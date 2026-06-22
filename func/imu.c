#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "imu.h"
#include "icm45686.h"

/* ────────────────────────────────────────────────────────────────────────────
 *  方案 A：寄存器直读 + UART DMA 异步外发（非阻塞）
 *
 *  数据流：上层在帧边界(frame_ready)调用 icm45686_stream_sample()
 *          → SPI 阻塞直读 14 字节(accel+gyro+temp，~9µs @12MHz)
 *          → 启动 UART TX DMA 把这 14 字节搬进 UART FIFO，立即返回
 *          → DMA 完成中断(共享 DMA_IRQ_0)清 busy 标志
 *
 *  说明：
 *   - SPI 直读 14B 很短，沿用阻塞接口；不值得为它再上一套 DMA+IRQ。
 *   - UART 发送走 DMA + 完成中断，主循环(摄像头发送链)不被阻塞。
 *   - 单帧缓冲 icm45686_stream_buf 在 DMA 搬运期间不可改写，由 busy 标志保护。
 *   - FIFO 路径整体暂缓，完整实现保留在文件末尾的 #if 0 块中。
 * ──────────────────────────────────────────────────────────────────────────*/

/* —— UART TX DMA —— */
static int                icm45686_uart_tx_dma_chan = -1;
static dma_channel_config icm45686_uart_tx_cfg;

/* 单帧发送缓冲 + 在途标志（volatile：ISR 与主循环共享） */
static uint8_t            icm45686_stream_buf[ICM45686_DataNum];
static volatile bool      icm45686_tx_inflight = false;

/* ────────────────────────────────────────────────────────────────────────────
 *  SPI 片选控制（IMU 读总线）
 * ──────────────────────────────────────────────────────────────────────────*/
static void icm45686_spi_select(void)
{
    gpio_put(ICM45686_SPI_CS, 0);
}

static void icm45686_spi_deselect(void)
{
    gpio_put(ICM45686_SPI_CS, 1);
}

/* 单字节寄存器写入：消除散落的 &(uint8_t){...} 写法 */
static uint8_t icm45686_write_u8(uint8_t reg, uint8_t val)
{
    return icm45686_write_reg(reg, &val, 1u);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  UART TX DMA 完成中断
 *  与摄像头共用 DMA_IRQ_0：shared handler 必须只认领/应答自己的通道
 * ──────────────────────────────────────────────────────────────────────────*/
static void icm45686_uart_dma_irq_handler(void)
{
    if (dma_channel_get_irq0_status((uint)icm45686_uart_tx_dma_chan)) {
        dma_channel_acknowledge_irq0((uint)icm45686_uart_tx_dma_chan);
        icm45686_tx_inflight = false;
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 *  UART TX DMA 初始化
 * ──────────────────────────────────────────────────────────────────────────*/
static void icm45686_uart_dma_init(void)
{
    if (icm45686_uart_tx_dma_chan >= 0) {
        return;
    }

    icm45686_uart_tx_dma_chan = dma_claim_unused_channel(true);

    /* 从内存读取数据发送到 UART：读地址递增、写地址固定(UART DR)、按 UART TX DREQ 节流 */
    icm45686_uart_tx_cfg = dma_channel_get_default_config((uint)icm45686_uart_tx_dma_chan);
    channel_config_set_transfer_data_size(&icm45686_uart_tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&icm45686_uart_tx_cfg, true);
    channel_config_set_write_increment(&icm45686_uart_tx_cfg, false);
    channel_config_set_dreq(&icm45686_uart_tx_cfg, uart_get_dreq(ICM45686_UART_INST, true));

    /* 完成中断：与 cam_pio.c 共用 DMA_IRQ_0（各自在 handler 里判断自己的通道） */
    dma_channel_set_irq0_enabled((uint)icm45686_uart_tx_dma_chan, true);
    irq_add_shared_handler(DMA_IRQ_0, icm45686_uart_dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  SPI 初始化（IMU 读总线，SPI0）
 * ──────────────────────────────────────────────────────────────────────────*/
void icm45686_spi_init(void)
{
    spi_init(ICM45686_SPI_INST, 12000000u);
    spi_set_format(ICM45686_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(ICM45686_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(ICM45686_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(ICM45686_SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(ICM45686_SPI_CS);
    gpio_set_dir(ICM45686_SPI_CS, GPIO_OUT);
    icm45686_spi_deselect();
}

void icm45686_uart_init(void)
{
    uart_init(ICM45686_UART_INST, ICM45686_UART_BAUD);
    uart_set_format(ICM45686_UART_INST, 8, 1, UART_PARITY_NONE);
    gpio_set_function(ICM45686_UART_TX_PIN, GPIO_FUNC_UART);
    /* UART RX 引脚未使用，保持默认输入状态即可 */
}

/* ────────────────────────────────────────────────────────────────────────────
 *  寄存器写入 - 阻塞操作（不使用 DMA）
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t icm45686_write_reg(uint8_t reg, const uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0u) || (length > ICM45686_SPI_MAX_TRANSFER)) {
        return 1u;
    }

    uint8_t tx[1u + ICM45686_SPI_MAX_TRANSFER];
    tx[0] = (uint8_t)(reg & 0x7Fu);

    for (size_t i = 0u; i < length; ++i) {
        tx[1u + i] = data[i];
    }

    icm45686_spi_select();
    int written = spi_write_blocking(ICM45686_SPI_INST, tx, length + 1u);
    icm45686_spi_deselect();

    return (written == (int)(length + 1u)) ? 0u : 1u;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  寄存器读取 - 阻塞操作（不使用 DMA）
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t icm45686_read_regs(uint8_t start_reg, uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0u) || (length > ICM45686_SPI_MAX_TRANSFER)) {
        return 1u;
    }

    uint8_t tx[1u + ICM45686_SPI_MAX_TRANSFER];
    uint8_t rx[1u + ICM45686_SPI_MAX_TRANSFER];
    tx[0] = (uint8_t)(0x80u | (start_reg & 0x7Fu));

    for (size_t i = 0u; i < length; ++i) {
        tx[1u + i] = 0x00u;
    }

    icm45686_spi_select();
    int transferred = spi_write_read_blocking(ICM45686_SPI_INST, tx, rx, length + 1u);
    icm45686_spi_deselect();

    if (transferred != (int)(length + 1u)) {
        return 1u;
    }

    for (size_t i = 0u; i < length; ++i) {
        data[i] = rx[1u + i];
    }

    return 0u;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  设备 ID 校验（确认 SPI/MISO 链路通）
 * ──────────────────────────────────────────────────────────────────────────*/
int32_t icm45686_verify_id(void)
{
    uint8_t who_am_i = 0u;
    if (icm45686_read_regs(ICM45686_WHO_AM_I, &who_am_i, 1u) != 0u) {
        return -1; /* 读取失败 */
    }
    return (who_am_i == ICM45686_WHO_AM_I_VALUE) ? 0 : -1;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  IMU 数据读取 - 直接读寄存器（不通过 FIFO）
 *  0x00~0x0D 连续 14 字节 = Accel(6) + Gyro(6) + Temp(2)
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t icm45686_getdata(uint8_t *data)
{
    if (data == NULL) {
        return 1u;
    }

    return icm45686_read_regs(ICM45686_ACCEL_DATA_X1_UI, data, ICM45686_DataNum);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  采一帧并经 UART DMA 异步外发（非阻塞）
 *  返回 0：已发起本帧发送；返回 1：上一帧未发完(跳过本帧) 或 读取失败
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t icm45686_stream_sample(void)
{
    /* 上一帧 DMA 还在读 stream_buf，跳过本帧，避免改写在途缓冲 */
    if (icm45686_tx_inflight) {
        return 1u;
    }

    if (icm45686_getdata(icm45686_stream_buf) != 0u) {
        return 1u;
    }

    icm45686_tx_inflight = true;
    dma_channel_configure(
        (uint)icm45686_uart_tx_dma_chan,
        &icm45686_uart_tx_cfg,
        &uart_get_hw(ICM45686_UART_INST)->dr,   /* 写：UART 数据寄存器 */
        icm45686_stream_buf,                     /* 读：样本缓冲 */
        ICM45686_DataNum,
        true);                                   /* 立即启动 */

    return 0u;
}

bool icm45686_uart_tx_busy(void)
{
    return icm45686_tx_inflight;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  IMU 初始化
 * ──────────────────────────────────────────────────────────────────────────*/
int32_t icm45686_init(void)
{
    icm45686_spi_init();
    icm45686_uart_init();
    icm45686_uart_dma_init();

    /* 1) 软复位：REG_MISC2 bit1=SOFT_RST，置 1 触发，硬件自清零 */
    (void)icm45686_write_u8(ICM45686_REG_MISC2, ICM45686_REG_MISC2_SOFT_RST);
    sleep_ms(2);

    /* 2) 校验设备 ID（确认 SPI/MISO 链路），失败直接返回让上层停机 */
    if (icm45686_verify_id() != 0) {
        return -1;
    }

    /* 3) 大端：让数据寄存器/FIFO/count 的高低字节与手册命名一致 */
    (void)icm45686_write_u8(ICM45686_SREG_CTRL, ICM45686_SREG_CTRL_BIG_ENDIAN);

    /* 4) 量程 / ODR / 电源模式 */
    (void)icm45686_write_u8(ICM45686_ACCEL_CONFIG0,
                            ICM45686_ACCEL_CONFIG0_FS_8G | ICM45686_ODR_400HZ);
    (void)icm45686_write_u8(ICM45686_GYRO_CONFIG0,
                            ICM45686_GYRO_CONFIG0_FS_1000DPS | ICM45686_ODR_400HZ);
    (void)icm45686_write_u8(ICM45686_PWR_MGMT0,
                            ICM45686_PWR_MGMT0_GYRO_LN | ICM45686_PWR_MGMT0_ACCEL_LN);

    /* 5) 等待传感器启动有效（加速度计 ~10ms，陀螺更久），留足余量 */
    sleep_ms(50);

    return 0;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  FIFO 路径（暂缓，保留以便后续扩展）
 *
 *  整体改用方案 A(直读)后，下面这套 SPI-DMA + FIFO burst 搬运暂不启用。
 *  若日后要走 FIFO（高 ODR / 批量 / 低功耗批处理），在此基础上还需：
 *    - count 是“包数”而非字节数：实际字节 = count × 16 + 1（accel+gyro 帧=16B）
 *    - 每个 16B 包首字节是 Header，需跳过/解析
 *    - FIFO_MAX_BURST 偏小(仅 2 包)，要按单次取包数放大缓冲
 *    - 在 icm45686_init() 中按手册顺序写 FIFO_CONFIG0/1/2/3 并 flush
 *    - 若要 IRQ 化，可复用上面的共享 DMA_IRQ_0 模式
 * ════════════════════════════════════════════════════════════════════════════*/
#if 0
/* FIFO / SPI-DMA 通道与缓冲 */
static int icm45686_tx_dma_chan = -1;
static int icm45686_rx_dma_chan = -1;
static dma_channel_config icm45686_tx_dma_cfg;
static dma_channel_config icm45686_rx_dma_cfg;

static uint8_t icm45686_fifo_tx_buf[1u + ICM45686_FIFO_MAX_BURST];
static uint8_t icm45686_fifo_rx_buf[1u + ICM45686_FIFO_MAX_BURST];
static size_t  icm45686_data_count = 0u;

uint8_t icm45686_cached_data[ICM45686_FIFO_MAX_BURST];

/* SPI TX/RX DMA 初始化 - 仅用于 FIFO 搬运 */
static void icm45686_dma_init(void)
{
    if ((icm45686_tx_dma_chan >= 0) && (icm45686_rx_dma_chan >= 0)) {
        return;
    }

    icm45686_tx_dma_chan = dma_claim_unused_channel(true);
    icm45686_rx_dma_chan = dma_claim_unused_channel(true);

    /* TX DMA 配置：从内存读取命令发送到 SPI */
    icm45686_tx_dma_cfg = dma_channel_get_default_config((uint)icm45686_tx_dma_chan);
    channel_config_set_transfer_data_size(&icm45686_tx_dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&icm45686_tx_dma_cfg, true);
    channel_config_set_write_increment(&icm45686_tx_dma_cfg, false);
    channel_config_set_dreq(&icm45686_tx_dma_cfg, spi_get_dreq(ICM45686_SPI_INST, true));

    /* RX DMA 配置：从 SPI 读取数据存入内存 */
    icm45686_rx_dma_cfg = dma_channel_get_default_config((uint)icm45686_rx_dma_chan);
    channel_config_set_transfer_data_size(&icm45686_rx_dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&icm45686_rx_dma_cfg, false);
    channel_config_set_write_increment(&icm45686_rx_dma_cfg, true);
    channel_config_set_dreq(&icm45686_rx_dma_cfg, spi_get_dreq(ICM45686_SPI_INST, false));
}

/* FIFO 读取 - 使用 DMA 搬运数据 */
uint8_t icm45686_read_FIFO(uint8_t *data)
{
    if ((data == NULL)) {
        return 1u;
    }

    /* 第一步：读取 FIFO 计数器（使用阻塞 SPI，非 DMA） */
    uint8_t fifo_cnt[2] = {0u};
    if (icm45686_read_regs(ICM45686_FIFO_COUNT_0, fifo_cnt, 2u) != 0u) {
        return 1u;
    }

    uint16_t count = ((uint16_t)fifo_cnt[0] << 8) | (uint16_t)fifo_cnt[1];
    if (count == 0u) {
        icm45686_data_count = 0u;   /* 空 FIFO 也要清零，避免 uart_send 重发旧缓冲 */
        return 0u;
    }

    if ((count > ICM45686_FIFO_MAX_BURST)) {
        return 1u;
    }
    icm45686_data_count = count;

    /* 第二步：初始化 DMA 通道（仅在第一次调用时） */
    icm45686_dma_init();

    /* 第三步：准备 FIFO 读取命令 */
    icm45686_fifo_tx_buf[0] = (uint8_t)(0x80u | (ICM45686_FIFO_DATA & 0x7Fu));
    for (size_t i = 0u; i < count; ++i) {
        icm45686_fifo_tx_buf[1u + i] = 0x00u;
    }

    /* 第四步：配置 DMA 搬运参数 */
    dma_channel_configure(
        (uint)icm45686_rx_dma_chan,
        &icm45686_rx_dma_cfg,
        icm45686_fifo_rx_buf,
        &spi_get_hw(ICM45686_SPI_INST)->dr,
        count + 1u,
        false);

    dma_channel_configure(
        (uint)icm45686_tx_dma_chan,
        &icm45686_tx_dma_cfg,
        &spi_get_hw(ICM45686_SPI_INST)->dr,
        icm45686_fifo_tx_buf,
        count + 1u,
        false);

    /* 第五步：启动 FIFO 搬运 */
    while(dma_channel_is_busy((uint)icm45686_tx_dma_chan) == 1u);
    while(dma_channel_is_busy((uint)icm45686_rx_dma_chan) == 1u);
    icm45686_spi_select();
    dma_start_channel_mask((1u << icm45686_rx_dma_chan) | (1u << icm45686_tx_dma_chan));
    dma_channel_wait_for_finish_blocking((uint)icm45686_tx_dma_chan);
    dma_channel_wait_for_finish_blocking((uint)icm45686_rx_dma_chan);
    icm45686_spi_deselect();

    /* 第六步：提取结果 */
    for (size_t i = 0u; i < count; ++i) {
        data[i] = icm45686_fifo_rx_buf[1u + i];
    }

    return 0u;
}

/* 使用串口，将 FIFO 的数据发送（旧的阻塞实现） */
uint8_t icm45686_uart_send(const uint8_t *data)
{
    if ((data == NULL) || (icm45686_data_count == 0u)) {
        return 1u;
    }

    while(dma_channel_is_busy((uint)icm45686_uart_tx_dma_chan) == 1u);
    dma_channel_configure(
        (uint)icm45686_uart_tx_dma_chan,
        &icm45686_uart_tx_cfg,
        &uart_get_hw(ICM45686_UART_INST)->dr,
        data,
        icm45686_data_count,
        true);

    dma_channel_wait_for_finish_blocking((uint)icm45686_uart_tx_dma_chan);
    icm45686_data_count = 0;
    return 0;
}
#endif /* FIFO 路径暂缓 */
