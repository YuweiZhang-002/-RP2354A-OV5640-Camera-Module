/*
 * cam_pio.c  —  PIO 摄像头采集 + DMA 行级三缓冲环实现
 *
 * 数据流（单向闭合）：
 *   OV5640 DVP 并行口 (8-bit data + PCLK/HREF/VSYNC)
 *     ▼  GPIO 12-19 / 11 / 10 / 20  ← pio_gpio_init() 接管复用
 *   PIO0 SM0                       ← cam_pio.pio 程序
 *     ▼  RX FIFO 非空 → 拉高 RX DREQ
 *   DMA Channel (8-bit/byte)       ← 写入 cam_frame_buf[w]
 *     ▼  写满一行 (CAPTURE_BYTES) → DMA 完成中断
 *   完成中断：把该块标记为"就绪"，立即重装到环里下一块（无忙等、无旧 HSTX 调用）
 *     ▼
 *   主循环 cam_acquire_line() 取走就绪块 → PIO1 发送 → cam_release_line() 归还
 *
 * 关键设计：
 *   - 采集层只生产 + 打标记，不直接驱动发送层（解耦）。
 *   - 行完成事件取自 DMA 传输完成（权威），而非 HREF 边沿。
 *   - 重装在中断里以两条寄存器写完成（write_addr + 触发 trans_count），
 *     行间间隙极短，配合 PIO RX FIFO 足以避免溢出。
 *     （如需绝对零间隙，可改用两条 DMA 通道 CHAIN_TO 硬件乒乓。）
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "cam_pio.h"
#include "cam_pio.pio.h"


/* ────────────────────────────────────────────────────────────────────────────
 *  模块内部状态（对外不可见）
 * ──────────────────────────────────────────────────────────────────────────*/
static PIO  cam_pio      = pio0;
static uint cam_sm       = 0u;
static int  cam_dma_chan = -1;
static dma_channel_config cam_dma_cfg;

static void cam_gpio_irq_callback(uint gpio, uint32_t events);
static void cam_dma_irq_handler(void);

/* 行级三缓冲环 */
static uint8_t cam_frame_buf[CAM_NUM_BUFFERS][CAPTURE_BYTES];

/*
 * 用三个单调递增序号描述生产/消费进度（无符号差值在回绕下仍成立）：
 *   cam_prod_seq — 已写满的行数；DMA 当前写入 cam_frame_buf[prod % N]
 *   cam_send_seq — 已派发给 PIO1 的行数；发送中的块下标 = (send-1) % N
 *   cam_cons_seq — 已发送完成并归还的行数
 * 不变式：cam_cons_seq <= cam_send_seq <= cam_prod_seq，且 send-cons <= 1（单条在飞）。
 *
 * 写者：prod 仅 DMA 中断写；send/cons 仅消费者（主循环）写。
 * 32-bit 对齐读写在 Cortex-M33 上原子，跨上下文无需加锁。
 */
static volatile uint32_t cam_prod_seq = 0u;
static volatile uint32_t cam_send_seq = 0u;
static volatile uint32_t cam_cons_seq = 0u;

volatile uint8_t  frame_ready       = 0u;
volatile uint32_t cam_overrun_count = 0u;
volatile uint32_t cam_frame_count   = 0u;
volatile uint32_t cam_line_count    = 0u;
volatile uint32_t cam_linem1_count   = 0u;
volatile uint32_t cam_line0_count    = 0u;
volatile uint32_t cam_linep1_count   = 0u;
volatile uint8_t  cam_filter_ready   = 0u;
volatile bool     dma_flag          = false;
volatile bool     comp_flag         = false;
static volatile uint8_t cam_warmup_discard_frames = 0u;

/* ────────────────────────────────────────────────────────────────────────────
 *  GPIO 初始化
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_gpio_init(void)
{
    for (uint pin = CAM_DATA_PIN_BASE; pin < CAM_DATA_PIN_BASE + CAM_DATA_PIN_COUNT; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_set_pulls(pin, true, false);
    }

    gpio_init(CAM_PCLK_PIN);
    gpio_set_dir(CAM_PCLK_PIN, GPIO_IN);
    gpio_set_pulls(CAM_PCLK_PIN, true, false);

    gpio_init(CAM_HREF_PIN);
    gpio_set_dir(CAM_HREF_PIN, GPIO_IN);
    gpio_set_pulls(CAM_HREF_PIN, true, false);

    gpio_init(CAM_VSYNC_PIN);
    gpio_set_dir(CAM_VSYNC_PIN, GPIO_IN);
    gpio_set_pulls(CAM_VSYNC_PIN, true, false);

    gpio_init(CAM_DEBUG_PIN);
    gpio_set_dir(CAM_DEBUG_PIN, GPIO_OUT);
    gpio_put(CAM_DEBUG_PIN, 0);

}

/* ────────────────────────────────────────────────────────────────────────────
 *  PIO 初始化
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_pio_init(void)
{
    uint offset = pio_add_program(cam_pio, &cam_capture_program);

    for (uint i = 0u; i < CAM_DATA_PIN_COUNT; ++i) {
        pio_gpio_init(cam_pio, CAM_DATA_PIN_BASE + i);
    }
    pio_gpio_init(cam_pio, CAM_PCLK_PIN);
    pio_gpio_init(cam_pio, CAM_HREF_PIN);
    pio_gpio_init(cam_pio, CAM_VSYNC_PIN);

    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm,
                                   CAM_DATA_PIN_BASE, CAM_DATA_PIN_COUNT, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_PCLK_PIN,  1u, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_HREF_PIN,  1u, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_VSYNC_PIN, 1u, false);

    cam_capture_program_init(cam_pio, cam_sm, offset);

    pio_sm_set_enabled(cam_pio, cam_sm, false);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  DMA 重装（中断内快速路径）
 *
 *  读地址固定为 PIO RX FIFO（read_increment=false，传输后不变），
 *  因此重装只需重写 write_addr 并通过 trans_count 触发寄存器重置计数并启动。
 *  仅两条寄存器写，行间间隙最小。
 * ──────────────────────────────────────────────────────────────────────────*/
static inline void cam_dma_rearm(uint32_t idx)
{
    dma_channel_set_write_addr((uint)cam_dma_chan, cam_frame_buf[idx], false);
    dma_channel_set_transfer_count((uint)cam_dma_chan, CAPTURE_BYTES, true);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  DMA 完成中断：一行写满时触发
 *    - 仅做轻量交接：发布刚写满的块，重装到环里下一块
 *    - 环满（消费者落后）则丢弃最新一行并累加计数，绝不阻塞采集
 * ──────────────────────────────────────────────────────────────────────────*/
static void cam_dma_irq_handler(void)
{
    if (!dma_channel_get_irq0_status((uint)cam_dma_chan)) {
        return;
    }
    dma_channel_acknowledge_irq0((uint)cam_dma_chan);

    cam_line_count = cam_prod_seq; /* 行计数 */

    if (cam_prod_seq % CAPTURE_LINES > 2u) {
        cam_linem1_count = (cam_prod_seq - 2u) % CAM_NUM_BUFFERS;    /* 采集链滤波计算行计数器 */
        cam_line0_count  = (cam_prod_seq - 1u) % CAM_NUM_BUFFERS;    /* 采集链滤波计算行计数器 */
        cam_linep1_count = cam_prod_seq % CAM_NUM_BUFFERS;           /* 采集链滤波计算行计数器 */
        cam_filter_ready = 1u;                                       /* 采集链滤波计算就绪标记 */
    }
    uint32_t next = cam_prod_seq + 1u;

    if ((next - cam_cons_seq) >= CAM_NUM_BUFFERS) {
        /* 环满：下一块仍被下游占用。丢弃刚采到的这一行，重写同一块。 */
        cam_overrun_count++;
        cam_dma_rearm(cam_prod_seq % CAM_NUM_BUFFERS);
    } else {
        /* 发布刚写满的块（prod_seq 前进），重装到下一块继续采集。 */
        cam_prod_seq = next;
        cam_dma_rearm(cam_prod_seq % CAM_NUM_BUFFERS);
        dma_flag = !dma_flag;
        gpio_put(CAM_DEBUG_PIN, dma_flag);
        
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 *  DMA 初始化：通道 + DREQ + 完成中断 + VSYNC 帧边界中断
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_dma_init(void)
{
    if (cam_dma_chan >= 0) {
        return;
    }

    cam_dma_chan = dma_claim_unused_channel(true);

    cam_dma_cfg = dma_channel_get_default_config((uint)cam_dma_chan);
    channel_config_set_transfer_data_size(&cam_dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cam_dma_cfg, false);
    channel_config_set_write_increment(&cam_dma_cfg, true);
    channel_config_set_dreq(&cam_dma_cfg, pio_get_dreq(cam_pio, cam_sm, false));

    /* 行写满 → DMA 完成中断（DMA_IRQ_0），中断里自维持地重装下一块 */
    dma_channel_set_irq0_enabled((uint)cam_dma_chan, true);
    irq_add_shared_handler(DMA_IRQ_0, cam_dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    /* VSYNC 作为帧边界事件：仅做轻量记账与帧就绪标记。
     * 注意：HREF 不参与 CPU 交接，数据流仍由 PIO + DMA 负责。 */
    gpio_set_irq_enabled_with_callback(CAM_VSYNC_PIN,
                                       GPIO_IRQ_EDGE_RISE,
                                       true,
                                       cam_gpio_irq_callback);
    gpio_set_irq_enabled(CAM_VSYNC_PIN, GPIO_IRQ_EDGE_FALL, true);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  启动采集
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_capture_start(void)
{
    if (cam_dma_chan < 0) {
        cam_dma_init();
    }

    /* 复位缓冲环进度 */
    cam_prod_seq      = 0u;
    cam_send_seq      = 0u;
    cam_cons_seq      = 0u;
    cam_overrun_count = 0u;
    cam_frame_count   = 0u;
    frame_ready       = 0u;
    cam_line_count    = 0u;
    dma_flag          = false;
    gpio_put(CAM_DEBUG_PIN, 0);

    /* 首块完整配置（设定读地址=RX FIFO，写地址=buf0，计数与触发）；
     * 之后的重装走 cam_dma_rearm() 快速路径。 */
    dma_channel_configure(
        (uint)cam_dma_chan,
        &cam_dma_cfg,
        cam_frame_buf[0],
        &cam_pio->rxf[cam_sm],
        CAPTURE_BYTES,
        true);

    pio_sm_set_enabled(cam_pio, cam_sm, true);
}

void cam_discard_next_frames(uint8_t frames)
{
    cam_warmup_discard_frames = frames;
    frame_ready = 0u;
}

const uint8_t *cam_get_buffer(uint32_t index)
{
    return cam_frame_buf[index % CAM_NUM_BUFFERS];
}

/* ────────────────────────────────────────────────────────────────────────────
 *  关闭采集
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_capture_stop(void)
{
    pio_sm_set_enabled(cam_pio, cam_sm, false);

    if (cam_dma_chan >= 0) {
        dma_channel_abort((uint)cam_dma_chan);
    }

    pio_sm_clear_fifos(cam_pio, cam_sm);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  采集/发送单点交接（消费者侧）
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t *cam_acquire_line(void)
{
    if (cam_send_seq != cam_cons_seq) {
        return NULL;                       /* 还有一行在飞，等其归还 */
    }
    if (cam_prod_seq == cam_send_seq) {
        return NULL;                       /* 没有就绪行 */
    }

    uint8_t *buf = cam_frame_buf[cam_send_seq % CAM_NUM_BUFFERS];
    cam_send_seq++;                        /* 标记为在飞 */
    return buf;
}

void cam_release_line(void)
{
    if (cam_send_seq != cam_cons_seq) {
        cam_cons_seq++;                    /* 归还在飞缓冲，供采集复用 */
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 *  GPIO 中断回调：仅处理 VSYNC 帧边界（轻量标记）
 * ──────────────────────────────────────────────────────────────────────────*/
static void cam_gpio_irq_callback(uint gpio, uint32_t events)
{
    if (gpio == CAM_VSYNC_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        comp_flag = false;
        cam_frame_count++;
    }

    if (gpio == CAM_VSYNC_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        if (cam_warmup_discard_frames > 0u) {
            cam_warmup_discard_frames--;
            frame_ready = 0u;
            return;
        }
        comp_flag = true;
        frame_ready = 1u;
    }
}

/* ------------------------------------------------------------------
 * 4x4-buffer scaffold implementation
 * - 提供预留函数以便未来将行级三缓冲替换为 4 个 4-line blocks。
 * - 当前实现为占位：记录意图并在将来扩展点放置 TODO。
 * - 该函数不会在当前默认路径中自动切换行为，需由上层显式调用并
 *   在切换前完成内存与时序验证。
 */
void cam_enable_4x4_scaffold(void)
{
    /* TODO: 实现迁移逻辑：
     *  - 分配或重构缓冲数组为 [CAM_4X4_NUM_BUFFERS][CAPTURE_CHUNK_LINES * CAPTURE_BYTES]
     *  - 更新 cam_dma_rearm() 以按 chunk 写地址与计数重装
     *  - 在 DMA 完成中断里维护行计数并在 chunk 完成时发布
     *  - 更新 cam_acquire_line()/cam_release_line() 以按 chunk 返回指针
     *  - 验证 RAM 占用（CAM_4X4_NUM_BUFFERS * CAPTURE_CHUNK_LINES * CAPTURE_BYTES）
     */
    /* 占位注释：当前仅作文档记录，不修改运行时数据结构 */
}
