/**
 * @file hstx.c
 * @brief RP2354 HSTX 并行输出（SDR 单沿数据 + 差分时钟）
 *
 * ============================================================
 * ⚠ 当前实际配置 (以代码 hstx_init() 为准，下方旧注释多已过时)
 * ============================================================
 *   时钟源:   clk_hstx = 48 MHz (timer.c: clk_sys 144MHz / 3)
 *   输出时钟: 48 MHz = clk_hstx / CLKDIV(=1)，GPIO12/13 差分对
 *   数据:     GPIO16-19 单端 SDR(单沿)，每个上升沿送 4 bit，2 个周期 1 字节
 *   CSR:      CLKDIV=1, SHIFT=4, N_SHIFTS=2
 *   注: 数据脚 INV=0 即普通输出；仅时钟脚为差分(bit1 = CLK|INV)。
 *       下面关于"差分对/INV=1/SHIFT=2/CLKDIV=2"的描述属于旧版本，仅作历史参考。
 *
 * ============================================================
 * 原理说明 (Architecture Overview) —— 以下为旧版本说明，已过时
 * ============================================================
 *
 * 数据流:
 *   buffer[] (uint8_t)
 *     → hstx_dma_cfg            以 8-bit 单元按字节搬运
 *     → DMA (读增量, 写固定)    通过 DREQ_HSTX 节流
 *     → hstx_fifo_hw->fifo      HSTX FIFO 写端口 (地址 0x50100004)
 *     → 8×32-bit 硬件 FIFO     硬件内部缓冲
 *     → 32-bit 移位寄存器       每周期右旋 SHIFT=2 bit
 *     → bit[4..7] 输出寄存器   选择数据位并可选反相
 *     → GPIO16-19 差分驱动器   输出差分信号
 *
 * ============================================================
 * HSTX 初始化调用链 (Initialization Call Chain)
 * ============================================================
 *
 * 1) hstx_gpio_init()
 *    gpio_set_function(pin, GPIO_FUNC_HSTX)
 *    → 写 IO_BANK0 的 GPIO_CTRL 寄存器 FUNCSEL 字段 = 0 (HSTX)
 *    → GPIO16/17/18/19 切换到 HSTX 外设驱动
 *
 * 2) hstx_init()
 *    a. hstx_ctrl_hw->csr = 0          → 关闭 HSTX 输出 (EN=0)
 *    b. hstx_ctrl_hw->bit[N] = ...     → 配置每个输出位的数据来源和极性
 *       寄存器字段:
 *         SEL_P [4:0]  = 移位寄存器的哪一位驱动正半周
 *         SEL_N [12:8] = 移位寄存器的哪一位驱动负半周
 *         INV   [16]   = 对输出取反 (用于差分负极)
 *    c. hstx_fifo_hw->stat = WOF_BITS  → 清除 FIFO 溢出 sticky 位 (W1C)
 *    d. hstx_ctrl_hw->csr = ...        → 配置时钟分频/移位参数并使能 (EN=1)
 *       CSR 字段:
 *         CLKDIV   [31:28] = 2  → 输出时钟 = HSTX_CLK / 2
 *         N_SHIFTS [20:16] = 16 → 每个 FIFO 字旋转 16 次后取下一字
 *         SHIFT    [12:8]  = 2  → 每次右旋 2 bit
 *         EN       [0]     = 1  → 使能 HSTX
 *    移位规则: 32 bit / 2 bit = 16 次恰好消耗整个 32-bit 字 ✓
 *
 * 3) hstx_dma_init()
 *    dma_claim_unused_channel()         → 从 SDK 分配一个空闲 DMA 通道
 *    channel_config_set_dreq(&c, DREQ_HSTX)
 *    → DREQ_HSTX = 52 (dreq.h)
 *    → DMA 只在 HSTX FIFO 未满时才发起传输，防止覆盖
 *    dma_channel_configure(dst=&fifo, src=hstx_tx_words, count=0, trigger=false)
 *    → 预配置通道，count=0 表示先不触发
 *
 * 4) hstx_tx_start()
 *    dma_channel_configure(..., count=words, trigger=true)
 *    → trigger=true 立即启动 DMA
 *    → DMA 每当 DREQ_HSTX 有效 (FIFO 有空位) 就搬运一个 uint8_t
 *    → 全部 words 搬完后 DMA 停止，hstx_dma_busy() 返回 false
 *
 * ============================================================
 * FIFO 位置 (FIFO Location)
 * ============================================================
 *
 *   HSTX_FIFO_BASE = 0x50100000 (RP2350 APB 地址空间)
 *   hstx_fifo_hw->stat = HSTX_FIFO_BASE + 0x00  (状态: LEVEL/FULL/EMPTY/WOF)
 *   hstx_fifo_hw->fifo = HSTX_FIFO_BASE + 0x04  (写入口, 只写)
 *   FIFO 深度: 8 × 32-bit 字
 *
 * ============================================================
 * 差分机制详解 (Differential Output Mechanism)
 * ============================================================
 *
 *  GPIO 与 HSTX bit 的对应 (GPIO N → HSTX_bit[N-12]):
 *    GPIO16 → bit[4]   GPIO17 → bit[5]
 *    GPIO18 → bit[6]   GPIO19 → bit[7]
 *
 *  差分对配置:
 *    lane 0 (LCD_ADDRESS):
 *      bit[7] GPIO19 正极: SEL_P=0, SEL_N=0, INV=0 → 输出移位器 bit0
 *      bit[4] GPIO16 负极: SEL_P=0, SEL_N=0, INV=1 → 输出移位器 bit0 取反
 *    lane 1 (LCD_ADDRESS_B):
 *      bit[6] GPIO18 正极: SEL_P=1, SEL_N=1, INV=0 → 输出移位器 bit1
 *      bit[5] GPIO17 负极: SEL_P=1, SEL_N=1, INV=1 → 输出移位器 bit1 取反
 *
 *  一个字节如何拆分为差分输出 (以 0xAA = 10101010b 为例):
 *
 *    HSTX 每个时钟周期取移位寄存器最低2位输出，然后右旋2位:
 *
 *    时钟周期  低2位(从32bit字)  lane0(bit0)  lane1(bit1)
 *    ────────  ────────────────  ──────────   ──────────
 *      1       b1=1, b0=0        GPIO19=0,16=1  GPIO18=1,17=0
 *      2       b3=1, b2=0        GPIO19=0,16=1  GPIO18=1,17=0
 *      3       b5=1, b4=0        GPIO19=0,16=1  GPIO18=1,17=0
 *      4       b7=1, b6=0        GPIO19=0,16=1  GPIO18=1,17=0
 *    (一个字节需要4个时钟周期，因为每周期输出2bit，4×2=8bit)
 *
 *    以 0x55 = 01010101b 为例:
 *      每个周期: b1=0,b0=1 → lane0=1(正), lane1=0(正)
 *
 * ============================================================
 * 示例 buffer (Test Buffer Example)
 * ============================================================
 *
 *   uint8_t buffer[256];
 *   for (int i = 0; i < 256; i++)
 *       buffer[i] = (i & 1) ? 0x55 : 0xAA;
 *   // 0xAA=10101010: lane0持续低,lane1持续高
 *   // 0x55=01010101: lane0持续高,lane1持续低
 *   // 交替产生两个差分对互补跳变的测试波形
 *
 * ============================================================
 */

#include <string.h>

/* pico-sdk 硬件抽象层 */
#include "hardware/dma.h"              /* dma_claim_unused_channel, dma_channel_configure 等 */
#include "hardware/gpio.h"             /* gpio_set_function, GPIO_FUNC_HSTX */
#include "hardware/regs/dreq.h"        /* DREQ_HSTX = 52 */
#include "hardware/regs/hstx_ctrl.h"  /* HSTX_CTRL_CSR_*, HSTX_CTRL_BIT0_* 宏定义 */
#include "hardware/regs/hstx_fifo.h"  /* HSTX_FIFO_STAT_WOF_BITS */
#include "hardware/structs/hstx_ctrl.h" /* hstx_ctrl_hw (指向 HSTX_CTRL_BASE) */
#include "hardware/structs/hstx_fifo.h" /* hstx_fifo_hw (指向 HSTX_FIFO_BASE) */

#include "hstx.h"

/* ============================================================
 * 内部常量与静态存储
 * ============================================================ */

/* HSTX_TX_BYTES defined in header (byte units) */

/** DMA 通道编号，-1 表示未初始化 */
static int hstx_dma_chan = -1;

/** 保存 DMA 通道配置，供 hstx_tx_start() 复用 */
static dma_channel_config hstx_dma_cfg;

/* 内部示例 buffer (以字节为单位) */
static uint8_t hstx_demo_buffer[HSTX_TX_BYTES];


/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * 构造 HSTX BITx 寄存器的值，用于差分数据输出模式。
 *
 * HSTX_CTRL_BITx 寄存器布局:
 *   [4:0]  SEL_P — 选择移位寄存器的第几位驱动 GPIO 正半周输出
 *   [12:8] SEL_N — 选择移位寄存器的第几位驱动 GPIO 负半周输出
 *   [16]   INV   — 对最终输出电平取反 (差分负极需置1)
 *   [17]   CLK   — 置1则输出时钟而非数据 (本demo不用)
 *
 * 在 RAW 模式下 SEL_P 和 SEL_N 设为相同值即可（无 DDR）。
 *
 * @param data_bit  选取移位寄存器的第几位 (0=lane0, 1=lane1)
 * @param invert    true → 差分负极 (电平取反)
 */
static uint32_t make_hstx_bit_cfg(uint32_t data_bit_P, uint32_t data_bit_N, bool invert) {
    uint32_t v = (data_bit_P << HSTX_CTRL_BIT0_SEL_P_LSB) |
                 (data_bit_N << HSTX_CTRL_BIT0_SEL_N_LSB);
    if (invert) {
        v |= HSTX_CTRL_BIT0_INV_BITS;
    }
    return v;
}


/* ============================================================
 * 公开函数实现
 * ============================================================ */

/**
 * 第1步: GPIO 初始化
 *
 * gpio_set_function() 写 IO_BANK0 的 GPIOx_CTRL 寄存器:
 *   FUNCSEL = 0 → GPIO_FUNC_HSTX
 * 之后 HSTX 外设直接驱动这4个引脚，无需软件控制电平。
 */
void hstx_gpio_init(void) {
    gpio_set_function(HSTX_NEG_0_PIN, GPIO_FUNC_HSTX); /* GPIO16: LCD_ADDRESS  负极 */
    gpio_set_function(HSTX_NEG_1_PIN, GPIO_FUNC_HSTX); /* GPIO17: LCD_ADDRESS_B 负极 */
    gpio_set_function(HSTX_POS_1_PIN, GPIO_FUNC_HSTX); /* GPIO18: LCD_ADDRESS_B 正极 */
    gpio_set_function(HSTX_POS_0_PIN, GPIO_FUNC_HSTX); /* GPIO19: LCD_ADDRESS  正极 */
    gpio_set_function(12u, GPIO_FUNC_HSTX); // HSTX bit0
    gpio_set_function(13u, GPIO_FUNC_HSTX); // HSTX bit1
}

/**
 * 第2步: HSTX 控制器初始化
 *
 * 寄存器操作顺序:
 *   1. csr=0       — 关闭输出，安全配置
 *   2. bit[7..4]   — 设置差分对的数据选择和极性
 *   3. stat=WOF    — 清除 FIFO 溢出 sticky 位
 *   4. csr=配置|EN — 设置移位参数并启动
 *
 * 移位参数选择依据:
 *   每个 32-bit FIFO 字需要完整输出:
 *   SHIFT=2, N_SHIFTS=16 → 2 × 16 = 32 bit，恰好覆盖整字。
 *
 * CLKDIV=2 → HSTX 输出时钟 = CLK_HSTX / 2。
 * CSR 寄存器地址: HSTX_CTRL_BASE + 0x00
 */
void hstx_init(void) {
    /* 关闭 HSTX (EN=0)，所有寄存器在此期间可安全写入 */
    hstx_ctrl_hw->csr = 0;

    /*
     * GPIO N 对应 HSTX bit[N-12]，因此 GPIO16-19 应写 bit[4..7]。
     *
     * SDR 单沿单端输出：SEL_P == SEL_N，使输出在整个时钟周期内保持稳定
     * （不再像 DDR 那样在周期中间翻转），FPGA 上升沿采样一次即可。
     * 每个时钟周期送 4 bit（GPIO16-19 共 4 根数据线）。
     */
    hstx_ctrl_hw->bit[7] = make_hstx_bit_cfg(0u, 0u, false); // GPIO19 = 数据 bit0
    hstx_ctrl_hw->bit[6] = make_hstx_bit_cfg(1u, 1u, false); // GPIO18 = 数据 bit1
    hstx_ctrl_hw->bit[5] = make_hstx_bit_cfg(2u, 2u, false); // GPIO17 = 数据 bit2
    hstx_ctrl_hw->bit[4] = make_hstx_bit_cfg(3u, 3u, false); // GPIO16 = 数据 bit3

    /* 差分时钟对：GPIO12 = CLK，GPIO13 = CLK 反相（差分负极），保留 */
    hstx_ctrl_hw->bit[0] = HSTX_CTRL_BIT0_CLK_BITS;
    hstx_ctrl_hw->bit[1] = HSTX_CTRL_BIT1_CLK_BITS | HSTX_CTRL_BIT1_INV_BITS;

    /*
     * 清除 FIFO 溢出 sticky 位 (WOF, 写1清零 W1C)。
     * HSTX_FIFO_STAT 地址: HSTX_FIFO_BASE + 0x00
     */
    hstx_fifo_hw->stat = HSTX_FIFO_STAT_WOF_BITS;

    /*
     * 配置 CSR 并使能 HSTX (SDR 单沿模式):
     *
     *   CLKDIV   [31:28] = 1  → 输出时钟 = clk_hstx / 1 = 48 MHz
     *                          (SDR 每周期只能移位 1 次，CLKDIV 必须为 1；
     *                           调频靠 timer.c 对 clk_hstx 分频，不靠 CLKDIV)
     *   SHIFT    [12:8]  = 4  → 每个时钟周期右旋 4 位 (GPIO16-19 共 4 根数据线)
     *   N_SHIFTS [20:16] = 2  → 移位 2 次 (4×2=8bit) 消耗一个字节后从 FIFO 取下一字节
     *   EN       [0]     = 1  → 使能 HSTX
     *
     * 宏来自 hardware/regs/hstx_ctrl.h
     */
    hstx_ctrl_hw->csr =
        (1u << HSTX_CTRL_CSR_CLKDIV_LSB)   |    /* bit[31:28] */
        (4u << HSTX_CTRL_CSR_SHIFT_LSB)    |    /* bit[12:8]  */
        (2u << HSTX_CTRL_CSR_N_SHIFTS_LSB) |    /* bit[20:16] */
        HSTX_CTRL_CSR_EN_BITS;                  /* bit[0]     */
}





/**
 * 第3步: DMA 通道初始化
 *
 * DMA 如何触发 HSTX (DREQ 机制):
 *   - DREQ_HSTX = 52，定义于 hardware/regs/dreq.h
 *   - HSTX FIFO 每有一个空位，就向 DMA 发出一次 DREQ 请求
 *   - DMA 收到 DREQ 后搬运1个 32-bit 字到 FIFO 写入口
 *   - 硬件自动节流：FIFO 满时 DREQ 无效，DMA 暂停等待
 *
 * DMA 通道配置:
 *   - 传输单元: 8-bit (DMA_SIZE_8)
 *   - 读地址自增: true  (遍历 uint8_t buffer[])
 *   - 写地址固定: false (始终写到同一个 FIFO 入口)
 *   - DREQ: DREQ_HSTX (= 52)
 *   - 目标地址: &hstx_fifo_hw->fifo (0x50100004)
 */
void hstx_dma_init(void) {
    if (hstx_dma_chan >= 0) {
        return; /* 已初始化，避免重复申请 */
    }

    /* 从 SDK 的通道位图中分配一个空闲 DMA 通道，失败则 panic */
    hstx_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config((uint)hstx_dma_chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);      /* uint8_t transfer unit */
    channel_config_set_read_increment(&c, true);                 /* 读指针自增遍历源数组 */
    channel_config_set_write_increment(&c, false);           /* 写指针固定在 FIFO 入口 */
    channel_config_set_dreq(&c, DREQ_HSTX);                 /* HSTX FIFO 空位触发 DMA      */
    hstx_dma_cfg = c;

    /*
* 预配置通道 (count=0, trigger=false 不立即发送):
     *   读: hstx_tx_words (后续 hstx_tx_start() 时才确定实际字数)
     *   写: hstx_fifo_hw->fifo — HSTX FIFO 唯一写入口
*/
    dma_channel_configure(
        (uint)hstx_dma_chan,
        &c,
        &hstx_fifo_hw->fifo,    /* 目标: HSTX FIFO 写入口 (只写寄存器) */
        NULL,                   /* 源: 从DVP传输过来的uint8_t数组,初始化阶段不写入 */
        0,                      /* 传输字数 = 0，此处不启动 */
        false);                 /* 不触发 */
}

/**
 * 第4步: 启动一次 DMA 传输
 *
 * 本函数会:
 *   1. 将 buffer 打包为 uint32_t (小端对齐)
 *   2. 重新配置 DMA 传输字数并触发发送
 *
 * 触发 (trigger=true) 后 DMA 立即运行:
 *   每当 HSTX FIFO 有空位 → DREQ_HSTX → DMA 搬运1字 → HSTX 移位输出
 *   → 直到 words 个字全部搬完，DMA 自动停止。
 *
 * 注意: 调用前需确保上一次传输已完成 (hstx_dma_busy() == false)。
 *
 * @param buffer    待发送的字节数组
 * @param len_bytes 字节数 (不超过 HSTX_TX_DEMO_BYTES=2560 < DMA_COUNT_MAX=65535)
*/
void hstx_tx_start(const uint8_t *buffer) {
    if (hstx_dma_chan < 0) {
        hstx_dma_init(); /* 懒初始化: 若未调用 hstx_dma_init() 则自动初始化 */
    }

    while (dma_channel_is_busy((uint)hstx_dma_chan));
    /*
     * 重新配置并立即触发 DMA:
     *   trigger=true → 等价于调用完后再调用 dma_channel_start()
     *   DMA_SIZE_8: 每次传输 1 字节到 HSTX FIFO
     *   HSTX 每个 FIFO entry 只消费低 8 bit
     */
    dma_channel_configure(
        (uint)hstx_dma_chan,
        &hstx_dma_cfg,
        &hstx_fifo_hw->fifo,    /* 写地址: HSTX FIFO */
        buffer,                 /* read address: uint8_t byte stream */
        HSTX_TX_DMA_COUNT,      /* DMA_SIZE_8 transfer_count is bytes */
        true);                  /* 立即触发 */
}


void hstx_tx_stop(void) {
    if (hstx_dma_chan >= 0) {
        dma_channel_abort((uint)hstx_dma_chan);
    }
}
/**
 * 查询 DMA 传输是否仍在进行。
 *
 * dma_channel_is_busy() 读取 DMA 通道的 CTRL_TRIG.BUSY 位。
 * 返回 true 表示 DMA 尚未完成全部字的搬运。
 */
bool hstx_dma_busy(void) {
    return (hstx_dma_chan >= 0) && dma_channel_is_busy((uint)hstx_dma_chan);
}
