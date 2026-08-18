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
 *   Core 0 主循环 cam_acquire_line() 认领三行窗口 → Sobel → cam_release_line() 归还旧行
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
#include "hardware/sync.h"

#include "cam_pio.h"
#include "cam_pio.pio.h"


/* ────────────────────────────────────────────────────────────────────────────
 *  模块内部状态（对外不可见）
 * ──────────────────────────────────────────────────────────────────────────*/
static PIO  cam_pio      = pio0;
static uint cam_sm       = 0u;
static uint cam_program_offset = 0u;
static int  cam_dma_chan = -1;
static dma_channel_config cam_dma_cfg;

/*
 * SM1 = VSYNC 脉宽鉴别器（cam_pio.pio 的 vsync_gate 程序）。
 * 它把"高电平连续超过 58.7us"的 VSYNC 脉冲的下降沿变成两个 PIO IRQ 标志：
 *   IRQ4 → SM0 的丢帧序列消费（wait 1 irq 4 命中时硬件自动清零）
 *   IRQ0 → 路由到 PIO0_IRQ_0，CPU 只对合格边界记账
 * 10ns 毛刺（实测上电 2 个、稳态 1 个）在鉴别器内部就被丢弃，
 * 不会进入 SM0，也不会进入 CPU。
 */
static uint cam_vsync_sm = 1u;
static uint cam_vsync_program_offset = 0u;

/* SM1 → SM0 的合格帧边界标志 */
#define CAM_VSYNC_GATE_SM0_IRQ  4u
/* SM1 → CPU 的合格帧边界标志 */
#define CAM_VSYNC_GATE_CPU_IRQ  0u

static void cam_pio_irq_handler(void);
static void cam_dma_irq_handler(void);

/* 行级三缓冲环 */
static uint8_t cam_frame_buf[CAM_NUM_BUFFERS][CAPTURE_BYTES];

/*
 * 用三个单调递增序号描述生产/消费进度（无符号差值在回绕下仍成立）：
 *   cam_prod_seq — 已写满的行数；DMA 当前写入 cam_frame_buf[prod % N]
 *   cam_send_seq — Core 0 当前窗口处理完成后可发布的消费位置
 *   cam_cons_seq — 已发布给 DMA IRQ 的消费位置
 * 不变式：cam_cons_seq == cam_send_seq 表示没有窗口在处理。
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
volatile bool     cam_line_ready    = false;
static volatile uint32_t cam_filter_p1_idx = 0u;
static volatile uint8_t  cam_filter_ready  = 0u;

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

}

/* ────────────────────────────────────────────────────────────────────────────
 *  PIO 初始化
 * ──────────────────────────────────────────────────────────────────────────*/
void cam_pio_init(void)
{
    cam_program_offset = pio_add_program(cam_pio, &cam_capture_program);
    cam_vsync_program_offset = pio_add_program(cam_pio, &vsync_gate_program);

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
    pio_sm_set_consecutive_pindirs(cam_pio, cam_vsync_sm, CAM_VSYNC_PIN, 1u, false);

    cam_capture_program_init(cam_pio, cam_sm, cam_program_offset);
    vsync_gate_program_init(cam_pio, cam_vsync_sm, cam_vsync_program_offset);

    /* 只有鉴别器判定合格的 VSYNC 才会打到 NVIC，CPU 不再看原始引脚边沿。 */
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_CPU_IRQ);
    pio_set_irq0_source_enabled(cam_pio, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, cam_pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_sm_set_enabled(cam_pio, cam_sm, false);
    pio_sm_set_enabled(cam_pio, cam_vsync_sm, false);
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

    /* p1 帧内序号 2..479 可构成窗口 [p1-2,p1-1,p1]。只有确认采集环
     * 仍有空间后才发布该窗口，避免把即将被重写的槽标记为可处理。 */
    uint32_t p1_frame_idx = cam_prod_seq % CAPTURE_LINES;
    uint32_t next = cam_prod_seq + 1u;

    if ((next - cam_cons_seq) >= CAM_NUM_BUFFERS) {
        /* 环满：下一块仍被下游占用。丢弃刚采到的这一行，重写同一块。 */
        cam_overrun_count++;
        /* 保留此前尚未认领的 ready 状态，使 Core 0 仍可消费旧窗口并释放空间。 */
        cam_dma_rearm(cam_prod_seq % CAM_NUM_BUFFERS);
    } else {
        /* 发布刚写满的块（prod_seq 前进），重装到下一块继续采集。 */
        if (p1_frame_idx >= 2u) {
            cam_filter_p1_idx = cam_prod_seq;
            cam_filter_ready = 1u;
            cam_line_ready = true;
        }
        cam_prod_seq = next;
        cam_dma_rearm(cam_prod_seq % CAM_NUM_BUFFERS);
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

    /*
     * 不再使用 VSYNC 的 GPIO 边沿中断：原始引脚上有 10ns 毛刺，
     * GPIO 中断会把它们一并计入 cam_frame_count / frame_ready。
     * 帧边界记账改由 PIO 鉴别器（SM1）经 PIO0_IRQ_0 送达，见 cam_pio_irq_handler()。
     */
}

/* ────────────────────────────────────────────────────────────────────────────
 *  启动采集：清空状态并把 PIO 拉回入口；前三帧跳过由 PIO 入口等待完成
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
    cam_line_ready    = false;
    cam_filter_p1_idx = 0u;
    cam_filter_ready  = 0u;

    /* 清除旧状态并把PC拉回程序入口。是否正式开始采集由 PIO 入口的
     * 四次 `wait 1 irq 4` 控制，边界来自 SM1 鉴别器；
     * 进入wrap后不再读取VSYNC，仅由HREF/PCLK连续驱动。 */
    pio_sm_set_enabled(cam_pio, cam_sm, false);
    pio_sm_set_enabled(cam_pio, cam_vsync_sm, false);
    dma_channel_abort((uint)cam_dma_chan);
    pio_sm_clear_fifos(cam_pio, cam_sm);
    pio_sm_restart(cam_pio, cam_sm);
    pio_sm_restart(cam_pio, cam_vsync_sm);
    pio_sm_exec(cam_pio, cam_sm, pio_encode_jmp(cam_program_offset));
    pio_sm_exec(cam_pio, cam_vsync_sm,
                pio_encode_jmp(cam_vsync_program_offset));

    /* 上一轮遗留的合格边界标志必须清掉，否则丢帧序列会立刻消耗掉一个。 */
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_SM0_IRQ);
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_CPU_IRQ);

    if (dma_channel_get_irq0_status((uint)cam_dma_chan)) {
        dma_channel_acknowledge_irq0((uint)cam_dma_chan);
    }
    dma_channel_configure(
        (uint)cam_dma_chan,
        &cam_dma_cfg,
        cam_frame_buf[0],
        &cam_pio->rxf[cam_sm],
        CAPTURE_BYTES,
        true);

    /* 鉴别器必须先跑起来：SM0 停在第一个 `wait 1 irq 4` 上等它。 */
    pio_sm_set_enabled(cam_pio, cam_vsync_sm, true);
    pio_sm_set_enabled(cam_pio, cam_sm, true);
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
    pio_sm_set_enabled(cam_pio, cam_vsync_sm, false);

    if (cam_dma_chan >= 0) {
        dma_channel_abort((uint)cam_dma_chan);
    }

    pio_sm_clear_fifos(cam_pio, cam_sm);
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_SM0_IRQ);
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_CPU_IRQ);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  采集/发送单点交接（消费者侧）
 * ──────────────────────────────────────────────────────────────────────────*/
bool cam_acquire_line(uint32_t *p1_abs_row_idx)
{
    if (p1_abs_row_idx == NULL) {
        return false;
    }

    uint32_t irq_state = save_and_disable_interrupts();
    if (cam_send_seq != cam_cons_seq) {
        restore_interrupts(irq_state);
        return false;                      /* 上一个窗口尚未归还 */
    }
    if (cam_filter_ready == 0u) {
        cam_line_ready = false;
        restore_interrupts(irq_state);
        return false;                      /* 尚未形成新的三行窗口 */
    }

    uint32_t p1_idx = cam_filter_p1_idx;
    cam_send_seq = p1_idx - 1u;
    cam_filter_ready = 0u;
    cam_line_ready = false;
    restore_interrupts(irq_state);

    *p1_abs_row_idx = p1_idx;
    return true;
}

void cam_release_line(void)
{
    if (cam_send_seq != cam_cons_seq) {
        /* 原始窗口读取必须先完成，再允许 DMA IRQ 复用 p-2 及更早的行。 */
        __dmb();
        cam_cons_seq = cam_send_seq;
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 *  GPIO 中断回调：仅做边界记账，不参与采集启动或PIO/DMA控制
 * ──────────────────────────────────────────────────────────────────────────*/
static void cam_pio_irq_handler(void)
{
    if (!pio_interrupt_get(cam_pio, CAM_VSYNC_GATE_CPU_IRQ)) {
        return;
    }
    pio_interrupt_clear(cam_pio, CAM_VSYNC_GATE_CPU_IRQ);

    /* 每个合格 VSYNC 边界一次。毛刺已在 SM1 内部被丢弃，不会到这里。 */
    cam_frame_count++;
    frame_ready = 1u;
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
