/*
 * cam_pio.c — OV5640 PIO/DMA采集、物理帧边界与行描述符队列
 *
 * 权威关系（简单、单向、不可能死锁）：
 *   VSYNC 合格下降沿 -> 无条件：发布FRAME_END、重置PIO/FIFO/DMA、
 *                        行号归零、帧号递增
 *   PIO 一条HREF      -> 无条件产出640字节（HREF提前结束就在PIO内补零），
 *                        因此 一个HREF <=> 一次640-byte DMA完成
 *   DMA 640-byte完成  -> 该物理行边界的到达通知 + 数据buffer
 *
 * 固件不再在任何地方要求“必须凑满480行”：
 *   之前的版本在VSYNC判定里要求 rows_seen==CAPTURE_LINES 才认边界，
 *   于是只要出现一次短帧，之后所有边界都被拒绝，帧再也关不掉 —— 卡死。
 *   现在帧是否满480行完全交给FPGA侧扫描判定，固件只负责无条件重新同步。
 *   cam_line_end_count / cam_last_frame_diag.rows_seen 仅作为记录保留。
 *
 * CPU完全不在PIO的逐行关键路径上：
 *   - PIO行尾不使用 `irq wait`，PIO永远不会等待CPU；
 *   - CPU中断延迟不会让 `wait 0 gpio HREF` 吞掉一条已经开始的物理行；
 *   - 没有空闲buffer时该行落入drop buffer，行号照常推进，只是数据标记为缺失。
 *   DMA ISR唯一的硬期限是“在下一条HREF开始前重装好下一个640-byte块”，
 *   在HTS=1562/PCLK=12MHz下有约76.8us的消隐窗口。
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"

#include "cam_pio.h"
#include "cam_pio.pio.h"

typedef enum {
    CAM_BUF_FREE = 0u,
    CAM_BUF_DMA,
    CAM_BUF_QUEUED,
} cam_buffer_owner_t;

static PIO cam_pio = pio0;
static uint cam_sm = 0u;
static uint cam_program_offset = 0u;
static int cam_dma_chan = -1;
static dma_channel_config cam_dma_cfg;

static uint8_t cam_frame_buf[CAM_NUM_BUFFERS][CAPTURE_BYTES];
static uint8_t cam_drop_buf[CAPTURE_BYTES];
static volatile uint8_t cam_buffer_owner[CAM_NUM_BUFFERS];
static volatile uint8_t cam_dma_target_idx = CAM_INVALID_BUFFER;

static cam_descriptor_t cam_desc_queue[CAM_DESC_QUEUE_DEPTH];
static volatile uint32_t cam_desc_head = 0u;
static volatile uint32_t cam_desc_tail = 0u;
static uint32_t cam_descriptor_seq = 0u;

static volatile bool cam_runtime_enabled = false;
static volatile bool cam_frame_active = false;
static volatile bool cam_startup_pending = false;
static volatile uint8_t cam_skip_boundaries_left = 0u;
/* cam_begin_physical_frame()先递增再发布；UINT32_MAX保证首次有效帧为0。 */
static volatile uint32_t cam_physical_frame_id = UINT32_MAX;
static volatile uint16_t cam_physical_row_idx = 0u;
static volatile uint16_t cam_physical_rows_seen = 0u;
static volatile uint16_t cam_frame_error_flags = 0u;
static volatile uint16_t cam_frame_rows_published = 0u;
static volatile uint16_t cam_frame_dataless_rows = 0u;

volatile uint8_t frame_ready = 0u;
volatile uint32_t cam_overrun_count = 0u;
volatile uint32_t cam_descriptor_overrun_count = 0u;
volatile uint32_t cam_pio_rxover_count = 0u;
volatile uint32_t cam_href_error_count = 0u;
volatile uint32_t cam_line_end_count = 0u;
volatile uint32_t cam_line_publish_count = 0u;
volatile uint32_t cam_dataless_row_count = 0u;
volatile uint32_t cam_partial_line_count = 0u;
volatile uint32_t cam_skip_done_count = 0u;
volatile uint32_t cam_discarded_frame_count = 0u;
volatile uint32_t cam_vsync_glitch_count = 0u;
volatile uint32_t cam_short_line_count = 0u;
volatile uint32_t cam_frame_rollover_count = 0u;
volatile uint32_t cam_startup_error_count = 0u;
volatile uint32_t cam_frame_count = 0u;
volatile cam_frame_diag_t cam_last_frame_diag;

/*
 * 实测 OV5640 时序 (docs/2026-08-11_21-51-15.bin，HTS=1562/VTS=512/PCLK=12MHz)：
 *   行周期 130.167us；HREF高 = 恰好640个PCLK；每两个真VSYNC下降沿之间恰好480行
 *   VSYNC 高电平宽度 260.34us；周期 66.645ms
 *   最后一条HREF下降 -> VSYNC下降 808.0us；VSYNC下降 -> 下一条HREF上升 3434.2us
 *
 * 实测异常 (docs/2026-08-11_22-21-22.bin)：
 *   VSYNC 10ns 脉冲 @985.651ms / 986.877ms / 3345.066ms（最后一个在稳态运行期）
 *   VSYNC 1033.53us 超宽脉冲 @1002.363ms
 *   176条异常HREF行，全部落在 805.4~991.5ms；1000ms之后23258行全为640 PCLK
 */
/* 真沿260.33us，13倍余量；只用来挡10ns级毛刺。 */
#define CAM_VSYNC_HIGH_MIN_US         20u
/* 第1个合格边界只做对齐，之后丢弃3个完整帧（等价8月5日的set x,2循环+对齐wait对）。 */
#define CAM_STARTUP_SKIP_BOUNDARIES    4u
/* 超时后强制起跑，绝不返回失败：4个边界约267ms，加上等第一个合格脉冲。 */
#define CAM_STARTUP_TIMEOUT_MS      1000u
/*
 * 活性看门狗：VSYNC是唯一的帧边界权威，但如果它长时间一个都不合格，
 * 行号会一路涨过CAPTURE_LINES，之后每条行都被丢弃 —— 表现为静默停摆。
 * 行号到达该上限时在采集侧强制翻页（只做描述符记账，不碰PIO/DMA硬件），
 * 保证数据流永远继续。正常帧永远碰不到它。
 */
#define CAM_FRAME_ROW_WATCHDOG      (CAPTURE_LINES * 2u)

static volatile uint64_t cam_last_vsync_fall_us = 0u;
static volatile uint64_t cam_vsync_rise_us = 0u;
static volatile bool cam_vsync_rise_valid = false;

static void cam_gpio_irq_callback(uint gpio, uint32_t events);
static void cam_dma_irq_handler(void);
static void cam_service_dma_completion(void);

static inline uint32_t cam_dma_remaining(void)
{
    return dma_channel_hw_addr((uint)cam_dma_chan)->transfer_count;
}

static inline void cam_mark_frame_error(uint16_t error_flags)
{
    if (cam_frame_active) {
        cam_frame_error_flags |= error_flags;
    }
}

static uint16_t cam_take_pio_rxover(void)
{
    /* RP2350将RX FIFO满导致的状态机停顿记录为RXSTALL。对摄像头连续
     * PCLK而言，该停顿等价于采样字节丢失，按RXOVER语义处理。 */
    const uint32_t mask = 1u << (PIO_FDEBUG_RXSTALL_LSB + cam_sm);
    if ((cam_pio->fdebug & mask) == 0u) {
        return 0u;
    }

    cam_pio->fdebug = mask;
    cam_pio_rxover_count++;
    return CAM_FRAME_ERR_PIO_RXOVER;
}

static uint8_t cam_claim_free_buffer(void)
{
    for (uint8_t idx = 0u; idx < CAM_NUM_BUFFERS; ++idx) {
        if (cam_buffer_owner[idx] == CAM_BUF_FREE) {
            cam_buffer_owner[idx] = CAM_BUF_DMA;
            return idx;
        }
    }
    return CAM_INVALID_BUFFER;
}

static void cam_dma_arm(uint8_t buffer_idx)
{
    cam_dma_target_idx = buffer_idx;
    void *write_addr = (buffer_idx == CAM_INVALID_BUFFER)
        ? (void *)cam_drop_buf
        : (void *)cam_frame_buf[buffer_idx];

    dma_channel_set_write_addr((uint)cam_dma_chan, write_addr, false);
    dma_channel_set_transfer_count((uint)cam_dma_chan, CAPTURE_BYTES, true);
}

static void cam_arm_next_buffer(void)
{
    uint8_t next = cam_claim_free_buffer();
    if (next == CAM_INVALID_BUFFER) {
        cam_overrun_count++;
        cam_mark_frame_error(CAM_FRAME_ERR_DMA_OVERRUN | CAM_FRAME_ERR_NO_BUFFER);
    }
    cam_dma_arm(next);
}

static bool cam_queue_push(const cam_descriptor_t *descriptor, bool frame_boundary)
{
    uint32_t count = cam_desc_head - cam_desc_tail;
    uint32_t limit = frame_boundary ? CAM_DESC_QUEUE_DEPTH : (CAM_DESC_QUEUE_DEPTH - 2u);
    if (count >= limit) {
        cam_descriptor_overrun_count++;
        cam_overrun_count++;
        cam_mark_frame_error(CAM_FRAME_ERR_DESC_QUEUE_FULL);
        return false;
    }

    cam_desc_queue[cam_desc_head % CAM_DESC_QUEUE_DEPTH] = *descriptor;
    __dmb();
    cam_desc_head++;
    return true;
}

/* 在HREF低电平或VSYNC blanking中调用，将硬件恢复到运行期line_loop。 */
static void cam_reset_runtime_capture(void)
{
    pio_sm_set_enabled(cam_pio, cam_sm, false);
    dma_channel_abort((uint)cam_dma_chan);

    if (dma_channel_get_irq0_status((uint)cam_dma_chan)) {
        dma_channel_acknowledge_irq0((uint)cam_dma_chan);
    }

    if (cam_dma_target_idx < CAM_NUM_BUFFERS &&
        cam_buffer_owner[cam_dma_target_idx] == CAM_BUF_DMA) {
        cam_buffer_owner[cam_dma_target_idx] = CAM_BUF_FREE;
    }
    cam_dma_target_idx = CAM_INVALID_BUFFER;

    pio_sm_clear_fifos(cam_pio, cam_sm);
    pio_sm_restart(cam_pio, cam_sm);
    (void)cam_take_pio_rxover();
    pio_interrupt_clear(cam_pio, 1u);   /* 丢弃残留的补零标志 */
    pio_sm_exec(cam_pio, cam_sm,
                pio_encode_jmp(cam_program_offset + cam_capture_wrap_target));

    cam_arm_next_buffer();
    pio_sm_set_enabled(cam_pio, cam_sm, true);
}

/*
 * 一次640-byte DMA完成 == 一条PIO确认的物理HREF行。
 * 行号在这里唯一地产生并推进；数据是否可用只影响描述符的valid/error_flags，
 * 绝不影响行号，也绝不允许后一行前移补位。
 */
/*
 * 只做描述符记账的强制翻页：发布FRAME_END、递增帧号、行号归零。
 * 不触碰PIO/DMA硬件，因此可以安全地在采集DMA中断内调用
 * （cam_reset_runtime_capture() 里的 dma_channel_abort() 会自旋等通道停止，
 *  在自己的ISR里调用可能挂死）。
 */
static void cam_force_frame_rollover(void);

static void cam_publish_physical_line(uint8_t buffer_idx, uint16_t error_flags)
{
    /* 活性保护：VSYNC长时间不合格时不允许无限累积行号。 */
    if (cam_physical_row_idx >= CAM_FRAME_ROW_WATCHDOG) {
        cam_force_frame_rollover();
    }

    uint16_t row = cam_physical_row_idx;

    cam_physical_row_idx++;
    cam_physical_rows_seen++;
    cam_line_end_count++;

    if (row >= CAPTURE_LINES) {
        /* 本帧HREF多于480条：只记录错误，不产生越界行描述符。
         * 帧是否够480行由FPGA侧扫描判定。 */
        cam_mark_frame_error(CAM_FRAME_ERR_TOO_MANY_ROWS);
        cam_href_error_count++;
        if (buffer_idx < CAM_NUM_BUFFERS) {
            cam_buffer_owner[buffer_idx] = CAM_BUF_FREE;
        }
        return;
    }

    uint16_t line_err = error_flags;
    if (buffer_idx >= CAM_NUM_BUFFERS) {
        /* 该物理行落进了drop buffer：行身份成立，数据不可用。 */
        line_err |= CAM_FRAME_ERR_NO_BUFFER | CAM_FRAME_ERR_DMA_OVERRUN;
    }
    cam_mark_frame_error(line_err);

    uint8_t payload_idx = CAM_INVALID_BUFFER;
    if (buffer_idx < CAM_NUM_BUFFERS) {
        if (line_err == 0u) {
            payload_idx = buffer_idx;
        } else {
            cam_buffer_owner[buffer_idx] = CAM_BUF_FREE;
        }
    }

    if (payload_idx == CAM_INVALID_BUFFER) {
        cam_dataless_row_count++;
        cam_frame_dataless_rows++;
    }

    cam_descriptor_t descriptor = {
        .frame_id = cam_physical_frame_id,
        .descriptor_seq = cam_descriptor_seq++,
        .row_idx = row,
        .rows_seen = cam_physical_rows_seen,
        .error_flags = line_err,
        .buffer_idx = payload_idx,
        .type = CAM_DESC_LINE,
        /* valid只描述payload可用性；行号无论如何都是权威的。 */
        .valid = (payload_idx == CAM_INVALID_BUFFER) ? 0u : 1u,
        .reserved = 0u,
    };

    if (cam_queue_push(&descriptor, false)) {
        if (payload_idx != CAM_INVALID_BUFFER) {
            cam_buffer_owner[payload_idx] = CAM_BUF_QUEUED;
        }
        cam_line_publish_count++;
        cam_frame_rows_published++;
    } else if (payload_idx != CAM_INVALID_BUFFER) {
        cam_buffer_owner[payload_idx] = CAM_BUF_FREE;
    }
}

/* 发布本物理帧的FRAME_END并锁存诊断。纯描述符记账，不动硬件。 */
static void cam_publish_frame_end(void)
{
    cam_frame_error_flags |= cam_take_pio_rxover();
    if (cam_physical_rows_seen < CAPTURE_LINES) {
        cam_frame_error_flags |= CAM_FRAME_ERR_TOO_FEW_ROWS;
    } else if (cam_physical_rows_seen > CAPTURE_LINES) {
        cam_frame_error_flags |= CAM_FRAME_ERR_TOO_MANY_ROWS;
    }

    cam_last_frame_diag.rows_seen = cam_physical_rows_seen;
    cam_last_frame_diag.rows_published = cam_frame_rows_published;
    cam_last_frame_diag.error_flags = cam_frame_error_flags;
    cam_last_frame_diag.dataless_rows = cam_frame_dataless_rows;
    __dmb();
    cam_last_frame_diag.frame_id = cam_physical_frame_id;

    cam_descriptor_t descriptor = {
        .frame_id = cam_physical_frame_id,
        .descriptor_seq = cam_descriptor_seq++,
        .row_idx = (cam_physical_rows_seen == 0u)
            ? UINT16_MAX
            : (uint16_t)(cam_physical_rows_seen - 1u),
        .rows_seen = cam_physical_rows_seen,
        .error_flags = cam_frame_error_flags,
        .buffer_idx = CAM_INVALID_BUFFER,
        .type = CAM_DESC_FRAME_END,
        .valid = (cam_frame_error_flags == 0u &&
                  cam_physical_rows_seen == CAPTURE_LINES) ? 1u : 0u,
        .reserved = 0u,
    };

    /* 行描述符最多占用CAM_NUM_BUFFERS个buffer；队列额外预留
     * 两个帧边界位置，保证过载帧仍能发布FRAME_END。 */
    (void)cam_queue_push(&descriptor, true);
}

/*
 * VSYNC 上的强制刷新：发布FRAME_END，然后把PIO/FIFO/DMA清回行循环入口。
 * 在垂直消隐（VSYNC下降沿到下一条HREF约3.43ms）里执行，绝不会打断一条行。
 */
static void cam_close_physical_frame(void)
{
    if (!cam_runtime_enabled || !cam_frame_active) {
        return;
    }

    /* DMA应当正好停在块边界；否则说明VSYNC落在一条行的中间。 */
    if (cam_dma_chan >= 0 && cam_dma_remaining() != CAPTURE_BYTES) {
        cam_partial_line_count++;
        cam_mark_frame_error(CAM_FRAME_ERR_HREF_LENGTH | CAM_FRAME_ERR_BOUNDARY);
    }

    cam_publish_frame_end();
    cam_frame_active = false;
    cam_reset_runtime_capture();
}

static void cam_reset_frame_counters(void)
{
    cam_physical_frame_id++;
    cam_physical_row_idx = 0u;
    cam_physical_rows_seen = 0u;
    cam_frame_error_flags = 0u;
    cam_frame_rows_published = 0u;
    cam_frame_dataless_rows = 0u;
    cam_frame_active = true;
    cam_frame_count++;
    frame_ready = 1u;

    if (cam_dma_target_idx == CAM_INVALID_BUFFER) {
        cam_mark_frame_error(CAM_FRAME_ERR_DMA_OVERRUN | CAM_FRAME_ERR_NO_BUFFER);
    }
}

static void cam_begin_physical_frame(void)
{
    if (!cam_runtime_enabled) {
        return;
    }
    if (cam_frame_active) {
        cam_frame_error_flags |= CAM_FRAME_ERR_BOUNDARY;
        cam_close_physical_frame();
    }
    cam_reset_frame_counters();
}

/* 见声明处注释：活性看门狗用的纯记账翻页，不碰PIO/DMA。 */
static void cam_force_frame_rollover(void)
{
    cam_frame_rollover_count++;
    cam_frame_error_flags |= CAM_FRAME_ERR_BOUNDARY | CAM_FRAME_ERR_TOO_MANY_ROWS;
    cam_publish_frame_end();
    cam_reset_frame_counters();
}

void cam_gpio_init(void)
{
    for (uint pin = CAM_DATA_PIN_BASE;
         pin < CAM_DATA_PIN_BASE + CAM_DATA_PIN_COUNT; ++pin) {
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

void cam_pio_init(void)
{
    cam_program_offset = pio_add_program(cam_pio, &cam_capture_program);

    for (uint i = 0u; i < CAM_DATA_PIN_COUNT; ++i) {
        pio_gpio_init(cam_pio, CAM_DATA_PIN_BASE + i);
    }
    pio_gpio_init(cam_pio, CAM_PCLK_PIN);
    pio_gpio_init(cam_pio, CAM_HREF_PIN);
    pio_gpio_init(cam_pio, CAM_VSYNC_PIN);

    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm,
                                   CAM_DATA_PIN_BASE, CAM_DATA_PIN_COUNT, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_PCLK_PIN, 1u, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_HREF_PIN, 1u, false);
    pio_sm_set_consecutive_pindirs(cam_pio, cam_sm, CAM_VSYNC_PIN, 1u, false);

    cam_capture_program_init(cam_pio, cam_sm, cam_program_offset);
    pio_sm_set_enabled(cam_pio, cam_sm, false);

    /* 运行期不再有逐行PIO中断：行边界由640-byte DMA完成给出，
     * PIO永远不等待CPU。IRQ0/IRQ1只在启动握手时被CPU轮询。 */
}

static void cam_service_dma_completion(void)
{
    if (!dma_channel_get_irq0_status((uint)cam_dma_chan)) {
        return;
    }
    dma_channel_acknowledge_irq0((uint)cam_dma_chan);

    uint8_t completed = cam_dma_target_idx;

    /*
     * 先重装下一个640-byte块。PIO在行尾不会等待CPU，最迟在下一条HREF
     * 的前几个PCLK就需要DMA已经就绪（HTS=1562时消隐窗口约76.8us）。
     */
    cam_arm_next_buffer();

    uint16_t line_err = cam_take_pio_rxover();

    /*
     * PIO在补零时置IRQ1。该标志只可能在刚刚完成的这个640-byte块
     * 填充期间被置位（置位后PIO立刻去等下一条HREF），因此与行严格一一对应。
     * 行号和块边界依然正确，只是该行尾部是补出来的0。
     */
    if (pio_interrupt_get(cam_pio, 1u)) {
        pio_interrupt_clear(cam_pio, 1u);
        line_err |= CAM_FRAME_ERR_HREF_LENGTH;
        cam_short_line_count++;
    }

    if (!cam_runtime_enabled || !cam_frame_active) {
        if (completed < CAM_NUM_BUFFERS) {
            cam_buffer_owner[completed] = CAM_BUF_FREE;
        }
        if (cam_runtime_enabled) {
            /* 消隐期内出现完整行：该行不属于任何已打开的物理帧。 */
            cam_mark_frame_error(CAM_FRAME_ERR_BOUNDARY);
        }
        return;
    }

    cam_publish_physical_line(completed, line_err);
}

static void cam_dma_irq_handler(void)
{
    cam_service_dma_completion();
}

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
    /* Camera RX is loss-intolerant and cannot be back-pressured by the sensor.
     * Give it precedence over the recoverable FPGA TX DMA channel. */
    channel_config_set_high_priority(&cam_dma_cfg, true);

    dma_channel_set_irq0_enabled((uint)cam_dma_chan, true);
    irq_add_shared_handler(DMA_IRQ_0, cam_dma_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    /*
     * 采集DMA是唯一有硬期限的中断：下一条HREF开始前必须重装好buffer
     * （HTS=1562时约有76.8us消隐窗口）。VSYNC处理与它保持同一优先级，
     * 两者互不抢占：VSYNC里的 dma_channel_abort() 会自旋等待通道停止，
     * 若被采集DMA的ISR重新触发通道就可能挂死。VSYNC只在垂直消隐
     * （约4.2ms，其间没有任何HREF）执行，因此不抢占不会错过行边界。
     */
    irq_set_priority(DMA_IRQ_0, 0x20u);
    irq_set_enabled(DMA_IRQ_0, true);

    /* 两个沿都要：帧边界由“VSYNC高电平宽度≈260us”裁定，
     * 只看下降沿无法把耦合尖峰和真沿区分开。 */
    gpio_set_irq_enabled_with_callback(CAM_VSYNC_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       false, cam_gpio_irq_callback);
    irq_set_priority(IO_IRQ_BANK0, 0x20u);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

static bool cam_wait_for_pio_irq(uint irq_index, uint32_t timeout_ms)
{
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);
    while (!pio_interrupt_get(cam_pio, irq_index)) {
        if (time_reached(timeout)) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

void cam_capture_start(void)
{
    if (cam_dma_chan < 0) {
        cam_dma_init();
    }

    cam_runtime_enabled = false;
    cam_frame_active = false;
    frame_ready = 0u;
    cam_overrun_count = 0u;
    cam_descriptor_overrun_count = 0u;
    cam_pio_rxover_count = 0u;
    cam_href_error_count = 0u;
    cam_line_end_count = 0u;
    cam_line_publish_count = 0u;
    cam_dataless_row_count = 0u;
    cam_partial_line_count = 0u;
    cam_vsync_glitch_count = 0u;
    cam_short_line_count = 0u;
    cam_frame_rollover_count = 0u;
    cam_discarded_frame_count = 0u;
    cam_frame_count = 0u;
    memset((void *)&cam_last_frame_diag, 0, sizeof(cam_last_frame_diag));
    cam_desc_head = 0u;
    cam_desc_tail = 0u;
    cam_descriptor_seq = 0u;
    /* 三个完整丢弃帧不占用线上frame_id；首次放行必须从0开始。 */
    cam_physical_frame_id = UINT32_MAX;
    cam_physical_row_idx = 0u;
    cam_physical_rows_seen = 0u;
    cam_frame_error_flags = 0u;
    cam_frame_rows_published = 0u;
    cam_frame_dataless_rows = 0u;
    memset((void *)cam_buffer_owner, CAM_BUF_FREE, sizeof(cam_buffer_owner));

    const uint32_t vsync_edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    cam_startup_pending = false;
    cam_skip_boundaries_left = 0u;
    cam_last_vsync_fall_us = 0u;
    cam_vsync_rise_us = 0u;
    cam_vsync_rise_valid = false;

    gpio_set_irq_enabled(CAM_VSYNC_PIN, vsync_edges, false);
    gpio_acknowledge_irq(CAM_VSYNC_PIN, vsync_edges);
    pio_sm_set_enabled(cam_pio, cam_sm, false);
    dma_channel_abort((uint)cam_dma_chan);
    pio_sm_clear_fifos(cam_pio, cam_sm);
    pio_sm_restart(cam_pio, cam_sm);
    pio_sm_exec(cam_pio, cam_sm, pio_encode_jmp(cam_program_offset));
    pio_interrupt_clear(cam_pio, 0u);
    pio_interrupt_clear(cam_pio, 1u);

    if (dma_channel_get_irq0_status((uint)cam_dma_chan)) {
        dma_channel_acknowledge_irq0((uint)cam_dma_chan);
    }

    cam_buffer_owner[0] = CAM_BUF_DMA;
    cam_dma_target_idx = 0u;
    dma_channel_configure((uint)cam_dma_chan, &cam_dma_cfg,
                          cam_frame_buf[0], &cam_pio->rxf[cam_sm],
                          CAPTURE_BYTES, true);

    pio_sm_set_enabled(cam_pio, cam_sm, true);
    if (!cam_wait_for_pio_irq(0u, 100u)) {
        cam_startup_error_count++;
        cam_capture_stop();
        return;
    }

    /*
     * 启动丢帧由CPU完成，不再交给PIO的 wait 1/wait 0 gpio VSYNC ——
     * 那两条指令会被10ns毛刺满足，丢帧循环可能瞬间跑完（实测输出
     * 曾从相机第4帧的第386行开始）。
     *
     * 放行PIO后它立刻开始按HREF采样，但 cam_runtime_enabled 仍为 false，
     * 所以 cam_service_dma_completion() 只回收buffer、不发布任何描述符。
     * 这段时间正好覆盖传感器上电的异常行窗口（实测805.4~991.5ms）。
     */
    cam_skip_boundaries_left = CAM_STARTUP_SKIP_BOUNDARIES;
    cam_startup_pending = true;
    __dmb();
    gpio_acknowledge_irq(CAM_VSYNC_PIN, vsync_edges);
    gpio_set_irq_enabled(CAM_VSYNC_PIN, vsync_edges, true);
    pio_interrupt_clear(cam_pio, 0u);

    /*
     * 超时不再返回失败：宁可从帧中间起跑（那一帧会被FPGA按480行判掉，
     * 下一个真VSYNC自然重新同步），也绝不停掉采集造成整机停摆。
     */
    absolute_time_t timeout = make_timeout_time_ms(CAM_STARTUP_TIMEOUT_MS);
    while (cam_startup_pending) {
        if (time_reached(timeout)) {
            uint32_t irq_state = save_and_disable_interrupts();
            cam_startup_error_count++;
            cam_skip_boundaries_left = 0u;
            cam_startup_pending = false;
            cam_runtime_enabled = true;
            cam_begin_physical_frame();
            restore_interrupts(irq_state);
            return;
        }
        tight_loop_contents();
    }
}

void cam_capture_stop(void)
{
    cam_runtime_enabled = false;
    cam_startup_pending = false;
    cam_frame_active = false;
    gpio_set_irq_enabled(CAM_VSYNC_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    pio_sm_set_enabled(cam_pio, cam_sm, false);

    if (cam_dma_chan >= 0) {
        dma_channel_abort((uint)cam_dma_chan);
        if (dma_channel_get_irq0_status((uint)cam_dma_chan)) {
            dma_channel_acknowledge_irq0((uint)cam_dma_chan);
        }
    }
    pio_sm_clear_fifos(cam_pio, cam_sm);
}

bool cam_descriptor_pop(cam_descriptor_t *descriptor)
{
    if (descriptor == NULL) {
        return false;
    }

    uint32_t irq_state = save_and_disable_interrupts();
    if (cam_desc_tail == cam_desc_head) {
        restore_interrupts(irq_state);
        return false;
    }

    *descriptor = cam_desc_queue[cam_desc_tail % CAM_DESC_QUEUE_DEPTH];
    __dmb();
    cam_desc_tail++;
    restore_interrupts(irq_state);
    return true;
}

const uint8_t *cam_get_buffer(uint8_t buffer_idx)
{
    if (buffer_idx >= CAM_NUM_BUFFERS) {
        return NULL;
    }
    return cam_frame_buf[buffer_idx];
}

void cam_release_buffer(uint8_t buffer_idx)
{
    if (buffer_idx >= CAM_NUM_BUFFERS) {
        return;
    }

    uint32_t irq_state = save_and_disable_interrupts();
    if (cam_buffer_owner[buffer_idx] == CAM_BUF_QUEUED) {
        __dmb();
        cam_buffer_owner[buffer_idx] = CAM_BUF_FREE;
    }
    restore_interrupts(irq_state);
}

/*
 * 唯一的VSYNC判定：高电平宽度必须 >= CAM_VSYNC_HIGH_MIN_US。
 *
 * 这条判定只为挡掉 10ns 级毛刺（实测 985.651 / 986.877 / 3345.066 ms 各一个）。
 * 真沿恒为 260.33us，余量 13 倍。1033.53us 的超宽脉冲被当作真沿接受 ——
 * 反正它的作用就是强制刷新，接受它比拒绝它更安全。
 *
 * 刻意不做的判定：不看周期、不看行数、不看行空闲时间。
 * 任何"必须恰好480行才认边界"的条件都会在出现一次短帧后
 * 永久拒绝所有后续边界，帧再也关不掉 —— 这就是之前卡死的机制。
 * 帧是否够480行由FPGA侧扫描判定，固件只负责无条件重新同步。
 */
static bool cam_vsync_fall_is_genuine(uint64_t now_us)
{
    if (!cam_vsync_rise_valid) {
        /* 没有配对的上升沿：使能时抓到半个脉冲，或毛刺的上升沿没被锁存。 */
        cam_vsync_glitch_count++;
        return false;
    }

    uint64_t high_us = now_us - cam_vsync_rise_us;
    cam_vsync_rise_valid = false;
    if (high_us < CAM_VSYNC_HIGH_MIN_US) {
        cam_vsync_glitch_count++;
        return false;
    }
    return true;
}

static void cam_gpio_irq_callback(uint gpio, uint32_t events)
{
    if (gpio != CAM_VSYNC_PIN) {
        return;
    }

    uint64_t now_us = time_us_64();

    if ((events & GPIO_IRQ_EDGE_RISE) != 0u) {
        /* 每个上升沿都刷新时间戳：即使某个毛刺的下降沿没被锁存，
         * 下一个真上升沿也会把状态带回正确值，不会长期失效。 */
        cam_vsync_rise_us = now_us;
        cam_vsync_rise_valid = true;
    }

    if ((events & GPIO_IRQ_EDGE_FALL) == 0u) {
        return;
    }
    if (!cam_runtime_enabled && !cam_startup_pending) {
        return;
    }
    if (!cam_vsync_fall_is_genuine(now_us)) {
        return;
    }

    cam_last_vsync_fall_us = now_us;

    if (cam_startup_pending) {
        /*
         * 启动丢帧：只数合格边界，不看行数、不看周期，因此不存在
         * "凑不出480行就永远启动不了"的可能。
         * 第1个边界只做对齐，之后丢弃3个完整帧，与8月5日PIO里
         * "1对对齐 wait + set x,2 三次循环"完全等价。
         */
        if (cam_skip_boundaries_left > 1u) {
            cam_skip_boundaries_left--;
            cam_discarded_frame_count++;
            return;
        }
        cam_skip_boundaries_left = 0u;
        cam_startup_pending = false;

        /* 消隐期内清掉丢帧期间的PIO/DMA残留，再开启frame 0。 */
        cam_reset_runtime_capture();
        cam_pio_rxover_count = 0u;
        cam_short_line_count = 0u;
        cam_overrun_count = 0u;
        cam_descriptor_overrun_count = 0u;
        cam_skip_done_count++;
        __dmb();
        cam_runtime_enabled = true;
        cam_begin_physical_frame();
        return;
    }

    /* 运行期：无条件关旧帧 + 开新帧。DMA/行号/帧号在这里强制刷新。 */
    cam_close_physical_frame();
    cam_begin_physical_frame();
}

void cam_enable_4x4_scaffold(void)
{
    /* 预留接口保持不变；当前使用CAM_NUM_BUFFERS个独立行buffer。 */
}
