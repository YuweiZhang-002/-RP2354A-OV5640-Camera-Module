/*
 * RP2350A Camera pipeline
 *
 * OV5640 -> PIO0/DMA -> physical line descriptors -> Core0 Sobel
 *        -> Core1 threshold/bit pack -> 128-byte packet -> PIO1/FPGA
 *
 * 行节奏契约（本文件的核心不变量）：
 *   完整物理帧按 row_idx=0..479 发出恰好480个job。采集侧为每个
 *   HREF都发布描述符；payload丢失时同一行就地变成INVALID_ROW，仍是
 *   “一个Camera HREF -> 一个FPGA行包”。
 *
 *   严禁在FRAME_END无上限补到row479。实测2026-08-11_21-43-48.bin中
 *   有31835个包的起始间隔小于20us：这是伪VSYNC在约308行关帧后，
 *   无上限帧尾补齐一次生成约172个job的直接结果。现在只修复不超过3行的
 *   局部描述符缺口；大缺口直接结束为INCOMPLETE，不制造发送突发。
 */

#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "timer.h"
#include "header/ov5640.h"
#include "cam_pio.h"
#include "ov5640_set.h"
#include "fpga_pio.h"
#include "image_process.h"

/* 一条已到达的原始行。ring 按 row_idx % 3 索引，构成真正的三行滑窗。 */
typedef struct {
    bool     valid;          /* payload 可用 */
    uint16_t row_idx;
    uint8_t  buffer_idx;
    uint16_t error_flags;
} core0_row_slot_t;

typedef struct {
    bool     active;
    uint32_t frame_id;
    uint16_t next_emit_row;      /* 下一个必须发出的行号 */
    uint16_t expected_row;       /* 下一个期望到达的原始行号 */
    uint16_t output_count;
    uint16_t placeholder_rows;   /* 因数据缺失而占位的行数 */
    uint16_t error_flags;
    uint16_t last_rows_seen;
    uint32_t last_descriptor_seq;
    bool     has_last_job;
    uint32_t last_job_seq;
    core0_row_slot_t ring[3];
} core0_frame_state_t;

static uint8_t packet_buf[PACKET_BYTES];
static core0_frame_state_t core0_frame;
static uint32_t core1_last_tx_start_us = 0u;
static volatile bool core1_tx_irq_ready = false;

/*
 * 128 byte @12MHz字节率约10.67us。正常job由Camera仦约130.17us间隔
 * 生成；100us只用来抑制局部两三个job同时就绪的短突发，仍保留约30%
 * 的追赶余量。大批积压由上游禁止生成，不再靠提高发送速度掩盖。
 */
#define FPGA_TX_MIN_START_INTERVAL_US   100u
#define CORE0_MAX_INLINE_REPAIR_ROWS      3u

static void fpga_pio_core1_entry(void);

void sys_loop(void)
{
    while (true) {
        tight_loop_contents();
    }
}

static void core0_push_job(uint32_t job_seq)
{
    multicore_fifo_push_blocking(job_seq);
}

static void core0_release_ring(void)
{
    for (uint8_t i = 0u; i < 3u; ++i) {
        if (core0_frame.ring[i].valid) {
            cam_release_buffer(core0_frame.ring[i].buffer_idx);
        }
        memset(&core0_frame.ring[i], 0, sizeof(core0_frame.ring[i]));
    }
}

static void core0_start_frame(uint32_t frame_id)
{
    memset(&core0_frame, 0, sizeof(core0_frame));
    core0_frame.active = true;
    core0_frame.frame_id = frame_id;
}

static image_row_job_meta_t core0_make_job_meta(uint16_t row_idx,
                                                uint16_t extra_error_flags)
{
    image_row_job_meta_t meta = {
        .frame_id = core0_frame.frame_id,
        .descriptor_seq = core0_frame.last_descriptor_seq,
        .descriptor_overrun_count = cam_descriptor_overrun_count,
        .capture_overrun_count = cam_overrun_count,
        .row_idx = row_idx,
        .physical_rows_seen = core0_frame.last_rows_seen,
        .capture_error_flags =
            (uint16_t)(core0_frame.error_flags | extra_error_flags),
        .skip_done_count = (uint16_t)cam_skip_done_count,
        .row_flags = (row_idx == 0u) ? PKT_ROW_FLAG_FIRST_LINE : 0u,
        .has_sobel = 0u,
        .job_type = IMAGE_JOB_ROW,
        .frame_complete = 0u,
    };
    return meta;
}

/* 三行滑窗查询：row 必须到达过、payload 可用，且槽位仍属于该 row。 */
static const core0_row_slot_t *core0_row_data(uint16_t row)
{
    if (row >= CAPTURE_LINES) {
        return NULL;
    }
    const core0_row_slot_t *slot = &core0_frame.ring[row % 3u];
    if (!slot->valid || slot->row_idx != row) {
        return NULL;
    }
    return slot;
}

/*
 * 发出行 row 的唯一出口。row 必须等于 next_emit_row，
 * 因此行号只可能连续、单调、不重复。
 */
static void core0_emit_row(uint16_t row)
{
    const core0_row_slot_t *r0 = (row >= 2u)
        ? core0_row_data((uint16_t)(row - 2u))
        : NULL;
    const core0_row_slot_t *r1 = (row >= 1u)
        ? core0_row_data((uint16_t)(row - 1u))
        : NULL;
    const core0_row_slot_t *r2 = core0_row_data(row);

    /* 恢复8月5日已验证的行语义：row0/1为零边界，row2..479使用
     * [row-2,row-1,row]做Sobel。这里只改调度/标签，不改fused_row_sq数学实现。 */
    bool boundary_row = row < 2u;
    bool sobel_ok = !boundary_row && r0 != NULL && r1 != NULL && r2 != NULL;

    uint16_t extra_err = 0u;
    if (r2 != NULL) {
        extra_err |= r2->error_flags;
    }

    image_row_job_meta_t meta = core0_make_job_meta(row, extra_err);
    uint32_t job_seq;

    if (sobel_ok) {
        const uint8_t *p0 = cam_get_buffer(r0->buffer_idx);
        const uint8_t *p1 = cam_get_buffer(r1->buffer_idx);
        const uint8_t *p2 = cam_get_buffer(r2->buffer_idx);
        job_seq = image_prepare_sobel_job(p0, p1, p2, &meta);
        pipeline_timing_stats.core0_sobel_rows++;
    } else {
        if (!boundary_row) {
            /* 行号保留、payload 缺失：显式标记，不静默跳号。 */
            meta.row_flags |= PKT_ROW_FLAG_INVALID_ROW;
            meta.capture_error_flags |= CAM_FRAME_ERR_ROW_JUMP;
            core0_frame.placeholder_rows++;
            core0_frame.error_flags |= CAM_FRAME_ERR_ROW_JUMP;
            pipeline_timing_stats.core0_repair_rows++;
        }
        job_seq = image_prepare_zero_job(&meta);
    }

    core0_frame.last_job_seq = job_seq;
    core0_frame.has_last_job = true;
    core0_frame.output_count++;
    core0_frame.next_emit_row = (uint16_t)(row + 1u);
    pipeline_timing_stats.core0_row_jobs++;
    core0_push_job(job_seq);
}

/* 只允许修复三行窗口可能造成的局部缺口。超过三行说明帧边界
 * 或描述符流已经失步，继续补齐只会制造一批与Camera HREF无关的包。 */
static bool core0_emit_rows_through(uint16_t last_row)
{
    if (last_row >= CAPTURE_LINES) {
        last_row = (uint16_t)(CAPTURE_LINES - 1u);
    }
    if (core0_frame.next_emit_row > last_row) {
        return true;
    }
    uint16_t count =
        (uint16_t)(last_row - core0_frame.next_emit_row + 1u);
    if (count > CORE0_MAX_INLINE_REPAIR_ROWS) {
        core0_frame.error_flags |= CAM_FRAME_ERR_ROW_JUMP;
        return false;
    }
    while (core0_frame.next_emit_row <= last_row) {
        core0_emit_row(core0_frame.next_emit_row);
    }
    return true;
}

static void core0_store_row(const cam_descriptor_t *line)
{
    core0_row_slot_t *slot = &core0_frame.ring[line->row_idx % 3u];

    /* 覆盖槽位前归还上一个占用者（row_idx - 3，已经发完）。 */
    if (slot->valid) {
        cam_release_buffer(slot->buffer_idx);
    }

    bool has_payload = (line->valid != 0u) && (line->buffer_idx < CAM_NUM_BUFFERS);
    slot->valid = has_payload;
    slot->row_idx = line->row_idx;
    slot->buffer_idx = has_payload ? line->buffer_idx : CAM_INVALID_BUFFER;
    slot->error_flags = line->error_flags;

    if (!has_payload) {
        cam_release_buffer(line->buffer_idx);
    }
}

static bool core0_error_is_overflow(uint16_t error_flags)
{
    const uint16_t overflow_errors =
        CAM_FRAME_ERR_DESC_QUEUE_FULL |
        CAM_FRAME_ERR_DMA_OVERRUN |
        CAM_FRAME_ERR_PIO_RXOVER |
        CAM_FRAME_ERR_NO_BUFFER;
    return (error_flags & overflow_errors) != 0u;
}

static void core0_finish_frame(uint32_t frame_id, uint16_t rows_seen,
                               uint16_t capture_errors, bool capture_complete)
{
    if (!core0_frame.active) {
        core0_start_frame(frame_id);
    }
    if (core0_frame.frame_id != frame_id) {
        core0_frame.error_flags |= CAM_FRAME_ERR_BOUNDARY;
    }

    core0_frame.error_flags |= capture_errors;
    core0_frame.last_rows_seen = rows_seen;

    /* 走过正常row479描述符后next_emit_row已经是480，此处不生成job。
     * 只有局部不超过3行的尾部描述符缺口才会占位；大缺口按
     * INCOMPLETE收束，绝不在VSYNC中断中生成上百个job。 */
    bool tail_emitted =
        core0_emit_rows_through((uint16_t)(CAPTURE_LINES - 1u));

    bool complete = tail_emitted && capture_complete &&
                    core0_frame.error_flags == 0u &&
                    rows_seen == CAPTURE_LINES &&
                    core0_frame.placeholder_rows == 0u &&
                    core0_frame.output_count == CAPTURE_LINES;

    if (core0_frame.has_last_job) {
        uint8_t flags = PKT_ROW_FLAG_FINAL_LINE;
        if (!complete) {
            flags |= PKT_ROW_FLAG_FRAME_INCOMPLETE;
        }
        if (core0_error_is_overflow(core0_frame.error_flags)) {
            flags |= PKT_ROW_FLAG_OVERFLOW;
        }
        /* Core1 仍在 hold 这个 job，尚未组包，因此这里改 flags 是安全的。 */
        image_finalize_job(core0_frame.last_job_seq, flags,
                           rows_seen, core0_frame.error_flags);
    }

    image_row_job_meta_t end_meta =
        core0_make_job_meta((uint16_t)(CAPTURE_LINES - 1u), 0u);
    end_meta.row_flags = 0u;
    end_meta.job_type = IMAGE_JOB_FRAME_END;
    end_meta.frame_complete = complete ? 1u : 0u;
    uint32_t end_job = image_prepare_frame_end_job(&end_meta);
    core0_push_job(end_job);

    core0_release_ring();
    memset(&core0_frame, 0, sizeof(core0_frame));
}

static void core0_force_close_current_frame(void)
{
    if (!core0_frame.active) {
        return;
    }
    core0_frame.error_flags |= CAM_FRAME_ERR_BOUNDARY;
    core0_finish_frame(core0_frame.frame_id, core0_frame.last_rows_seen,
                       CAM_FRAME_ERR_BOUNDARY, false);
}

static void core0_handle_line(const cam_descriptor_t *line)
{
    if (!core0_frame.active) {
        core0_start_frame(line->frame_id);
    } else if (line->frame_id != core0_frame.frame_id) {
        /* 没有收到 FRAME_END 就来了新帧的行：先把旧帧走完再开新帧。 */
        core0_force_close_current_frame();
        core0_start_frame(line->frame_id);
        core0_frame.error_flags |= CAM_FRAME_ERR_BOUNDARY;
    }

    if (line->row_idx >= CAPTURE_LINES) {
        core0_frame.error_flags |= CAM_FRAME_ERR_TOO_MANY_ROWS | line->error_flags;
        cam_release_buffer(line->buffer_idx);
        return;
    }

    if (line->row_idx < core0_frame.expected_row) {
        /* 行号必须单调；重复行只记错误，绝不回退已发出的行节奏。 */
        core0_frame.error_flags |= CAM_FRAME_ERR_DUPLICATE_ROW;
        cam_release_buffer(line->buffer_idx);
        return;
    }

    if (line->row_idx > core0_frame.expected_row) {
        core0_frame.error_flags |= CAM_FRAME_ERR_ROW_JUMP;
    }
    core0_frame.expected_row = (uint16_t)(line->row_idx + 1u);
    core0_frame.last_rows_seen = line->rows_seen;
    core0_frame.last_descriptor_seq = line->descriptor_seq;
    /* 采集侧的错误在本物理帧内是粘性的，会带进后续所有行包的 metadata。 */
    core0_frame.error_flags |= line->error_flags;

    core0_store_row(line);

    /* 8月5日语义中，收到row N后[row N-2,N-1,N]已完整，因此每个
     * Camera行描述符恰好触发一个同行号job。row0/1是已知零边界。 */
    (void)core0_emit_rows_through(line->row_idx);
}

static void core0_handle_descriptor(const cam_descriptor_t *descriptor)
{
    if (descriptor->type == CAM_DESC_LINE) {
        core0_handle_line(descriptor);
        return;
    }

    if (descriptor->type == CAM_DESC_FRAME_END) {
        if (core0_frame.active && core0_frame.frame_id != descriptor->frame_id) {
            core0_force_close_current_frame();
        }
        core0_finish_frame(descriptor->frame_id, descriptor->rows_seen,
                           descriptor->error_flags, descriptor->valid != 0u);
    }
}

int main(void)
{
    timer_config();
    stdio_init_all();

    cam_gpio_init();
    fpga_gpio_init();
    cam_pio_init();
    fpga_pio_init();
    cam_dma_init();
    fpga_dma_init();

    ov5640_i2c_init();
    ov5640_pin_init();
    system_init_buffers();

    int status = OV5640_Init(BMP_640x480, OV5640_Y8, OV5640_Polarity_4);
    if (status != 0) {
        sys_loop();
    }

    multicore_launch_core1(fpga_pio_core1_entry);

    /* 发送完成中断必须落在 Core1：它内部会自旋等待 PIO1 TXSTALL，
     * 留在 Core0 会直接抢占 Sobel 与采集 DMA 的行交接。 */
    while (!core1_tx_irq_ready) {
        tight_loop_contents();
    }

    ov5640_start_capture();

    while (true) {
        cam_descriptor_t descriptor;
        if (!cam_descriptor_pop(&descriptor)) {
            tight_loop_contents();
            continue;
        }

        core0_handle_descriptor(&descriptor);
    }
}

static void core1_send_job(uint32_t job_seq)
{
    const image_row_job_meta_t *meta = image_get_job_meta(job_seq);
    const uint8_t *row_bits = image_get_row_bits(job_seq);
    pkt_row_header_t *header = (pkt_row_header_t *)packet_buf;
    pkt_row_payload_t *payload =
        (pkt_row_payload_t *)(packet_buf + sizeof(pkt_row_header_t));
    plt_row_trailer_t *trailer =
        (plt_row_trailer_t *)(packet_buf + sizeof(pkt_row_header_t) +
                              sizeof(pkt_row_payload_t));

    while (fpga_dma_busy()) {
        tight_loop_contents();
    }

    if (core1_last_tx_start_us != 0u) {
        while ((time_us_32() - core1_last_tx_start_us) <
               FPGA_TX_MIN_START_INTERVAL_US) {
            tight_loop_contents();
        }
    }

    packet_generator(row_bits, meta, header, payload, trailer);
    core1_last_tx_start_us = time_us_32();
    fpga_tx_start(packet_buf, sizeof(packet_buf));
    pipeline_timing_stats.core1_packets_sent++;
    /* DMA读取的是packet_buf；组包完成后Sobel/row槽即可归还。 */
    image_core1_release_job(job_seq);
}

static void fpga_pio_core1_entry(void)
{
    bool held_valid = false;
    uint32_t held_job = 0u;

    fpga_dma_irq_init_this_core();
    __dmb();
    core1_tx_irq_ready = true;

    while (true) {
        uint32_t job_seq = multicore_fifo_pop_blocking();
        const image_row_job_meta_t *meta = image_get_job_meta(job_seq);

        if (meta->job_type == IMAGE_JOB_FRAME_END) {
            if (held_valid) {
                core1_send_job(held_job);
                held_valid = false;
            }
            image_core1_end_frame(meta->frame_complete != 0u);
            image_core1_release_job(job_seq);
        } else {
            /*
             * 上一行的 job 一直 hold 到下一个 job 到达才发送：这样 Core0
             * 才有机会在帧尾把 FINAL_LINE / FRAME_INCOMPLETE 打到它上面。
             */
            if (held_valid) {
                core1_send_job(held_job);
            }
            image_core1_process_job(job_seq);
            pipeline_timing_stats.core1_row_jobs++;
            held_job = job_seq;
            held_valid = true;
        }
    }
}
