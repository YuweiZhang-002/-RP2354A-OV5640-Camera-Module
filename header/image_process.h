#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define ROW_BYTES       80u
#define ROW_FIFO_DEPTH  16u
#define PACKET_BYTES    128u

/* 行包标志位：帧边界、完整性与局部占位行语义 */
#define PKT_ROW_FLAG_OVERFLOW    (1u << 0)  /* 对应 cam_overrun_count 增量 */
#define PKT_ROW_FLAG_FINAL_LINE  (1u << 1)  /* 当前帧最后一行 */
#define PKT_ROW_FLAG_FIRST_LINE  (1u << 2)  /* 当前帧第一行 */
#define PKT_ROW_FLAG_FRAME_INCOMPLETE (1u << 3) /* 当前物理帧最后有效包，但帧不完整 */
#define PKT_ROW_FLAG_INVALID_ROW (1u << 4) /* 原始行/三行窗口缺失，payload为零占位 */

typedef struct __attribute__((packed)) {
    uint16_t sync0;
    uint16_t sync1;
    uint8_t  cam_id;
    uint16_t frame_id;
    uint16_t row_idx;
    uint8_t  row_flags;   /* 行包控制位：见 PKT_ROW_FLAG_* */
    uint8_t  payload_len;
    uint16_t row_seq;
    uint8_t  reserved[11];
} pkt_row_header_t;

typedef struct __attribute__((packed)) {
    uint8_t payload[ROW_BYTES];
} pkt_row_payload_t;

/*
 * 128-byte wire layout is unchanged: header[0..23], payload[24..103],
 * trailer[104..127].  The former trailer fields were:
 *   104..113 pad[10] (10 bytes)
 *   114..117 m00 (4 bytes)
 *   118..119 xc_q4 (2 bytes)
 *   120..121 yc_q4 (2 bytes)
 *   122..123 vx_q8 (2 bytes)
 *   124..125 vy_q8 (2 bytes)
 *   126..127 crc16 (2 bytes)
 * They only described the removed XOR/motion-moment path.  The same 24 bytes
 * now carry capture consistency information; packet size and CRC offset stay
 * fixed.
 */
typedef struct __attribute__((packed)) {
    uint32_t physical_frame_id;       /* 104..107 */
    uint16_t physical_rows_seen;      /* 108..109 */
    uint16_t capture_error_flags;     /* 110..111 */
    uint32_t descriptor_seq;          /* 112..115 */
    uint32_t descriptor_overrun_count;/* 116..119 */
    uint32_t capture_overrun_count;   /* 120..123 */
    uint16_t skip_done_count;         /* 124..125 */
    uint16_t crc16;                   /* 126..127 */
} plt_row_trailer_t;

typedef struct {
    uint32_t frame_id;
    uint32_t descriptor_seq;
    uint32_t descriptor_overrun_count;
    uint32_t capture_overrun_count;
    uint16_t row_idx;
    uint16_t physical_rows_seen;
    uint16_t capture_error_flags;
    uint16_t skip_done_count;
    uint8_t row_flags;
    uint8_t has_sobel;
    uint8_t job_type;
    uint8_t frame_complete;
} image_row_job_meta_t;

#define IMAGE_JOB_ROW       0u
#define IMAGE_JOB_FRAME_END 1u

typedef struct {
    volatile uint32_t core0_row_jobs;      /* 应恒等于 core1_packets_sent + 在途 */
    volatile uint32_t core0_sobel_rows;    /* 带真实三行窗口的行数 */
    volatile uint32_t core0_repair_rows;   /* INVALID_ROW 占位行数 */
    volatile uint32_t core1_row_jobs;
    volatile uint32_t core1_packets_sent;
    volatile uint32_t core0_service_last_us;
    volatile uint32_t core0_service_max_us;
    volatile uint32_t core0_push_wait_last_us;
    volatile uint32_t core0_push_wait_max_us;
    volatile uint32_t core1_service_last_us;
    volatile uint32_t core1_service_max_us;
    volatile uint32_t core1_fpga_busy_wait_last_us;
    volatile uint32_t core1_fpga_busy_wait_max_us;
    volatile uint32_t core1_pacing_wait_last_us;
    volatile uint32_t core1_pacing_wait_max_us;
    volatile uint32_t core1_tx_wait_last_us;
    volatile uint32_t core1_tx_wait_max_us;
} pipeline_timing_stats_t;

/* Wire-format contract: header + payload + trailer must remain 24 + 80 + 24. */
_Static_assert(sizeof(pkt_row_header_t) == 24u, "packet header must be 24 bytes");
_Static_assert(sizeof(pkt_row_payload_t) == 80u, "packet payload must be 80 bytes");
_Static_assert(sizeof(plt_row_trailer_t) == 24u, "packet trailer must be 24 bytes");
_Static_assert(offsetof(plt_row_trailer_t, physical_frame_id) == 0u,
               "physical_frame_id wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, physical_rows_seen) == 4u,
               "physical_rows_seen wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, capture_error_flags) == 6u,
               "capture_error_flags wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, descriptor_seq) == 8u,
               "descriptor_seq wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, descriptor_overrun_count) == 12u,
               "descriptor_overrun_count wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, capture_overrun_count) == 16u,
               "capture_overrun_count wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, skip_done_count) == 20u,
               "skip_done_count wire offset changed");
_Static_assert(offsetof(plt_row_trailer_t, crc16) == 22u,
               "crc16 wire offset changed");
_Static_assert(sizeof(pkt_row_header_t) + sizeof(pkt_row_payload_t) +
               sizeof(plt_row_trailer_t) == PACKET_BYTES,
               "packet layout must match PACKET_BYTES");

void system_init_buffers(void);
void debug_gpio_init(void);
size_t rle_encode_row(const uint8_t *bits, uint8_t *out, size_t max_len);
uint16_t crc16_ccitt(const void *d1, size_t n1, const void *d2, size_t n2, const void *d3, size_t n3);
void packet_generator(const uint8_t *row_bits, const image_row_job_meta_t *meta,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, plt_row_trailer_t *trailer);
uint32_t image_prepare_sobel_job(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2,
                                 const image_row_job_meta_t *meta);
uint32_t image_prepare_zero_job(const image_row_job_meta_t *meta);
uint32_t image_prepare_frame_end_job(const image_row_job_meta_t *meta);
void image_finalize_job(uint32_t job_seq, uint8_t additional_flags,
                        uint16_t rows_seen, uint16_t capture_error_flags);
void image_core1_process_job(uint32_t job_seq);
void image_core1_release_job(uint32_t job_seq);
void image_core1_end_frame(bool complete);
const uint8_t *image_get_row_bits(uint32_t job_seq);
const image_row_job_meta_t *image_get_job_meta(uint32_t job_seq);

extern pipeline_timing_stats_t pipeline_timing_stats;

#endif /* IMAGE_PROCESS_H */
