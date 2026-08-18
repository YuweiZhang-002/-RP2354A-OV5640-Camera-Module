#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define ROW_BYTES       80u
#define ROW_FIFO_DEPTH  8u
#define PACKET_BYTES    128u

/* 行包标志位：只保留与当前流水线直接相关的两种语义 */
#define PKT_ROW_FLAG_OVERFLOW    (1u << 0)  /* 对应 cam_overrun_count 增量 */
#define PKT_ROW_FLAG_FINAL_LINE  (1u << 1)  /* 当前帧最后一行 */
#define PKT_ROW_FLAG_FIRST_LINE  (1u << 2)  /* 当前帧第一行 */

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
 * 128-byte 线上格式不变：header[0..23] / payload[24..103] / trailer[104..127]。
 * XOR 与质心检测已移除，原来那 12 个字节（包内偏移 114..125，
 * 曾经是 m00 / xc_q4 / yc_q4 / vx_q8 / vy_q8）现在全部填同步图案，
 * 供 FPGA 侧定位包尾、校验没有发生字节滑移。crc16 偏移不变。
 */
#define PKT_TRAILER_SYNC   0xA55Au
#define PKT_TRAILER_SYNC_WORDS 6u

typedef struct __attribute__((packed)) {
    uint8_t  pad[10];                            /* 104..113 */
    uint16_t sync[PKT_TRAILER_SYNC_WORDS];       /* 114..125，全部 = PKT_TRAILER_SYNC */
    uint16_t crc16;                              /* 126..127 */
} plt_row_trailer_t;

/* Wire-format contract: header + payload + trailer must remain 24 + 80 + 24. */
_Static_assert(sizeof(pkt_row_header_t) == 24u, "packet header must be 24 bytes");
_Static_assert(sizeof(pkt_row_payload_t) == 80u, "packet payload must be 80 bytes");
_Static_assert(sizeof(plt_row_trailer_t) == 24u, "packet trailer must be 24 bytes");
_Static_assert(sizeof(pkt_row_header_t) + sizeof(pkt_row_payload_t) +
               sizeof(plt_row_trailer_t) == PACKET_BYTES,
               "packet layout must match PACKET_BYTES");
_Static_assert(offsetof(plt_row_trailer_t, sync) == 10u,
               "trailer sync must start at packet offset 114");
_Static_assert(offsetof(plt_row_trailer_t, crc16) == 22u,
               "crc16 wire offset changed");

void system_init_buffers(void);
void debug_gpio_init(void);
size_t rle_encode_row(const uint8_t *bits, uint8_t *out, size_t max_len);
uint16_t crc16_ccitt(const void *d1, size_t n1, const void *d2, size_t n2, const void *d3, size_t n3);
void packet_generator(const uint8_t *row_bits, uint32_t row_idx, uint32_t frame_id, bool overflow, bool first_line, bool final_line,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, plt_row_trailer_t *trailer);
void process_frame_row(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2, uint32_t row_idx);
void image_core1_process_row(uint32_t abs_row_idx, uint32_t frame_row_idx);
const uint8_t *image_get_row_bits(uint32_t row_idx);

#endif /* IMAGE_PROCESS_H */
