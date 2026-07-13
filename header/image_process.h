#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <stddef.h>
#include <stdint.h>

#define ROW_BYTES      80u
#define ROW_FIFO_DEPTH  8u

typedef struct __attribute__((packed)) {
    uint16_t sync0;
    uint16_t sync1;
    uint16_t frame_id;
    uint16_t row_idx;
    uint8_t  flags;
    uint8_t  payload_len;
    uint16_t row_seq;
    uint8_t  reserved[4];
} pkt_row_header_t;

typedef struct __attribute__((packed)) {
    uint8_t payload[ROW_BYTES];
} pkt_row_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t crc16;
    uint8_t  pad[2];
} pkt_row_trailer_t;

typedef struct __attribute__((packed)) {
    uint16_t sync;
    uint16_t frame_id;
    uint32_t m00;
    uint16_t xc_q4;
    uint16_t yc_q4;
    int16_t  vx_q8;
    int16_t  vy_q8;
    uint16_t crc;
} pkt_meta_t;

void system_init_buffers(void);
void fused_row_sq(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2,
                  uint8_t *bits_out, int32_t T_sq_val, int width,
                  uint32_t *edge_count);
void xor_row_moments(const uint8_t *new_row, const uint8_t *old_row,
                     uint32_t y, uint32_t *m00, uint32_t *m10, uint32_t *m01);
size_t rle_encode_row(const uint8_t *bits, uint8_t *out, size_t max_len);
uint16_t crc16_ccitt(const void *d1, size_t n1, const void *d2, size_t n2);
void packet_generator(const uint8_t *row_bits, uint32_t row_idx, uint32_t frame_id,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, pkt_row_trailer_t *trailer);
void packet_send_meta(uint32_t m00, uint32_t m10, uint32_t m01);
void process_frame_row(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2, uint32_t row_idx);
void update_threshold(void);
const uint8_t *image_get_row_bits(uint32_t row_idx);

#endif /* IMAGE_PROCESS_H */