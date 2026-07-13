#include "image_process.h"

#include <string.h>

#include "pico/stdlib.h"

static uint8_t row_fifo[ROW_FIFO_DEPTH][ROW_BYTES];
static uint8_t e_ref[480][ROW_BYTES];
static uint32_t frame_m00;
static uint32_t frame_m10;
static uint32_t frame_m01;
static int32_t T_sq = 14400;
static uint32_t frame_edge_count;
static uint8_t pkt_buf[100];
static uint16_t frame_id;
static uint16_t row_seq;

void system_init_buffers(void)
{
    memset(e_ref, 0, sizeof(e_ref));
    memset(row_fifo, 0, sizeof(row_fifo));
    frame_m00 = 0u;
    frame_m10 = 0u;
    frame_m01 = 0u;
    frame_edge_count = 0u;
    frame_id = 0u;
    row_seq = 0u;
}

uint16_t crc16_ccitt(const void *d1, size_t n1, const void *d2, size_t n2)
{
    uint16_t crc = 0xFFFFu;
    const uint8_t *parts[2] = { (const uint8_t *)d1, (const uint8_t *)d2 };
    const size_t lens[2] = { n1, n2 };

    for (int part = 0; part < 2; ++part) {
        if (parts[part] == NULL || lens[part] == 0u) {
            continue;
        }
        for (size_t i = 0; i < lens[part]; ++i) {
            crc ^= (uint16_t)parts[part][i] << 8;
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

void fused_row_sq(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2,
                  uint8_t *bits_out, int32_t T_sq_val, int width,
                  uint32_t *edge_count)
{
    int16_t v_m1 = r0[0] + (r1[0] << 1) + r2[0];
    int16_t h_m1 = r2[0] - r0[0];
    int16_t v_0 = r0[1] + (r1[1] << 1) + r2[1];
    int16_t h_0 = r2[1] - r0[1];
    uint32_t acc = 0u;
    int nb = 1;
    uint8_t *out = bits_out;
    *out = 0u;

    for (int x = 1; x < width - 1; ++x) {
        int16_t v_p1 = r0[x + 1] + (r1[x + 1] << 1) + r2[x + 1];
        int16_t h_p1 = r2[x + 1] - r0[x + 1];
        int16_t gx = v_p1 - v_m1;
        int16_t gy = h_m1 + (h_0 << 1) + h_p1;
        int32_t gx2 = (int32_t)gx * (int32_t)gx;
        int32_t g_sq = (int32_t)gy * (int32_t)gy + gx2;
        uint32_t bit = (uint32_t)(g_sq > T_sq_val);
        if (edge_count != NULL) {
            *edge_count += bit;
        }
        acc = (acc << 1) | bit;
        if (++nb == 8) {
            *out++ = (uint8_t)acc;
            acc = 0u;
            nb = 0;
        }
        v_m1 = v_0;
        v_0 = v_p1;
        h_m1 = h_0;
        h_0 = h_p1;
    }

    acc <<= 1;
    *out++ = (uint8_t)(acc << (8 - nb - 1));
}

void xor_row_moments(const uint8_t *new_row, const uint8_t *old_row,
                     uint32_t y, uint32_t *m00, uint32_t *m10, uint32_t *m01)
{
    const uint32_t *a = (const uint32_t *)new_row;
    const uint32_t *b = (const uint32_t *)old_row;

    for (int wi = 0; wi < (int)(ROW_BYTES / 4u); ++wi) {
        uint32_t m = a[wi] ^ b[wi];
        if (m == 0u) {
            continue;
        }
        int xbase = wi * 32;
        while (m != 0u) {
            int bit = __builtin_clz(m);
            int x = xbase + bit;
            (*m00)++;
            *m10 += (uint32_t)x;
            *m01 += y;
            m &= ~(0x80000000u >> bit);
        }
    }
}

size_t rle_encode_row(const uint8_t *bits, uint8_t *out, size_t max_len)
{
    size_t written = 0u;

    for (size_t i = 0u; i < ROW_BYTES && written < max_len; ++i) {
        out[written++] = bits[i];
    }

    return written;
}

void packet_send_meta(uint32_t m00, uint32_t m10, uint32_t m01)
{
    pkt_meta_t t = { 0xA55Au, frame_id, m00, 0u, 0u, 0, 0, 0u };

    if (m00 > 0u) {
        float xc = (float)m10 / (float)m00;
        float yc = (float)m01 / (float)m00;
        t.xc_q4 = (uint16_t)(xc * 16.0f + 0.5f);
        t.yc_q4 = (uint16_t)(yc * 16.0f + 0.5f);
    }

    t.crc = crc16_ccitt(&t, sizeof(t) - 2u, NULL, 0u);
    memcpy(pkt_buf, &t, sizeof(t));
}

void packet_generator(const uint8_t *row_bits, uint32_t row_idx, uint32_t frame_id_in,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, pkt_row_trailer_t *trailer)
{
    header->sync0 = 0xA5A5u;
    header->sync1 = 0xA5A5u;
    header->frame_id = (uint16_t)frame_id_in;
    header->row_idx = (uint16_t)row_idx;
    header->flags = 0x03u;
    header->payload_len = (uint8_t)ROW_BYTES;
    header->row_seq = row_seq++;
    memset(header->reserved, 0, sizeof(header->reserved));

    memcpy(payload->payload, row_bits, ROW_BYTES);
    trailer->crc16 = crc16_ccitt(header, sizeof(*header), payload, sizeof(*payload));
    trailer->pad[0] = 0u;
    trailer->pad[1] = 0u;
}

void process_frame_row(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2, uint32_t row_idx)
{
    uint8_t *bits = row_fifo[row_idx % ROW_FIFO_DEPTH];
    uint32_t edge_count = 0u;

    fused_row_sq(r0, r1, r2, bits, T_sq, 640, &edge_count);
    frame_edge_count += edge_count;
    xor_row_moments(bits, e_ref[row_idx], row_idx, &frame_m00, &frame_m10, &frame_m01);
    memcpy(e_ref[row_idx], bits, ROW_BYTES);
}

void update_threshold(void)
{
    const uint32_t target = (640u * 480u / 1000000u) * 40000u;

    if (frame_edge_count > (target * 6u) / 5u) {
        T_sq = (T_sq * 12) / 11;
    } else if (frame_edge_count < (target * 4u) / 5u) {
        T_sq = (T_sq * 11) / 12;
    }

    if (T_sq < 400) {
        T_sq = 400;
    }
    if (T_sq > 200000) {
        T_sq = 200000;
    }

    frame_edge_count = 0u;
}

const uint8_t *image_get_row_bits(uint32_t row_idx)
{
    return row_fifo[row_idx % ROW_FIFO_DEPTH];
}