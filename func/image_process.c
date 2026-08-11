#include "image_process.h"    /* 图像处理阶段对外声明：行处理、组包、CRC、阈值 */

#include <string.h>            /* memset / memcpy，用于缓冲清零与行数据复制 */

#include "pico.h"             /* Pico 统一入口：提供 __not_in_flash_func 与 CMSIS 内建 */
#include "pico/stdlib.h"      /* Pico 基础类型与 __SMULBB / __SMLABB 等内建支持 */
#include "hardware/sync.h"    /* __dmb: 跨核job发布/归还内存顺序 */
#include "cam_pio.h"          /* CAPTURE_BYTES / CAPTURE_LINES / 摄像头行参数 */

/* Core1私有：threshold及完整有效帧的动态阈值状态。 */
static int32_t  T_sq = 14400;              /* 初值=120²,与卷一标定值接轨 */
static uint32_t frame_edge_count = 0;

/* Core0写/Core1读：Sobel、二值行以及与槽位绑定的packet metadata。 */
static uint32_t sobel_fifo[ROW_FIFO_DEPTH][CAPTURE_BYTES] __attribute__((aligned(4)));
static uint8_t row_fifo[ROW_FIFO_DEPTH][ROW_BYTES];
static image_row_job_meta_t job_meta_fifo[ROW_FIFO_DEPTH];

/* job序号连续递增，与物理frame/row身份解耦；缺行不会破坏槽位所有权。 */
static volatile uint32_t image_job_prod_seq = 0u;
static volatile uint32_t image_job_consumed_seq = 0u;

/* Core1私有：线上行包序号。 */
static uint16_t row_seq = 0u;

pipeline_timing_stats_t pipeline_timing_stats;

static void update_threshold(void);

static inline uint16_t wire_u16(uint16_t value)
{
    return __builtin_bswap16(value);
}

static inline uint32_t wire_u32(uint32_t value)
{
    return __builtin_bswap32(value);
}

const uint32_t target = (uint32_t)((uint64_t)640 * 480 * 40000u / 1000000u);  /* = 12288 */


void debug_gpio_init(void)
{
    /* GPIO8/9已固定为FPGA PCLK/HREF；保留接口但不再占用这两个引脚。 */
}

/**
 * @brief  初始化图像处理模块的静态缓冲区和状态变量。
 * @note   作用：在系统启动初期，清零所有相关内存，重置状态变量。
 *         位置：在 main() 函数中调用，确保在图像处理开始前，所有状态都处于已知的初始值。
 */
void system_init_buffers(void)
{
    memset(sobel_fifo, 0, sizeof(sobel_fifo));
    memset(row_fifo, 0, sizeof(row_fifo));
    memset(job_meta_fifo, 0, sizeof(job_meta_fifo));
    memset(&pipeline_timing_stats, 0, sizeof(pipeline_timing_stats));
    frame_edge_count = 0u;
    row_seq = 0u;
    image_job_prod_seq = 0u;
    image_job_consumed_seq = 0u;
}

/**
 * @brief  预留的逐字节 CRC-16-CCITT 扩展入口。
 * @note   当前线上 crc16 仍固定写 0xFFFF，本函数不在实时发送路径中调用。
 *         如后续启用，可按 header/payload/trailer 三段以同一字节顺序计算。
 */
uint16_t crc16_ccitt(const void *d1, size_t n1, const void *d2, size_t n2, const void *d3, size_t n3)
{
    uint16_t crc = 0xFFFFu;
    const uint8_t *parts[3] = { (const uint8_t *)d1, (const uint8_t *)d2, (const uint8_t *) d3 };
    const size_t lens[3] = { n1, n2, n3 };
    const uint8_t *p;

    for (int part = 0; part < 3; part++) {
        p = parts[part];
        for (size_t i = 0; i < lens[part]; i++) {
            crc ^= (uint16_t)p[i] << 8;
            for (int b = 0; b < 8; b++) {
                crc = (uint16_t)((crc & 0x8000u)
                    ? (((uint32_t)crc << 1) ^ 0x1021u)
                    : ((uint32_t)crc << 1));
            }
        }
    }

    return crc;
}

/**
 * @brief  对输入的三行图像数据执行 Sobel 边缘检测。
 * @param  r0          上一行 (y-1) 的原始灰度数据指针。
 * @param  r1          当前行 (y)   的原始灰度数据指针。
 * @param  r2          下一行 (y+1) 的原始灰度数据指针。
 * @param  sobel_pairs 输出的 Sobel 中间结果缓冲区，每个像素保存一组 (gx, gy) 打包值。
 * @param  width       图像宽度。
 * @note   作用：这是行级图像处理的第一段，只负责 Sobel 滑窗和梯度中间值生成。
 *         位置：fused_row_sq() 直接执行该 Sobel 阶段，结果供 Core 1 Threshold 阶段复用。
 */
static void __attribute__((noinline))  __not_in_flash_func(fused_row_sq)(const uint8_t *restrict r0, const uint8_t *restrict r1, const uint8_t *restrict r2,
                  uint32_t *restrict sobel_pairs, int width)
{    
    /* 指令③: 滑窗与梯度局部量统一 int32_t, 避免逐步 16 位截断(SXTH) */
    int32_t v_m1 = r0[0] + (r1[0] << 1) + r2[0], h_m1 = r2[0] - r0[0];
    int32_t v_0  = r0[1] + (r1[1] << 1) + r2[1], h_0  = r2[1] - r0[1];
    sobel_pairs[0] = 0u;

    int x = 1;
    for (; x + 3 < width - 1; x += 4)
    {
        /* 指令②: 3 次完整 32 位 load。从 x+1 起始取词, 一次拿到本轮所需的
         * 4 个新列 (x+1..x+4), 省去原先越界的 r?_4 字节 load。
         * ARM 小端: 词内 低字节=列x+1, 次字节=x+2, 第三字节=x+3, 高字节=x+4。 */
        const uint32_t w0 = *(const uint32_t *)(const void *)(r0 + x + 1);
        const uint32_t w1 = *(const uint32_t *)(const void *)(r1 + x + 1);
        const uint32_t w2 = *(const uint32_t *)(const void *)(r2 + x + 1);

        {
            /* 像素 x, x+1: 用列 x+1(词低字节)、x+2(次字节)。
             * 指令②: 字节即取即用, 不跨块存活; 指令③: 局部量 int32_t。 */
            int32_t vp1_0 = (uint8_t)w0 + ((uint8_t)w1 << 1) + (uint8_t)w2;
            int32_t hp1_0 = (uint8_t)w2 - (uint8_t)w0;
            int32_t gx0 = vp1_0 - v_m1;
            int32_t gy0 = h_m1 + (h_0 << 1) + hp1_0;

            int32_t vp1_1 = (uint8_t)(w0 >> 8) + ((uint8_t)(w1 >> 8) << 1) + (uint8_t)(w2 >> 8);
            int32_t hp1_1 = (uint8_t)(w2 >> 8) - (uint8_t)(w0 >> 8);
            int32_t gx1 = vp1_1 - v_0;
            int32_t gy1 = h_0 + (hp1_0 << 1) + hp1_1;

            sobel_pairs[x]   = ((uint32_t)(uint16_t)gx0) | ((uint32_t)(uint16_t)gy0 << 16);
            sobel_pairs[x+1] = ((uint32_t)(uint16_t)gx1) | ((uint32_t)(uint16_t)gy1 << 16);

            v_m1 = vp1_0;
            v_0  = vp1_1;
            h_m1 = hp1_0;
            h_0  = hp1_1;
        }

        {
            /* 像素 x+2, x+3: 用列 x+3(词第三字节)、x+4(高字节)。 */
            int32_t vp1_0 = (uint8_t)(w0 >> 16) + ((uint8_t)(w1 >> 16) << 1) + (uint8_t)(w2 >> 16);
            int32_t hp1_0 = (uint8_t)(w2 >> 16) - (uint8_t)(w0 >> 16);
            int32_t gx0 = vp1_0 - v_m1;
            int32_t gy0 = h_m1 + (h_0 << 1) + hp1_0;

            int32_t vp1_1 = (uint8_t)(w0 >> 24) + ((uint8_t)(w1 >> 24) << 1) + (uint8_t)(w2 >> 24);
            int32_t hp1_1 = (uint8_t)(w2 >> 24) - (uint8_t)(w0 >> 24);
            int32_t gx1 = vp1_1 - v_0;
            int32_t gy1 = h_0 + (hp1_0 << 1) + hp1_1;

            sobel_pairs[x+2] = ((uint32_t)(uint16_t)gx0) | ((uint32_t)(uint16_t)gy0 << 16);
            sobel_pairs[x+3] = ((uint32_t)(uint16_t)gx1) | ((uint32_t)(uint16_t)gy1 << 16);

            v_m1 = vp1_0;
            v_0  = vp1_1;
            h_m1 = hp1_0;
            h_0  = hp1_1;
        }
    }

    for (; x < width - 2; x += 2)
    {
        /* 指令③: 尾部 2 像素收口循环同样使用 int32_t */
        int32_t vp1_0 = r0[x + 1] + (r1[x + 1] << 1) + r2[x + 1];
        int32_t hp1_0 = r2[x + 1] - r0[x + 1];
        int32_t gx0 = vp1_0 - v_m1;
        int32_t gy0 = h_m1 + (h_0 << 1) + hp1_0;

        int32_t vp1_1 = r0[x + 2] + (r1[x + 2] << 1) + r2[x + 2];
        int32_t hp1_1 = r2[x + 2] - r0[x + 2];
        int32_t gx1 = vp1_1 - v_0;
        int32_t gy1 = h_0 + (hp1_0 << 1) + hp1_1;

        sobel_pairs[x]   = ((uint32_t)(uint16_t)gx0) | ((uint32_t)(uint16_t)gy0 << 16);
        sobel_pairs[x+1] = ((uint32_t)(uint16_t)gx1) | ((uint32_t)(uint16_t)gy1 << 16);

        v_m1 = vp1_0;
        v_0  = vp1_1;
        h_m1 = hp1_0;
        h_0  = hp1_1;
    }

    sobel_pairs[width - 1] = 0u;
    
}

static void __attribute__((noinline)) __not_in_flash_func(filter_pack_row_bits)(const uint32_t *restrict sobel_pairs,
                  uint8_t *restrict bits_out, int32_t T_sq_val, int width,
                  uint32_t *edge_count)
{
    
    /* 指令④: 定长 8 像素/字节打包。width=640 恰为 8 整除 (80 字节),
     * 无余数、无 nb 计数、无 if(++nb==8) 分支; 每字节一次性写回。
     * 输出位序与原实现完全一致(MSB 先入, 像素 p -> 字节 p>>3 的第 7-(p&7) 位);
     * 边界像素 sobel_pairs[0]/[width-1] 恒为 0, SMUAD 得 0, 阈值判定必为 0,
     * 因此逐 8 像素扫描全宽的结果与原来"跳过首末像素"完全等价。 */
    uint32_t local_edge = 0u;
    const int nbytes = width >> 3;

    for (int by = 0; by < nbytes; ++by) {
        const uint32_t *pp = &sobel_pairs[(uint32_t)by << 3];
        uint32_t acc = 0u;
        for (int k = 0; k < 8; ++k) {   /* 固定 8 次, 编译期展开, 无数据相关分支 */
            int32_t g_sq = (int32_t)__builtin_arm_smuad(pp[k], pp[k]);
            uint32_t bit = (uint32_t)(g_sq > T_sq_val);
            local_edge += bit;
            acc = (acc << 1) | bit;
        }
        bits_out[by] = (uint8_t)acc;
    }

    *edge_count += local_edge;
    
}

/**
 * @brief  对二值化行数据进行行程长度编码（RLE）。
 * @note   作用：压缩数据以减少传输量。
 *         当前实现：这是一个占位符，仅执行了直接复制，未实现真正的 RLE 压缩。
 */
size_t rle_encode_row(const uint8_t *bits, uint8_t *out, size_t max_len)
{
    size_t written = 0u;

    for (size_t i = 0u; i < ROW_BYTES && written < max_len; ++i) {
        out[written++] = bits[i];
    }

    return written;
}

/**
 * @brief  将处理后的行数据打包成一个完整的行数据包。
 * @note   作用：构建一个包含同步头、帧/行信息、数据载荷和 CRC 校验的标准化数据包。
 *         位置：在 core1 的发送循环中被调用，为每一行数据生成一个待发送的数据包。
 */
void packet_generator(const uint8_t *row_bits, const image_row_job_meta_t *meta,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, plt_row_trailer_t *trailer)
{
    header->sync0 = wire_u16(0xA5A0u);
    header->sync1 = wire_u16(0x5A50u);
    header->cam_id = 0u;  /* 摄像头ID，当前版本固定为0 */
    header->frame_id = wire_u16((uint16_t)meta->frame_id);
    header->row_idx = wire_u16(meta->row_idx);
    header->row_flags = meta->row_flags;
    header->payload_len = (uint8_t)ROW_BYTES;
    header->row_seq = wire_u16(row_seq++);
    memset(header->reserved, 0, sizeof(header->reserved));

    memcpy(payload->payload, row_bits, ROW_BYTES);

    /* 24-byte trailer保持原偏移/长度，运动矩字段改为一致性metadata。 */
    trailer->physical_frame_id = wire_u32(meta->frame_id);
    trailer->physical_rows_seen = wire_u16(meta->physical_rows_seen);
    trailer->capture_error_flags = wire_u16(meta->capture_error_flags);
    trailer->descriptor_seq = wire_u32(meta->descriptor_seq);
    trailer->descriptor_overrun_count = wire_u32(meta->descriptor_overrun_count);
    trailer->capture_overrun_count = wire_u32(meta->capture_overrun_count);
    trailer->skip_done_count = wire_u16(meta->skip_done_count);
    trailer->crc16 = wire_u16(0xFFFFu);  /* CRC16 校验码暂时置为65535，FPGA端计算 */
}

static uint32_t image_reserve_job(const image_row_job_meta_t *meta)
{
    while ((image_job_prod_seq - image_job_consumed_seq) >= ROW_FIFO_DEPTH) {
        tight_loop_contents();
    }

    uint32_t job_seq = image_job_prod_seq++;
    job_meta_fifo[job_seq % ROW_FIFO_DEPTH] = *meta;
    return job_seq;
}

uint32_t image_prepare_sobel_job(const uint8_t *r0, const uint8_t *r1,
                                 const uint8_t *r2,
                                 const image_row_job_meta_t *meta)
{
    uint32_t job_seq = image_reserve_job(meta);
    image_row_job_meta_t *stored = &job_meta_fifo[job_seq % ROW_FIFO_DEPTH];
    stored->has_sobel = 1u;
    stored->job_type = IMAGE_JOB_ROW;
    fused_row_sq(r0, r1, r2, sobel_fifo[job_seq % ROW_FIFO_DEPTH], CAPTURE_BYTES);
    __dmb();
    return job_seq;
}

uint32_t image_prepare_zero_job(const image_row_job_meta_t *meta)
{
    uint32_t job_seq = image_reserve_job(meta);
    image_row_job_meta_t *stored = &job_meta_fifo[job_seq % ROW_FIFO_DEPTH];
    stored->has_sobel = 0u;
    stored->job_type = IMAGE_JOB_ROW;
    __dmb();
    return job_seq;
}

uint32_t image_prepare_frame_end_job(const image_row_job_meta_t *meta)
{
    uint32_t job_seq = image_reserve_job(meta);
    image_row_job_meta_t *stored = &job_meta_fifo[job_seq % ROW_FIFO_DEPTH];
    stored->has_sobel = 0u;
    stored->job_type = IMAGE_JOB_FRAME_END;
    __dmb();
    return job_seq;
}

void image_finalize_job(uint32_t job_seq, uint8_t additional_flags,
                        uint16_t rows_seen, uint16_t capture_error_flags)
{
    image_row_job_meta_t *meta = &job_meta_fifo[job_seq % ROW_FIFO_DEPTH];
    meta->row_flags |= additional_flags;
    meta->physical_rows_seen = rows_seen;
    meta->capture_error_flags |= capture_error_flags;
    meta->descriptor_overrun_count = cam_descriptor_overrun_count;
    meta->capture_overrun_count = cam_overrun_count;
    meta->skip_done_count = (uint16_t)cam_skip_done_count;
    __dmb();
}

/**
 * @brief  在一帧处理完成后，根据总边缘像素数自适应更新阈值。
 * @note   作用：通过动态调整阈值 T_sq，使边缘检测算法能适应不同光照和场景复杂度，保持边缘点数量大致稳定。
 *         位置：在 main() 主循环中，每处理完一帧的最后一行时调用。
 */
static void update_threshold(void)
{
    
    if (frame_edge_count > (target * 6u) / 5u) {
        T_sq = (T_sq * 12) / 11;
    } else if (frame_edge_count < (target * 4u) / 5u) {
        T_sq = (T_sq * 11) / 12;
    }

    if (T_sq < 400) T_sq = 400;       // T_SQ_MIN
    if (T_sq > 200000) T_sq = 200000; // T_SQ_MAX

    frame_edge_count = 0u;
}

void image_core1_end_frame(bool complete)
{
    if (complete) {
        update_threshold();
    } else {
        /* 不完整帧不参与阈值反馈，也不能把累计量带到下一物理帧。 */
        frame_edge_count = 0u;
    }
}

void image_core1_process_job(uint32_t job_seq)
{
    uint32_t slot = job_seq % ROW_FIFO_DEPTH;
    uint8_t *bits = row_fifo[slot];
    const image_row_job_meta_t *meta = &job_meta_fifo[slot];

    if (meta->has_sobel == 0u) {
        /* 8月5日语义中的row0/row1边界行和INVALID_ROW占位行不读取Sobel，
         * payload固定为80-byte 0。 */
        memset(bits, 0, ROW_BYTES);
    } else {
        uint32_t edge_count = 0u;
        filter_pack_row_bits(sobel_fifo[slot], bits, T_sq,
                             CAPTURE_BYTES, &edge_count);
        frame_edge_count += edge_count;
    }
    __dmb();
}

void image_core1_release_job(uint32_t job_seq)
{
    __dmb();
    image_job_consumed_seq = job_seq + 1u;
}

const uint8_t *image_get_row_bits(uint32_t job_seq)
{
    return row_fifo[job_seq % ROW_FIFO_DEPTH];
}

const image_row_job_meta_t *image_get_job_meta(uint32_t job_seq)
{
    return &job_meta_fifo[job_seq % ROW_FIFO_DEPTH];
}
