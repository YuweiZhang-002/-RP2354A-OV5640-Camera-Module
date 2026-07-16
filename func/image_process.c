#include "image_process.h"    /* 图像处理阶段对外声明：行处理、组包、CRC、阈值 */

#include <string.h>            /* memset / memcpy，用于缓冲清零与行数据复制 */

#include "pico.h"             /* Pico 统一入口：提供 __not_in_flash_func 与 CMSIS 内建 */
#include "pico/stdlib.h"      /* Pico 基础类型与 __SMULBB / __SMLABB 等内建支持 */
#include "cam_pio.h"          /* CAPTURE_BYTES / CAPTURE_LINES / 摄像头行参数 */
#include "hardware/gpio.h"    /* GPIO9 拥堵调试脚 */

/*
 * ===================================================================
 *  全局状态: 按七层数据流模型划分
 * ===================================================================
 */

/* 层1 核0私有: Sobel+阈值+动态T状态 */
static int32_t  T_sq = 14400;              /* 初值=120²,与卷一标定值接轨 */
static uint32_t frame_edge_count = 0;

/* 层2 核0写核1读: Sobel 中间结果环（共享 SRAM） */
static uint32_t sobel_fifo[ROW_FIFO_DEPTH][CAPTURE_BYTES] __attribute__((aligned(4)));

/* 层2 核0写核1读: 行级二值结果环（共享 SRAM） */
static uint8_t row_fifo[ROW_FIFO_DEPTH][ROW_BYTES];

/* 层2 跨核同步: Sobel 结果消费计数器，由 core1 在处理后推进 */
static volatile uint32_t sobel_fifo_consumed_count = 0;

/* 层3 核1私有: 参考帧(方案A, 单缓冲原地覆写) & 图像矩 */
static uint8_t e_ref[480][ROW_BYTES];
static uint32_t frame_m00, frame_m10, frame_m01;

/* 层5 & 6 核1私有: 组包序列与位置状态 */
static uint16_t frame_id = 0, row_seq = 0;
static uint16_t prev_xc_q4 = 0;
static uint16_t prev_yc_q4 = 0;
static uint8_t prev_meta_valid = 0u;

const uint32_t target = (uint32_t)((uint64_t)640 * 480 * 40000u / 1000000u);  /* = 12288 */


void debug_gpio_init(void)
{
    gpio_init(9u);
    gpio_set_dir(9u, GPIO_OUT);
    gpio_put(9u, 0);
}

/**
 * @brief  初始化图像处理模块的静态缓冲区和状态变量。
 * @note   作用：在系统启动初期，清零所有相关内存，重置状态变量。
 *         位置：在 main() 函数中调用，确保在图像处理开始前，所有状态都处于已知的初始值。
 */
void system_init_buffers(void)
{

    memset(e_ref, 0, sizeof(e_ref));
    memset(sobel_fifo, 0, sizeof(sobel_fifo));
    memset(row_fifo, 0, sizeof(row_fifo));
    /* 其他状态变量在声明时已初始化为0 */
    frame_m00 = 0u;
    frame_m10 = 0u;
    frame_m01 = 0u;
    frame_edge_count = 0u;
    frame_id = 0u;
    row_seq = 0u;
    prev_xc_q4 = 0u;
    prev_yc_q4 = 0u;
    prev_meta_valid = 0u;
}

/**
 * @brief  计算输入数据的 CRC-16-CCITT 校验和 (X.25)。
 * @note   作用：为数据包生成校验码，确保传输完整性。支持对两块不连续内存进行连续计算。
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
                crc = (crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1;
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
 *         位置：在 fused_row_sq() 中先执行，用于后续 Threshold 阶段复用。
 */
static void __attribute__((noinline))  __not_in_flash_func(threshold_row_sq)(const uint8_t *restrict r0, const uint8_t *restrict r1, const uint8_t *restrict r2,
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

static void __not_in_flash_func(fused_row_sq)(const uint8_t *restrict r0, const uint8_t *restrict r1, const uint8_t *restrict r2,
                  uint32_t *restrict sobel_pairs, int width)
{
    threshold_row_sq(r0, r1, r2, sobel_pairs, width);   
}

/**
 * @brief  通过异或（XOR）操作计算两行之间的差异，并累加图像矩。
 * @note   作用：通过比较当前帧与参考帧的差异，快速计算出运动区域的面积和质心相关的矩。
 *         位置：在 process_frame_row() 中被调用，用于运动目标检测。
 */
void xor_row_moments(const uint8_t *new_row, const uint8_t *old_row,
                     uint32_t y, uint32_t *m00, uint32_t *m10, uint32_t *m01)
{
    
    const uint32_t *a = (const uint32_t *)new_row;
    const uint32_t *b = (const uint32_t *)old_row;

    for (int wi = 0; wi < (int)(ROW_BYTES / 4u); ++wi) {
        uint32_t m = a[wi] ^ b[wi];
        if (!m) {
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
void packet_generator(const uint8_t *row_bits, uint32_t row_idx, uint32_t frame_id_in, bool overflow, bool final_line,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, plt_row_trailer_t *trailer)
{
    
    header->sync0 = 0xA5A5u;
    header->sync1 = 0x5A5Au;
    header->frame_id = (uint16_t)frame_id_in;
    header->row_idx = (uint16_t)row_idx;
    header->row_flags = 0u;
    if (overflow) {
        header->row_flags |= PKT_ROW_FLAG_OVERFLOW;  /* 仅保留行级溢出标志 */
    }
    if (final_line) {
        header->row_flags |= PKT_ROW_FLAG_FINAL_LINE; /* 仅保留帧尾标志 */
    }
    header->payload_len = (uint8_t)ROW_BYTES;
    header->row_seq = row_seq++;
    memset(header->reserved, 0, sizeof(header->reserved));

    memcpy(payload->payload, row_bits, ROW_BYTES);
    trailer->pad[0] = 0u;
    trailer->pad[1] = 0u;

    trailer->m00 = frame_m00;
    if (frame_m00 > 0u) {
        uint16_t xc_q4 = (uint16_t)(((uint64_t)frame_m10 * 16u + (frame_m00 / 2u)) / frame_m00);
        uint16_t yc_q4 = (uint16_t)(((uint64_t)frame_m01 * 16u + (frame_m00 / 2u)) / frame_m00);
        trailer->xc_q4 = xc_q4;
        trailer->yc_q4 = yc_q4;

        if (prev_meta_valid != 0u) {
            trailer->vx_q8 = (int16_t)(((int32_t)xc_q4 - (int32_t)prev_xc_q4) << 4);
            trailer->vy_q8 = (int16_t)(((int32_t)yc_q4 - (int32_t)prev_yc_q4) << 4);
        } else {
            trailer->vx_q8 = 0;
            trailer->vy_q8 = 0;
            prev_meta_valid = 1u;
        }

        prev_xc_q4 = xc_q4;
        prev_yc_q4 = yc_q4;
    } else {
        trailer->xc_q4 = 0u;
        trailer->yc_q4 = 0u;
        trailer->vx_q8 = 0;
        trailer->vy_q8 = 0;
    }
    trailer->crc16 = crc16_ccitt(header, sizeof(*header), 
                                payload, sizeof(*payload), 
                                trailer, sizeof(*trailer) - 2);
    
}

/**
 * @brief  处理一帧中的单行数据，是行级处理流程的入口。
 * @note   作用：协调调用边缘检测和矩计算，完成对单行数据的核心处理。
 *         位置：在 main() 主循环中被调用，是 CPU 侧图像处理的起点。
 *         效果：处理后的二值化行数据存入 `row_fifo`，参考帧 `e_ref` 被更新，帧级矩和边缘计数被累加。
 */
void process_frame_row(const uint8_t *r0, const uint8_t *r1, const uint8_t *r2, uint32_t row_idx)
{

    /* 增加 sobel_fifo 溢出保护：等待消费者 Core 1 释放空间 */
    /* (row_idx - sobel_fifo_consumed_count) 是已生产但未消费的行数 */
    while ((row_idx - sobel_fifo_consumed_count) >= ROW_FIFO_DEPTH) {
        // FIFO已满，Core0在此等待Core1消费。
        // tight_loop_contents() 或 sleep_us(1) 可避免总线锁死。
        tight_loop_contents();
    }
    uint32_t *sobel_pairs = sobel_fifo[row_idx % ROW_FIFO_DEPTH];

    fused_row_sq(r0, r1, r2, sobel_pairs, 640);
    /* 内存屏障，确保对 sobel_fifo 的写入在后续跨核通知前完成 */
    __asm volatile("dmb sy" ::: "memory");

    
}

/**
 * @brief  在一帧处理完成后，根据总边缘像素数自适应更新阈值。
 * @note   作用：通过动态调整阈值 T_sq，使边缘检测算法能适应不同光照和场景复杂度，保持边缘点数量大致稳定。
 *         位置：在 main() 主循环中，每处理完一帧的最后一行时调用。
 */
void update_threshold(void)
{
    
    if (frame_edge_count > (target * 6u) / 5u) {
        T_sq = (T_sq * 12) / 11;
    } else if (frame_edge_count < (target * 4u) / 5u) {
        T_sq = (T_sq * 11) / 12;
    }

    if (T_sq < 400) T_sq = 400;       // T_SQ_MIN
    if (T_sq > 200000) T_sq = 200000; // T_SQ_MAX

    frame_edge_count = 0u;
    frame_m00 = 0u;
    frame_m10 = 0u;
    frame_m01 = 0u;
}

/**
 * @brief  获取指定行的已处理（二值化）数据。
 * @note   作用：为发送核心（core1）提供访问已处理行数据的接口。
 */
const uint8_t *image_get_row_bits(uint32_t row_idx)
{
    return row_fifo[row_idx % ROW_FIFO_DEPTH];
}

/**
 * @brief  (Core 1) 执行XOR和参考帧更新
 */
void image_core1_process_row(uint32_t abs_row_idx, uint32_t frame_row_idx, bool is_final_line)
{
    const uint32_t *sobel_pairs = sobel_fifo[abs_row_idx % ROW_FIFO_DEPTH];
    uint8_t *bits = row_fifo[abs_row_idx % ROW_FIFO_DEPTH];
    uint32_t edge_count = 0u;

    filter_pack_row_bits(sobel_pairs, bits, T_sq, 640, &edge_count);
    frame_edge_count += edge_count;

    xor_row_moments(bits, e_ref[frame_row_idx], abs_row_idx, &frame_m00, &frame_m10, &frame_m01);
    memcpy(e_ref[frame_row_idx], bits, ROW_BYTES);

    /* 推进消费计数器，通知 Core 0 该行对应的 sobel_fifo 槽位可以被重用 */
    sobel_fifo_consumed_count = abs_row_idx + 1;

    if (is_final_line) {
        update_threshold();
    }
}
