#include "image_process.h"    /* 图像处理阶段对外声明：行处理、组包、CRC、阈值 */

#include <string.h>            /* memset / memcpy，用于缓冲清零与行数据复制 */

#include "pico.h"             /* Pico 统一入口：提供 __not_in_flash_func 与 CMSIS 内建 */
#include "pico/stdlib.h"      /* Pico 基础类型与 __SMULBB / __SMLABB 等内建支持 */
#include "cam_pio.h"          /* CAPTURE_BYTES / CAPTURE_LINES / 摄像头行参数 */

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

/* 层5 & 6 核1私有: 组包序列与位置状态 */
static uint16_t frame_id = 0, row_seq = 0;

static void update_threshold(void);

static inline uint16_t wire_u16(uint16_t value)
{
    return __builtin_bswap16(value);
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
    /* 其他状态变量在声明时已初始化为0 */
    frame_edge_count = 0u;
    frame_id = 0u;
    row_seq = 0u;
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
void packet_generator(const uint8_t *row_bits, uint32_t row_idx, uint32_t frame_id_in, bool overflow,
                      bool first_line, bool final_line,
                      pkt_row_header_t *header, pkt_row_payload_t *payload, plt_row_trailer_t *trailer)
{
    
    header->sync0 = wire_u16(0xA5A0u);
    header->sync1 = wire_u16(0x5A50u);
    header->cam_id = 0u;  /* 摄像头ID，当前版本固定为0 */
    header->frame_id = wire_u16((uint16_t)frame_id_in);
    header->row_idx = wire_u16((uint16_t)row_idx);
    header->row_flags = 0u;
    if (overflow) {
        header->row_flags |= PKT_ROW_FLAG_OVERFLOW;  /* 保留行级溢出标志 */
    }
    if (final_line) {
        header->row_flags |= PKT_ROW_FLAG_FINAL_LINE; /* 保留帧尾标志 */
    } else if (first_line) {
        header->row_flags |= PKT_ROW_FLAG_FIRST_LINE; /* 保留帧首标志 */
    }
    header->payload_len = (uint8_t)ROW_BYTES;
    header->row_seq = wire_u16(row_seq++);
    memset(header->reserved, 0, sizeof(header->reserved));

    memcpy(payload->payload, row_bits, ROW_BYTES);
    memset(trailer->pad, 0, sizeof(trailer->pad));

    /* XOR/质心已移除：原运动矩字段改为包尾同步图案，供FPGA定位包尾。 */
    for (uint32_t i = 0u; i < PKT_TRAILER_SYNC_WORDS; ++i) {
        trailer->sync[i] = wire_u16(PKT_TRAILER_SYNC);
    }
    trailer->crc16 = wire_u16(0xFFFFu);  /* CRC16 校验码暂时置为65535，FPGA端计算 */

    if (final_line) {
        update_threshold();
    }
    
}

/**
 * @brief  处理一帧中的单行数据，是行级处理流程的入口。
 * @note   作用：协调调用边缘检测和矩计算，完成对单行数据的核心处理。
 *         位置：在 main() 主循环中被调用，是 CPU 侧图像处理的起点。
 *         效果：Sobel 梯度中间结果存入 `sobel_fifo`，供 Core 1 做阈值化与打包。
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

/**
 * @brief  获取指定行的已处理（二值化）数据。
 * @note   作用：为发送核心（core1）提供访问已处理行数据的接口。
 */
const uint8_t *image_get_row_bits(uint32_t row_idx)
{
    return row_fifo[row_idx % ROW_FIFO_DEPTH];
}

/**
 * @brief  (Core 1) 阈值化 + 位打包，并推进 sobel_fifo 消费计数
 */
void image_core1_process_row(uint32_t abs_row_idx, uint32_t frame_row_idx)
{
    uint8_t *bits = row_fifo[abs_row_idx % ROW_FIFO_DEPTH];

    if ((frame_row_idx == 0u) || (frame_row_idx ==  1u)) {
        /* 固定边界行不读取 sobel_fifo；80-byte payload 置 0x00。 */
        memset(bits, 0, ROW_BYTES);
    } else {
        const uint32_t *sobel_pairs = sobel_fifo[abs_row_idx % ROW_FIFO_DEPTH];
        uint32_t edge_count = 0u;

        filter_pack_row_bits(sobel_pairs, bits, T_sq, 640, &edge_count);
        frame_edge_count += edge_count;
    }

    /* 推进消费计数器，通知 Core 0 该行对应的 sobel_fifo 槽位可以被重用 */
    sobel_fifo_consumed_count = abs_row_idx + 1;
}
