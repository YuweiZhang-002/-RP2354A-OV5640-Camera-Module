/*
 * main.c  —  RP2354 PIO 摄像头采集最小 Demo
 *
 * 完整数据流（单向闭合）：
 *   GPIO 0-7/8/10/9 → PIO 状态机 → RX FIFO → DMA → 行级三缓冲环
 *     → 主循环 cam_acquire_line() → PIO1 发送 → cam_release_line()
 *
 * 三条链各管各的，主循环只做单点编排：
 *   - 采集链：PIO+DMA 在完成中断里自维持地逐行采集（cam_pio.c）
 *   - 发送链：主循环把就绪行交给 PIO1 发送，互不在同一中断路径（fpga_pio.c）
 *   - IMU 链：按 VSYNC 帧边界(frame_ready)自采样、FIFO、串口外发（imu.c）
 *
 * 初始化职责分离：
 *   cam_gpio_init()     — 引脚方向/上拉（cam_pio.c）
 *   cam_pio_init()      — PIO 程序加载 + 行计数器预装（cam_pio.c）
 *   cam_dma_init()      — DMA 通道申请 + DREQ + 完成中断 + VSYNC 中断（cam_pio.c）
 *   ov5640_i2c_init()   — SCCB/I2C 总线初始化（ov5640.c）
 *   ov5640_pin_init()   — 上电时序（ov5640.c）
 *   OV5640_Init()       — 寄存器配置（ov5640_set.c）
 */

#include "pico/multicore.h"   /* 双核 FIFO / core1 启动，用于 core0-core1 行号交接 */
#include "pico/stdlib.h"      /* Pico 基础库：GPIO、sleep、stdio 初始化等通用接口 */
#include "timer.h"            /* 项目自定义定时器与系统节拍初始化 */
#include "header/ov5640.h"    /* OV5640 SCCB/I2C 读写、上电与启动采集接口 */
#include "cam_pio.h"          /* PIO0 + DMA 摄像头采集链路与行缓冲状态 */
#include "ov5640_set.h"       /* OV5640 分辨率、格式、极性等配置入口 */
#include "fpga_pio.h"         /* PIO1 + DMA 发送链路，负责把组包数据发出 */
#include "image_process.h"    /* Sobel、XOR 矩、组包、阈值更新等图像处理接口 */

static uint8_t packet_buf[PACKET_BYTES];


/* 系统循环，用于强制终止出现ERROR的程序 */
void sys_loop(void)
{
    while(1);
}

static void fpga_pio_core1_entry(void);

int main(void)
{
    int status = 0;

    /* 初始化 USB/UART 串口（用于调试打印） */
    timer_config();
    stdio_init_all();

    /* 1. GPIO 复用与上拉/下拉初始化 */
    cam_gpio_init();
    fpga_gpio_init();
    debug_gpio_init();

    /* 2. PIO 程序和 DMA 初始化 */
    cam_pio_init();
    fpga_pio_init();

    /* 3. DMA 通道申请与 DREQ 绑定，但不立即触发 */
    cam_dma_init();
    fpga_dma_init();

    /* 4. OV5640 SCCB/I2C 与上电时序 */
    ov5640_i2c_init();
    ov5640_pin_init();

    system_init_buffers();

    /* 唤醒窗口内的前 3 帧不参与后续处理 */
    cam_discard_next_frames(3u);

    /* 5. 传感器参数配置：若芯片 ID 或寄存器写入失败则直接停机 */
    status = OV5640_Init(BMP_640x480, OV5640_Y8, OV5640_Polarity_4);
    if (status != 0) {
        sys_loop();
    }

    multicore_launch_core1(fpga_pio_core1_entry); /* 启动 PIO1 发送核心 */

    /* 6. 启动连续采集（DMA+PIO0） */
    ov5640_start_capture(); 

    /*
     * ===================================================================
     *  主循环 (Core 0): 图像处理与任务分发
     * ===================================================================
     *
     * 1. 等待标志 `cam_filter_ready`:
     *    - 此标志由 `cam_pio.c` 中的 DMA 完成中断置位，表示新的一行原始数据已采集完成，
     *      并且形成了可供 3x3 卷积计算的 "上-中-下" 三行数据。
     *
     * 2. 执行 `process_frame_row()`:
     *    - 从行级三缓冲环中获取上、中、下三行原始数据。
     *    - 对这三行数据进行 Sobel 边缘检测，生成二值化的边缘行。
     *    - 将新生成的边缘行与参考帧（上一帧的边缘图）进行 XOR，计算运动目标的图像矩。
     *
     * 3. 跨核通信 `multicore_fifo_push_blocking()`:
     *    - 将处理完成的行号（在行 FIFO 中的索引）推送到核间 FIFO，通知 Core 1 可以发送这一行了。
     *
     * 4. 帧尾处理:
     *    - 当一帧的最后一行处理完毕时 (cam_line_count % CAPTURE_LINES == CAPTURE_LINES - 1)，
     *      调用 `update_threshold()` 根据本帧的边缘总数调整下一帧的阈值。
     */
    while (true) {
        if (cam_filter_ready) {
            uint32_t abs_row_idx = cam_line_count; /* 绝对行号：用于队列、丢行和消费计数 */
            uint32_t frame_row_idx = abs_row_idx % CAPTURE_LINES; /* 帧内行号：用于参考帧和包内行号 */
            cam_filter_ready = 0u; /* 清标记，等待下一行采集完成中断 */
            process_frame_row(cam_get_buffer(cam_linem1_count),
                              cam_get_buffer(cam_line0_count),
                              cam_get_buffer(cam_linep1_count),
                              abs_row_idx);
            multicore_fifo_push_blocking(abs_row_idx);

        }

    }
}


/*
 * ===================================================================
 *  发送核心 (Core 1): 数据打包与 PIO 发送
 * ===================================================================
 *
 * 1. 等待数据 `multicore_fifo_pop_blocking()`:
 *    - 阻塞等待，直到 Core 0 向核间 FIFO 推送了新的行号。
 *
 * 2. 获取数据 `image_get_row_bits()`:
 *    - 使用收到的行号，从 `image_process.c` 的行 FIFO 中获取对应的二值化边缘行数据。
 *
 * 3. 打包数据 `packet_generator()`:
 *    - 将行数据、帧号、行号等信息填充到一个标准的数据包结构中，并计算 CRC。
 *
 * 4. 启动发送 `fpga_tx_start()`:
 *    - 将打包好的数据包交给 PIO1 的 DMA 通道，由硬件自动、高速地发送出去，发送期间 Core 1 不被阻塞。
 *
 * 5. Core 1 任务扩展:
 *    - 执行 `image_core1_process_row()`，进行 XOR 运算和参考帧 `e_ref` 的更新。
 *    - 在帧尾发送 `meta` 包。
 */
static void fpga_pio_core1_entry(void)
{
    uint32_t last_overrun_count = 0;

    while (true) {
        uint32_t abs_row_idx = multicore_fifo_pop_blocking();
        uint32_t frame_row_idx = abs_row_idx % CAPTURE_LINES;
        uint32_t frame_id = abs_row_idx / CAPTURE_LINES;
        bool is_final_line = frame_row_idx == (CAPTURE_LINES - 1u);
        bool has_overflow = (cam_overrun_count > last_overrun_count);
        last_overrun_count = cam_overrun_count;

        /* Core 1 先更新参考帧，再把本行的控制位写进 packet header。 */
        image_core1_process_row(abs_row_idx, frame_row_idx, is_final_line);

        const uint8_t *row_bits = image_get_row_bits(abs_row_idx);
        pkt_row_header_t *header = (pkt_row_header_t *)packet_buf;
        pkt_row_payload_t *payload = (pkt_row_payload_t *)(packet_buf + sizeof(pkt_row_header_t));
        plt_row_trailer_t *trailer = (plt_row_trailer_t *)(packet_buf + sizeof(pkt_row_header_t) + sizeof(pkt_row_payload_t));
        
        /* 等待上一次 DMA 传输完成，确保 packet_buf 不会被覆盖 */
        while (fpga_dma_busy()) {
            tight_loop_contents();
        }

        packet_generator(row_bits, frame_row_idx, frame_id, has_overflow, is_final_line, header, payload, trailer);
        fpga_tx_start(packet_buf, sizeof(packet_buf));
    }
}