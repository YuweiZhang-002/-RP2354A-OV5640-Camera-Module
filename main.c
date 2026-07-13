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

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "timer.h"
#include "header/ov5640.h"
#include "cam_pio.h"
#include "ov5640_set.h"
#include "fpga_pio.h"
#include "image_process.h"

#define PKT_PAYLOAD_SIZE  100u


uint8_t packet_payload[PKT_PAYLOAD_SIZE];
static uint8_t packet_buf[sizeof(pkt_row_header_t) + sizeof(pkt_row_payload_t) + sizeof(pkt_row_trailer_t)];


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

    while (true) {
        if (cam_filter_ready) {
            uint32_t row_idx = cam_line0_count;
            uint32_t frame_idx = cam_line_count / CAPTURE_LINES;
            cam_filter_ready = 0u; /* 清标记，等待下一行采集完成中断 */
            process_frame_row(cam_get_buffer(cam_linem1_count),
                              cam_get_buffer(cam_line0_count),
                              cam_get_buffer(cam_linep1_count),
                              row_idx);
            multicore_fifo_push_blocking(row_idx);

            if ((cam_line_count % CAPTURE_LINES) == (CAPTURE_LINES - 1u)) {
                update_threshold();
                packet_send_meta(0u, 0u, 0u);
                (void)frame_idx;
            }
        }

    }
}


/* ================= 核1: 帧时间域(软实时) ================= */
static void fpga_pio_core1_entry(void)
{
    for (;;) {
        uint32_t cur = multicore_fifo_pop_blocking();
        const uint8_t *row_bits = image_get_row_bits(cur);
        pkt_row_header_t *header = (pkt_row_header_t *)packet_buf;
        pkt_row_payload_t *payload = (pkt_row_payload_t *)(packet_buf + sizeof(pkt_row_header_t));
        pkt_row_trailer_t *trailer = (pkt_row_trailer_t *)(packet_buf + sizeof(pkt_row_header_t) + sizeof(pkt_row_payload_t));

        packet_generator(row_bits, cur, (uint32_t)(cam_line_count / CAPTURE_LINES), header, payload, trailer);
        fpga_tx_start(packet_buf, sizeof(packet_buf));
    }
}