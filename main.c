/*
 * main.c  —  RP2354 PIO 摄像头采集最小 Demo
 *
 * 完整数据流（单向闭合）：
 *   GPIO 22-29/6/5/7 → PIO 状态机 → RX FIFO → DMA → 行级三缓冲环
 *     → 主循环 cam_acquire_line() → HSTX 发送 → cam_release_line()
 *
 * 三条链各管各的，主循环只做单点编排：
 *   - 采集链：PIO+DMA 在完成中断里自维持地逐行采集（cam_pio.c）
 *   - 发送链：主循环把就绪行交给 HSTX，互不在同一中断路径（hstx.c）
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

#include "pico/stdlib.h"
#include "timer.h"
#include "header/ov5640.h"
#include "cam_pio.h"
#include "ov5640_set.h"
#include "imu.h"
#include "hstx.h"


/* 系统循环，用于强制终止出现ERROR的程序 */
void sys_loop(void)
{
    while(1);
}

/* 初始化FPGA关联引脚 */
void fpga_gpio_init(void) {
    /* FPGA开发板传来1个引脚：GPIO 11，配置为输入并上拉 */
    gpio_init(11);
    gpio_set_dir(11, GPIO_IN);
    gpio_set_pulls(11, true, false);
}

/* 检测FPGA_EN电平机制
 * EN 低有效：GPIO 11 = 0 表示进入工作状态，= 1（含 FPGA 未驱动时的上拉默认）表示关闭。
 * 状态保存在文件全局 is_fpga_en，只在电平跳变时启停，避免重复写寄存器。 */
bool is_fpga_en = false;

void check_fpga_en(void) {
    bool en_level = (gpio_get(11) == 0);   /* 低有效：低=工作 */

    if (!is_fpga_en && en_level) {
        /* 关→开：先唤醒传感器，待 PLL/AEC 稳定后再启动采集，避开首帧垃圾/采集卡死 */
        if (OV5640_SetActivation(OV5640_PowerUp) != OV5640_OK) {
            return;                         /* I2C 失败：保持关闭，下一轮重试 */
        }
        sleep_ms(20);
        cam_capture_start();
        is_fpga_en = true;
    }
    else if (is_fpga_en && !en_level) {
        /* 开→关：先干净停采集（此时 PCLK 仍在，DMA abort 可靠），再休眠传感器 */
        cam_capture_stop();
        OV5640_SetActivation(OV5640_PowerDown);
        is_fpga_en = false;
    }
}


int main(void)
{
    int status = 0;

    /* 初始化 USB/UART 串口（用于调试打印） */
    timer_config();
    stdio_init_all();

    /* 1. GPIO 复用与上拉/下拉初始化 */
    cam_gpio_init();
    hstx_gpio_init();
    /* 补充：添加FPGA开发板传来的引脚初始化 */
    fpga_gpio_init();

    /* 2. PIO 程序和 HSTX 控制器初始化 */
    cam_pio_init();
    hstx_init();

    /* 3. DMA 通道申请与 DREQ 绑定，但不立即触发 */
    cam_dma_init();
    hstx_dma_init();

    /* 4. OV5640 SCCB/I2C 与上电时序 */
    ov5640_i2c_init();
    ov5640_pin_init();

    /* 5. 传感器参数配置：若芯片 ID 或寄存器写入失败则直接停机 */
    status = OV5640_Init(BMP_1280x720, OV5640_RGB565, OV5640_Polarity_4);
    if (status != 0) {
        sys_loop();
    }

    /* 6. IMU 初始化：SPI / UART / UART-TX-DMA；WHO_AM_I 校验失败则停机 */
    status = icm45686_init();
    if (status != 0) {
        sys_loop();
    }


    /* 当前是否有一行正在经 HSTX 发送（在飞） */
    bool hstx_line_inflight = false;

    while (true) {
        /* 验证是否进入工作状态（检测电平） */
        check_fpga_en();

        /* —— 发送链：与采集链解耦，仅通过 acquire/release 单点交接 —— */

        /* 1) 上一行发送完成 → 归还缓冲，供采集复用 */
        if (hstx_line_inflight && !hstx_dma_busy()) {
            cam_release_line();
            hstx_line_inflight = false;
        }

        /* 2) 空闲且有就绪行 → 启动一次 HSTX 发送 */
        if (!hstx_line_inflight) {
            uint8_t *line = cam_acquire_line();
            if (line != NULL) {
                hstx_tx_start(line);
                hstx_line_inflight = true;
            }
        }

        /* —— IMU 链：按帧边界独立采样并外发，不阻塞发送链 —— */
        if (frame_ready == 1u) {
            frame_ready = 0u;
            /* 直读一帧并经 UART DMA 异步外发；非阻塞，发送在后台进行 */
            (void)icm45686_stream_sample();
        }
    }
}
