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

#include "pico/stdlib.h"
#include "timer.h"
#include "header/ov5640.h"
#include "cam_pio.h"
#include "ov5640_set.h"
#include "fpga_pio.h"


/* 系统循环，用于强制终止出现ERROR的程序 */
void sys_loop(void)
{
    while(1);
}


/* */
volatile bool work = false;

/*
 * 注意：`FPGA_CTRL_PIN` 在设计语义上是一个由 FPGA/板上逻辑提供的
 * 输入信号，用以指示 OV5640 与采集/发送链的工作状态（例如：进入
 * 采集模式或停止）。该信号在当前开发阶段暂不使用——我们在
 * header/fpga_pio.h 中将其宏注释掉以避免误用。未来若需重新启用，
 * 可在此处添加基于该输入的启动/停止逻辑或其它策略。
 */

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

    /* 唤醒窗口内的前 3 帧不参与后续处理 */
    cam_discard_next_frames(3u);

    /* 5. 传感器参数配置：若芯片 ID 或寄存器写入失败则直接停机 */
    status = OV5640_Init(BMP_1280x720, OV5640_Y8, OV5640_Polarity_4);
    if (status != 0) {
        sys_loop();
    }

    /* 当前是否有一行正在经 PIO1 发送（在飞） */
    bool fpga_line_inflight = false;
    /* 6. 启动连续采集（DMA+PIO0） */
    ov5640_start_capture(); 

    while (true) {
        /* —— 发送链：与采集链解耦，仅通过 acquire/release 单点交接 —— */

        /* 1) 上一行发送完成 → 归还缓冲，供采集复用 */
        if (fpga_line_inflight && !fpga_dma_busy()) {
            cam_release_line();
            fpga_line_inflight = false;
        }

        /* 2) 空闲且有就绪行 → 启动一次 PIO1 发送 */
        if (!fpga_line_inflight) {
            uint8_t *line = cam_acquire_line();
            if (line != NULL) {
                fpga_tx_start(line);
                fpga_line_inflight = true;
            }
        }

        /*
         * IMU 采样链已在当前设计中停用（imu.c 保留但不被 main 调用）。
         * 若未来需要按帧边界采样，请在此处恢复调用。
         */
    }
}
