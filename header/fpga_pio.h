#ifndef FPGA_PIO_H
#define FPGA_PIO_H

#include <stdbool.h>
#include <stdint.h>

/* PIO1 bridge output: 8 contiguous data pins + clock + control output */
#define FPGA_DATA_PIN_BASE   0u
#define FPGA_DATA_PIN_COUNT  8u
#define FPGA_CLK_PIN         8u
/*
 * FPGA control input (外部输入):
 *  - 语义上这是一个外部来自 FPGA 或板上逻辑的输入引脚，
 *    用于指示 OV5640 / cam_pio / hstx_pio 的工作状态。
 *  - 在当前开发阶段该信号不被本固件主动驱动，故在头文件中
 *    注释掉宏定义，避免将其作为输出控制用到 PIO/CPU 代码里。
 *  - 若将来重新启用，请在这里恢复宏定义并在 `main.c` 中添加
 *    适当的输入配置与处理逻辑。
 */
// #define FPGA_CTRL_PIN        9u

/* One camera line: 1280px * Y8(1 byte) */
#define FPGA_FRAME_BYTES     1280u
#define FPGA_FRAME_WORDS     (FPGA_FRAME_BYTES / 4u)

void fpga_gpio_init(void);
void fpga_pio_init(void);
void fpga_dma_init(void);
void fpga_tx_start(const uint8_t *buffer);
bool fpga_dma_busy(void);
void fpga_tx_stop(void);

#endif /* FPGA_PIO_H */