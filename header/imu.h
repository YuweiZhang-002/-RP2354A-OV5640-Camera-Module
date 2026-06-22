#ifndef IMU_H
#define IMU_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/spi.h"
#include "icm45686.h"

/* ────────────────────────────────────────────────────────────────────────────
 *  GPIO 层
 *  - UART0 TX / SPI0 MOSI-MISO-SCK-CS
 *  当前状态：由 icm45686_spi_init() / icm45686_uart_init() 统一复用
 * ──────────────────────────────────────────────────────────────────────────*/

/* ── IMU → FPGA 数据链：UART(数据) + PWM 参考时钟(FPGA 矫正用) ──
 * UART 数据走 GP0；另给 FPGA 一路与 UART 同源(clk_sys=144MHz)的参考时钟，
 * 频率 = 16×波特率，FPGA 把它÷16 得到与数据频率锁定的位时钟 → 无波特率漂移。
 * 选 1Mbps：144M/(16×1M)=9 整除，UART 与 16MHz 参考时钟都精确。
 */
#define ICM45686_UART_INST    uart0
#define ICM45686_UART_TX_PIN  0u
#define ICM45686_UART_BAUD    1000000u

/*
 * RP2040/RP2350 的 SPI 引脚功能是固定 pinmux，不能随意命名：
 *   GP2 = SPI0 SCK, GP3 = SPI0 TX(MOSI), GP4 = SPI0 RX(MISO)。
 * CS 用普通 GPIO 软件控制，可放在任意空闲脚。
 */
#define ICM45686_SPI_INST  spi0
#define ICM45686_SPI_SCK   2u   /* GP2 = SPI0 SCK */
#define ICM45686_SPI_MOSI  3u   /* GP3 = SPI0 TX  */
#define ICM45686_SPI_MISO  4u   /* GP4 = SPI0 RX  */
#define ICM45686_SPI_CS    1u   /* 软件片选（普通 GPIO） */

#define ICM45686_SPI_MAX_TRANSFER  32u
#define ICM45686_FIFO_MAX_BURST    32u   /* FIFO 路径暂缓，宏保留供后续扩展 */

/* ────────────────────────────────────────────────────────────────────────────
 *  公共接口（方案 A：寄存器直读 + UART DMA 异步外发）
 *  - 采样由上层在帧边界(frame_ready)触发，无需 IMU 硬件 DRDY 引脚
 *  - icm45686_stream_sample() 非阻塞：直读 14B 后用 DMA 经 UART 发出，立即返回
 *  - icm45686_uart_tx_busy() 查询上一帧是否仍在搬运
 *  FIFO 路径整体暂缓，实现保留在 imu.c 末尾的 #if 0 块中
 * ──────────────────────────────────────────────────────────────────────────*/

int32_t icm45686_init(void);                 /* 0=成功；-1=WHO_AM_I 校验失败 */
int32_t icm45686_verify_id(void);            /* 0=ID 正确；-1=读失败或不匹配 */
void    icm45686_spi_init(void);
uint8_t icm45686_getdata(uint8_t *data);     /* 直读 14B(accel+gyro+temp)，阻塞 SPI */
uint8_t icm45686_stream_sample(void);        /* 采一帧并经 UART DMA 异步外发（非阻塞） */
bool    icm45686_uart_tx_busy(void);         /* UART TX DMA 是否仍在搬运 */
uint8_t icm45686_read_regs(uint8_t start_reg, uint8_t *data, size_t length);
uint8_t icm45686_write_reg(uint8_t reg, const uint8_t *data, size_t length);

#endif /* IMU_H */
