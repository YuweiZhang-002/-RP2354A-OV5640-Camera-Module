#ifndef _USER_TIMER_H_
#define _USER_TIMER_H_

#include <stdint.h>

/*
 * RP2354 时钟规划（当前配置：clk_sys = 144 MHz）
 *   clk_sys = 144 MHz  （PLL_SYS: VCO 576 MHz / 4 = 144 MHz）
 *   clk_peri = 144 MHz （SPI/UART/I2C 主时钟，跟随 clk_sys）
 *   clk_hstx = 48 MHz  （144/3；HSTX SDR 单沿，输出时钟 = clk_hstx/CLKDIV(=1) = 48 MHz）
 *   clk_usb  = 48 MHz （来自 PLL_USB）
 *   clk_adc  = 48 MHz （来自 PLL_USB）
 *
 *   OV5640 XCLK = 24 MHz
 *   ├─ 硬件生成：GPCLK 在 GPIO21 输出（clk_sys / 4）
 *   └─ 调用：timer_config() 已内置 clock_DCMI_config()
 *
 * DMA 吞吐能力（相对于当前链路）：
 *   PIO capture 侧：24 MHz × 8 bit = 192 Mbit/s  ← 缓冲区吸收峰值
 *   HSTX output 侧：96 MHz / 2 / 2 = 24 Mbit/s 双 lane  ← 系统时钟 96 MHz 充足
 */
void timer_config(void);
void clock_DCMI_config(void);

#endif /* _USER_TIMER_H_ */
