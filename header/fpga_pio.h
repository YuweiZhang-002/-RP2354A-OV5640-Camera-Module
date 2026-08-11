#ifndef FPGA_PIO_H
#define FPGA_PIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* PIO1 packet output: GPIO0-7 data, GPIO8 PCLK, GPIO9 packet HREF. */
#define FPGA_DATA_PIN_BASE   0u
#define FPGA_DATA_PIN_COUNT  8u
#define FPGA_CLK_PIN         8u   /* PIO1 side-set: 12 MHz byte clock */
#define FPGA_HREF_PIN        9u   /* SIO output: high for one 128-byte packet */

extern volatile bool fpga_tx_busy;

void fpga_gpio_init(void);
void fpga_pio_init(void);
void fpga_dma_init(void);
/* 必须由实际发送数据的核（Core1）调用，安装并使能 DMA_IRQ_1。 */
void fpga_dma_irq_init_this_core(void);
void fpga_tx_start(const void *buffer, size_t length);
bool fpga_dma_busy(void);
void fpga_tx_stop(void);

#endif /* FPGA_PIO_H */
