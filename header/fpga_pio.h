#ifndef FPGA_PIO_H
#define FPGA_PIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* PIO1 packet output: 8 contiguous data pins + strobe */
#define FPGA_DATA_PIN_BASE   0u
#define FPGA_DATA_PIN_COUNT  8u
#define FPGA_CLK_PIN         8u

void fpga_gpio_init(void);
void fpga_pio_init(void);
void fpga_dma_init(void);
void fpga_tx_start(const void *buffer, size_t length);
bool fpga_dma_busy(void);
void fpga_tx_stop(void);

#endif /* FPGA_PIO_H */