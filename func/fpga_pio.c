/*
 * fpga_pio.c — PIO1 bridge output for the FPGA side
 *
 * Data flow:
 *   line buffer (RGB565, 1600 bytes)
 *     -> DMA (8-bit, read increment)
 *     -> PIO1 TX FIFO
 *     -> fpga_out.pio (8 data pins + clock side-set)
 *     -> FPGA receiver
 *
 * GP20 is driven by the PIO program as a control/window line.
 */

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "fpga_pio.h"
#include "fpga_pio.pio.h"

static PIO fpga_pio = pio1;
static uint fpga_sm = 1u;
static int fpga_dma_chan = -1;
static int fpga_program_offset = -1;
static dma_channel_config fpga_dma_cfg;

static void fpga_pio_hw_init(void)
{
    if (fpga_program_offset >= 0) {
        return;
    }

    fpga_program_offset = pio_add_program(fpga_pio, &fpga_out_program);

    for (uint i = 0u; i < FPGA_DATA_PIN_COUNT; ++i) {
        pio_gpio_init(fpga_pio, FPGA_DATA_PIN_BASE + i);
    }
    pio_gpio_init(fpga_pio, FPGA_CLK_PIN);
#ifdef FPGA_CTRL_PIN
    pio_gpio_init(fpga_pio, FPGA_CTRL_PIN);
#endif

    pio_sm_set_consecutive_pindirs(fpga_pio, fpga_sm, FPGA_DATA_PIN_BASE, FPGA_DATA_PIN_COUNT, true);
    pio_sm_set_consecutive_pindirs(fpga_pio, fpga_sm, FPGA_CLK_PIN, 1u, true);
#ifdef FPGA_CTRL_PIN
    pio_sm_set_consecutive_pindirs(fpga_pio, fpga_sm, FPGA_CTRL_PIN, 1u, false);
#endif

    pio_sm_config c = fpga_out_program_get_default_config((uint)fpga_program_offset);
    sm_config_set_out_pins(&c, FPGA_DATA_PIN_BASE, FPGA_DATA_PIN_COUNT);
    sm_config_set_sideset_pins(&c, FPGA_CLK_PIN);
#ifdef FPGA_CTRL_PIN
    sm_config_set_set_pins(&c, FPGA_CTRL_PIN, 1);
#endif
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_clkdiv(&c, 1.5f); /* Use system clock (clk_sys) as the PIO clock source */

    pio_sm_init(fpga_pio, fpga_sm, (uint)fpga_program_offset, &c);
    pio_sm_clear_fifos(fpga_pio, fpga_sm);
    pio_sm_set_enabled(fpga_pio, fpga_sm, false);
}

void fpga_gpio_init(void)
{
    for (uint pin = FPGA_DATA_PIN_BASE; pin < FPGA_DATA_PIN_BASE + FPGA_DATA_PIN_COUNT; ++pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    gpio_init(FPGA_CLK_PIN);
    gpio_set_dir(FPGA_CLK_PIN, GPIO_OUT);
    gpio_put(FPGA_CLK_PIN, 0);

#ifdef FPGA_CTRL_PIN
    gpio_init(FPGA_CTRL_PIN);
    gpio_set_dir(FPGA_CTRL_PIN, GPIO_IN);
    gpio_put(FPGA_CTRL_PIN, 0);
#endif
}

void fpga_pio_init(void)
{
    fpga_pio_hw_init();
}

void fpga_dma_init(void)
{
    if (fpga_dma_chan >= 0) {
        return;
    }

    fpga_pio_hw_init();

    fpga_dma_chan = dma_claim_unused_channel(true);
    fpga_dma_cfg = dma_channel_get_default_config((uint)fpga_dma_chan);

    channel_config_set_transfer_data_size(&fpga_dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&fpga_dma_cfg, true);
    channel_config_set_write_increment(&fpga_dma_cfg, false);
    channel_config_set_dreq(&fpga_dma_cfg, pio_get_dreq(fpga_pio, fpga_sm, true));
}

bool fpga_dma_busy(void)
{
    return (fpga_dma_chan >= 0) && dma_channel_is_busy((uint)fpga_dma_chan);
}

void fpga_tx_start(const uint8_t *buffer)
{
    if (fpga_dma_chan < 0) {
        fpga_dma_init();
    }

    dma_channel_configure(
        (uint)fpga_dma_chan,
        &fpga_dma_cfg,
        &fpga_pio->txf[fpga_sm],
        buffer,
        FPGA_FRAME_BYTES,
        true);

    pio_sm_set_enabled(fpga_pio, fpga_sm, true);
}

void fpga_tx_stop(void)
{
    if (fpga_dma_chan >= 0) {
        dma_channel_abort((uint)fpga_dma_chan);
    }

    pio_sm_set_enabled(fpga_pio, fpga_sm, false);
    pio_sm_clear_fifos(fpga_pio, fpga_sm);
#ifdef FPGA_CTRL_PIN
    gpio_put(FPGA_CTRL_PIN, 0);
#endif
}
