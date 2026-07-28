/*
 * fpga_pio.c — PIO1 + DMA 数据包发送模块
 *
 * Data flow:
 *   Core 1 组装好的数据包 (pkt_buf)
 *     -> DMA (8-bit, 读地址自增)
 *     -> PIO1 SM0 TX FIFO
 *     -> packet_tx.pio (8-bit 数据 + 1-bit STROBE 时钟)
 *     -> 外部接收端 (FPGA)
 *
 */

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"

#include "fpga_pio.h"
#include "fpga_pio.pio.h"

static PIO fpga_pio = pio1;         // 使用独立的 PIO1
static uint fpga_sm = 0u;           // 使用 PIO1 的 SM0
static uint fpga_program_offset = 0u;
static int fpga_dma_chan = -1;      // DMA 通道
static dma_channel_config fpga_dma_cfg; // DMA 配置
volatile bool fpga_tx_busy = false; // 发送忙标志，由主循环轮询

static void fpga_dma_irq_handler(void);

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

    gpio_init(FPGA_HREF_PIN);
    /* 先写低电平锁存值再打开输出，避免接管GPIO9时产生窄高脉冲。 */
    gpio_put(FPGA_HREF_PIN, 0);
    gpio_set_dir(FPGA_HREF_PIN, GPIO_OUT);
}

void fpga_pio_init(void)
{
    fpga_program_offset = pio_add_program(fpga_pio, &packet_tx_program);

    pio_sm_config c = packet_tx_program_get_default_config(fpga_program_offset);
    sm_config_set_out_pins(&c, FPGA_DATA_PIN_BASE, 8);
    sm_config_set_sideset_pins(&c, FPGA_CLK_PIN);
    sm_config_set_out_shift(&c, false, true, 8);  /* MSB先出, autopull阈值8bit */
    sm_config_set_clkdiv(&c, 3.0f);                /* 144MHz/3.0=48MHz SM时钟 -> 4周期/字节 = 12MHz */

    for (uint i = FPGA_DATA_PIN_BASE; i < FPGA_DATA_PIN_BASE + 8; i++) {
        pio_gpio_init(fpga_pio, i);
    }
    pio_gpio_init(fpga_pio, FPGA_CLK_PIN);

    pio_sm_set_consecutive_pindirs(fpga_pio, fpga_sm, FPGA_DATA_PIN_BASE, 8, true);
    pio_sm_set_consecutive_pindirs(fpga_pio, fpga_sm, FPGA_CLK_PIN, 1, true);

    pio_sm_init(fpga_pio, fpga_sm, fpga_program_offset, &c);
    pio_sm_set_enabled(fpga_pio, fpga_sm, true);
}

void fpga_dma_init(void)
{
    fpga_dma_chan = dma_claim_unused_channel(true);
    fpga_dma_cfg = dma_channel_get_default_config((uint)fpga_dma_chan);

    channel_config_set_transfer_data_size(&fpga_dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&fpga_dma_cfg, true);
    channel_config_set_write_increment(&fpga_dma_cfg, false);
    channel_config_set_dreq(&fpga_dma_cfg, pio_get_dreq(fpga_pio, fpga_sm, true));

    /* 行写满 → DMA 完成中断（DMA_IRQ_1），中断里清 busy 标志 */
    dma_channel_set_irq1_enabled((uint)fpga_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, fpga_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    // 预配置DMA，但不启动
    dma_channel_configure((uint)fpga_dma_chan, &fpga_dma_cfg, &fpga_pio->txf[fpga_sm], NULL, 0, false);
}


static void fpga_dma_irq_handler(void)
{   
    if (!dma_channel_get_irq1_status((uint)fpga_dma_chan)) {
        return;
    }

    dma_channel_acknowledge_irq1((uint)fpga_dma_chan);

    /*
     * DMA完成只表示最后一个字节已经写入PIO TX FIFO。清除旧TXSTALL后等待
     * 状态机再次因FIFO空而停顿，才能确认最后一个GPIO8时钟已经结束。
     */
    const uint32_t txstall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + fpga_sm);
    fpga_pio->fdebug = txstall_mask;
    while ((fpga_pio->fdebug & txstall_mask) == 0u) {
        tight_loop_contents();
    }

    gpio_put(FPGA_HREF_PIN, 0);
    __dmb();
    /* 最后清busy，避免Core1启动下一包后又被本IRQ把GPIO9拉低。 */
    fpga_tx_busy = false;
}

bool fpga_dma_busy(void)
{
    return fpga_tx_busy || ((fpga_dma_chan >= 0) && dma_channel_is_busy((uint)fpga_dma_chan));
}

void fpga_tx_start(const void *pkt, size_t len)
{
    fpga_tx_busy = true;
    pio_sm_set_enabled(fpga_pio, fpga_sm, true);
    gpio_put(FPGA_HREF_PIN, 1);
    dma_channel_set_read_addr((uint)fpga_dma_chan, pkt, false);
    dma_channel_set_trans_count((uint)fpga_dma_chan, len, true);
}

void fpga_tx_stop(void)
{
    /* 停止路径无论DMA处于何种状态都必须先撤销外部HREF。 */
    gpio_put(FPGA_HREF_PIN, 0);

    if (fpga_dma_chan >= 0) {
        dma_channel_abort((uint)fpga_dma_chan);
        if (dma_channel_get_irq1_status((uint)fpga_dma_chan)) {
            dma_channel_acknowledge_irq1((uint)fpga_dma_chan);
        }
    }

    pio_sm_set_enabled(fpga_pio, fpga_sm, false);
    pio_sm_clear_fifos(fpga_pio, fpga_sm);
    pio_sm_restart(fpga_pio, fpga_sm);
    pio_sm_exec(fpga_pio, fpga_sm, pio_encode_jmp(fpga_program_offset));
    fpga_tx_busy = false;
}
