#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/structs/clocks.h"

#include "../header/timer.h"


static const uint32_t camera_core_hz = 144u * 1000u * 1000u;
static const uint32_t camera_usb_adc_hz = 48u * 1000u * 1000u;
static const uint32_t camera_ov5640_xclk_gpio = 21u;

void timer_config(void)
{
    /*
     * RP2354 时钟初始化 —— 最小三步骤 + 外设级分频
     *
     *
     *   150 MHz 是 RP2354 数据手册给出的 clk_sys 硬上限，再往上推不保证可靠。
     */

    /* 关 resus，避免初始化过程被强制切回 clk_ref */
    clocks_hw->resus.ctrl = 0;

    /* ── 步骤 1：启动 XOSC（12 MHz） ─────────────────────────── */
    xosc_init();

    /* 把 clk_sys / clk_ref 的 glitchless mux 切回 src=0，
       下一步动 aux mux 时才不会带毛刺 */
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1) tight_loop_contents();

    hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x1) tight_loop_contents();

    /* ── 步骤 2：配置 PLL_SYS / PLL_USB ───────────────────────── */
    // void pll_init(PLL pll, uint refdiv, uint vco_freq, uint post_div1, uint post_div2)
    // VCO = 12MHz × FBDIV(72) = 864MHz  (在 750~1600MHz 合法区间内)
    // out = 864MHz / (POSTDIV1=6 × POSTDIV2=1) = 144MHz，精确无余数
    pll_init(pll_sys, 1, 864 * MHZ, 6, 1);
    // PLL_USB 用官方验证过的标准默认值即可，不需要自己凑
    // VCO = 12MHz × FBDIV(100) = 1200MHz / (5×5) = 48MHz
    pll_init(pll_usb, 1, 1200 * MHZ, 5, 5);
    /* ── 步骤 3：clk_ref ← XOSC，clk_sys ← PLL_SYS ────────────── */
    clock_configure_undivided(clk_ref,
        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
        0,
        XOSC_HZ);
    clock_configure_undivided(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        camera_core_hz);
    /* ── 外设级分频：用每条 clk_xxx 自己的分频器，绝不回去改 PLL ── */

    /* clk_peri ← clk_sys / 1 = 144 MHz （SPI/UART/I2C 主时钟） */
    clock_configure_int_divider(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        camera_core_hz,
        1);

    /* clk_usb ← PLL_USB = 48 MHz */
    clock_configure_undivided(clk_usb,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        camera_usb_adc_hz);

    /* clk_adc ← PLL_USB = 48 MHz */
    clock_configure_undivided(clk_adc,
        0,
        CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        camera_usb_adc_hz);

    /* clk_hstx ← clk_sys / 3 = 48 MHz
     * SDR 单沿模式下 HSTX CSR.CLKDIV=1，故输出时钟 = clk_hstx = 48 MHz，
     * 低于 Artix-7 FPGA 100 MHz 上限，留 ~52 MHz 余量。 */
    clock_configure_int_divider(clk_hstx,
        0,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,
        camera_core_hz,
        3);

    /* clk_gpout0 ← clk_sys / 6 = 24 MHz */
    clock_DCMI_config();
}

void clock_DCMI_config(void)
{
    /*
     * OV5640 XCLK：系统时钟树在 GPIO21 输出 clk_sys / 6 = 24 MHz
     * clock_gpio_init_int_frac16() 一行完成：
     *   - GPIO21 是合法的 GPCLK 输出引脚（定义于 hardware/clocks.h）
     *   - 时钟源：CLK_SYS = 144 MHz
     *   - 分频：div_int = 6，div_frac16 = 0 → 输出 144 / 6 = 24 MHz
     *   - 硬件自动生成，无 CPU 干预，精度最好
     */
    clock_gpio_init_int_frac16(camera_ov5640_xclk_gpio,
        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
        6u,
        0u);

    gpio_set_drive_strength(camera_ov5640_xclk_gpio, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(camera_ov5640_xclk_gpio, GPIO_SLEW_RATE_FAST);
}
