/*
 * ov5640.c  —  OV5640 摄像头传感器 SCCB/I2C 驱动（最小可运行版）
 *
 * 职责范围：
 *   1. 初始化 Pico 的 I2C0 外设作为 SCCB 主机
 *   2. 实现 16-bit 地址 + 8-bit 数据的寄存器读写
 *   3. OV5640 上电复位及 RGB565 格式配置
 *
 * 依赖：
 *   - pico/stdlib.h        (sleep_ms)
 *   - hardware/i2c.h       (i2c_init / i2c_write_blocking / i2c_read_blocking)
 *   - hardware/gpio.h      (gpio_set_function / gpio_pull_up)
 *   - cam_pio.h            (cam_capture_start)
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "ov5640.h"
#include "cam_pio.h"

/* ────────────────────────────────────────────────────────────────────────────
 *  I2C / SCCB 初始化
 *
 *  SCCB（Serial Camera Control Bus）协议由 OmniVision 定义，与 I2C 高度兼容：
 *    - 起始/停止条件、时钟拉伸行为与标准 I2C 一致
 *    - 最大时钟频率 400 kHz（本 demo 使用 100 kHz 保证稳定性）
 *    - 写时序：START | SlaveAddr+W | RegH | RegL | Data | STOP
 *    - 读时序：START | SlaveAddr+W | RegH | RegL | STOP
 *              START | SlaveAddr+R | Data | STOP（重复起始）
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_i2c_init(void)
{
    /* 初始化 i2c0，速率 100 kHz */
    i2c_init(OV5640_I2C_INST, 100000u);

    /* 将宏定义的 SDA/SCL 引脚复用为 I2C 功能 */
    gpio_set_function(OV5640_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OV5640_I2C_SCL, GPIO_FUNC_I2C);

    /* I2C 总线需要上拉电阻；Pico 内部上拉约 50 kΩ，满足低速 SCCB 需求 */
    gpio_pull_up(OV5640_I2C_SDA);
    gpio_pull_up(OV5640_I2C_SCL);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  寄存器写入
 *
 *  数据包格式（共 3 字节）：
 *    [0] = addr[15:8]   — 16-bit 寄存器地址高字节
 *    [1] = addr[7:0]    — 16-bit 寄存器地址低字节
 *    [2] = data         — 写入值
 *
 *  返回 0 = 成功，1 = 失败（NAK 或总线错误）
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t ov5640_write_reg(uint16_t addr, uint8_t data)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(addr >> 8);
    buf[1] = (uint8_t)(addr & 0xFFu);
    buf[2] = data;

    int ret = i2c_write_blocking(OV5640_I2C_INST, OV5640_SCCB_ADDR, buf, 3, false);
    return (ret == 3) ? 0u : 1u;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  寄存器读取
 *
 *  SCCB 读时序（两阶段）：
 *    阶段 1（写寄存器地址，保持总线）：
 *      START | 0x3C+W | RegH | RegL | STOP(*)
 *    阶段 2（重新寻址读数据）：
 *      START | 0x3C+R | Data | NACK | STOP
 *    (*) 使用 nostop=true 保持 I2C 总线以产生重复 START
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t ov5640_read_reg(uint16_t addr)
{
    uint8_t reg[2];
    uint8_t val = 0u;

    reg[0] = (uint8_t)(addr >> 8);
    reg[1] = (uint8_t)(addr & 0xFFu);

    /* 写寄存器地址，nostop=true 产生重复 START */
    i2c_write_blocking(OV5640_I2C_INST, OV5640_SCCB_ADDR, reg, 2, true);

    /* 重复 START 后读一字节数据 */
    i2c_read_blocking(OV5640_I2C_INST, OV5640_SCCB_ADDR, &val, 1, false);

    return val;
}

/* ────────────────────────────────────────────────────────────────────────────
 *  引脚控制（上电时序）
 *
 *  控制信号（均由 OV5640 模块板拉出，需经 GPIO 输出推挽驱动）：
 *    - PWDN  低有效：HIGH=待机/掉电，LOW=正常工作
 *    - RESETB 低复位：LOW=复位，HIGH=运行
 *
 *  OV5640 datasheet 推荐上电时序：
 *    1) RST↓, PWDN↑              (复位并进入掉电)
 *    2) ≥1 ms 后 PWDN↓          (退出掉电，等待内部电源稳定)
 *    3) ≥1 ms 后 RST↑           (释放复位)
 *    4) ≥20 ms 后才能访问 SCCB  (等待内部上电完成)
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_pin_init(void)
{
    /* GPIO 复用为 SIO（默认）+ 设为输出 */
    gpio_init(OV5640_PWDN_PIN);
    gpio_init(OV5640_RST_PIN);
    gpio_set_dir(OV5640_PWDN_PIN, GPIO_OUT);
    gpio_set_dir(OV5640_RST_PIN,  GPIO_OUT);

    /* 阶段 1：进入复位 + 掉电状态 */
    gpio_put(OV5640_RST_PIN,  0);   /* RST  LOW  */
    gpio_put(OV5640_PWDN_PIN, 1);   /* PWDN HIGH */
    sleep_ms(10);

    /* 阶段 2：退出掉电 */
    gpio_put(OV5640_PWDN_PIN, 0);   /* PWDN LOW  */
    sleep_ms(10);

    /* 阶段 3：释放复位 */
    gpio_put(OV5640_RST_PIN, 1);    /* RST  HIGH */
    sleep_ms(20);                   /* ≥20 ms after RST↑ before SCCB */
}

/* ────────────────────────────────────────────────────────────────────────────
 *  软件复位
 *
 *  寄存器 0x3008[7] = 1 → 软复位
 *  寄存器 0x3008    = 0x02 → 正常工作模式（退出待机）
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_reset(void)
{
    ov5640_write_reg(0x3008u, 0x80u);   /* 触发软复位 */
    sleep_ms(10);
    ov5640_write_reg(0x3008u, 0x02u);   /* 进入正常工作模式 */
    sleep_ms(30);                        /* 等待 ISP 稳定 */
}

/* ────────────────────────────────────────────────────────────────────────────
 *  读取芯片 ID
 *
 *  正常情况应读到 PIDH=0x56, PIDL=0x40（即 OV5640）
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_read_id(OV5640_IDTypeDef *id)
{
    id->PIDH = ov5640_read_reg(OV5640_REG_PIDH);
    id->PIDL = ov5640_read_reg(OV5640_REG_PIDL);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  RGB565 格式配置
 *
 *  0x4300 FORMAT_CTRL00：
 *    [7:4] = 0x6 → RGB 输出
 *    [3:0] = 0xF → RGB565 字节序
 *  0x501F ISP FORMAT MUXING CTRL：
 *    0x01 → ISP 输出到 DVP（并行接口）
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_rgb565_config(void)
{
    ov5640_write_reg(0x4300u, 0x6Fu);
    ov5640_write_reg(0x501Fu, 0x01u);
}

/* ────────────────────────────────────────────────────────────────────────────
 *  触发一次图像采集
 *  委托给 cam_pio 模块启动 DMA 传输
 * ──────────────────────────────────────────────────────────────────────────*/
void ov5640_start_capture(void)
{
    cam_capture_start();
}
