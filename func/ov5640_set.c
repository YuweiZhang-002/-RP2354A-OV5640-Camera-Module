/* Includes ------------------------------------------------------------------*/
#include "ov5640_set.h"
#include "ov5640.h"        /* ov5640_write_reg() — Pico SCCB writer */
#include "ov5640_regs.h"   /* sensor_reg_t + register address #defines */

volatile uint8_t isInitialised = 0;



int32_t OV5640_Init(uint32_t Resolution, uint32_t PixelFormat, uint32_t Polarity)
{
  int32_t ret = OV5640_OK;

  /* 1) 软件复位：确保后续寄存器写入处于干净状态 */
  ov5640_reset();

  /* 2) 读取并验证芯片 ID：这是当前 I2C/SCCB 是否正常的最直接证据 */
  OV5640_IDTypeDef id;
  ov5640_read_id(&id);
  if (id.PIDH != 0x56u || id.PIDL != 0x40u)
  {
    return OV5640_ERROR; /* 芯片 ID 不匹配，初始化失败 */
  }

  /* 3) 配置寄存器（分模块写入，便于调试和后续调整）
   *    这部分决定传感器是否真的会输出 DVP 数据。
   *    写失败则说明 SCCB 事务或寄存器依赖链异常。
   */
  for (size_t i = 0; i < sizeof(ov5640_init_common) / sizeof(ov5640_init_common[0]); i++)
  {
    if (ov5640_write_reg(ov5640_init_common[i].reg, (uint8_t)ov5640_init_common[i].value) != 0U)
    {
      return OV5640_ERROR; /* 任何寄存器写入失败都立即返回错误 */
    }
  }
  
  /* 4) 启用 DVP 模式：切到并行口输出 */
  if (OV5640_EnableDVPMode() != OV5640_OK)
  {
    return OV5640_ERROR;
  }

  /* 5) 设置分辨率 */
  if (OV5640_SetResolution(Resolution) != OV5640_OK)
  {
    return OV5640_ERROR;
  }

  /* 6) 设置像素格式 */
  if (OV5640_SetPixelFormat(PixelFormat) != OV5640_OK)
  {
    return OV5640_ERROR;
  }

  /* 7) 设置极性 */
  if (OV5640_SetPolarity(Polarity) != OV5640_OK)
  {
    return OV5640_ERROR;
  }

  isInitialised = 1; /* 初始化成功，更新状态 */

  return ret;
}


/**
  * @brief  Set OV5640 camera Pixel Format.
  *
  *         Writes the matching FORMAT_CTRL00 / FORMAT_MUX_CTRL register pair
  *         over SCCB using ov5640_write_reg() (defined in ov5640.c).
  *
  * @param  PixelFormat  one of OV5640_RGB565 / OV5640_RGB888 / OV5640_YUV422
  *                      / OV5640_Y8 / OV5640_JPEG (see ov5640_set.h).
  * @retval OV5640_OK    all writes acknowledged on the bus.
  * @retval OV5640_ERROR unsupported PixelFormat or a write failed.
  */
int32_t OV5640_SetPixelFormat(uint32_t PixelFormat)
{
  const sensor_reg_t *regs;
  uint32_t count;
  uint32_t i;

  switch (PixelFormat)
  {
    case OV5640_RGB565:
      regs  = ov5640_PF_RGB565;
      count = sizeof(ov5640_PF_RGB565) / sizeof(ov5640_PF_RGB565[0]);
      break;

    case OV5640_RGB888:
      regs  = ov5640_PF_RGB888;
      count = sizeof(ov5640_PF_RGB888) / sizeof(ov5640_PF_RGB888[0]);
      break;

    case OV5640_YUV422:
      regs  = ov5640_PF_YUV422;
      count = sizeof(ov5640_PF_YUV422) / sizeof(ov5640_PF_YUV422[0]);
      break;

    case OV5640_Y8:
      regs  = ov5640_PF_Y8;
      count = sizeof(ov5640_PF_Y8) / sizeof(ov5640_PF_Y8[0]);
      break;

    case OV5640_JPEG:
      regs  = ov5640_PF_JPEG;
      count = sizeof(ov5640_PF_JPEG) / sizeof(ov5640_PF_JPEG[0]);
      break;

    default:
      return OV5640_ERROR;
  }

  for (i = 0U; i < count; i++)
  {
    if (ov5640_write_reg(regs[i].reg, (uint8_t)regs[i].value) != 0U)
    {
      return OV5640_ERROR;
    }
  }

  return OV5640_OK;
}



int32_t OV5640_SetResolution(uint32_t Resolution)
{
  const sensor_reg_t *regs;
  uint32_t count;
  uint32_t i;

  switch (Resolution)
  {
    case BMP_320x240:
      regs  = ov5640_qvga_regs;
      count = sizeof(ov5640_qvga_regs) / sizeof(ov5640_qvga_regs[0]);
      break;

    case BMP_640x480:
      regs  = ov5640_vga_regs;
      count = sizeof(ov5640_vga_regs) / sizeof(ov5640_vga_regs[0]);
      break;

    case BMP_800x480:
      regs  = ov5640_wvga_regs;
      count = sizeof(ov5640_wvga_regs) / sizeof(ov5640_wvga_regs[0]);
      break;

    case BMP_1280x720:
      regs  = ov5640_svga_regs;
      count = sizeof(ov5640_svga_regs) / sizeof(ov5640_svga_regs[0]);
      break;

    case BMP_DEFAULT:
      regs  = ov5640_wvga_regs;
      count = sizeof(ov5640_wvga_regs) / sizeof(ov5640_wvga_regs[0]);
      break;

    default:
      return OV5640_ERROR;
  }

  for (i = 0U; i < count; i++)
  {
    if (ov5640_write_reg(regs[i].reg, (uint8_t)regs[i].value) != 0U)
    {
      return OV5640_ERROR;
    }
  }

  return OV5640_OK;
}


/**
  * @brief  Enable OV5640 DVP (parallel) mode.
  *
  *         Applies DVP interface configuration registers and hard-disables
  *         the MIPI PHY since this project uses parallel DVP output only.
  *         Writes the DVP mode register array over SCCB using ov5640_write_reg().
  *
  * @retval OV5640_OK    all writes acknowledged on the bus.
  * @retval OV5640_ERROR a register write failed.
  */
int32_t OV5640_EnableDVPMode(void)
{
  uint32_t i;
  uint32_t count = sizeof(ov5640_init_dvp) / sizeof(ov5640_init_dvp[0]);

  for (i = 0U; i < count; i++)
  {
    if (ov5640_write_reg(ov5640_init_dvp[i].reg, (uint8_t)ov5640_init_dvp[i].value) != 0U)
    {
      return OV5640_ERROR;
    }
  }

  return OV5640_OK;
}


/**
  * @brief  Set OV5640 camera signal polarity.
  *
  *         Selects one of six predefined polarity configurations for
  *         PCLK, VSYNC, and HREF signals. Each case corresponds to a
  *         specific combination of active-high/active-low settings.
  *         Writes the matching polarity register using ov5640_write_reg().
  *
  * @param  Polarity  polarity configuration index (0-5) (Reference from ov5640_reg.h):
  *                   0: All signals active low
  *                   1: VSYNC active high, HREF/PCLK active low
  *                   2: VSYNC/HREF active high, PCLK active low
  *                   3: HREF/VSYNC active low, PCLK active high
  *                   4: HREF active low, VSYNC/PCLK active high
  *                   5: All signals active high (default)
  * @retval OV5640_OK    write acknowledged on the bus.
  * @retval OV5640_ERROR unsupported Polarity or write failed.
  */
int32_t OV5640_SetPolarity(uint32_t Polarity)
{
  if (Polarity > 5U)
  {
    return OV5640_ERROR;
  }

  if (ov5640_write_reg(ov5640_polarity_regs[Polarity].reg, (uint8_t)ov5640_polarity_regs[Polarity].value) != 0U)
  {
    return OV5640_ERROR;
  }

  return OV5640_OK;
}



int32_t OV5640_SetActivation(uint32_t Activation)
{
  if (Activation > 3U)
  {
    return OV5640_ERROR;
  }

  if (ov5640_write_reg(ov5640_power_regs[Activation].reg, (uint8_t)ov5640_power_regs[Activation].value) != 0U)
  {
    return OV5640_ERROR;
  }

  return OV5640_OK;
}