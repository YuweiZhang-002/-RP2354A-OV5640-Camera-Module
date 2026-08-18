#ifndef __OV5640_SET_H
#define __OV5640_SET_H

#include <stdint.h>

#define OV5640_OK                      (0)
#define OV5640_ERROR                   (-1)

/** @defgroup OV5640 Pixel Format Identifiers
  * @{
  *   Format selectors consumed by OV5640_SetPixelFormat().
  */
#define OV5640_RGB565                  0x00U
#define OV5640_RGB888                  0x01U
#define OV5640_YUV422                  0x02U
#define OV5640_Y8                      0x03U
#define OV5640_JPEG                    0x04U
/**
  * @}
  */

/** @defgroup OV5640 SetPixelFormat API
  * @{
  */
int32_t OV5640_SetPixelFormat(uint32_t PixelFormat);


/** @defgroup OV5640 Resolution Identifiers
  * @{
  *   Format selectors consumed by OV5640_SetResolution().
  */
typedef enum{
	BMP_320x240 = 0,
	BMP_640x480 = 1,
  BMP_800x480 = 2,
  BMP_1280x720 = 3,
  BMP_800x600 = 4,
  BMP_DEFAULT = 5,
}ImageFormatTypeDef;


/** @defgroup OV5640 Polarity Identifiers
  * @{
  *   Format selectors consumed by OV5640_SetPolarity().
  *   Specific register values please refer to ov5640_polarity_regs[] in ov5640_regs.h 
  *   For the polarity settings, Refer to OV5640_SetPolarity(uint32_t Polarity) API for details.
  */

#define OV5640_Polarity_1       0x00U
#define OV5640_Polarity_2       0x01U
#define OV5640_Polarity_3       0x02U
#define OV5640_Polarity_4       0x03U
#define OV5640_Polarity_5       0x04U
#define OV5640_Polarity_6       0x05U
  
/** @defgroup OV5640 System Operation Identifiers
  * @{
  *   Format selectors consumed by OV5640_SetActivation().
  */
 #define OV5640_PowerUp         0x00U
 #define OV5640_PowerDown       0x01U 
 #define OV5640_Reset           0x02U


/** @defgroup OV5640 SetResolution API
  * @{
  */
int32_t OV5640_SetResolution(uint32_t Resolution);


/** @defgroup OV5640 SetPolarity API
  * @{
  */
int32_t OV5640_SetPolarity(uint32_t Polarity);

/** @defgroup OV5640 EnableDVPMode API
  * @{
  */
int32_t OV5640_EnableDVPMode(void);

extern volatile uint8_t isInitialised;


/** @defgroup OV5640 Init API
  * @{
  */
int32_t OV5640_Init(uint32_t Resolution, uint32_t PixelFormat, uint32_t Polarity);

/** @defgroup OV5640 Control System status API
  * @{
  */
 int32_t OV5640_SetActivation(uint32_t Activation);


/** @defgroup OV5640 AEC runtime read-back
  * @{
  *
  * 运行期读回 AEC 实际工作点。用途：判断"动起来撕裂"里有多少是运动模糊。
  *   - 若 exposure_us 顶在 8330 / 10020 附近 → 曝光受限于上限，模糊是主因，
  *     光照不足；加光或按需进一步压上限（代价是必须关防频闪）。
  *   - 若 exposure_us 远低于上限（例如 2000）→ 模糊不是主因，
  *     剩下的就是卷帘快门剪切（480 行 x 130.167us = 62.5 ms 读出），
  *     只能靠提高 PCLK 或降分辨率缩短读出时间。
  *
  * 走 SCCB 阻塞读，约 6 次 I2C 事务，**不要放在逐行热路径里**；
  * 在断点处调用或按秒级调用即可。
  */
typedef struct {
    uint32_t exposure_q4;    /* 0x3500..0x3502 原始值，单位 1/16 行 */
    uint32_t exposure_us;    /* 换算后的实际曝光时间 */
    uint16_t gain_q4;        /* 0x350A/0x350B，单位 1/16 倍 */
    uint16_t hts;            /* 0x380C/0x380D */
    uint16_t vts;            /* 0x380E/0x380F */
    uint8_t  aec_ctrl00;     /* 0x3A00，bit2 应恒为 0（夜视关闭） */
} ov5640_aec_status_t;

/* PCLK 实测 12.000 MHz（clk_sys/6 = 24 MHz XCLK 经传感器 PLL）。 */
#define OV5640_PCLK_MHZ  12u

void OV5640_ReadAecStatus(ov5640_aec_status_t *status);

#endif /* __OV5640_SET_H */
