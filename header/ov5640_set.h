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
  BMP_DEFAULT = 4,
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

#endif /* __OV5640_SET_H */
