/**
  ******************************************************************************
  * @file    ov5640_reg.h
  * @author  MCD Application Team
  * @brief   Header of ov5640_reg.c
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OV5640_REG_H
#define OV5640_REG_H

#include <stdint.h>
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup OV5640
  * @{
  */

/** @defgroup OV5640_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup OV5640_Exported_Constants OV5640 Exported Constants
  * @{
  */


/**
  * @}
  */

/************** Generic Function  *******************/
/* ------------------------------------------------------------------------- */
/* Minimal OV5640 SCCB/I2C helper API used by src/user/func/ov5640.c         */
/* ------------------------------------------------------------------------- */

/* OV5640 7-bit SCCB address (SADDR pin low -> 0x3C). */
#define OV5640_I2C_INST   i2c0
#define OV5640_I2C_SDA    10u
#define OV5640_I2C_SCL    9u
#define OV5640_SCCB_ADDR  0x3Cu

/* 上电控制引脚（低电平有效的 PWDN/RESETB 由 GPIO 直接驱动） */
#define OV5640_PWDN_PIN   14u
#define OV5640_RST_PIN    8u

/* Chip ID registers. */
#define OV5640_REG_PIDH   0x300Au
#define OV5640_REG_PIDL   0x300Bu
#define OV5640_CHIP_ID    0x5640u

typedef struct {
    uint8_t PIDH;
    uint8_t PIDL;
} OV5640_IDTypeDef;

void ov5640_i2c_init(void);
void ov5640_pin_init(void);
void ov5640_reset(void);
uint8_t ov5640_read_reg(uint16_t addr);
uint8_t ov5640_write_reg(uint16_t addr, uint8_t data);
void ov5640_read_id(OV5640_IDTypeDef *id);
void ov5640_rgb565_config(void);
void ov5640_start_capture(void);


/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* OV5640_REG_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
