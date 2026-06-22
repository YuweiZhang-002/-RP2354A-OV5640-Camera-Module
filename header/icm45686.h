#ifndef ICM45686_H
#define ICM45686_H

#include <stdint.h>

/*
 * ICM-45686 register definitions for the UI path.
 * Source: TDK_DS_000577_ICM_45686.pdf
 *
 * This header keeps the UI data registers and the core control registers that
 * are relevant for later SPI bring-up.
 */

/* UI sensor data registers */
#define ICM45686_ACCEL_DATA_X1_UI    0x00U
#define ICM45686_ACCEL_DATA_X0_UI    0x01U
#define ICM45686_ACCEL_DATA_Y1_UI    0x02U
#define ICM45686_ACCEL_DATA_Y0_UI    0x03U
#define ICM45686_ACCEL_DATA_Z1_UI    0x04U
#define ICM45686_ACCEL_DATA_Z0_UI    0x05U
#define ICM45686_GYRO_DATA_X1_UI     0x06U
#define ICM45686_GYRO_DATA_X0_UI     0x07U
#define ICM45686_GYRO_DATA_Y1_UI     0x08U
#define ICM45686_GYRO_DATA_Y0_UI     0x09U
#define ICM45686_GYRO_DATA_Z1_UI     0x0AU
#define ICM45686_GYRO_DATA_Z0_UI     0x0BU
#define ICM45686_TEMP_DATA1_UI       0x0CU
#define ICM45686_TEMP_DATA0_UI       0x0DU
#define ICM45686_TMST_FSYNCH         0x0EU
#define ICM45686_TMST_FSYNCL         0x0FU

/* Core power and output configuration */
#define ICM45686_PWR_MGMT0           0x10U
#define ICM45686_FIFO_COUNT_0        0x12U
#define ICM45686_FIFO_COUNT_1        0x13U
#define ICM45686_FIFO_DATA           0x14U
#define ICM45686_INT1_CONFIG0        0x16U
#define ICM45686_INT1_CONFIG1        0x17U
#define ICM45686_INT1_CONFIG2        0x18U
#define ICM45686_INT1_STATUS0        0x19U
#define ICM45686_INT1_STATUS1        0x1AU
#define ICM45686_ACCEL_CONFIG0       0x1BU
#define ICM45686_GYRO_CONFIG0        0x1CU

/* FIFO configuration */
#define ICM45686_FIFO_CONFIG0        0x1DU
#define ICM45686_FIFO_CONFIG1_0      0x1EU
#define ICM45686_FIFO_CONFIG1_1      0x1FU
#define ICM45686_FIFO_CONFIG2        0x20U
#define ICM45686_FIFO_CONFIG3        0x21U
#define ICM45686_FIFO_CONFIG4        0x22U
#define ICM45686_TMST_WOM_CONFIG     0x23U

/* Interface / SPI-related control */
#define ICM45686_INTF_CONFIG0        0x2CU
#define ICM45686_INTF_CONFIG1_OVRD   0x2DU
#define ICM45686_INTF_AUX_CONFIG     0x2EU
#define ICM45686_SREG_CTRL           0x67U

/* Device identity */
#define ICM45686_WHO_AM_I            0x72U
#define ICM45686_REG_HOST_MSG        0x73U

/* UI accel + gyro + temperature frame length */
#define ICM45686_DataNum             14u

/* Common values for bring-up */
#define ICM45686_WHO_AM_I_VALUE      0xE9U

#define ICM45686_PWR_MGMT0_GYRO_OFF      0x00U
#define ICM45686_PWR_MGMT0_GYRO_STANDBY   0x04U
#define ICM45686_PWR_MGMT0_GYRO_LP       0x08U
#define ICM45686_PWR_MGMT0_GYRO_LN       0x0CU

#define ICM45686_PWR_MGMT0_ACCEL_OFF      0x00U
#define ICM45686_PWR_MGMT0_ACCEL_LP       0x02U
#define ICM45686_PWR_MGMT0_ACCEL_LN       0x03U

#define ICM45686_ACCEL_CONFIG0_FS_32G     0x00U
#define ICM45686_ACCEL_CONFIG0_FS_16G     0x10U
#define ICM45686_ACCEL_CONFIG0_FS_8G      0x20U
#define ICM45686_ACCEL_CONFIG0_FS_4G      0x30U
#define ICM45686_ACCEL_CONFIG0_FS_2G      0x40U

#define ICM45686_GYRO_CONFIG0_FS_4000DPS  0x00U
#define ICM45686_GYRO_CONFIG0_FS_2000DPS  0x10U
#define ICM45686_GYRO_CONFIG0_FS_1000DPS  0x20U
#define ICM45686_GYRO_CONFIG0_FS_500DPS   0x30U
#define ICM45686_GYRO_CONFIG0_FS_250DPS   0x40U
#define ICM45686_GYRO_CONFIG0_FS_125DPS   0x50U


/* ODR 选择（ACCEL_ODR / GYRO_ODR 共用 [3:0] 编码） */
#define ICM45686_ODR_400HZ                0x07U   /* 0111: 400Hz (LP/LN) */

/* 软复位 REG_MISC2 (0x7F) */
#define ICM45686_REG_MISC2                0x7FU
#define ICM45686_REG_MISC2_SOFT_RST       0x02U   /* bit1: 写 1 触发软复位，硬件自清零 */

/* 数据端序 SREG_CTRL (0x67) */
#define ICM45686_SREG_CTRL_BIG_ENDIAN     0x02U   /* bit1=1: 数据寄存器/FIFO/count 大端 */

/*
 * ICM-45686 驱动接口定义
 *  - icm45686_init()：初始化 SPI、UART、DMA，配置寄存器
 *  - icm45686_verify_id()：读取 WHO_AM_I 寄存器验证设备 ID
 *  - icm45686_getdata()：直接读取 UI 数据寄存器（不通过 FIFO）
 *
 * 注意：本驱动仅实现了最基本的功能，后续可根据需要添加更多接口，如：
 *   - FIFO 读取接口（icm45686_read_FIFO()）以支持更高效的数据采集
 *   - 中断配置和处理接口以支持事件驱动的数据读取
 *   - 更丰富的配置接口以支持不同的测量范围和输出数据率
 */

#endif /* ICM45686_H */