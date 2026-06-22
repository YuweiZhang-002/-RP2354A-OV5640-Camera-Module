/**
 * @file hstx.h
 * @brief RP2354 HSTX 差分输出最小 Demo — 公开接口
 *
 * ============================================================
 * 硬件拓扑 (Hardware Topology)
 * ============================================================
 *
 *  HSTX 外设内部结构:
 *
 *   ┌─────────────────────────────────────────────────────┐
 *   │                   HSTX 控制器                        │
 *   │                                                     │
 *   │  hstx_fifo_hw->fifo (0x50100004)                    │
 *   │        ↓  (DMA 写入)                                │
 *   │  8×32-bit FIFO 队列                                 │
 *   │        ↓                                            │
 *   │  32-bit 移位寄存器 (右旋 SHIFT=2 bit/周期)          │
 *   │        ↓          ↓                                 │
 *   │  bit[0]→BIT4寄存器  bit[1]→BIT5寄存器  ...          │
 *   │     (GPIO16/19)       (GPIO17/18)                   │
 *   └─────────────────────────────────────────────────────┘
 *
 * ============================================================
 * 差分引脚分配 (Differential Pin Assignment)
 * ============================================================
 *
 *  HSTX bit 序号与 GPIO 的对应关系 (GPIO N = HSTX bit (N-12)):
 *
 *   GPIO16 = HSTX_4  →  hstx_ctrl_hw->bit[4]  → 差分负极 (INV=1)
 *   GPIO17 = HSTX_5  →  hstx_ctrl_hw->bit[5]  → 差分负极 (INV=1)
 *   GPIO18 = HSTX_6  →  hstx_ctrl_hw->bit[6]  → 差分正极 (INV=0)
 *   GPIO19 = HSTX_7  →  hstx_ctrl_hw->bit[7]  → 差分正极 (INV=0)
 *
 *  两个差分对 (对应两条 lane):
 *
 *   LCD_ADDRESS  (lane 0):  GPIO19(+) / GPIO16(-)  ← 移位寄存器 data bit 0
 *   LCD_ADDRESS_B(lane 1):  GPIO18(+) / GPIO17(-)  ← 移位寄存器 data bit 1
 *
 *  一个字节如何拆分到两对差分信号:
 *
 *   byte = b7 b6 b5 b4 b3 b2 b1 b0
 *   HSTX 每个时钟周期右旋2bit并输出低2位:
 *     周期1: 输出 b1(lane1) b0(lane0)
 *     周期2: 输出 b3(lane1) b2(lane0)
 *     周期3: 输出 b5(lane1) b4(lane0)
 *     周期4: 输出 b7(lane1) b6(lane0)
 *   → 4个周期输出完一个字节，2个周期输出4bit给每条lane
 *
 * ============================================================
 */

#ifndef USER_HSTX_H
#define USER_HSTX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ----------------------------------------------------------
 * 引脚定义 (Pin Definitions)
 * LCD_ADDRESS  → GPIO16(负极) / GPIO19(正极)  差分对
 * LCD_ADDRESS_B→ GPIO17(负极) / GPIO18(正极)  差分对
 * ---------------------------------------------------------- */
#define HSTX_NEG_0_PIN  16u   /* LCD_ADDRESS   负极: GPIO16 = HSTX_4 */
#define HSTX_NEG_1_PIN  17u   /* LCD_ADDRESS_B 负极: GPIO17 = HSTX_5 */
#define HSTX_POS_1_PIN  18u   /* LCD_ADDRESS_B 正极: GPIO18 = HSTX_6 */
#define HSTX_POS_0_PIN  19u   /* LCD_ADDRESS   正极: GPIO19 = HSTX_7 */

#define HSTX_GPIO_FIRST HSTX_NEG_0_PIN  /* 连续 GPIO 的起始编号 */
#define HSTX_GPIO_COUNT 4u              /* 共4个 GPIO */

/* 每次传输的默认字节数 (与 hstx.c 中的实现保持一致) */
#define HSTX_TX_BYTES 1600u /* 1600字节 = 400个32-bit字，适合一次完整的帧数据传输 */
/* 兼容旧名称 */
#define HSTX_TX_WORDS (HSTX_TX_BYTES / 4u)
#define HSTX_TX_DMA_COUNT HSTX_TX_BYTES

/* ----------------------------------------------------------
 * 公开函数接口 (Public API)
 *
 * 典型调用顺序:
 *   hstx_gpio_init();        // 1. GPIO 复用为 HSTX
 *   hstx_init();             // 2. 配置 HSTX 控制器寄存器
 *   hstx_dma_init();         // 3. 申请并配置 DMA 通道
 *   hstx_tx_start(buf);      // 4. 启动 DMA 传输（按字节流发送）
 *   while (hstx_dma_busy()); // 5. 等待完成
 * ---------------------------------------------------------- */

/** 将 GPIO16-19 切换到 HSTX 复用功能 (GPIO_FUNC_HSTX = 0) */
void hstx_gpio_init(void);

/** 配置 HSTX 控制器: lane 映射、差分极性、时钟、移位参数 */
void hstx_init(void);

/** 申请 DMA 通道并绑定 DREQ_HSTX (DREQ=52)，目标为 HSTX FIFO */
void hstx_dma_init(void);

/**
 * 将 buffer 作为字节流通过 DMA 发送到 HSTX FIFO
 * @param buffer    源数据指针 (任意字节对齐)
 */
void hstx_tx_start(const uint8_t *buffer);

/** 返回 DMA 通道是否仍在传输 */
bool hstx_dma_busy(void);

#endif /* USER_HSTX_H */
