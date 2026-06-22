#ifndef CAM_PIO_H
#define CAM_PIO_H

/*
 * cam_pio.h  —  PIO 摄像头采集接口声明
 *
 * 职责（单一生产者）：
 *   - GPIO 初始化（数据线 + PCLK/HREF/VSYNC）
 *   - PIO 状态机加载与启动
 *   - DMA 通道申请、DREQ 绑定、完成中断里自维持地逐行重装
 *   - 行级三缓冲环：采集只负责"生产并打标记"，发送侧自行取用
 *
 * 本模块不直接驱动 HSTX。采集链与发送链通过
 *   cam_acquire_line() / cam_release_line()
 * 这一对函数做单点交接，保持单向闭合：
 *   PIO → DMA → 完成中断(置标记) → 主循环取行 → HSTX
 *
 * 引脚分配（实际值，注意与旧注释中的 2-9 不同）：
 *   数据总线  GPIO 22-29  (CAM_DATA_PIN_BASE = 22, COUNT = 8)
 *   像素时钟  GPIO 6      (CAM_PCLK_PIN)
 *   行有效    GPIO 5      (CAM_HREF_PIN)
 *   场同步    GPIO 7      (CAM_VSYNC_PIN)
 *   调试输出  GPIO 11     (CAM_FRAME_VALID)
 */

#include <stdint.h>
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"

/* ── 引脚定义 ─────────────────────────────────────────────────────────────── */
#define CAM_DATA_PIN_BASE   22u
#define CAM_DATA_PIN_COUNT  8u
#define CAM_PCLK_PIN        6u
#define CAM_HREF_PIN        5u
#define CAM_VSYNC_PIN       7u
#define CAM_FRAME_VALID     11u
/* PIO 输出：用于由 PIO 状态机驱动的同步输出（VSYNC/HREF 观测/转发） */
#define CAM_VSYNC_OUT_PIN   17u
#define CAM_HREF_OUT_PIN    18u

/* ── 每次 DMA 传输的字节数 / 字数 ────────────────────────────────────────────── */
/* 当前分辨率 800x600 RGB565：一行 = 800px x 2B = 1600B = 400 word = 一次 DMA。 */
#define CAPTURE_BYTES        1600u
#define CAPTURE_WORDS       (CAPTURE_BYTES / 4u)
/* PIO 在 VSYNC 后加载的帧内行计数器（通过 TX FIFO 送入）：600 行/帧 */
#define FRAME_LOOPS         600u

/* ── 行级三缓冲环 ─────────────────────────────────────────────────────────────
 * 采集 DMA 在 N 块行缓冲间循环写入：
 *   - 完成一行 → 该块标记为"就绪"，DMA 立即重装到下一块（中断内，无忙等）
 *   - 主循环取走"就绪"块交给 HSTX 发送，发送完归还，供采集复用
 * 三块给一行的流水留出余量：采集不必等发送，发送也不会读到正在写的块。
 * 若消费者落后到环满，采集丢弃最新一行并累加 cam_overrun_count（不阻塞采集）。
 * ──────────────────────────────────────────────────────────────────────────*/
#define CAM_NUM_BUFFERS      3u

extern volatile uint8_t  frame_ready;       /* VSYNC 帧边界标记，供 IMU 按帧采样 */
extern volatile uint32_t cam_overrun_count; /* 环满丢行计数（调试用） */

/* ── 函数声明 ─────────────────────────────────────────────────────────────── */

/* 设置数据/PCLK/HREF/VSYNC 引脚方向与上拉，配置调试输出引脚 */
void cam_gpio_init(void);

/* 加载 PIO 程序，配置并预装行计数器（不使能状态机） */
void cam_pio_init(void);

/* 申请 DMA 通道、绑定 PIO RX DREQ、注册 DMA 完成中断与 VSYNC 中断（不启动传输） */
void cam_dma_init(void);

/* 复位缓冲环、武装首块、使能状态机，开始连续采集 */
void cam_capture_start(void);

/* 关闭状态机并终止 DMA，清空 FIFO */
void cam_capture_stop(void);

/* ── 采集/发送交接（消费者侧调用，通常在主循环）────────────────────────────────
 * cam_acquire_line(): 若有就绪行且上一行已归还，返回该行缓冲指针并标记为"在飞"；
 *                     否则返回 NULL。
 * cam_release_line(): 发送完成后归还在飞缓冲，使其可被采集复用。
 * 一次只允许一行在飞（对应单条 HSTX DMA）。
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t *cam_acquire_line(void);
void cam_release_line(void);

#endif /* CAM_PIO_H */
