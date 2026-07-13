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
 * 本模块不直接驱动发送链。采集链与发送链通过
 *   cam_acquire_line() / cam_release_line()
 * 这一对函数做单点交接，保持单向闭合：
 *   PIO → DMA → 完成中断(置标记) → 主循环取行 → PIO1
 *
 * 引脚分配（最终版）：
 *   数据总线  GPIO 12-19  (CAM_DATA_PIN_BASE = 12, COUNT = 8)
 *   像素时钟  GPIO 11     (CAM_PCLK_PIN)
 *   场同步    GPIO 20     (CAM_VSYNC_PIN, CPU 侧 GPIO 边沿中断)
 *   行有效    GPIO 10     (CAM_HREF_PIN, 兼作 jmp pin)
 *   传感器控制 GPIO 22/28 (OV5640_RST_PIN / OV5640_PWDN_PIN)
 */

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"

/* ── 引脚定义 ─────────────────────────────────────────────────────────────── */
#define CAM_DATA_PIN_BASE   12u
#define CAM_DATA_PIN_COUNT  8u
#define CAM_PCLK_PIN        11u
#define CAM_HREF_PIN        20u
#define CAM_VSYNC_PIN       10u
#define CAM_DEBUG_PIN       9u   /* DMA 完成中断调试输出 */

/* ── 每次 DMA 传输的字节数 / 字数 ────────────────────────────────────────────── */
/* 当前分辨率 640x480 Y8：一行 = 640px x 1B = 640B = 160 word = 一次 DMA。 */
#define CAPTURE_BYTES         640u
#define CAPTURE_LINES         480u
#define CAPTURE_WORDS       (CAPTURE_BYTES / 4u)
/* ── 行级三缓冲环 ─────────────────────────────────────────────────────────────
 * 采集 DMA 在 N 块行缓冲间循环写入：
 *   - 完成一行 → 该块标记为"就绪"，DMA 立即重装到下一块（中断内，无忙等）
 *   - 主循环取走"就绪"块交给 PIO1 发送，发送完归还，供采集复用
 * 三块给一行的流水留出余量：采集不必等发送，发送也不会读到正在写的块。
 * 若消费者落后到环满，采集丢弃最新一行并累加 cam_overrun_count（不阻塞采集）。
 * ──────────────────────────────────────────────────────────────────────────*/
#define CAM_NUM_BUFFERS      8u

/* ---------------------------------------------------------------------
 * 4x4-buffer (scaffold)
 *  - 预留用于将现有的 3-lines 环（CAM_NUM_BUFFERS）替换为
 *    4 个每个包含 4 行的块（总计 16 行的环），以降低中断频率
 *    并为后续压缩算法/批处理提供更大窗口。
 *  - 本头仅提供预置常量与接口声明；实际切换须在 cam_pio.c
 *    中完成并需要验证内存占用与时序约束。
 */
#define CAPTURE_CHUNK_LINES  4u
#define CAM_4X4_NUM_BUFFERS  4u


/* 启用 4x4 scaffold（当前为预留实现，调用不会影响默认 3-buffer 行为） */
void cam_enable_4x4_scaffold(void);

/* 丢弃接下来若干个 VSYNC 帧，用于唤醒后的稳定期 */
void cam_discard_next_frames(uint8_t frames);


extern volatile uint8_t  frame_ready;       /* VSYNC 帧边界标记，供 IMU 按帧采样 */
extern volatile uint32_t cam_overrun_count; /* 环满丢行计数（调试用） */


/* 行计数器（用于滤波计算） */ 
extern volatile uint32_t cam_linem1_count;
extern volatile uint32_t cam_line0_count;     
extern volatile uint32_t cam_linep1_count;
extern volatile uint8_t  cam_filter_ready;        /* 采集链滤波计算就绪标记 */

extern volatile uint32_t cam_line_count;   /* 采集帧计数（调试用） */

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

/* 访问内部行缓冲，供图像处理层读取已采集的行 */
const uint8_t *cam_get_buffer(uint32_t index);

/* ── 采集/发送交接（消费者侧调用，通常在主循环）────────────────────────────────
 * cam_acquire_line(): 若有就绪行且上一行已归还，返回该行缓冲指针并标记为"在飞"；
 *                     否则返回 NULL。
 * cam_release_line(): 发送完成后归还在飞缓冲，使其可被采集复用。
 * 一次只允许一行在飞（对应单条 PIO1 DMA）。
 * ──────────────────────────────────────────────────────────────────────────*/
uint8_t *cam_acquire_line(void);
void cam_release_line(void);

#endif /* CAM_PIO_H */
