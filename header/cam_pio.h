#ifndef CAM_PIO_H
#define CAM_PIO_H

/*
 * cam_pio.h  —  PIO 摄像头采集接口声明
 *
 * 职责（单一生产者）：
 *   - GPIO 初始化（数据线 + PCLK/HREF/VSYNC）
 *   - PIO 状态机加载与启动
 *   - DMA 通道申请、DREQ 绑定、完成中断里自维持地逐行重装
 *   - 以物理 VSYNC/HREF 为权威生成帧/行身份
 *   - 发布只携带 ownership 的行描述符，不复制 640-byte 图像本体
 *
 * 行身份的硬件保证：
 *   cam_capture.pio 在每条 HREF 内精确计数 640 个 PCLK，行首用
 *   wait 0 / wait 1 gpio HREF 重新对齐。因此
 *       一个 HREF  <=>  640 个采样字节  <=>  一次 640-byte DMA 完成
 *   行号由帧内 DMA 块完成次数产生，与 HREF/PCLK 的异步相位无关，
 *   也与 CPU 中断延迟无关（PIO 行尾不再等待 CPU）。
 *
 * 本模块不直接驱动发送链。采集链与处理链通过描述符队列交接：
 *   PIO(640B/行) → DMA完成=行边界 → descriptor queue → Core0 → Core1 → PIO1
 *
 * 引脚分配（最终版）：
 *   数据总线  GPIO 12-19  (CAM_DATA_PIN_BASE = 12, COUNT = 8)
 *   像素时钟  GPIO 11     (CAM_PCLK_PIN)
 *   场同步    GPIO 10     (CAM_VSYNC_PIN, 启动和运行期物理帧边界)
 *   行有效    GPIO 20     (CAM_HREF_PIN, PIO wait/jmp；CPU不配置GPIO IRQ)
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
#define CAM_DEBUG_PIN       9u   /* 旧调试别名；GPIO9现由FPGA_HREF使用，采集链不得驱动 */

/* ── 每次 DMA 传输的字节数 / 字数 ────────────────────────────────────────────── */
/* 当前分辨率 640x480 Y8：一行 = 640px x 1B = 640B = 160 word = 一次 DMA。 */
#define CAPTURE_BYTES         640u
#define CAPTURE_LINES         480u
#define CAPTURE_WORDS       (CAPTURE_BYTES / 4u)
/* ── 行级缓冲环 ───────────────────────────────────────────────────────────────
 * 采集 DMA 在 N 块行缓冲间循环写入：
 *   - 完成一行 → 该块标记为"就绪"，DMA 立即重装到下一块（中断内，无忙等）
 *   - 主循环取走"就绪"块交给 PIO1 发送，发送完归还，供采集复用
 * 16块给采集/双核处理的瞬时抖动留出余量，发送不会读到正在写的块。
 * 若消费者落后到环满，采集丢弃最新一行并累加 cam_overrun_count（不阻塞采集）。
 * ──────────────────────────────────────────────────────────────────────────*/
#define CAM_NUM_BUFFERS      16u
#define CAM_DESC_QUEUE_DEPTH 32u
#define CAM_INVALID_BUFFER   0xffu

typedef enum {
    CAM_DESC_LINE = 0u,
    CAM_DESC_FRAME_END = 1u,
} cam_descriptor_type_t;

/* 帧失效原因；可组合写入 packet consistency metadata。 */
#define CAM_FRAME_ERR_DESC_QUEUE_FULL (1u << 0)
#define CAM_FRAME_ERR_DMA_OVERRUN      (1u << 1)
#define CAM_FRAME_ERR_PIO_RXOVER       (1u << 2)
#define CAM_FRAME_ERR_HREF_LENGTH      (1u << 3)
#define CAM_FRAME_ERR_ROW_JUMP         (1u << 4)
#define CAM_FRAME_ERR_DUPLICATE_ROW    (1u << 5)
#define CAM_FRAME_ERR_TOO_MANY_ROWS    (1u << 6)
#define CAM_FRAME_ERR_TOO_FEW_ROWS     (1u << 7)
#define CAM_FRAME_ERR_NO_BUFFER        (1u << 8)
#define CAM_FRAME_ERR_BOUNDARY         (1u << 9)

/*
 * CAM_DESC_LINE 语义：
 *   row_idx / frame_id 永远权威——它们来自 VSYNC 与 PIO 确认的 HREF 行，
 *   与数据是否搬成功无关。
 *   valid == 1 : buffer_idx 指向可用的 640-byte 原始行
 *   valid == 0 : 该物理行确实存在，但 payload 缺失（无空闲 buffer / RXOVER）；
 *                下游必须为它保留行位置，禁止用后一行前移补位。
 */
typedef struct {
    uint32_t frame_id;
    uint32_t descriptor_seq;
    uint16_t row_idx;
    uint16_t rows_seen;
    uint16_t error_flags;
    uint8_t  buffer_idx;
    uint8_t  type;
    uint8_t  valid;
    uint8_t  reserved;
} cam_descriptor_t;

/* VSYNC时锁存的上一个物理帧诊断。调试器可直接观察该结构，
 * 无需在实时路径中使用printf。 */
typedef struct {
    uint32_t frame_id;
    uint16_t rows_seen;       /* 本帧PIO确认的HREF行数，正常=CAPTURE_LINES */
    uint16_t rows_published;  /* 进入描述符队列的行数 */
    uint16_t error_flags;
    uint16_t dataless_rows;   /* 行号成立但payload缺失(drop buffer/RXOVER)的行数 */
} cam_frame_diag_t;

/* ---------------------------------------------------------------------
 * 4x4-buffer (scaffold)
 *  - 预留用于将现有的 line 环（CAM_NUM_BUFFERS）替换为
 *    4 个每个包含 4 行的块（总计 16 行的环），以降低中断频率
 *    并为后续压缩算法/批处理提供更大窗口。
 *  - 本头仅提供预置常量与接口声明；实际切换须在 cam_pio.c
 *    中完成并需要验证内存占用与时序约束。
 */
#define CAPTURE_CHUNK_LINES  4u
#define CAM_4X4_NUM_BUFFERS  4u


/* 启用 4x4 scaffold（当前为预留实现，不改变默认line-buffer行为） */
void cam_enable_4x4_scaffold(void);

extern volatile uint8_t  frame_ready;       /* VSYNC 帧边界标记，供 IMU 按帧采样 */
extern volatile uint32_t cam_overrun_count; /* 缓冲/描述符资源不足累计次数 */
extern volatile uint32_t cam_descriptor_overrun_count;
extern volatile uint32_t cam_pio_rxover_count;
extern volatile uint32_t cam_href_error_count;
extern volatile uint32_t cam_line_end_count;      /* PIO确认的HREF行总数 */
extern volatile uint32_t cam_line_publish_count;
extern volatile uint32_t cam_dataless_row_count;  /* 行号成立但payload缺失 */
extern volatile uint32_t cam_partial_line_count;  /* VSYNC时DMA不在块边界 */
extern volatile uint32_t cam_skip_done_count;
extern volatile uint32_t cam_discarded_frame_count;
/* VSYNC 边沿判定诊断（现场用它确认耦合是否存在）：
 *   glitch  = 高电平宽度不像真 VSYNC（≈260us）的伪下降沿，通常来自电气耦合
 *   reject  = 通过了宽度判定但落在行活动期或周期过短
 *   period  = 候选边界周期超出 [50,90]ms
 *   row     = 候选周期不是恰好480行，禁止用它关帧 */
extern volatile uint32_t cam_vsync_glitch_count;
extern volatile uint32_t cam_vsync_reject_count;
extern volatile uint32_t cam_vsync_period_error_count;
extern volatile uint32_t cam_vsync_row_error_count;
extern volatile uint32_t cam_startup_error_count;
extern volatile uint32_t cam_frame_count;
extern volatile cam_frame_diag_t cam_last_frame_diag;


/* ── 函数声明 ─────────────────────────────────────────────────────────────── */

/* 设置数据/PCLK/HREF/VSYNC 引脚方向与上拉，配置调试输出引脚 */
void cam_gpio_init(void);

/* 加载 PIO 程序，配置并预装行计数器（不使能状态机） */
void cam_pio_init(void);

/* 申请 DMA 通道、绑定 PIO RX DREQ、注册 DMA 完成中断与 VSYNC 中断（不启动传输） */
void cam_dma_init(void);

/* 复位采集状态，完成PIO/CPU握手并稳定跳过三个完整物理帧。 */
void cam_capture_start(void);

/* 关闭状态机并终止 DMA，清空 FIFO */
void cam_capture_stop(void);

/* Core0单消费者接口：pop不会释放图像buffer；三行窗口不再需要时显式归还。 */
bool cam_descriptor_pop(cam_descriptor_t *descriptor);
const uint8_t *cam_get_buffer(uint8_t buffer_idx);
void cam_release_buffer(uint8_t buffer_idx);

#endif /* CAM_PIO_H */
