# RP2354A OV5640 相机模块

[English README](README.md) | 中文

本仓库包含 RP2354A 级控制器固件：采集 OV5640 8-bit DVP 灰度流，在双核之间完成二值 Sobel 处理，并通过 PIO/DMA 向 FPGA 发送“每图像行一个固定长度包”。本文描述 2026-08-18 里程碑后的有效链路；早期 XOR 参考帧与质心机制不属于当前构建。

## 当前能力

- OV5640：640×480、Y8，约 15 fps。
- PIO 对 VSYNC 资格化并完成 DVP 捕获，避免逐像素 CPU 中断。
- 启动阶段消费四个合格帧边界，从第四个有效帧开始输出。
- 8-slot 原始行 ring 与 8-slot 处理行 ring 连接 DMA、Core0 和 Core1。
- Core0 使用三行窗口计算 Sobel；Core1 自适应阈值化并把 640 pixel 打包为 80 byte。
- PIO1/DMA 提供 8-bit FPGA 数据总线与 12 MHz byte clock。
- 每行包固定 128 byte，每帧 480 个包。

## 有效数据通路

```text
OV5640 DVP (D/PCLK/HREF/VSYNC)
  -> PIO0 VSYNC qualification
  -> PIO0 capture + DMA
  -> 8-slot raw-line ring
  -> Core0: three-line Sobel
  -> 8-slot Sobel ring + multicore FIFO notification
  -> Core1: threshold + 1-bit packing + row packet
  -> PIO1 + DMA: 8-bit data / byte clock / packet HREF
  -> FPGA receiver
```

`vsync_gate` 以 36 MHz 采样 VSYNC，只有连续 32 次为高才接受候选边界，对应约 58.7 µs 的资格窗口。相机 capture 状态机随后消费一个对齐边界和三个完整跳过帧；进入持续捕获后以 HREF/PCLK 处理行，不在每帧用 VSYNC 重新对齐。

每行 640 byte 经 DMA 写入八槽 ring。ring 满时捕获保持非阻塞，overrun counter 增加并复用当前槽。Core0 对三行窗口做 Sobel；row0/row1 输出零边界，真正 Sobel output 从 row2 开始。Core1 经 multicore FIFO 收到行通知，每帧调整阈值，把结果压成 1 bit/pixel 并组包。

PIO1 运行在 48 MHz，每 byte 使用四个 state-machine cycle，因此 byte clock 为 12 MHz。8-bit data 按 MSB first 输出。GPIO9 在完整 128-byte packet 期间保持高，并在最后一个 byte 真正离开 PIO 后降低。

## 硬件配置

| 项目 | 当前值 |
|---|---|
| Target | RP2350 platform / Pico 2 board |
| 系统与 peripheral clock | 144 MHz |
| OV5640 XCLK | 24 MHz |
| Camera mode | 640×480 Y8 |
| DVP PCLK | 约 12 MHz |
| Sensor timing | HTS 1562、VTS 512、约 15.006 fps |
| FPGA byte clock | 12 MHz |
| Packet rate | 每帧 480 包 |

## 引脚

| 功能 | GPIO |
|---|---|
| FPGA D0-D7 | GPIO0-GPIO7 |
| FPGA byte clock | GPIO8 |
| FPGA packet HREF/envelope | GPIO9 |
| OV5640 VSYNC | GPIO10 |
| OV5640 PCLK | GPIO11 |
| OV5640 D0-D7 | GPIO12-GPIO19 |
| OV5640 HREF | GPIO20 |
| OV5640 XCLK | GPIO21 |
| OV5640 RESET | GPIO22 |
| SCCB SDA/SCL | GPIO26/GPIO27 |
| OV5640 PWDN | GPIO28 |

## 128-byte 行包协议

所有多字节数值均为大端。

| Offset | Size | 字段 | 当前语义 |
|---:|---:|---|---|
| 0-1 | 2 | Sync0 | `A5 A0` |
| 2-3 | 2 | Sync1 | `5A 50` |
| 4 | 1 | Camera ID | MCU 当前写 0；FPGA 可按物理入口覆盖 |
| 5-6 | 2 | Frame ID | 每帧递增 |
| 7-8 | 2 | Row index | 0..479 |
| 9 | 1 | Row flags | bit0 overflow；bit1 final row；bit2 first processed row |
| 10 | 1 | Payload length | 80 |
| 11-12 | 2 | Row sequence | 全局行包序列 |
| 13-23 | 11 | Reserved | MCU 写 0；FPGA 可独立写 status |
| 24-103 | 80 | Binary payload | 640 pixel，1 bit/pixel，MSB first |
| 104-113 | 10 | Padding | 0 |
| 114-125 | 12 | Trailer sync | `A5 5A` 重复六次 |
| 126-127 | 2 | CRC | CCITT-FALSE over 0..125，大端 |

offset9 bit2 不是 frame-start；它标记 row2，即第一个拥有完整三行 Sobel window 的处理行。接收端必须用 `row_idx==0` 判断真正帧首行。

CRC-16/CCITT-FALSE 参数为 polynomial `0x1021`、initial `0xFFFF`、不反射、final XOR `0x0000`。MCU 对 offset0..125 计算并以大端写 126/127。若 FPGA 改写 offset4、13 或其他 CRC 覆盖字节，就必须在转发前重新计算出站 CRC。一帧在线路上包含 `480×128=61,440` byte，没有独立 metadata packet。

## 构建

前置条件：CMake 3.13+、Ninja（或其他 CMake generator）、Arm GNU embedded toolchain、兼容 Pico SDK 2.2.0 的环境。

tracked `pico_sdk_import.cmake` 通过 `PICO_SDK_PATH` 导入外部 SDK；不要把 SDK 复制进本仓库。

```powershell
$repo = 'D:\prg\blank_project\MCU_Camera_Module'
$env:PICO_SDK_PATH = 'C:\Users\<USER>\.pico-sdk\sdk\2.2.0' # <- 本机 SDK
Set-Location $repo
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350
if ($LASTEXITCODE -ne 0) { throw 'MCU configure 失败' }
cmake --build build --config Release
if ($LASTEXITCODE -ne 0) { throw 'MCU build 失败' }
```

可烧写文件为 `build/new_camera_project_app.uf2`。USB stdio 打开，UART stdio 关闭。

## 源码布局

| 路径 | 职责 |
|---|---|
| `main.c` | clock/init、双核 pipeline、threshold 与 packet scheduling |
| `cam_pio.pio` | VSYNC gate 与 OV5640 DVP capture SM |
| `fpga_pio.pio` | FPGA byte-output SM |
| `func/cam_pio.c` | PIO/DMA、ring ownership、capture recovery |
| `func/fpga_pio.c` | FPGA PIO/DMA 与 packet HREF completion |
| `func/image_process.c` | Sobel、threshold、packing、packet generation |
| `func/ov5640*.c` | sensor 初始化、控制和寄存器表 |
| `header/` | 接口和硬件常量 |

IMU/HSTX 文件仍在源码树，但当前 `CMakeLists.txt` 没有把它们链接进活动 executable。

## 已知限制

- 长时间相机→FPGA→Host 压测仍需绑定统一 run manifest。
- offset13 的 FPGA diagnostic 所有权必须与 MCU reserved 语义保持分离。
- 若长期运行发现帧边界漂移，需要受控 VSYNC realignment 设计与验证。
- 当前单一共享输出 packet buffer 限制更深的处理/发送重叠。
- 4×4 scaffold 与 RLE helper 不是当前 active compression feature。

## 跨仓库指南

- [MCU 架构与代码](https://github.com/YuweiZhang-002/FPGA-Based-Camera-Buffer/blob/main/docs/ZH/01_mcu_architecture_and_code_guide.zh-CN.md)
- [MCU 构建、运行与调试](https://github.com/YuweiZhang-002/FPGA-Based-Camera-Buffer/blob/main/docs/ZH/02_mcu_build_run_and_debug_guide.zh-CN.md)

协议判定仍以当前 `packet_generator()` 和本 README 的 packet table 为准；跨仓库教程不能覆盖实际发出的 byte。

## 许可证

Yuwei Zhang 的原创代码采用 [BSD 3-Clause License](LICENSE)。带上游声明的文件仍服从相应上游条款；发布前阅读 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 并保留所需声明。
