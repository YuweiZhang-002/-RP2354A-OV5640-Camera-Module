# New Camera Project / 新摄像头项目

This project is a bare-metal Raspberry Pi Pico / RP2350 camera bridge for an OV5640 DVP sensor and an FPGA-side output path.
这个项目是一个基于 Raspberry Pi Pico / RP2350 的裸机摄像头桥接工程，用于连接 OV5640 DVP 传感器和 FPGA 输出链路。

## Dataflow / 数据流

- 中文：OV5640 通过 PIO0 + DMA 完成并行图像采集，图像行先进入三缓冲环，再由主循环交给 PIO1 发送到 FPGA。
- English: OV5640 data is captured by PIO0 + DMA, stored in a three-buffer line ring, and then handed to PIO1 for output to the FPGA.

- 中文：当前发送链使用双 DMA / 队列式机制，减少逐行发送时的空档和 stall，保证行间时序更稳定。
- English: The output path uses a dual-DMA / queue-based mechanism to reduce stalls and line gaps, keeping timing more stable.

## Implementation / 实现机制

- 中文：采集侧由 `cam_pio.c` 负责 GPIO 初始化、PIO 程序加载、DMA 完成中断和行级缓冲管理；发送侧由 `fpga_pio.c` 负责 PIO1、DMA 和数据搬运。
- English: The capture side is handled by `cam_pio.c` for GPIO setup, PIO loading, DMA completion IRQs, and line-buffer management; the output side is handled by `fpga_pio.c` for PIO1, DMA, and data transfer.

- 中文：OV5640 的初始化和寄存器配置在 `ov5640.c` / `ov5640_set.c` 中完成，`main.c` 只做顶层编排和链路衔接。
- English: OV5640 initialization and register setup live in `ov5640.c` / `ov5640_set.c`, while `main.c` only orchestrates the top-level flow and connects the chains.

- 中文：设计参考以 `docs/DESIGN.md` 为准，当前代码优先保证采集、发送和时序控制的可维护性。
- English: The design reference is `docs/DESIGN.md`, and the current code focuses on maintainable capture, output, and timing control.

## Update / 更新
- (7/11)
- 中文：今天整理并恢复了当前版本中的引脚与调试残留，FPGA 发送桥路回到 GPIO0-7 数据、GPIO8 时钟；临时调试逻辑已清理，采集停止函数也已补回。
- English: Today’s cleanup restored the current pin mapping and removed debug leftovers. The FPGA bridge is back to GPIO0-7 for data and GPIO8 for clock; the temporary debug logic was removed, and the capture-stop function was restored.

- 中文：同时保留了当前的双 DMA 发送机制，用来减少行与行之间的随机空档。
- English: The current dual-DMA output mechanism was kept to reduce random gaps between lines.

- (7/13)
- 中文：预留给后续三天的修改记录，当前版本先保留这一段作为每天变更的落点。
- English: Reserved for the next three days of change notes; this section is kept as a daily log anchor for upcoming updates.

- (7/14)
- 中文：预留。
- English: Reserved.

- (7/15)
- 中文：预留。
- English: Reserved.

- (7/16)
- 中文：本轮把图像处理链进一步拆分为 Sobel 计算、Threshold 比对/写入和 Filter 打包三段，并把 GPIO9 探针切成独立窗口，方便分别观测各阶段耗时。
- English: This round further split the image-processing chain into Sobel, Threshold compare/write, and Filter packing stages, and separated the GPIO9 probe windows so each phase can be timed independently.

- 中文：同时把 Sobel 中间结果与二值化结果分离到不同的行缓冲中，Core 1 侧继续承担 XOR、参考帧更新和 packet 发送，便于后续按预算判断是否将 Threshold 整块迁移到 Core 1。
- English: Sobel intermediates and binarized rows were also moved into separate line buffers, while Core 1 continues to handle XOR, reference-frame updates, and packet sending, making it easier to decide later whether Threshold should move wholesale to Core 1 based on timing budget.

- (7/18)
- 中文：采集启动改为由 PIO 在入口等待两个完整 VSYNC 周期，严格跳过用于稳定的第一帧，并从第二帧开始进入持续 HREF/PCLK 采样；进入运行循环后 VSYNC 不再控制采集。
- English: Capture startup now waits through two complete VSYNC boundary pairs in PIO, strictly skipping the first stabilization frame and starting continuous HREF/PCLK sampling from the second frame; VSYNC no longer controls capture after the runtime loop begins.

- 中文：行交接统一为 `cam_acquire_line()` / `cam_release_line()` 三行滑动窗口。每帧行 0、1 作为 80-byte `0x00` 白边发送，行 2…479 执行 Sobel、Threshold、XOR 和组包，输出保持完整 480 行。
- English: Line ownership now uses the `cam_acquire_line()` / `cam_release_line()` three-line sliding window. Rows 0 and 1 are transmitted as 80-byte `0x00` borders, while rows 2 through 479 run Sobel, thresholding, XOR, and packet generation, preserving a complete 480-row output.

- 中文：行包已固定为 24-byte header + 80-byte payload + 24-byte trailer，共 128 byte，并由静态断言约束布局；当前 CRC 字段固定写入 `0xFFFF`，等待 FPGA 侧计算。当前发送实现经源码确认是单 DMA，并由 `fpga_tx_busy` 串行保护共享 `packet_buf`。
- English: The row packet is fixed at a 24-byte header, 80-byte payload, and 24-byte trailer (128 bytes total), with static assertions enforcing the layout. The CRC field is currently written as `0xFFFF` for FPGA-side calculation. Source review confirms that the current transmitter uses one DMA channel and serializes access to the shared `packet_buf` with `fpga_tx_busy`.

- 中文：新增 `docs/img_struct_v6.md`，用 Mermaid 记录当前采集、双核处理、状态量所有权和 FPGA PIO 发送链，并明确列出仍需确认的帧边界、CRC、占位接口和调试配置。
- English: Added `docs/img_struct_v6.md`, using Mermaid to document the current capture, dual-core processing, state ownership, and FPGA PIO transmit path, together with explicit open questions around frame boundaries, CRC, placeholder interfaces, and debug configuration.

## Build / 编译

- 中文：在 VS Code 中可直接运行 `Compile Project` 任务，或使用当前 build 目录继续 Ninja 构建。
- English: In VS Code, you can run the `Compile Project` task directly, or continue building with Ninja from the existing build directory.
