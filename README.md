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

## Update (7/11) / 更新（7/11）

- 中文：今天整理并恢复了当前版本中的引脚与调试残留，FPGA 发送桥路回到 GPIO0-7 数据、GPIO8 时钟；临时调试逻辑已清理，采集停止函数也已补回。
- English: Today’s cleanup restored the current pin mapping and removed debug leftovers. The FPGA bridge is back to GPIO0-7 for data and GPIO8 for clock; the temporary debug logic was removed, and the capture-stop function was restored.

- 中文：同时保留了当前的双 DMA 发送机制，用来减少行与行之间的随机空档。
- English: The current dual-DMA output mechanism was kept to reduce random gaps between lines.

## Build / 编译

- 中文：在 VS Code 中可直接运行 `Compile Project` 任务，或使用当前 build 目录继续 Ninja 构建。
- English: In VS Code, you can run the `Compile Project` task directly, or continue building with Ninja from the existing build directory.