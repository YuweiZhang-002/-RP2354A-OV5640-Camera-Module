# RP2354A OV5640 Camera Module

English | [Chinese README](README.zh-CN.md)

This repository contains firmware for an RP2354A-class controller that captures an OV5640 8-bit DVP stream, performs binary Sobel edge processing across both CPU cores, and sends one fixed-size packet per image row to an FPGA through PIO and DMA.

This document describes the source tree at the 2026-08-18 milestone (`8e12162`). The current data path no longer contains the earlier XOR reference-frame or centroid mechanism.

## Current capabilities

- OV5640 acquisition at 640 x 480, 8-bit grayscale (Y8), approximately 15 fps.
- PIO-based VSYNC qualification and DVP capture instead of per-pixel CPU interrupts.
- Four qualified frame boundaries are consumed at startup, so image output begins with the fourth valid frame.
- Eight-slot raw-line and processed-line rings connect capture, Core 0, and Core 1.
- Core 0 performs a three-line Sobel operation.
- Core 1 applies an adaptive threshold, packs 640 binary pixels into 80 bytes, and builds the row packet.
- PIO and DMA provide an 8-bit FPGA output bus with a 12 MHz byte clock.
- The wire protocol uses exactly 128 bytes per row and 480 row packets per frame.

## Data path

```text
OV5640 DVP
  |  GPIO data / PCLK / HREF / VSYNC
  v
PIO0 VSYNC gate ----> qualified startup frame boundaries
  |
PIO0 camera capture + DMA
  |
8-slot raw-line ring
  |
Core 0: 3-line Sobel filter
  |
8-slot Sobel result ring + multicore FIFO row notification
  |
Core 1: adaptive threshold + 1-bit packing + packet generation
  |
PIO1 + DMA: 8-bit data, byte clock, packet HREF
  v
FPGA receiver
```

### PIO capture and startup alignment

`vsync_gate` samples VSYNC at 36 MHz and accepts a candidate only after it remains high for 32 consecutive checks. This is an approximately 58.7 us qualification window and rejects the short startup glitches observed from the sensor.

The camera capture state machine then consumes four qualified boundaries: one alignment boundary followed by three complete skipped frames. Continuous HREF/PCLK line capture starts on the fourth valid frame. After the state machine enters its capture loop, HREF and PCLK control line acquisition; it does not use VSYNC to realign each later frame.

Each 640-byte row is transferred from the PIO RX FIFO into an eight-slot DMA-backed ring. If the ring is full, capture remains non-blocking, increments the overrun counter, and reuses the current slot.

### Dual-core image processing

Core 0 acquires a three-row window and generates the Sobel result. Rows 0 and 1 are emitted as zero-filled boundary rows; Sobel output starts at row 2. Core 1 receives row notifications through the multicore FIFO, adjusts the threshold once per frame, converts the result to one bit per pixel, and builds the packet.

The current implementation has no XOR reference-frame comparison, centroid calculation, or motion-vector fields.

### FPGA output

PIO1 runs at 48 MHz and uses four state-machine cycles per byte, producing a 12 MHz byte clock. The 8-bit data bus is sent MSB first. GPIO9 is high for the complete 128-byte packet and is lowered only after the PIO reports that the final transmitted byte has left the state machine.

## Hardware configuration

| Item | Current setting |
| --- | --- |
| Target | RP2350 platform / Pico 2 board definition |
| System and peripheral clocks | 144 MHz |
| OV5640 input clock | 24 MHz |
| Camera mode | 640 x 480 Y8 |
| DVP pixel clock | Approximately 12 MHz |
| Sensor timing | HTS 1562, VTS 512, approximately 15.006 fps |
| FPGA byte clock | 12 MHz |
| Packet rate | 480 packets per image frame |

## Pin assignment

| Function | GPIO |
| --- | --- |
| FPGA data D0-D7 | GPIO0-GPIO7 |
| FPGA byte clock | GPIO8 |
| FPGA packet HREF/envelope | GPIO9 |
| OV5640 VSYNC | GPIO10 |
| OV5640 PCLK | GPIO11 |
| OV5640 D0-D7 | GPIO12-GPIO19 |
| OV5640 HREF | GPIO20 |
| OV5640 XCLK | GPIO21 |
| OV5640 RESET | GPIO22 |
| OV5640 SCCB SDA | GPIO26 |
| OV5640 SCCB SCL | GPIO27 |
| OV5640 PWDN | GPIO28 |

## 128-byte row packet

All multi-byte numeric fields are transmitted in big-endian byte order.

| Offset | Size | Field | Current value or meaning |
| ---: | ---: | --- | --- |
| 0-1 | 2 | Sync word 0 | `A5 A0` |
| 2-3 | 2 | Sync word 1 | `5A 50` |
| 4 | 1 | Camera ID | Currently `0` |
| 5-6 | 2 | Frame ID | Increments per frame |
| 7-8 | 2 | Row index | `0` to `479` |
| 9 | 1 | Row flags | Bit 0: overflow; bit 1: final row; bit 2: first processed row (row 2) |
| 10 | 1 | Payload length | `80` |
| 11-12 | 2 | Row sequence | Global row-packet sequence counter |
| 13-23 | 11 | Reserved | Currently zero |
| 24-103 | 80 | Binary edge payload | 640 pixels, one bit per pixel, MSB first |
| 104-113 | 10 | Trailer padding | Zero |
| 114-125 | 12 | Trailer synchronization | `A5 5A` repeated six times |
| 126-127 | 2 | CRC field | Currently the placeholder `FF FF` |

Bit 2 at offset 9 is not a row-0 or frame-start flag. A receiver must identify the first row from `row_idx == 0`; bit 2 marks row 2, the first row with a complete three-line Sobel window.

The firmware contains a CRC-16/CCITT calculation routine, but packet generation does not currently call it. Until CRC is explicitly enabled, bytes 126-127 remain `0xFFFF` and must be treated as a placeholder rather than a successful CRC result. Offset 13 is also reserved in the RP2354 output and is currently written as zero; any downstream FPGA status-byte replacement must be defined and validated separately.

One frame contains 480 x 128 = 61,440 wire bytes. There is no separate metadata packet in the active data path.

## Build

### Prerequisites

- CMake 3.13 or newer
- Ninja or another supported CMake generator
- Arm GNU embedded toolchain
- Pico SDK 2.2.0-compatible environment

The repository currently expects the Pico SDK in `pico-sdk-master` and imports it through `pico_sdk_import.cmake`.

```powershell
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350
cmake --build build --config Release
```

The flashable image is generated as `build/new_camera_project_app.uf2`. USB standard I/O is enabled; UART standard I/O is disabled.

The project can also be configured and built through the Raspberry Pi Pico extension in Visual Studio Code.

## Source layout

| Path | Purpose |
| --- | --- |
| `main.c` | Clock setup, peripheral initialization, multicore pipeline, thresholding, and packet scheduling |
| `cam_pio.pio` | Qualified VSYNC gate and OV5640 DVP capture state machines |
| `fpga_pio.pio` | FPGA byte-output state machine |
| `func/cam_pio.c` | Camera PIO and DMA setup, line-ring ownership, and capture recovery |
| `func/fpga_pio.c` | FPGA PIO/DMA output and packet-HREF completion handling |
| `func/image_process.c` | Sobel filtering and current pass-through processing helpers |
| `func/ov5640.c` | OV5640 initialization and sensor control |
| `func/ov5640_set.c` | OV5640 register tables and mode settings |
| `header/` | Shared interfaces and hardware constants |
| `docs/` | Architecture notes, historical packet documents, and captured traces |

IMU and HSTX-related files remain in the source tree but are not part of the active executable configured by the current `CMakeLists.txt`.

## Development timeline

| Date | Commit | Milestone |
| --- | --- | --- |
| 2026-06-22 | `64d476b` | Initial multi-camera repository structure |
| 2026-07-11 | `0110ddb` | Pin restoration and source cleanup |
| 2026-07-18 | `9e55e53` | Frame-capture alignment and v6 architecture documentation |
| 2026-07-28 | `9b75d65` | Bit-order and HREF output corrections |
| 2026-08-05 | `da9990b` | Startup frame-skip correction |
| 2026-08-11 | `8233e57` | Boundary experiment retired in preparation for restoration |
| 2026-08-18 | `8e12162` | PIO alignment update and removal of XOR centroid processing |

## Known limitations and next steps

- Enable CRC-16/CCITT in packet generation and define receiver-side failure signaling after the protocol is frozen.
- Define whether offset 13 remains reserved or becomes a versioned FPGA diagnostic byte.
- Add post-startup VSYNC monitoring or controlled realignment if long-running tests show frame-boundary drift.
- Run long-duration camera-to-FPGA-to-host stress tests and record row jumps, duplicates, overruns, and packet errors.
- Replace the single shared output packet buffer if processing must overlap more deeply with FPGA transmission.
- The 4 x 4 buffering scaffold and RLE helper are not active compression features in the present build.

## Reference documents

- [System architecture report](docs/system_architecture_report.md)
- [Image structure v6](docs/img_struct_v6.md)
- [Image-processing notes](docs/img_proc_v1.md)

Some files under `docs/` describe earlier protocol revisions. For the active firmware behavior, use the current source and the packet table in this README as the authoritative reference.
