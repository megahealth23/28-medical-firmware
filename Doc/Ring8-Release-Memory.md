# Ring8 Release 内存基线

基于 SEGGER Embedded Studio 的 nRF52840 工程，核对了 `Firmware/project/SES/Output/Release/Exe/Ring8.map` 和同目录的 `Ring8.hex`。RAM 数值已按 2026-11-24 15:57 的 Release 重编译产物更新。以下地址区间均为左闭右开，1 KiB = 1024 B。

## Flash（ROM）

- 芯片 Flash 共 1 MiB。应用从 `0x26000` 开始，之前的 152 KiB 留给 MBR 和 S140 6.1.1。SES 给应用配置的链接窗口为 `[0x26000, 0x100000)`，共 872 KiB。
- Release 应用链接跨度为 `[0x26000, 0x4A498)`，共 **148,632 B（145.15 KiB）**。HEX 实际携带 **148,631 B**，相差 1 B 的对齐空洞。链接窗口内账面剩余 744,296 B（726.85 KiB）。
- 主要段为 `.text` 112,968 B、`.rodata` 30,327 B、`.vectors` 512 B、`.init` 468 B。RAM 初始化数据在 Flash 另占 `.data` 4,080 B 和 `.fs_data` 40 B。其余为小段及对齐。

FDS 配置 120 个 4 KiB 页（480 KiB），另从 Bootloader 起点向下预留 3 页。应用固定使用 `[0xEE000, 0xF1000)` 作为内部用户数据区。FDS 实际边界由板上 MBR/UICR 记录的 Bootloader 地址决定。

| Bootloader 配置 | FDS 区间 | 从当前应用末尾到 FDS 的空间 |
| --- | --- | ---: |
| Debug Bootloader 的 SES 工程，起点 `0xF1000` | `[0x76000, 0xEE000)` | **179,048 B（174.85 KiB）** |
| 普通 Bootloader 的 SES 工程，起点 `0xF6000` | `[0x7B000, 0xF3000)` | 199,528 B（194.85 KiB），该布局下 FDS 与用户数据区重叠 12 KiB |

`scripts/scripts/debug_app-sd-bl-bs-full-SES.bat` 将应用 Release 与第一行的 Debug Bootloader 组合。若烧录采用这套布局，按 **174.85 KiB** 估算应用增长余量，并保留 DFU 升级所需空间。普通 Bootloader 工程还为 MBR 参数页（`0xFE000`）和 Bootloader 设置页（`0xFF000`）各留出 4 KiB。当前目录只有应用 HEX 可供核对，整包和板上 Bootloader 地址待确认。

## RAM

物理 RAM 共 256 KiB。低地址 `[0x20000000, 0x20002F00)` 为 SoftDevice 预留 11.75 KiB，应用 RAM 预算 244.25 KiB。

| 应用段 | 字节 |
| --- | ---: |
| `.fs_data` | 40 |
| `.retained_section` | 3,072 |
| `.data` | 4,080 |
| `.bss` | 162,916 |
| `.tbss` | 4 |
| `.non_init` | 1,536 |
| Heap | 8,192 |
| Stack | 16,384 |
| **合计** | **196,224** |
| **未分配** | **53,888** |

应用静态分配 **191.63 KiB**，占应用 RAM 预算约 78.5%，已计入 Heap 和 Stack 的链接预留。空闲区为 `[0x2002ED80, 0x2003C000)`，共 **52.63 KiB**。Stack 位于 `[0x2003C000, 0x20040000)`。实际运行中的栈峰值、堆高水位和 SoftDevice RAM 需求需要上板测量。

`.bss` 的主要大项为 `mem_manager.o` 的 `m_memory`（96,560 B，约占 BSS 的 59%）、`DataProcEHR.o` 的 `ppg_diff`（8,192 B），以及 `DataProcHealth.o` 的 `stress_heart_green_smooth`、`health_green`、`green_smooth`（各 4,000 B）。
