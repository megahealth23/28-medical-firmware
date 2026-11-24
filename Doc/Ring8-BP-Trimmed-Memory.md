# 240325 血压算法首次瘦身后的内存评估

以 240325 的 `mega_bp_step.c` 和 `mega_bp_step_trimmed.c` 为输入，使用 SES 5.68 自带 Clang 和 SEGGER RTL 头文件分别重新编译 ARM 对象。目标为 Cortex-M4、Thumb、FPv4-SP hard-float，使用 `-O2 -DNDEBUG -ffunction-sections -fdata-sections -fstack-usage`。对比基准见 [Ring8 Release 内存基线](Ring8-Release-Memory.md)。

| 对象段 | 原版 | 裁剪版 | 减少 |
| --- | ---: | ---: | ---: |
| .text | 59,216 B | 50,262 B | 8,954 B |
| .rodata | 4,826 B | 4,826 B | 0 B |
| Flash 小计 | 64,042 B | 55,088 B | 8,954 B（14.0%） |
| 静态 RAM（.bss） | 9,529 B | 9,403 B | 126 B（1.3%） |

首次瘦身主要减少代码，常量表和主要静态工作区仍保留。裁剪版最大的静态对象为 `c_state`（2,500 B）、`B_M_Y`（1,444 B）、`ham_win`（800 B）、`vect_act_makor`（768 B）。对象未引用 malloc、calloc、realloc 或 free，仍引用数学库和双精度运行库符号。

| Ring8 预算 | 当前余量 | 裁剪版对象占用 | 按对象粗扣后的余量 |
| --- | ---: | ---: | ---: |
| Flash，到 FDS 起点 | 179,048 B | 55,088 B | 123,960 B（121.05 KiB） |
| RAM，现有 Heap/Stack 之外 | 53,888 B | 9,403 B | 44,485 B（43.44 KiB） |

上述余量基于 SES 整包脚本采用的 Bootloader 布局。最终增量还取决于数学库复用、段对齐和链接裁剪，Flash 还要保留实际 DFU 所需空间。

## 栈是接入约束

裁剪版 `mega_bp_step_trimmed()` 的单帧用栈为 7,296 B，函数内会调用 `c_fft4plot()`，后者单帧 2,128 B。该调用链两帧合计 **9,424 B**，超过原 8 KiB 栈配置 1,232 B。新 Release MAP 已确认栈为 16,384 B，这两帧之外还有 **6,960 B** 给其他栈开销，包括调用者、被调库函数和中断现场。原版的入口单帧为 4,080 B，独立的 `bp_algo()` 单帧为 7,208 B，两帧合计 11,288 B。裁剪版在本次编译中将 `bp_algo()` 内联进入口，因此入口单帧需结合调用链解读。

新 Release MAP 中尚无血压算法符号。接入后需从最终 MAP 核对实际内存增量，并在覆盖 FFT 路径和中断活动时测量栈高水位。

算法源码未改动。本次完成对象级编译和资源核对，未进行 Ring8 整体链接或板上测量。现有主机对比工具的 Cygwin 进程启动受沙箱限制，数据回放未复跑。
