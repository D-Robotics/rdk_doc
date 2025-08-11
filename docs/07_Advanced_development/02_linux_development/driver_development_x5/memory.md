---
sidebar_position: 1
---

# Linux 系统内存使用情况

默认情况下 Linux 系统可以访问的 DDR 范围是：

起始地址： 0x8400_0000

终止地址： 0x8000_0000 + DDR 容量

Linux Kernel 的设备树中会保留部分内存，作为某些模块的专用内存，下表列出设备树保留的所有内存：

| 内存划分 | 内存作用 | 内存区域 | 内存大小 |
| --- | ------ | ------- | ------- |
| kernel_ram | 存放内核代码和数据 | 0x8000_0000..0x87FFF_FFFF | 128MB(0x2000000) |
| uboot_log_reserved| 用于在内核获取启动日志 | 0x87FFC000..0x87FFFFFF | 16KB(0x4000) |
| cma_reserved | 用于 CMA 内存分配 | 0x88000000..0x9FDFFFFF | 382MB(0x17E00000) |
| firewall_ddr_default | 当发生违反 firewall 访问时被自动导向的地址 | 0x9FE7F000..0x9FE7FFFF | 4KB(0x1000) |
| adsp_ddr | ADSP 固件在内存使用的空间 | 0x9FE80000..0xA207FFFF | 34MB(0x2200000) |
| ramoops_reserved | 用于 ramoops 临时内存分配 | 0xA4080000..0xA40BFFFF | 256KB(0x40000) |
| ion_reserved | 用于 ION 内存分配 | 0xA4100000..0xD40FFFFF | 768MB(0x2E000000) |

- 保留的地址空间定义在 kernel 文件夹下的 arch/arm64/boot/dts/hobot/x5-memory.dtsi 文件
- 可以通过 cat /proc/iomem 命令获取系统内存的分配情况

保留内存默认情况下总共占用约 `1150MB` 的内存空间。

`free` 命令显示的可用内存已经排除了这部分保留内存，只反映内核可实际使用的内存容量。

## ION内存

### 概述

ION 系统 是 Linux 内核中用于管理设备之间内存共享的一个内存管理子系统。

RDK X5 系统中 ION 内存主要被下面四种情况使用：

- BPU 子系统
- HIFI 子系统（ DSP）
- 开发者通过 hbmem 接口分配 ION 内存
- 多媒体系统：包含的硬件加速单元有 VIN、 ISP、 VSE、 GDC、 VPU、 OSD、 GPU、 Display


RDK X5 中主要使用 3 个 ION 区域：cma_reserved、carveout、ion_cma

- cma_reserved 主要为多媒体系统提供内存
- carveout 主要为 BPU 子系统提供内存
- ion_cma 作为备用的区域，当上述两个区域没有空间时， ION 管理子系统会自动从 ion_cma 区域中申请内存

### ION 区域大小调整方法

RDK X5 通过srpi-config工具来调整 ION 区域的大小，具体操作请参考 [srpi-config](../../../System_configuration/srpi-config) ，`ION memory`一节