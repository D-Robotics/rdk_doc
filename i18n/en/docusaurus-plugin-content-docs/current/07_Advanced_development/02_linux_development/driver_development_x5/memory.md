---
sidebar_position: 1
---

# Linux System Memory Usage

By default, the DDR range accessible to the Linux system is:

Start address: 0x8400_0000

End address: 0x8000_0000 + DDR capacity

The Linux Kernel device tree reserves a portion of memory for dedicated use by certain modules. The table below lists all memory reserved in the device tree:

| Memory Partition | Memory Purpose | Memory Region | Memory Size |
| --- | ------ | ------- | ------- |
| kernel_ram | Stores kernel code and data | 0x8000_0000..0x87FFF_FFFF | 128MB (0x2000000) |
| uboot_log_reserved | Retrieves boot logs within the kernel | 0x87FFC000..0x87FFFFFF | 16KB (0x4000) |
| cma_reserved | Used for CMA memory allocation | 0x88000000..0x9FDFFFFF | 382MB (0x17E00000) |
| firewall_ddr_default | Address automatically directed when a firewall access violation occurs | 0x9FE7F000..0x9FE7FFFF | 4KB (0x1000) |
| adsp_ddr | Memory space used by ADSP firmware | 0x9FE80000..0xA207FFFF | 34MB (0x2200000) |
| ramoops_reserved | Used for ramoops temporary memory allocation | 0xA4080000..0xA40BFFFF | 256KB (0x40000) |
| ion_reserved | Used for ION memory allocation | 0xA4100000..0xD40FFFFF | 768MB (0x2E000000) |

- The reserved address space is defined in the `arch/arm64/boot/dts/hobot/x5-memory.dtsi` file under the kernel directory.
- You can view the system memory allocation status using the `cat /proc/iomem` command.

By default, the reserved memory occupies approximately `1150MB` of memory space.

The available memory displayed by the `free` command has already excluded this reserved portion, reflecting only the memory capacity actually usable by the kernel.

## ION Memory

### Overview

The ION system is a memory management subsystem within the Linux kernel used to manage memory sharing between devices.

In the RDK X5 system, ION memory is primarily used in the following four scenarios:

- BPU subsystem
- HIFI subsystem (DSP)
- ION memory allocated by developers via the hbmem interface
- Multimedia system: Includes hardware acceleration units such as VIN, ISP, VSE, GDC, VPU, OSD, GPU, and Display

Three main ION regions are used in the RDK X5: `cma_reserved`, `carveout`, and `ion_cma`.

- `cma_reserved` primarily provides memory for the multimedia system.
- `carveout` primarily provides memory for the BPU subsystem.
- `ion_cma` serves as a backup region. When the above two regions have no available space, the ION management subsystem automatically allocates memory from the `ion_cma` region.

### How to Adjust ION Region Size

The RDK X5 uses the `srpi-config` tool to adjust the size of ION regions. For specific operations, please refer to the `ION memory` section in [srpi-config](../../../System_configuration/srpi-config).