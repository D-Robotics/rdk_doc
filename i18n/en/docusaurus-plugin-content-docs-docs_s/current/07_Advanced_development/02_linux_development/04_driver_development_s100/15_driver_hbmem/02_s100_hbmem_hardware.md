# Hardware Information

The heap sizes for all Hbmem can be modified and configured in the DTS, but sufficient memory must be reserved for the system.

## Memory Sizes and Modes Supported by S100X

Currently, the S100X supports two memory sizes: 12GB and 24GB. The address spaces under different modes are shown below:

| Model             | Address Space                                |
|-------------------|---------------------------------------------|
| 12G interleave    | 0x80000000~0xFFFFFFFF                       |
|                   | 0x400000000~0x4FFFFFFFF                     |
|                   | 0x800000000~0x8FFFFFFFF                     |
|                   | 0xC80000000~0xCFFFFFFFF                     |
| 24G interleave    | 0x80000000~0xFFFFFFFF                       |
|                   | 0x400000000~0x5FFFFFFFF                     |
|                   | 0x800000000~0x9FFFFFFFF                     |
|                   | 0xC80000000~0xDFFFFFFFF                     |

## Default ION Reserved Memory on S100X

For users, the S100X defaults to supporting three heap types: carveout / cma / cma_reserved. All of them allocate memory using reserved memory, enabling faster allocation. The default reserved heap sizes under different memory modes are shown below:

| Model             | ION_HEAP_TYPE_CMA_RESERVED (cma_reserved)    | ION_HEAP_TYPE_CARVEOUT (carveout)           | ION_HEAP_TYPE_DMA (cma)                    |
|-------------------|----------------------------------------------|---------------------------------------------|--------------------------------------------|
| 12G interleave    | 0x0000000400000000..0x000000043FFFFFFF (1GiB) | 0x0000000440000000..0x000000045FFFFFFF (512MiB) | 0x0000000460000000..0x000000047FFFFFFF (512MiB) |
| 24G interleave    | 0x0000000400000000..0x000000043FFFFFFF (1GiB) | 0x0000000440000000..0x000000045FFFFFFF (512MiB) | 0x0000000460000000..0x000000047FFFFFFF (512MiB) |

For ION usage instructions, please refer to [Heap Introduction](./03_s100_hbmem_software.md#heap-introduction).

### Memory Map under 12G Interleave Mode

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/02_12G_interleve_memory_layout.png)

### Memory Map under 24G Interleave Mode

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/02_24G_interleve_memory_layout.png)