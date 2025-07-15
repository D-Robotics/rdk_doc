# 硬件信息

Hbmem所有的heap大小都可以在dts中对其进行修改配置，但是需要给系统保留足够的内存

## S100X支持的内存大小和模式

目前S100X支持12G/24G两种内存大小。如下所示为不同模式下的地址空间：

| model             | 地址空间                                      |
|-------------------|---------------------------------------------|
| 12G interleave    | 0x80000000~0xFFFFFFFF                       |
|                   | 0x400000000~0x4FFFFFFFF                     |
|                   | 0x800000000~0x8FFFFFFFF                     |
|                   | 0xC80000000~0xCFFFFFFFF                     |
| 24G interleave    | 0x80000000~0xFFFFFFFF                       |
|                   | 0x400000000~0x5FFFFFFFF                     |
|                   | 0x800000000~0x9FFFFFFFF                     |
|                   | 0xC80000000~0xDFFFFFFFF                     |

## S100X 默认的ION预留内存

对于用户来说，S100X默认支持3种heap类型，carveout/cma/cma_reserved，其均采用预留内存的方式分配，分配速度较快；如下所示为不同内存模式下默认预留的heap大小：

| model             | ION_HEAP_TYPE_CMA_RESERVED(cma_reserved)     | ION_HEAP_TYPE_CARVEOUT(carveout)            | ION_HEAP_TYPE_DMA(cma)                     |
|-------------------|----------------------------------------------|---------------------------------------------|--------------------------------------------|
| 12G interleave    | 0x0000000400000000..0x000000043FFFFFFF(1GiB)  | 0x0000000440000000..0x000000045FFFFFFF(512MiB)  | 0x0000000460000000..0x000000047FFFFFFF(512MiB)  |
| 24G interleave    | 0x0000000400000000..0x000000043FFFFFFF(1GiB)  | 0x0000000440000000..0x000000045FFFFFFF(512MiB)  | 0x0000000460000000..0x000000047FFFFFFF(512MiB)  |

ION的使用说明，请参考[heap介绍](./03_s100_hbmem_software.md#heap介绍)

### 12G interleave模式下的memory map

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/02_12G_interleve_memory_layout.png)

### 24G interleave模式下的memory map

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/02_24G_interleve_memory_layout.png)
