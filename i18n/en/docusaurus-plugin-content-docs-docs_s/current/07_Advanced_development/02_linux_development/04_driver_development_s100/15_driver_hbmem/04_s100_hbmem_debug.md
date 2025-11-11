# Debug Information

Hbmem supports querying relevant debug information via sysfs nodes.

## How to Check Current ION Memory Allocation Status

```shell
cat /sys/kernel/debug/ion/heaps/ion_cma
cat /sys/kernel/debug/ion/heaps/cma_reserved
cat /sys/kernel/debug/ion/heaps/carveout
cat /sys/kernel/debug/ion/heaps/all_heap_info
```

## How to Check Current ION Reserved Memory Status

The reserved memory status for ION_HEAP_TYPE_CARVEOUT (HB_MEM_USAGE_PRIV_HEAP_RESERVED), ION_HEAP_TYPE_CMA_RESERVED (HB_MEM_USAGE_PRIV_HEAP_2_RESERVED), and ION_HEAP_TYPE_DMA (HB_MEM_USAGE_PRIV_HEAP_DMA) heaps can also be found in the boot logs, as shown below. The first line indicates the start address and size of ION_HEAP_TYPE_CARVEOUT; the second line shows those of ION_HEAP_TYPE_CMA_RESERVED; and the third line shows those of ION_HEAP_TYPE_DMA:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/hbmem/03_ion_region_kernel_print.png)

## How to Read/Write Memory Directly

### Using the devmem tool to access memory directly

```shell
root@ubuntu:~# devmem -h
BusyBox v1.30.1 (Ubuntu 1:1.30.1-7ubuntu3.1) multi-call binary.

Usage: devmem ADDRESS [WIDTH [VALUE]]

Read/write from physical address

        ADDRESS Address to act upon
        WIDTH   Width (8/16/...)
        VALUE   Data to be written
```

### Writing directly to memory

```shell
root@ubuntu:~# devmem 0xE0000000 32 0x12345678
root@ubuntu:~#
```

### Reading memory

```shell
root@ubuntu:~# devmem 0xE0000000
0x12345678
root@ubuntu:~#
```