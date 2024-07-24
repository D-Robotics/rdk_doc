---
sidebar_position: 1
---

# devmem

devmem是busybox中的一个命令。可以读写硬件寄存器的值，通过mmap函数对/dev/mem驱动中的mmap方法的使用，映射了设备的内存到用户空间，实现对这些物理地址的读写操作。

## 语法说明

```
devmem ADDRESS [WIDTH [VALUE]]
  
  Read/write from physical address

        ADDRESS Address to act upon
        WIDTH   Width (8/16/...)
        VALUE   Data to be written
```

- **ADDRESS：**要执行操作的物理地址。这是一个必需参数，用于指定要读取或写入的地址。
- **WIDTH：**可选参数，表示数据的位宽。可以指定为8、16、32，用于指定读取或写入的数据位宽。如果未提供此参数，默认为32位。
- **VALUE**：可选参数，表示要写入的数据值。如果提供了 `WIDTH` 参数，`VALUE` 应该与指定位宽相匹配。如果不提供 `VALUE`，则命令将执行读取操作。

------

## 常用命令

- 读寄存器

```shell
读32位: devmem 0xa600307c 32
读16位: devmem 0xa600307c 16
读8位: devmem 0xa600307c 8
```

- 写寄存器

```shell
写32位: devmem 0xa6003078 32 0x1000100
写16位: devmem 0xa6003078 16 0x1234
写8位: devmem 0xa6003078 8 0x12
```

