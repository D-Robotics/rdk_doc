---
sidebar_position: 1
---

# devmem

devmem is a command in busybox. It can read and write the values of hardware registers by using the mmap function on the mmap method of the /dev/mem driver, which maps the device's memory to the user space and enables read and write operations on these physical addresses.

## Syntax

```
devmem ADDRESS [WIDTH [VALUE]]
  
  Read/write from physical address

        ADDRESS Address to act upon
        WIDTH   Width (8/16/...)
        VALUE   Data to be written
```

- **ADDRESS:** The physical address to perform the operation on. This is a required parameter used to specify the address to read from or write to.
- **WIDTH:** Optional parameter that indicates the width of the data. It can be specified as 8, 16, or 32 to specify the width of the data to be read or written. If this parameter is not provided, the default width is 32 bits.
- **VALUE:** Optional parameter that represents the data value to be written. If the `WIDTH` parameter is provided, the `VALUE` should match the specified width. If `VALUE` is not provided, the command will perform a read operation.

------

## Common Commands

- Read register

```shell
Read 32-bit: devmem 0xa600307c 32
Read 16-bit: devmem 0xa600307c 16
Read 8-bit: devmem 0xa600307c 8
```

- Write register

```shell
Write 32-bit: devmem 0xa6003078 32 0x1000100
Write 16-bit: devmem 0xa6003078 16 0x1234
Write 8-bit: devmem 0xa6003078 8 0x12
```