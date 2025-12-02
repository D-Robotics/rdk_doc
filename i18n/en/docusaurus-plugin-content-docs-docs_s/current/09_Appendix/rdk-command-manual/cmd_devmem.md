---
sidebar_position: 1
---

# devmem

`devmem` is a command in BusyBox. It can read from and write to hardware registers by using the `mmap` function to map device memory into user space via the `mmap` method of the `/dev/mem` driver, thereby enabling read/write operations on physical addresses.

## Syntax

```
devmem ADDRESS [WIDTH [VALUE]]
  
  Read/write from physical address

        ADDRESS Address to act upon
        WIDTH   Width (8/16/...)
        VALUE   Data to be written
```

- **ADDRESS:** The physical address on which to perform the operation. This is a required parameter specifying the address to read from or write to.
- **WIDTH:** An optional parameter indicating the data width. It can be specified as 8, 16, or 32, defining the bit-width of the data to read or write. If omitted, it defaults to 32 bits.
- **VALUE:** An optional parameter representing the data value to write. If `WIDTH` is provided, `VALUE` must match the specified width. If `VALUE` is omitted, the command performs a read operation.

------

## Common Commands

- Reading registers

```shell
Read 32-bit: devmem 0xa600307c 32
Read 16-bit: devmem 0xa600307c 16
Read 8-bit:  devmem 0xa600307c 8
```

- Writing registers

```shell
Write 32-bit: devmem 0xa6003078 32 0x1000100
Write 16-bit: devmem 0xa6003078 16 0x1234
Write 8-bit:  devmem 0xa6003078 8 0x12
```