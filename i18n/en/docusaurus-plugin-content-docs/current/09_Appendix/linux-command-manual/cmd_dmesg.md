---
sidebar_position: 1
---

# dmesg

The `dmesg` command is used to view or control the kernel ring buffer.

The kernel stores the kernel boot logs in the ring buffer. If you didn't have time to view the information during boot-up, you can use `dmesg` to view it.

## Syntax

```
dmesg [options]

dmesg -C / dmesg --clear
dmesg -c / dmesg --read-clear [options]
```

------

## Option Explanation

- -c, --read-clear: Display the information and then clear the contents of the ring buffer.
- -C, --clear: Clear the contents of the ring buffer.

## Common Commands

- Display all kernel log content in the ring buffer

```
dmesg
```

- Save the kernel log to a file

```
dmesg > kernel.log
```

- Clear the cached logs; useful for reducing log content when debugging drivers

```
dmesg -C
```