---
sidebar_position: 1
---

# dmesg

`dmesg` 命令用于查看或控制内核环形缓冲区。

kernel 会将内核启动日志存储在 ring buffer 中。您若是开机时来不及查看信息，可利用 dmesg 来查看。

## 语法说明

```
 dmesg [options]

       dmesg -C / dmesg --clear
       dmesg -c / dmesg --read-clear [options]
```

------

## 选项说明

- -c, --read-clear　显示信息后，清除 ring buffer 中的内容。
- -C, --clear　清除 ring buffer 中的内容。

## 常用命令

- 显示所有 ring buffer 中的内核日志内容

  ```
  dmesg
  ```

- 把内核日志保存到文件中

  ```
  dmesg > kernel.log
  ```

- 清空缓存日志，在调试驱动时，可以减少日志内容

  ```
  dmesg -C
  ```

  

