---
sidebar_position: 1
---

# rdkos_info

**rdkos_info命令** 用于一次性收集`RDK`系统的软件、硬件版本，驱动加载清单，RDK软件包安装清单和最新的系统日志，方便用户快速获取当前系统的状态信息。

## 语法说明

```
sudo rdkos_info [options]
```

## 选项说明

选项都是可选的，非必须。如果不带任何选项参数运行，`rdkos_info`会默认安装简洁模式输出信息。

- `-b`：基础输出模式，不会收集系统日志。
- `-s`：简洁输出模式（默认），输出30行最新的系统日志。
- `-d`：详细输出模式，输出300行最新的系统日志。
- `-v`：显示版本信息。
- `-h`：显示帮助信息。

## 常用命令

默认用法

```
sudo rdkos_info
```

部分输出如下：

```
================ RDK System Information Collection ================

[Hardware Model]:
        Hobot X3 PI V2.1 (Board Id = 8)

[CPU And BPU Status]:
        =====================1=====================
        temperature-->
                CPU      : 56.6 (C)
        cpu frequency-->
                      min       cur     max
                cpu0: 240000    1500000 1500000
                cpu1: 240000    1500000 1500000
                cpu2: 240000    1500000 1500000
                cpu3: 240000    1500000 1500000
        bpu status information---->
                     min        cur             max             ratio
                bpu0: 400000000 1000000000      1000000000      0
                bpu1: 400000000 1000000000      1000000000      0

[Total Memory]:         1.9Gi
[Used Memory]:          644Mi
[Free Memory]:          986Mi
[ION Memory Size]:      672MB


[RDK OS Version]:
        2.1.0

[RDK Kernel Version]:
        Linux ubuntu 4.14.87 #3 SMP PREEMPT Sun Nov 26 18:38:22 CST 2023 aarch64 aarch64 aarch64 GNU/Linux

[RDK Miniboot Version]:
        U-Boot 2018.09-00012-g5e7d58f7-dirty (Nov 26 2023 - 18:47:14 +0800)
```
