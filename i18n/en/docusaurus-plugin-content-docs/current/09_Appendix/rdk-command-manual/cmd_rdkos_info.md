---
sidebar_position: 1
---

# rdkos_info

The **rdkos_info command** is used to collect the software and hardware versions, driver loading list, RDK software package installation list, and the latest system log of the RDK system at once, so that users can quickly obtain the current system's status information.

## Syntax

```
sudo rdkos_info [options]
```

## Options

All options are optional. If you run `rdkos_info` without any option parameter, it will default to the concise output mode.

- `-b`: Basic output mode, does not collect system logs.
- `-s`: Concise output mode (default), outputs the 30 latest lines of the system log.
- `-d`: Detailed output mode, outputs the 300 latest lines of the system log.
- `-v`: Display version information.
- `-h`: Display help information.

## Common Usage

Default usage

```
sudo rdkos_info
```

Partial output is as follows:

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