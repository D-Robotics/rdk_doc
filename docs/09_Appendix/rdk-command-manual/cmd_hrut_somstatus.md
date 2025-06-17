---
sidebar_position: 1
---

# hrut_somstatus

**hrut_somstatus** 命令可以获取温度传感器温度、CPU\BPU的运行频率以及BPU负载。

## 语法说明

```
sudo hrut_somstatus
```

## 常用命令

```shell
sunrise@ubuntu:~$ sudo hrut_somstatus
=====================1=====================
temperature-->
        CPU      : 61.3 (C)
cpu frequency-->
              min       cur     max
        cpu0: 240000    240000  1800000
        cpu1: 240000    240000  1800000
        cpu2: 240000    240000  1800000
        cpu3: 240000    240000  1800000
bpu status information---->
             min        cur             max             ratio
        bpu0: 400000000 1000000000      1000000000      0
        bpu1: 400000000 1000000000      1000000000      0
```

**temperature（温度）**：

- **CPU**：表示 CPU 温度，当前值为 61.3 摄氏度（C）。

**cpu frequency（CPU 频率）**：

- `min`：CPU 可运行的最低频率。
- `cur`：CPU 的当前运行频率。
- `max`：CPU 可运行的最大频率。
- 这些信息表示了每个 CPU 核心的频率范围，包括最小、当前和最大频率。

**bpu status information（BPU 状态信息）**：

- `min`：BPU 可运行的最低频率。
- `cur`：BPU 的当前运行频率。
- `max`：BPU 可运行的最大频率。
- `ratio`：BPU运行时的负载率。
- 这些信息表示了 BPU 的频率范围，包括最小、当前和最大频率和负载。
