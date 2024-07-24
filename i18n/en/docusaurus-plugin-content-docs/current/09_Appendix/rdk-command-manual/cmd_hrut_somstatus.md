---
sidebar_position: 1
---
# hrut_somstatus

The **hrut_somstatus** command can obtain the temperature of the temperature sensor, the operating frequency of the CPU/BPU, and the BPU load.

## Syntax

```
sudo hrut_somstatus
```

## Common Commands

```shell
sunrise@ubuntu:~$ sudo hrut_somstatus
=====================1=====================
temperature -->
        CPU      : 61.3 (C)
cpu frequency -->
              min       cur     max
        cpu0: 240000    240000  1800000
        cpu1: 240000    240000  1800000
        cpu2: 240000    240000  1800000
        cpu3: 240000    240000  1800000
bpu status information ---->
             min        cur             max             ratio
        bpu0: 400000000 1000000000      1000000000      0
        bpu1: 400000000 1000000000      1000000000      0
```

**temperature**:

- **CPU**: Represents the CPU temperature, with the current value being 61.3 degrees Celsius (C).

**cpu frequency**:

- `min`: The minimum frequency at which the CPU can operate.
- `cur`: The current operating frequency of the CPU.
- `max`: The maximum frequency at which the CPU can operate.
- These information represent the frequency ranges for each CPU core, including the minimum, current, and maximum frequencies.

**bpu status information**:

- `min`: The minimum frequency at which the BPU can operate.
- `cur`: The current operating frequency of the BPU.
- `max`: The maximum frequency at which the BPU can operate.
- `ratio`: The load ratio of the BPU during operation.
- These pieces of information indicate the frequency range of the BPU, including the minimum, current, and maximum frequency and load.