---
sidebar_position: 11
---

# 低功耗模式调试指南

## overview

芯片的功耗与工作电压，时钟频率以及环境温度密切相关。在负载一定的情况下，电压、时钟频率、环境温度的升高会导致功耗增加。所以降低功耗可以从降低电压、降低时钟频率以及散热三个方面入手。本文中提到的低功耗方法主要是对时钟频率和电压的调节和控制。 同时，负载减轻也可以降低功耗，客户可根据自己的产品优化电路。另外低功耗会对性能有负面影响，所以根据使用场景来选择是否要进入低功耗。

## DVFS调频调压

在X5中，共有CPU/GPU 3D/BPU/DDR支持调频，CPU支持调压

### BPU 调频

通过以下路径可以控制对bpu的调频

```
/sys/class/devfreq/3a000000.bpu
```

目前BPU支持两个频点

```
root@buildroot:/$ cat /sys/class/devfreq/3a000000.bpu/available_frequencies
500000000 1000000000
```

BPU支持的governor包括

- userspace：根据客户的配置指定频率
- performance：设置为最高频率
- powersave：以最低频率运行

若要调整BPU的governor，可以通过以下命令

```
echo userspace >/sys/class/devfreq/3a000000.bpu/governor
echo performance >/sys/class/devfreq/3a000000.bpu/governor
echo powersave >/sys/class/devfreq/3a000000.bpu/governor
```

若想手动调整频率

```
echo userspace >/sys/class/devfreq/3a000000.bpu/governor
echo 500000000 >/sys/class/devfreq/3a000000.bpu/userspace/set_freq
```

查看当前频率

```
cat /sys/class/devfreq/3a000000.bpu/cur_freq
```

### GPU 3D 调频

通过以下路径可以控制对GPU 3D的调频

```
/sys/class/devfreq/3c000000.gc8000
```

目前GPU 3D支持4个频点

```
cat /sys/class/devfreq/3c000000.gc8000/available_frequencies
200000000 400000000 750000000 1000000000
```

GPU 3D支持的governor包括

- userspace：根据客户的配置指定频率
- performance：GPU 3D设置为最高频率
- powersave：以最低频率运行
- simple_ondemand：结合工作负载，按需调整频率

若要调整GPU 3D的governor，可以通过以下命令

```
echo userspace >/sys/class/devfreq/3c000000.gc8000/governor
echo performance >/sys/class/devfreq/3c000000.gc8000/governor
echo powersave >/sys/class/devfreq/3c000000.gc8000/governor
echo simple_ondemand >/sys/class/devfreq/3c000000.gc8000/governor
```

若想手动调整频率
```
echo userspace >/sys/class/devfreq/3c000000.gc8000/governor
echo 750000000 >/sys/class/devfreq/3c000000.gc8000/userspace/set_freq
```

查看当前频率
```
cat /sys/class/devfreq/3c000000.gc8000/cur_freq
```

### DDR 调频

通过以下路径可以控制对DDR的调频

```
/sys/class/devfreq/soc\:ddrc-freq
```

不同的DDR颗粒支持的频点可能不一致，通过以下命令可获取当前支持的频点

```
root@buildroot:/$ cat /sys/class/devfreq/soc\:ddrc-freq/available_frequencies
266000000 1066000000 2133000000 3200000000
```

DDR支持的governor包括

- userspace：根据客户的配置指定频率
- performance：DDR设置为最高频率
- simple_ondemand：根据DDR的负载调整频率
- powersave：以最低频率运行

若要调整ddr的governor，可以通过以下命令

```
echo userspace >/sys/class/devfreq/soc\:ddrc-freq/governor
echo performance >/sys/class/devfreq/soc\:ddrc-freq/governor
echo simple_ondemand >/sys/class/devfreq/soc\:ddrc-freq/governor
echo powersave >/sys/class/devfreq/soc\:ddrc-freq/governor
```

若想手动调整频率

```
echo userspace >/sys/class/devfreq/soc\:ddrc-freq/governor
echo 266000000 >/sys/class/devfreq/soc\:ddrc-freq/userspace/set_freq
```

查看当前频率

```
cat /sys/class/devfreq/soc\:ddrc-freq/cur_freq
```

### CPU的调频调压

通过以下路径可以控制对cpu的调频

```
/sys/devices/system/cpu/cpufreq/policy0/
```

目前支持的频率包括

```
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
300000 600000 1200000 1500000
```

除此之外，还提供了超频的功能，超频频率为1.8G。

通过以下命令可以打开超频功能

```
echo 1 >/sys/devices/system/cpu/cpufreq/boost
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

1.8G超频时，CPU工作电压为1V，其余频点对应的工作电压为800mV 目前CPU支持的governor

- performance：以最高频率执行
- ondemand：按照负载调整频率
- userspace：根据用户的设置频率
- powersave：以最低频率执行
- schedutil：按照负载调整频率，它是与CPU调度器结合来使用

若要调整CPU的governor，可以通过以下命令

```
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo ondemand >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo userspace >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo powersave >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo schedutil >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

若想手动调整频率

```
echo userspace >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1200000 >/sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
```

查看当前频率

```
cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq
```

## CPU hotplug

CPU core在不使用时，可以将其拔出，也可以进一步降低功耗，除了core0不支持hotpulg，core1~core7均支持hotplug。

```
echo 0 >/sys/devices/system/cpu/cpu1/online  #disable cpu1
echo 1 >/sys/devices/system/cpu/cpu1/online  #enable cpu1
```

## 设备的电源控制

对于BPU/GPU/ISP/VIDEO CODEC/JPEG/HIFI都是有独立的电源控制，目前这些设备只有在使用时（驱动open）上电，使用结束（驱动close）就会下电。可尽量降低X5 SOC的整体功耗。

## 休眠唤醒

休眠可以通过长按按键，也可以通过输入`hobot-suspend`指令；短按按键，设备唤醒；