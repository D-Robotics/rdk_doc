---
sidebar_position: 10
---
# Low Power Mode Debugging Guide

## Overview

The power consumption of a chip is closely related to its operating voltage, clock frequency, and ambient temperature. Under a certain load, increasing voltage, clock frequency, and ambient temperature will result in higher power consumption. Therefore, to reduce power consumption, we can focus on lowering the voltage, clock frequency, and improving heat dissipation. The low-power methods discussed in this document mainly involve adjusting and controlling clock frequencies and voltages. Additionally, reducing the load can also help reduce power consumption, and customers can optimize circuits based on their product requirements. However, it's important to note that low-power modes may negatively impact performance, so the decision to enter low power mode should depend on the use case.

## DVFS Frequency and Voltage Scaling

In the X5, CPU, GPU 3D, BPU, and DDR support frequency scaling, while the CPU also supports voltage scaling.

### BPU Frequency Scaling

You can control the frequency scaling of the BPU through the following path:


```
/sys/class/devfreq/3a000000.bpu
```

Currently, the BPU supports two frequency points.
```
root@buildroot:/$ cat /sys/class/devfreq/3a000000.bpu/available_frequencies
500000000 1000000000
```

The BPU-supported governors include:

- **userspace**: Specifies the frequency based on the user's configuration.
- **performance**: Sets the highest frequency.
- **powersave**: Runs at the lowest frequency.

To adjust the BPU governor, you can use the following command:



```
echo userspace >/sys/class/devfreq/3a000000.bpu/governor
echo performance >/sys/class/devfreq/3a000000.bpu/governor
echo powersave >/sys/class/devfreq/3a000000.bpu/governor
```
If you wish to manually adjust the frequency, you can use the following command:


```
echo userspace >/sys/class/devfreq/3a000000.bpu/governor
echo 500000000 >/sys/class/devfreq/3a000000.bpu/userspace/set_freq
```

To check the current frequency, you can use the following command:


```
cat /sys/class/devfreq/3a000000.bpu/cur_freq
```

### GPU 3D Frequency Scaling

You can control the GPU 3D frequency scaling through the following path:


```
/sys/class/devfreq/3c000000.gc8000
```

Currently, the GPU 3D supports 4 frequency points:


```
cat /sys/class/devfreq/3c000000.gc8000/available_frequencies
200000000 400000000 750000000 1000000000
```
The GPU 3D-supported governors include:

- **userspace**: Specifies the frequency based on the user's configuration.
- **performance**: Sets the GPU 3D to the highest frequency.
- **powersave**: Runs at the lowest frequency.
- **simple_ondemand**: Adjusts the frequency on-demand based on the workload.

To adjust the GPU 3D governor, you can use the following command:


```
echo userspace >/sys/class/devfreq/3c000000.gc8000/governor
echo performance >/sys/class/devfreq/3c000000.gc8000/governor
echo powersave >/sys/class/devfreq/3c000000.gc8000/governor
echo simple_ondemand >/sys/class/devfreq/3c000000.gc8000/governor
```

If you wish to manually adjust the frequency for GPU 3D, you can use the following command:

```
echo userspace >/sys/class/devfreq/3c000000.gc8000/governor
echo 750000000 >/sys/class/devfreq/3c000000.gc8000/userspace/set_freq
```

To check the current frequency of GPU 3D, you can use the following command:

```
cat /sys/class/devfreq/3c000000.gc8000/cur_freq
```

### DDR Frequency Scaling

You can control the DDR frequency scaling through the following path:


```
/sys/class/devfreq/soc\:ddrc-freq
```

The supported frequency points may vary depending on the DDR chip. You can obtain the current supported frequency points with the following command:


```
root@buildroot:/$ cat /sys/class/devfreq/soc\:ddrc-freq/available_frequencies
266000000 1066000000 2133000000 3200000000
```

The DDR-supported governors include:

- **userspace**: Specifies the frequency based on the user's configuration.
- **performance**: Sets DDR to the highest frequency.
- **simple_ondemand**: Adjusts the frequency based on the DDR load.
- **powersave**: Runs DDR at the lowest frequency.

To adjust the DDR governor, you can use the following command:


```
echo userspace >/sys/class/devfreq/soc\:ddrc-freq/governor
echo performance >/sys/class/devfreq/soc\:ddrc-freq/governor
echo simple_ondemand >/sys/class/devfreq/soc\:ddrc-freq/governor
echo powersave >/sys/class/devfreq/soc\:ddrc-freq/governor
```

If you wish to manually adjust the DDR frequency, you can use the following command:


```
echo userspace >/sys/class/devfreq/soc\:ddrc-freq/governor
echo 266000000 >/sys/class/devfreq/soc\:ddrc-freq/userspace/set_freq
```

To check the current DDR frequency, you can use the following command:


```
cat /sys/class/devfreq/soc\:ddrc-freq/cur_freq
```

### CPU Frequency and Voltage Scaling

You can control the CPU frequency and voltage scaling through the following path:


```
/sys/devices/system/cpu/cpufreq/policy0/
```

The currently supported CPU frequencies can be viewed with the following command:

```
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
300000 600000 1200000 1500000
```

In addition, overclocking is supported with a frequency of 1.8 GHz.

You can enable the overclocking feature with the following command:


```
echo 1 >/sys/devices/system/cpu/cpufreq/boost
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

When overclocked to 1.8 GHz, the CPU operates at a voltage of 1V, while other frequency points correspond to a voltage of 800mV.

The currently supported CPU governors include:

- **performance**: Runs at the highest frequency.
- **ondemand**: Adjusts the frequency based on system load.
- **userspace**: Specifies the frequency based on the user's configuration.
- **powersave**: Runs at the lowest frequency.
- **schedutil**: Adjusts the frequency based on load, and is used in conjunction with the CPU scheduler.

To adjust the CPU governor, you can use the following command:


```
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo ondemand >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo userspace >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo powersave >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo schedutil >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

If you wish to manually adjust the CPU frequency, you can use the following command:


```
echo userspace >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1200000 >/sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
```

To check the current CPU frequency, you can use the following command:


```
cat /sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq
```

## CPU hotplug

When CPU cores are not in use, they can be hot-plugged (disabled) to further reduce power consumption. Except for core0, which does not support hotplug, cores 1 through 7 support hotplug.


```
echo 0 >/sys/devices/system/cpu/cpu1/online  #disable cpu1
echo 1 >/sys/devices/system/cpu/cpu1/online  #enable cpu1
```
## Power Control for Devices

Devices such as BPU, GPU, ISP, VIDEO CODEC, JPEG, and HIFI have independent power control. These devices are powered on only when in use (when the driver is opened) and powered off when not in use (when the driver is closed). This helps reduce the overall power consumption of the X5 SOC.

## Sleep and Wake-up

Sleep mode can be triggered by a long press of a button or by entering the `hobot-suspend` command. A short press of the button will wake the device up.
