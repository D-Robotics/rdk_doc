---
sidebar_position: 10
---

# Thermal 系统

## 温度传感器

在X3芯片上有一个温度传感器，直接反应的是X3芯片DIE的温度。

在/sys/class/hwmon/下有hwmon0目录下包含温度传感器的相关参数。

重要文件：name和temp1_input.

-   name是指温度传感器的名字。
-   temp1_input是指温度的值，默认精度是0.001摄氏度。

```
# cat /sys/class/hwmon/hwmon0/name
pvt_ts
# cat /sys/class/hwmon/hwmon0/temp1_input
55892
# 55892 代表55.892摄氏度
```

这个hwmon0设备的温度直接作用到  cat /sys/class/thermal/thermal_zone0/temp 设备，两者的数值是一摸一样。

## Thermal

Linux Thermal 是 Linux 系统下温度控制相关的模块，主要用来控制系统运行过程中芯片产生的热量，使芯片温度和设备外壳温度维持在一个安全、舒适的范围。

要想达到合理控制设备温度，我们需要了解以下三个模块：

- 获取温度的设备：在 Thermal 框架中被抽象为 Thermal Zone Device，这个就是温度传感器 thermal_zone0；
- 需要降温的设备：在 Thermal 框架中被抽象为 Thermal Cooling Device，有CPU和BPU；
- 控制温度策略：在 Thermal 框架中被抽象为 Thermal Governor;

以上模块的信息和控制都可以在 /sys/class/thermal 目录下获取。

在x3里面一共有三个cooling(降温)设备：

- cooling_device0: cnn0
- cooling_device1: cnn1
- cooling_device2: cpu

目前默认的策略通过以下命令可知是使用的 step_wise。

```
cat /sys/class/thermal/thermal_zone0/policy
```

 通过以下命令可看到支持的策略：user_space、step_wise一共两种。

```
cat /sys/class/thermal/thermal_zone0/available_policies
```

- user_space 是通过uevent将温区当前温度，温控触发点等信息上报到用户空间，由用户空间软件制定温控的策略。
- step_wise 是每个轮询周期逐级提高冷却状态，是一种相对温和的温控策略

具体选择哪种策略是根据产品需要自己选择。可在编译的时候指定或者通过sysfs动态切换。

例如：动态切换策略为 user_space模式

```
echo user_space > /sys/class/thermal/thermal_zone0/policy 
```

执行以下命令可看到有三个trip_point（触发温度）。

```
ls -l  /sys/devices/virtual/thermal/thermal_zone0
```

目前默认选择的trip-point是trip_point_1_temp（温度是75度）。

```
trip_point_*_hyst (*:0 - 2) # 滞后温度
trip_point_*_temp (*: 0 - 2) # 触发温度
trip_point_*_type (*: 0 - 2) # 触发点的类型
```

如果想要结温到85摄氏度才去降频：

```
echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
```

如果想要调整关机温度为105摄氏度： 

```
echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp
```

ps：以上设置断电重启后需要重新设置

## thermal参考文档

kernel/Documentation/thermal/
