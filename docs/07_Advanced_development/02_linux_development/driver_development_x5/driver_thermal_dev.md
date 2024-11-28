---
sidebar_position: 9
---

# Thermal 系统

## 温度传感器

在X5上有三个温度传感器，用于显示DDR/BPU/CPU的温度 在/sys/class/hwmon/下有hwmon0目录下包含温度传感器的相关参数 temp1_input是DDR的温度，temp2_input是BPU的温度，temp3_input是CPU的温度 温度的精度为0.001摄氏度

```
cat /sys/class/hwmon/hwmon0/temp1_input
46643
```

注：BPU的温度传感器位于bpu subsytem，bpu subsystem只有在bpu运行时才会上电，所以只有bpu运行时，bpu的温度才可以查看。

## Thermal

Linux Thermal 是 Linux 系统下温度控制相关的模块，主要用来控制系统运行过程中芯片产生的热量，使芯片温度和设备外壳温度维持在一个安全、舒适的范围。

要想达到合理控制设备温度，我们需要了解以下三个模块：

- 获取温度的设备：在 Thermal 框架中被抽象为 Thermal Zone Device，X5上有两个thermal zone，分别是thermal_zone0和thermal_zone1；
- 需要降温的设备：在 Thermal 框架中被抽象为 Thermal Cooling Device，有CPU、BPU、GPU和DDR；
- 控制温度策略：在 Thermal 框架中被抽象为 Thermal Governor;

以上模块的信息和控制都可以在 /sys/class/thermal 目录下获取。

在X5里面一共有四个cooling(降温)设备：

- cooling_device0: cpu
- cooling_device1: bpu
- cooling_device2: gpu
- cooling_device3: ddr

其中，cooling设备DDR与thermal_zone0关联，cooling 设备CPU/BPU/GPU与thermal_zone1关联。 目前默认的策略通过以下命令可知是使用的 step_wise。

```
cat /sys/class/thermal/thermal_zone0/policy
```

 通过以下命令可看到支持的策略：user_space、step_wise一共两种。

```
cat /sys/class/thermal/thermal_zone0/available_policies
```

- user_space 是通过uevent将温区当前温度，温控触发点等信息上报到用户空间，由用户空间软件制定温控的策略。
- step_wise 是每个轮询周期逐级提高冷却状态，是一种相对温和的温控策略

具体选择哪种策略是根据产品需要自己选择。可在编译的时候指定或者通过sysfs动态切换。 例如：动态切换thermal_zone0的策略为 user_space模式

```
echo user_space > /sys/class/thermal/thermal_zone0/policy
```

在thermal_zone0中有1个trip_point，用于控制cooling设备DDR的调频温度

可通过sysfs查看DDR的调频温度，当前配置的为95度

```
cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
```

若想调整DDR的调频温度，如85度，可通过如下命令：

```
echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
```

在thermal_zone1中有3个trip_point，其中trip_point_0_temp为预留作用；trip_point_1_temp是该thermal zone的调频温度，可控制CPU/BPU/GPU的频率，当前设置为95度。trip_point_2_temp为关机温度，当前设置为105度 例如想要结温到85摄氏度，CPU/BPU/GPU开始调频：

```
echo 85000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_1_temp
```

如果想要调整关机温度为105摄氏度：

```
echo 105000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_2_temp
```

<font color="red">注意：</font>以上设置断电重启后需要重新设置

## thermal参考文档

以下路径以kernel代码目录为根目录。

```
./Documentation/devicetree/bindings/thermal
./Documentation/driver-api/thermal
```
