---
sidebar_position: 4
---

# 2.4 Thermal和CPU频率管理

## RDK S100
### 温度传感器
在RDKS100芯片中有5个温度传感器，用于显示MCU域/BPU/MAIN域的温度，其中MAIN域和MCU各有两个温度传感器，BPU有一个温度传感器

在/sys/class/hwmon/下有hwmon0目录下包含温度传感器的相关参数
- temp1_input是MAIN域的第一个温度传感器，temp2_input是MAIN域的第二个温度传感器，
- temp3_input是MCU域的第一个温度传感器，temp4_input是MCU域的第二个温度传感器，
- temp5_input是BPU温度传感器。

温度的精度为0.001摄氏度

```
root@ubuntu:~# cat /sys/class/hwmon/hwmon0/temp1_input
46837
root@ubuntu:~#
```
### Thermal机制
Linux Thermal 是 Linux 系统下温度控制相关的模块，主要用来控制系统运行过程中芯片产生的热量，使芯片温度和设备外壳温度维持在一个安全、舒适的范围。

要想达到合理控制设备温度，我们需要了解以下三个模块：

  - 获取温度的设备：在 Thermal 框架中被抽象为 Thermal Zone Device，RDK S100上有5个thermal zone，分别是thermal_zone0~thermal_zone4，分别绑定5个温度传感器；
  - 进行降温的设备：在 Thermal 框架中被抽象为 Thermal Cooling Device，有CPU、BPU和风扇；
    - CPU/BPU等设备通过调频来进行降温；
    - 风扇则可以控制转速来进行降温；
  - 控制温度策略：在 Thermal 框架中被抽象为 Thermal Governor;

以上模块的信息和控制都可以在 `/sys/class/thermal` 目录下获取。

#### Thermal Zone简介
获取某一个thermal_zone的信息，以thermal_zone0为例，示例命令如下：
```shell
root@ubuntu:~# cat /sys/class/thermal/thermal_zone0/type
pvt_cmn_pvtc1_t1
```

获取某一个thermal_zone的当前的策略，以thermal_zone0为例，示例命令如下：
```shell
root@ubuntu:~# cat /sys/class/thermal/thermal_zone0/policy
step_wise
```

获取某一个thermal_zone支持的策略，以thermal_zone0为例，示例命令如下：
```shell
root@ubuntu:~# cat /sys/class/thermal/thermal_zone0/available_policies
user_space step_wise
```
可看到thermal支持的策略有：`user_space`和`step_wise`
- `user_space`： 是通过uevent将温区当前温度，温控触发点等信息上报到用户空间，由用户空间软件制定温控的策略。

- `step_wise`： 是每个轮询周期逐级提高冷却状态，是一种相对温和的温控策略

具体选择哪种策略客户可以根据产品自行选择。可在编译的时候指定或者通过sysfs动态切换。
例如：动态切换thermal_zone0的策略为`user_space`模式
```shell
echo user_space > /sys/class/thermal/thermal_zone0/policy
```

##### thermal_zone0简介
在thermal_zone0中有4个trip_point，
- trip_point_0_temp：关机温度，默认设置为120度
- trip_point_1_temp：用于控制风扇转速，默认为43度，风扇档位范围2~5，表示超过43度，风扇将从关闭状态调整为2档，最高可提升到5档。
- trip_point_2_temp：用于控制风扇转速，默认为65度，风扇档位范围6~10，表示超过65度，风扇将调整到6档，最高可提升到10档慢转速。
- trip_point_3_temp：用于控制CPU Acore频率，默认为95度，表示超过95度，CPU Acore会降频。
可通过sysfs查看相应的温度设置
```shell
root@ubuntu:~# cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
120000
```
若想调整相应的温度，如85度开始CPU调频，可通过如下命令：
```shell
echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_3_temp
```

##### thermal_zone1/2/3简介
在thermal_zone1/2/3中有1个trip_point，都表示的是关机温度，默认为120度

在thermal_zone4中有两个trip_point,其中
- trip_point_0_temp为关机温度，默认为120度。
- trip_point_1_temp为BPU的调频温度，默认为95度

例如想要结温到85摄氏度，BPU开始调频：
```shell
echo 85000 > /sys/devices/virtual/thermal/thermal_zone4/trip_point_1_temp
```

如果想要调整关机温度为105摄氏度， 可通过修改所有thermal_zone的trip_point_0_temp来实现
```shell
echo 105000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
echo 105000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_0_temp
echo 105000 > /sys/devices/virtual/thermal/thermal_zone2/trip_point_0_temp
echo 105000 > /sys/devices/virtual/thermal/thermal_zone3/trip_point_0_temp
echo 105000 > /sys/devices/virtual/thermal/thermal_zone4/trip_point_0_temp
```

:::info
PS：以上设置只在当前启动有效，<ins>重启后</ins>需要**重新**设置。
:::

#### 降温设备
在RDK S100中一共有四个cooling(降温)设备：

- cooling_device0: cpu cluster 0， 通过调整频率控制温度
- cooling_device1: cpu cluster 1， 通过调整频率控制温度
- cooling_device2: emc2305 fan，通过调整风扇转速档位来控制温度，档位从0~10，0表示关闭，10表示风扇满转速。
- cooling_device3: bpu， 通过调整频率控制温度

其中，cooling 设备CPU和风扇与thermal_zone0关联，cooling设备BPU与thermal_zone4关联，thermal_zone1/2/3没有绑定cooling设备

目前默认的策略用的是`step_wise`。

##### 风扇调节
RDK S100开发板上的emc2305风扇控制器，可以通过设备节点获取设备基本信息及控制转速：
1. 获取降温设备信息：
    ```shell
    root@ubuntu:~# cat /sys/class/thermal/cooling_device2/type
    emc2305_fan
    ```
2. 获取可配置的风扇档位：
    ```shell
    root@ubuntu:~# cat /sys/class/thermal/cooling_device2/max_state
    10
    ```
3. 获取当前风扇档位
   ```shell
   root@ubuntu:~# cat /sys/class/thermal/cooling_device2/cur_state
   5
   ```
4. 配置thermal_zone0的策略为`userspace`：
    ```
    echo user_space > /sys/class/thermal/thermal_zone0/policy
    ```
5. 配置当前风扇档位为10：
   ```shell
   root@ubuntu:~# echo 10 > /sys/class/thermal/cooling_device2/cur_state
   ```

:::info
**注意**：当thermal_zone0的策略为`step_wise`时，用户配置的风扇档位会被系统自动根据当前温度进行调节。如果客户需要将风扇固定为特定档位，请参考[Thermal Zone](#thermal-zone简介)章节，将thermal_zone0的策略改为`user_space`
:::

### CPU频率管理

在linux内核中，自带了cpufreq子系统用来控制cpu的频率和频率控制策略。

进入目录`/sys/devices/system/cpu/cpufreq/policy0`，`ls` 一下，会看到目录中有如下文件：

```shell
affected_cpus						// 当前控制影响的CPU核(没有显示处于offline状态的cpu)
cpuinfo_cur_freq					// 当前CPU频率(单位: KHz）
cpuinfo_max_freq					// 当前调频策略下CPU可用的最高频率(单位: KHz）
cpuinfo_min_freq					// 当前调频策略下CPU可用的最低频率(单位: KHz）
cpuinfo_transition_latency			// 处理器切换频率所需要的时间(单位:ns)
related_cpus						// 该控制策略影响到哪些CPU核(包括了online+offline的所有cpu)
scaling_available_frequencies		// CPU支持的主频率列表(单位: KHz）
scaling_available_governors			// 当前内核中支持的所有 governor(调频)类型
scaling_cur_freq					// 保存着 cpufreq 模块缓存的当前 CPU 频率，不会对 CPU 硬件寄存器进行检查。
scaling_driver						// 当前使用的调频驱动
scaling_governor					// governor(调频)策略
scaling_max_freq					// 当前调频策略下CPU可用的最高频率（从cpufreq模块缓存中读取）
scaling_min_freq					// 当前调频策略下CPU可用的最低频率（从cpufreq模块缓存中读取）
scaling_setspeed					// 需将governor切换为userspace才能使用，往这个文件echo数值，会切换频率
```

目前支持的频率包括
```shell
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
1500000 2000000
```

注：支持的频点可能在不同类型的芯片上有所差异。
RDK S100系统使用的linux内核支持以下种类的调频策略:

- 性能（performance）：总是将CPU置于最高能耗也是最高性能的状态，即硬件所支持的最高频。
- performance：以最高频率执行
- ondemand：按照负载调整频率
- userspace：根据用户的设置频率
- powersave：以最低频率执行
- schedutil：按照负载调整频率，它是与CPU调度器结合来使用
- conservative：类似Ondemand，不过频率调节的会平滑一下，不会有忽然调整为最大值又忽然调整为最小值的现象

用户可以通过控制目录`/sys/devices/system/cpu/cpu0/cpufreq/`下的对应设置来控制CPU的调频策略。

例如让CPU运行在性能模式：

```shell
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

或者控制CPU运行在一个固定的频率（1.5GHz）：

```shell
echo userspace >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1500000 >/sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
```

可通过`sudo hrut_somstatus`命令查看当前芯片工作频率、温度等状态：
