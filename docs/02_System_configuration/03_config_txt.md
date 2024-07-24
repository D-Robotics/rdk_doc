---
sidebar_position: 3
---

# 2.3 confit.txt 文件配置

RDK 使用配置文件`config.txt`来设置一些启动时候的系统配置。`config.txt` 会在`uboot`阶段被读取，支持修改设备树的配置，IO管脚状态，ION内存，CPU频率等。该文件通常可以从 Linux 访问`/boot/config.txt`，并且必须以`root`用户身份进行编辑。如果在`config.txt`配置设置，但是该文件还不存在，只需将其创建为新的文本文件即可。

## 注意事项

:::info 注意

1. `config.txt`配置文件仅适用于`RDK X3`和`RDK X3 Module`开发板，不适用于`RDK Ultra`开发板。

2. 系统版本不低于 `2.1.0`。

3. `miniboot`版本不能低于 `20231126`日期的版本。参考[rdk-miniboot-update](rdk-command-manual/cmd_rdk-miniboot-update)在板更新miniboot。

4. 如果您在本配置文件添加了过滤项，那么使用`srpi-config`工具时请注意配置项是否会被过滤掉。

:::

## 设备树配置

### dtdebug

`dtdebug` 如果非零，在`uboot`阶段的设备树配置过程中会输出配置日志。

```
dtdebug=1
```

### dtoverlay

支持设备树覆盖，提供更加灵活的设备树调整方式。

例如通过`ion_resize`调整`ION`内存的大小，以下配置会修改`ION`内存大小为 `1GB`。

```Shell
dtoverlay=ion_resize,size=0x40000000
```

### dtparam

支持设置uart、i2c、spi、i2s等总线的使能与关闭。

目前支持的选项参数：uart3, spi0, spi1, spi2, i2c0, i2c1, i2c2, i2c3, i2c4, i2c5, i2s0, i2s1

例如关闭串口3：

```
dtparam=uart3=off
```

例如打开`i2c5`:

```
dtparam=i2c5=on
```

## CPU频率

### arm_boost

当设置为1时，开启超频，RDK v1.x 版本最高频率提高到1.5GHz，RDK V2.0 和 RDK Module 最高频率提高到1.8GHz，通过 `cat /sys/devices/system/cpu/cpufreq/scaling_boost_frequencies` 获取使能boost后会开放哪些更高CPU频率。

默认不开启超频，设置`arm_boost` 为 `1`时开启，例如：

```
arm_boost=1
```

### governor

CPU 频率的调度方式，有 `conservative ondemand userspace powersave performance schedutil` 方式可以选择， 通过 `cat /sys/devices/system/cpu/cpufreq/scaling_available_governors` 获取可以设置的模式。

例如设置`CPU`运行在性能模式：

```
governor=performance
```

有关`CPU`调度方式的说明请查阅[CPU频率管理](frequency_management#cpu频率管理)。

### frequency

`governor`设置为 `userspace` 时，可以通过本选型设置`CPU`运行在一个固定的频率上，目前一般可以设置`240000 500000 800000 1000000 1200000 1500000 1800000`这些频率，具体可以通过`cat /sys/devices/system/cpu/cpufreq/scaling_available_frequencies` 获取可以设置的频率列表。

例如设置`CPU`降频运行在 `1GHz`：

```
governor=userspace
frequency=1000000
```

## IO初始化

### gpio

支持设置IO的功能复用，输出、输出模式，输出高、低电平，上下拉模式。

```shell
gpio:
ip - Input                             设置为输入模式
op - Output                            设置为输出模式
f0-f3 - Func0-Func3                    设置功能复用，f3功能都是设置为io模式，其他功能请查阅寄存器手册
dh - Driving high (for outputs)        输出高电平
dl - Driving low (for outputs)         输出低电平
pu - Pull up                           推挽上拉
pd - Pull down                         推挽下拉
pn/np - No pull                        无上下拉
```

### 示例

配置`40Pin`管脚上的 `GPIO5` 和 `GPIO6`为IO模式：

```
gpio=5=f3
gpio=6=f3
# 对于连续的管脚，也可以使用以下方式配置
gpio=5-6=f3
```

配置`40Pin`管脚上的 `GPIO5` 为输入模式：

```
gpio=5=f3
gpio=5=ip
```

配置`40Pin`管脚上的 `GPIO6` 为输出模式，并且输出低电平：

```
gpio=6=f3
gpio=6=op,dl
```

配置`40Pin`管脚上的 `GPIO6` 为输出模式，并且输出高电平，并且设置上拉：

```
gpio=6=f3
gpio=6=op,dl,pu
```

## 温度控制

### throttling_temp

系统CPU、BPU降频温度点，温度超过该温度点时，CPU和BPU会降低运行频率来减低功耗，CPU最低降到240MHz，BPU最低降到400MHz。 

### shutdown_temp

系统宕机温度点，如果温度超过该温度，为了保护芯片和硬件，系统会自动关机，建议对设备做好散热处理，避免设备宕机，因为宕机后设备不会自动重启。

## 选项过滤

支持使用 [] 设置过滤项，过滤项的设置需要在配置文件的尾部添加，因为文件前面未添加过滤项的部分属于 `all`，一旦添加过滤设置，则之后的配置只属于该过滤属性，直到配置文件结尾或者设置了另一个过滤项。

当前支持的过滤项以硬件型号为区分，支持以下过滤项：

| 过滤项    | 适配的型号               |
| --------- | ------------------------ |
| [all]     | 所有硬件，默认属性       |
| [rdkv1]   | RDK x3 v1.0，RDK x3 v1.1 |
| [rdkv1.2] | RDK x3 v1.2              |
| [rdkv2]   | RDK x3 v2.1              |
| [rdkmd]   | RDK x3 Module            |

## 电压域

### voltage_domain

配置40pin管脚的电压域，支持配置为 3.3V 和 1.8V，不配置时默认3.3V。

本配置项需要配合硬件上的电压域切换的跳线帽使用。

:::info 注意

仅RDK Modelu支持本项配置。

:::

例如配置`RDK Module`的`40Pin`工作在`3v3`电压模式，此处示例使用了`[rdkmd]`作为过滤项：

```
# Voltage domain configuration for 40 Pin, 3.3V or 1.8V, defualt 3.3V
# Only RDK Module supported
[rdkmd]
voltage_domain=3.3V
```

