---
sidebar_position: 3
---

# 2.3 config.txt 文件配置

RDK 使用配置文件`config.txt`来设置一些启动时候的系统配置。`config.txt` 会在`uboot`阶段被读取，支持修改设备树的配置，IO管脚状态，ION内存，CPU频率等。该文件通常可以从 Linux 访问`/boot/config.txt`，并且必须以`root`用户身份进行编辑。如果在`config.txt`配置设置，但是该文件还不存在，只需将其创建为新的文本文件即可。

## 注意事项

:::info 注意

1. `config.txt`配置文件仅适用于`RDK X3`、`RDK X5`和`RDK X3 Module`开发板，不适用于`RDK Ultra`开发板。

2. 系统版本不低于 `2.1.0`。

3. `miniboot`版本不能低于 `20231126`日期的版本。参考 [rdk-miniboot-update](../09_Appendix/rdk-command-manual/cmd_rdk-miniboot-update.md) 在板更新 miniboot。

4. 如果您在本配置文件添加了过滤项，那么使用`srpi-config`工具时请注意配置项是否会被过滤掉。

:::

## 设备树配置

### dtdebug

`dtdebug` 如果非零，在`uboot`阶段的设备树配置过程中会在串口输出配置日志。

```
dtdebug=1
```

### dtoverlay

支持设备树覆盖，提供更加灵活的设备树调整方式。

【RDK X3】例如通过`ion_resize`调整`ION`内存的大小，以下配置会修改`ION`内存大小为 `1GB`。

```Shell
dtoverlay=ion_resize,size=0x40000000
```

【RDK X5】通过dtoverlay_spi5_spidev增加/dev/spidev5.0（注意：can设备也接到了spi5，spidev和can只能二选一）

```Shell
dtoverlay=dtoverlay_spi5_spidev
```

### RDK X5 配置 ION

通过 ion_reserved_size，ion_carveout_size， ion_cma_size 修改 boot 环境变量，进而修改 ION 分区大小。

| boot环境变量名         | dts标签        | dts compatible字符串 | 默认大小 |
| ----------------- | ------------ | ----------------- | ---- |
| ion_reserved_size | ion_reserved | ion-pool          | 320M |
| ion_carveout_size | ion_carveout | ion-carveout      | 320M |
| ion_cma_size      | ion_cma      | ion-cma           | 128M |

```Shell
ion=ion_reserved_size=0x14000000
ion=ion_carveout_size=0x14000000
ion=ion_cma_size=0x08000000
```

可以通过启动信息查看各个ION区域的大小：

```Shell
root@ubuntu:~# dmesg | grep ION
[    0.187961] Reserved ION Carveout(ion-pool) MEM start 0xa4100000, size 0x14000000
[    0.187989] Reserved ION cma(ion-carveout) reserved MEM start 0xb8100000, size 0x14000000
[    0.188075] Reserved ION cma(ion-cma) reserved MEM start 0xcc100000, size 0x8000000
```

### dtparam

支持设置 uart、i2c、spi、i2s 等总线的使能与关闭。

目前支持的选项参数：

RDK X3 支持： uart3, spi0, spi1, spi2, i2c0, i2c1, i2c2, i2c3, i2c4, i2c5, i2s0, i2s1

RDK X5 支持： uart1, uart2, uart3, uart6，spi1, spi2, i2c0, i2c1, i2c5, i2c4, i2c5, dw_i2s1

例如关闭串口3：

```
dtparam=uart3=off
```

例如打开`i2c5`:

```
dtparam=i2c5=on
```

## X3 CPU频率

### arm_boost

当设置为1时，开启超频，RDK v1.x 版本最高频率提高到 1.5GHz，RDK V2.0 和 RDK Module 最高频率提高到1.8GHz，通过 `cat /sys/devices/system/cpu/cpufreq/scaling_boost_frequencies` 获取使能 boost 后会开放哪些更高 CPU 频率。

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

有关`CPU`调度方式的说明请查阅 [X3 CPU频率管理](x3_frequency_management#cpu频率管理)。

### frequency

`governor`设置为 `userspace` 时，可以通过本选型设置`CPU`运行在一个固定的频率上，目前一般可以设置`240000 500000 800000 1000000 1200000 1500000 1800000`这些频率，具体可以通过`cat /sys/devices/system/cpu/cpufreq/scaling_available_frequencies` 获取可以设置的频率列表。

例如设置`CPU`降频运行在 `1GHz`：

```
governor=userspace
frequency=1000000
```

## X5 CPU频率

有关`CPU`调度方式的说明请查阅 [X5 CPU频率管理](x5_frequency_management#cpu频率管理)，这里仅介绍config.txt的配置方法。

### arm_boost

当设置为1时，开启超频，RDK RDK X5 最高频率提高到1.8GHz，通过 `cat /sys/devices/system/cpu/cpufreq/policy0/scaling_boost_frequencies` 获取使能 boost 后会开放哪些更高 CPU 频率。

默认不开启超频，设置`arm_boost` 为 `1`时开启，例如：

```
arm_boost=1
```

### governor

CPU 频率的调度方式，有 `conservative ondemand userspace powersave performance schedutil ` 方式可以选择， 通过 `cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_governors` 获取可以设置的模式。

例如设置`CPU`运行在性能模式：

```
governor=performance
```

### frequency

`governor`设置为 `userspace` 时，可以通过本选型设置`CPU`运行在一个固定的频率上，目前一般可以设置`300000 600000 1200000 1500000`这些频率，具体可以通过`cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies` 获取可以设置的频率列表。

例如设置`CPU`降频运行在 `1.2GHz`：

```
governor=userspace
frequency=1200000
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

系统 CPU、BPU 降频温度点，温度超过该温度点时，CPU 和 BPU 会降低运行频率来减低功耗，CPU最低降到 240MHz，BPU 最低降到 400MHz。 

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
| [x5-rdk]  | RDK X5 V0.1             |

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

