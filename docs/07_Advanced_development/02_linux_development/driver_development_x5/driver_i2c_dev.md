---
sidebar_position: 3
---

# I2C调试指南

## 前言

X5芯片提供了标准的I2C总线，I2C总线控制器通过串行数据线（SDA）和串行时钟（SCL）线在连接到总线的器件间传递信息。每个器件都有一个唯一的地址（无论是微控制器——MCU、LCD控制器、存储器或键盘接口），而且都可以作为一个发送器和一个接收器（由器件的功能决定）。

I2C控制器支持以下功能：

- 支持三种速度模式
    - standard mode(0-100Kb/s)
    - fast mode(0-400Kb/s) && fast mode plus(0-1000Kb/s)
    - high-speed mode(0-3.4Mb/s)
- 支持主从模式配置
- 支持7位和10位寻址模式

## X5的I2C控制器

X5总共提供8个I2C控制器，其中7个（I2C0~6）位于LSIO子系统，1个(I2C7)位于DSP子系统。 所有8个I2C控制器只有I2C4支持high-speed mode。

## 驱动代码

```bash
drivers/i2c/i2c-dev.c 				# I2C字符设备接口代码
drivers/i2c/i2c-core-base.c 			# I2C框架代码
drivers/i2c/busses/i2c-designware-platdrv.c 	# I2C驱动代码源文件
```

### 内核配置位置

```bash
/* arch/arm64/configs/hobot_x5_soc_defconfig */
CONFIG_I2C_CHARDEV=y 			# I2C驱动应用层配置宏
CONFIG_I2C_DESIGNWARE_PLATFORM=y 	# DW I2C驱动配置宏
```

### 内核DTS节点配置

X5芯片总共支持8路I2C总线。 | X5 I2C控制器的设备树定义位于SDK包的kernel文件夹下的`arch/arm64/boot/dts/hobot/x5.dtsi`文件内。

<font color="red">备注：</font>x5.dtsi中的节点主要声明SoC共有特性，和具体电路板无关，一般情况下不用修改。

## I2C使用

对于I2C的使用说明在Linux Kernel的Documentation/i2c下有详细的说明，本文主要列出X3J3 I2C驱动接口特殊的部分。

### Kernel Space

X3J3 I2C驱动在Kernel Space下提供了可以设置I2C传输频率的接口，使用方法如下：

**I2C速度配置**

默认的I2C速率为100K，支持100k/200k/400k/1M/3.4M四种速率，可通过修改DTS中相应I2C节点的clock-frequency等字段完成速率修改。
以修改i2c4控制器的运行速率为例：

-   如果需要使用3.4MHz，需要在对应板卡对应的dts的i2c4的节点内配置以下参数：

```c
...
&i2c4 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_i2c4>;
	clock-frequency = <3400000>;
	i2c-scl-falling-time-ns = <78>;
	i2c-sda-falling-time-ns = <78>;
}
...
```

-   如果需要使用1MHz，需要在对应板卡对应的dts的i2c4的节点内配置以下参数：

```c
...
&i2c4 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_i2c4>;
	clock-frequency = <1000000>;
	i2c-scl-falling-time-ns = <60>;
	i2c-sda-falling-time-ns = <60>;
}
...
```

-   如果需要使用400kHz，需要在对应板卡对应的dts的i2c4的节点内配置以下参数：

```c
...
&i2c4 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_i2c4>;
	clock-frequency = <400000>;
	i2c-scl-falling-time-ns = <190>;
}
...
```

<font color="red">备注：</font>“i2c-scl-falling-time-ns”及“i2c-sda-falling-time-ns”两个参数在不同的PCB设计上可能需要通过示波器来调整。上述配置基于I2C IO使能上拉并且板端配置1k上拉电阻的硬件条件。

**I2C DMA配置**

如果需要配置I2C使用dma，可以参考以下dts配置：

```c
&i2c4 {
 	status = "okay";
 	pinctrl-names = "default";
 	pinctrl-0 = <&pinctrl_i2c4>;
	dma-names = "tx", "rx";
	dmas = <&axi_dmac 19>, <&axi_dmac 18>;
}
```

注意：I2C7位于DSP子系统，DTS内dma需要绑定DSP子系统的DMA：

```c
&i2c4 {
 	status = "okay";
 	pinctrl-names = "default";
 	pinctrl-0 = <&pinctrl_i2c4>;
	dma-names = "tx", "rx";
	dmas = <&dsp_axi_dma 31>, <&dsp_axi_dma 30>;
}
```

I2C DMA握手列表如下：
 |    | RX | TX |  
 | ---- | ---- | ---- | 
 | I2C0 | 	10 | 	11 | 
 | I2C1 | 	12 | 	13 | 
 | I2C2 | 	14 | 	15 | 
 | I2C3 | 	16 | 	17 | 
 | I2C4 | 	18 | 	19 | 
 | I2C5 | 	39 | 	40 | 
 | I2C6 | 	41 | 	42 | 
 | I2C7 | 	30 | 	31 | 

 ### User Space
 通常，I2C设备由内核驱动程序控制，但也可以从用户态访问总线上的所有设备，通过/dev/i2c-%d接口来访问，Kernel下面的Documentation/i2c/dev-interface文档里有详细的介绍。

 **开源工具：i2c-tools**

 i2c-tools是一套开源工具，该工具已经被交叉编译并包含在在X3J3 系统软件的rootfs中，客户可以直接使用：

-   i2cdetect — 用来列举I2C bus及该bus上的所有设备
-   i2cdump — 显示i2c设备的所有register值
-   i2cget — 读取i2c设备某个register的值
-   i2cset — 写入i2c设备某个register的值
-   i2ctransfer — 可以读、写i2c设备某个或者多个register的值

