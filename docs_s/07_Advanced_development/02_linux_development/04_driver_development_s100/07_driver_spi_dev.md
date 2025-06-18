---
sidebar_position: 7
---

# SPI调试指南

## SPI硬件支持

S100 Acore支持2路SPI，且SPI0，SPI1只能做SPI Master。

## 软件构架

![image-spi_software](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image-spi_software.png)


如上图为SPI软件架构图，从下到上依次可以分为硬件IP层，内核层和用户空间层，下面依次对各层进行介绍。

-   硬件IP层：该层为SPI硬件层。
-   内核层：又可以细分为3层。
    -   spi driver层：主要实现对SPI硬件IP的操作，另外还实现了spi
        framework定义的接口。
    -   spi framework层：可以理解为spi
        driver的适配层，对下层定义了一组driver层需要实现的接口，对上提供了通用接口屏蔽了硬件细节。
    -   spi char
        device层：为用户空间提供节点，方便用户空间与内核空间进行数据交换。目前使用kernel自带的spidev字符设备。
-   app层：为各种应用程序，这些应用程序通过调用字符设备驱动达到与内核空间数据交换的目的。


## 代码路径

### Hobot SPI 协议代码

hobot spi驱动相关代码都放在 **\$project/hobot-drivers/spi** 目录下

``` {.text}
oops@tiger$ tree . -L 1

├── Kconfig                       # Kconfig相关
├── README.md
└──spi_drv                        # spi driver相关
```

**\$project/hobot-drivers/spi/spi_drv** 目录说明

``` {.text}
oops@tiger$ tree . -L 1
├── Makefile
├── spi-dw.c                       # spi驱动核心代码
├── spi-dw.h
├── spi-dw-mmio.c                  # spi驱动mmio代码
└── spi-dw-mmio-dma.c              # spi驱动dma代码
```

### Linux SPI 框架代码

Linux spi协议相关代码都放在 **\$project/kernel/drivers/spi** 目录下

``` {.text}
oops@tiger$ tree kernel/drivers/spi/
drivers/spi/
├── spi.c                             # spi框架代码

oops@tiger$
```

### SPI 设备树代码

S100中涉及到spi配置相关的dts文件如下：

```C
|-- drobot-s100-pinctrl.dtsi       # spi pinctrl相关配置
|-- drobot-s100-soc.dtsi           # spi 设备节点配置
|-- drobot-s100-pdma.dtsi          # spi pdma使用配置
```

### SPI 设备树配置说明

```dts
spi0: spi@39800000 {
		compatible = "hobot,hb-dw-spi";
		reg-io-width = <4>;
		#address-cells = <1>;
		#size-cells = <0>;
		reg = <0x0 0x39800000 0x0 0x1000>;
		interrupts = <GIC_SPI PERISYS_SPI0_SSI_INTR PERISYS_SPI0_SSI_INTR_TRIG_TYPE>;
		status = "okay";
		num-cs = <2>;
		resets = <&smc_reset RST_IDX_IP_PERI_SPIM0>,
								<&smc_reset RST_IDX_IP_PERI_SPIM0_APB>;
		reset-names = "spi_reset";
		clocks = <&scmi_smc_clk CLK_IDX_TOP_PERI_SPI_M0>;
		clock-names = "spi_pclk";
		power-domains = <&scmi_smc_pd PD_IDX_LSPERI_TOP>;
		freq-pclk = <200000000>;
		sample-delay = <1>;
		pinctrl-names = "default";
		pinctrl-0 = <&peri_spi0>;
		dmas = <&pdma0 8                        /* read channel */
						&pdma0 9        >;              /* write channel */
		dma-names = "rx", "tx";
		spidev@0 {
				compatible = "rohm,dh2228fv";
				spi-max-frequency = <50000000>;
				reg = <0>;
		};
};

spi1: spi@39810000 {
		compatible = "hobot,hb-dw-spi";
		reg-io-width = <4>;
		#address-cells = <1>;
		#size-cells = <0>;
		reg = <0x0 0x39810000 0x0 0x1000>;
		interrupts = <GIC_SPI PERISYS_SPI1_SSI_INTR PERISYS_SPI1_SSI_INTR_TRIG_TYPE>;
		status = "okay";
		num-cs = <2>;
		resets = <&smc_reset RST_IDX_IP_PERI_SPIM1>,
								<&smc_reset RST_IDX_IP_PERI_SPIM1_APB>;
		reset-names = "spi_reset";
		clocks = <&scmi_smc_clk CLK_IDX_TOP_PERI_SPI_M1>;
		clock-names = "spi_pclk";
		power-domains = <&scmi_smc_pd PD_IDX_LSPERI_TOP>;
		freq-pclk = <200000000>;
		sample-delay = <1>;
		pinctrl-names = "default";
		pinctrl-0 = <&peri_spi1>;
		dmas = <&pdma0 10                       /* read channel */
						&pdma0 11       >;              /* write channel */
		dma-names = "rx", "tx";
		spidev@0 {
				compatible = "rohm,dh2228fv";
				spi-max-frequency = <50000000>;
				reg = <0>;
		};
};

```

这里着重说明S100 SPI新增的配置项

-   sample-delay：spi控制器作master时，对接收数据的采样延迟值，如果出现数据bit位错位的情况，可以调整该值。
-   num-cs：spi控制器作master时，支持cs个数，S100 SPI作master时，最多支持两个片选。

## SPI 验证及调试

本小节主要介绍S100 SPI基本功能如何验证，包括环境如何配置，测试命令的执行及测试代码存放位置等。

### 测试环境准备

spidev_test是一个开源的SPI测试工具，用户可以直接从linux源码如下目录获取并编译使用.

源码位置：kernel/tools/spi/spidev_test.c。

spidev_test常见参数说明如下：

```bash
root@ubuntu:/map# ./spidev_test -h
./spidev_test: invalid option -- 'h'
Usage: ./spidev_test [-DsbdlHOLC3vpNR24SI]
   -D --device   device to use (default /dev/spidev1.1)
   -s --speed    max speed (Hz)
   -d --delay    delay (usec)
   -b --bpw      bits per word
   -i --input    input data from a file (e.g. "test.bin")
   -o --output   output data to a file (e.g. "results.bin")
   -l --loop     loopback
   -H --cpha     clock phase
   -O --cpol     clock polarity
   -L --lsb      least significant bit first
   -C --cs-high  chip select active high
   -3 --3wire    SI/SO signals shared
   -v --verbose  Verbose (show tx buffer)
   -p            Send data (e.g. "1234\xde\xad")
   -N --no-cs    no chip select
   -R --ready    slave pulls low to pause
   -2 --dual     dual transfer
   -4 --quad     quad transfer
   -8 --octal    octal transfer
   -S --size     transfer size
   -I --iter     iterations
```

### SPI 内部回环测试

SPI内部回环测试仅SPI Master支持，其原理是SPI硬件IP的tx fifo将数据发给rx
fifo从而形成回环。

测试命令及结果参考如下：

```bash
root@ubuntu:/map# ./spidev_test -D /dev/spidev1.0 -s 1000000 -S 100 -l -v -p "\x01\x02\x03\x04"
spi mode: 0x20
bits per word: 8
max speed: 1000000 Hz (1000 kHz)
TX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
RX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
```

### SPI 外部回环测试

SPI外部回环测试指SPI Master接SPI Slave。

Master可以选择SPI1，SPI Slave选择外部SPI设备（客户自行选择）。

S100侧的发送测试命令参考如下：

```bash
root@ubuntu:/map# ./spidev_test -D /dev/spidev1.0 -s 1000000 -S 100  -v -p "\x01\x02\x03\x04"
spi mode: 0x0
bits per word: 8
max speed: 1000000 Hz (1000 kHz)
TX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
RX | FF FF FF FF __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
```

在Slave设备侧将收到S100侧Master发送的数据。

**注：在进行外部回环测试时，需要先执行SPI Slave程序，再执行SPI Master程序。假如先执行SPI Master程序，后执行SPI Slave程序，可能会由于Master与Slave不同步导致SPI接收数据出现丢失。**
