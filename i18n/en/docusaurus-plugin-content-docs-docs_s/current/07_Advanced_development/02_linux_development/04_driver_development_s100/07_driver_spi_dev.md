---
sidebar_position: 7
---

# SPI Debugging Guide

## SPI Hardware Support

The S100 Acore supports two SPI interfaces, and SPI0 and SPI1 can only operate as SPI Masters.

## Software Architecture

![image-spi_software](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image-spi_software.png)

The above figure illustrates the SPI software architecture. From bottom to top, it can be divided into the hardware IP layer, kernel layer, and user space layer. Each layer is introduced below.

-   **Hardware IP Layer**: This layer represents the SPI hardware.
-   **Kernel Layer**: This layer can be further subdivided into three sub-layers:
    -   **SPI Driver Layer**: Primarily implements operations for the SPI hardware IP and also implements interfaces defined by the SPI framework.
    -   **SPI Framework Layer**: Acts as an adaptation layer for the SPI driver. It defines a set of interfaces that the driver layer must implement for the lower layer and provides generic interfaces to the upper layer, abstracting hardware details.
    -   **SPI Char Device Layer**: Provides device nodes for user space, facilitating data exchange between user space and kernel space. Currently, the kernel’s built-in `spidev` character device is used.
-   **Application Layer**: Consists of various applications that interact with kernel space by calling character device drivers to exchange data.

## Code Paths

### Hobot SPI Protocol Code

All Hobot SPI driver-related code is located under the **\$project/hobot-drivers/spi** directory.

``` {.text}
oops@tiger$ tree . -L 1

├── Kconfig                       # Kconfig-related files
├── README.md
└──spi_drv                        # SPI driver-related files
```

Description of the **\$project/hobot-drivers/spi/spi_drv** directory:

``` {.text}
oops@tiger$ tree . -L 1
├── Makefile
├── spi-dw.c                       # Core SPI driver code
├── spi-dw.h
├── spi-dw-mmio.c                  # SPI driver MMIO code
└── spi-dw-mmio-dma.c              # SPI driver DMA code
```

### Linux SPI Framework Code

All Linux SPI protocol-related code resides in the **\$project/kernel/drivers/spi** directory.

``` {.text}
oops@tiger$ tree kernel/drivers/spi/
drivers/spi/
├── spi.c                             # SPI framework code

oops@tiger$
```

### SPI Device Tree Code

The following DTS files in S100 involve SPI-related configurations:

```C
|-- drobot-s100-pinctrl.dtsi       # SPI pinctrl configuration
|-- drobot-s100-soc.dtsi           # SPI device node configuration
|-- drobot-s100-pdma.dtsi          # SPI PDMA usage configuration
```

### SPI Device Tree Configuration Explanation

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

Key newly added configuration items for S100 SPI are explained below:

-   **sample-delay**: When the SPI controller operates as a master, this value specifies the sampling delay for received data. If bit misalignment occurs in received data, adjust this value accordingly.
-   **num-cs**: When the SPI controller operates as a master, this specifies the number of chip selects (CS) supported. The S100 SPI supports up to two chip selects in master mode.

## SPI Verification and Debugging

This section introduces how to verify basic SPI functionality on the S100, including environment setup, execution of test commands, and locations of test code.

### Test Environment Preparation

`spidev_test` is an open-source SPI testing utility. Users can directly obtain and compile it from the following directory in the Linux source code:

Source location: `kernel/tools/spi/spidev_test.c`.

Common parameters for `spidev_test` are explained below:

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

### SPI Internal Loopback Test

The SPI internal loopback test is supported only when operating as an SPI Master. The principle involves the SPI hardware IP’s TX FIFO sending data directly to the RX FIFO to form a loopback.

Example test command and results:

```bash
root@ubuntu:/map# ./spidev_test -D /dev/spidev1.0 -s 1000000 -S 100 -l -v -p "\x01\x02\x03\x04"
spi mode: 0x20
bits per word: 8
max speed: 1000000 Hz (1000 kHz)
TX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
RX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
```

### SPI External Loopback Test

SPI external loopback test refers to connecting an SPI Master to an SPI Slave.

The Master can be selected as SPI1, and the SPI Slave should be an external SPI device (chosen by the customer).

The command for sending test data from the S100 side is as follows:

```bash
root@ubuntu:/map# ./spidev_test -D /dev/spidev1.0 -s 1000000 -S 100  -v -p "\x01\x02\x03\x04"
spi mode: 0x0
bits per word: 8
max speed: 1000000 Hz (1000 kHz)
TX | 01 02 03 04 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
RX | FF FF FF FF __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __  |....|
```

The Slave device will receive the data sent by the Master on the S100 side.

**Note: When performing the external loopback test, the SPI Slave program must be executed first, followed by the SPI Master program. If the SPI Master program is executed before the SPI Slave program, data loss may occur due to desynchronization between the Master and Slave.**