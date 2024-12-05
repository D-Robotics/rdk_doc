---
sidebar_position: 3
---

# I2C Debugging Guide

## Foreword

The X5 chip provides a standard I2C bus, where the I2C bus controller transmits information between devices connected to the bus via the serial data line (SDA) and the serial clock line (SCL). Each device has a unique address (whether it is a microcontroller—MCU, LCD controller, memory, or keyboard interface), and can act as both a transmitter and a receiver (depending on the device's function).

The I2C controller supports the following features:

- Supports three speed modes:
    - Standard mode (0-100Kb/s)
    - Fast mode (0-400Kb/s) & Fast mode plus (0-1000Kb/s)
    - High-speed mode (0-3.4Mb/s)
- Supports master-slave configuration
- Supports 7-bit and 10-bit addressing modes

## X5's I2C Controller

The X5 provides a total of 8 I2C controllers, 7 of which (I2C0 to I2C6) are located in the LSIO subsystem, and 1 (I2C7) is located in the DSP subsystem. Out of these 8 controllers, only I2C4 supports high-speed mode.

## Driver Code


```bash
drivers/i2c/i2c-dev.c             # I2C Character Device Interface Code
drivers/i2c/i2c-core-base.c       # I2C Framework Code
drivers/i2c/busses/i2c-designware-platdrv.c # I2C Driver Source Code

```

### Kernel Configuration Location

```bash
/* arch/arm64/configs/hobot_x5_soc_defconfig */
CONFIG_I2C_CHARDEV=y              # I2C Driver User-Space Configuration Macro
CONFIG_I2C_DESIGNWARE_PLATFORM=y  # DW I2C Driver Configuration Macro

```


### Kernel DTS Node Configuration

The X5 chip supports a total of 8 I2C buses. The device tree definition for the X5 I2C controller is located in the SDK package's kernel folder at `arch/arm64/boot/dts/hobot/x5.dtsi`.

<font color="red">Note:</font> The nodes in `x5.dtsi` primarily declare SoC-level features and are not specific to any particular circuit board, so they generally do not need to be modified.

## I2C Usage

Detailed instructions for using I2C can be found in the Linux Kernel's `Documentation/i2c` folder. This document mainly highlights special parts of the X3J3 I2C driver interface.

### Kernel Space

The X3J3 I2C driver in Kernel Space provides an interface for setting the I2C transfer frequency. The usage is as follows:

**I2C Speed Configuration**

The default I2C speed is 100K, and it supports the following speeds: 100k, 200k, 400k, 1M, and 3.4M. The speed can be modified by adjusting the relevant I2C node parameters, such as `clock-frequency`, in the DTS.

For example, to modify the operating speed of the i2c4 controller:

- If 3.4 MHz is required, configure the following parameters in the corresponding `i2c4` node in the board's DTS:

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

- If you need to use 1MHz, configure the following parameters in the corresponding `i2c4` node in the board's DTS:
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

- If you need to use 400kHz, configure the following parameters in the corresponding `i2c4` node in the board's DTS:


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

<font color="red">Note:</font> The parameters "i2c-scl-falling-time-ns" and "i2c-sda-falling-time-ns" may need to be adjusted using an oscilloscope depending on the PCB design. The above configurations assume that the I2C IO has pull-up enabled and that a 1k pull-up resistor is used on the board.

**I2C DMA Configuration**

To configure I2C to use DMA, refer to the following DTS configuration:
```dts
&i2c4 {
    dma-enable = <1>;  // Enable DMA for I2C
};

```c
&i2c4 {
 	status = "okay";
 	pinctrl-names = "default";
 	pinctrl-0 = <&pinctrl_i2c4>;
	dma-names = "tx", "rx";
	dmas = <&axi_dmac 19>, <&axi_dmac 18>;
}
```

**Note:** I2C7 is located in the DSP subsystem. In the DTS, DMA needs to be bound to the DSP subsystem's DMA:


```c
&i2c4 {
 	status = "okay";
 	pinctrl-names = "default";
 	pinctrl-0 = <&pinctrl_i2c4>;
	dma-names = "tx", "rx";
	dmas = <&dsp_axi_dma 31>, <&dsp_axi_dma 30>;
}
```

I2C DMA Handshake List:

| I2C   | RX  | TX  |
| ----- | --- | --- |
| I2C0  | 10  | 11  |
| I2C1  | 12  | 13  |
| I2C2  | 14  | 15  |
| I2C3  | 16  | 17  |
| I2C4  | 18  | 19  |
| I2C5  | 39  | 40  |
| I2C6  | 41  | 42  |
| I2C7  | 30  | 31  |

### User Space

Normally, I2C devices are controlled by the kernel driver, but all devices on the bus can also be accessed from user space via the `/dev/i2c-%d` interface. For detailed information, refer to the `Documentation/i2c/dev-interface` file in the Kernel.

**Open Source Tool: i2c-tools**

The `i2c-tools` is a set of open-source tools that have been cross-compiled and included in the rootfs of the X3J3 system software. Customers can use these tools directly:

- `i2cdetect` — Used to list all I2C buses and the devices on them.
- `i2cdump` — Displays all register values of an I2C device.
- `i2cget` — Reads the value of a specific register of an I2C device.
- `i2cset` — Writes a value to a specific register of an I2C device.
- `i2ctransfer` — Allows reading and writing multiple registers of an I2C device.
