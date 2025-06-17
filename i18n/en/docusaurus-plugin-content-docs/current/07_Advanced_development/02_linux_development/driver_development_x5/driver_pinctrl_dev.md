---
sidebar_position: 5
---

# Pinctrl Debugging Guide

The X5 Pinctrl is driven by custom-developed Pinctrl code, primarily configured through the DTS (Device Tree Source) to enable Pinctrl functionality.

## Pin Query

You can check the IO pin multiplexing and configuration, including the default state on power-up, pin multiplexing, drive strength, pull-up/pull-down, Schmitt trigger configuration, and the corresponding GPIO register information in the [datasheets](https://archive.d-robotics.cc/downloads/datasheets/) by referring to the document "X5 PIN SW Reg-V1.1.xlsx".

### Driver Code


```bash
kernel/drivers/pinctrl/hobot/aon.c  # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/common.c  # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/display.c  # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/dsp.c  # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/hsio.c  # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/lsio.c # Pinctrl driver source file
kernel/drivers/pinctrl/hobot/common.h  # Pinctrl driver header file
```
### Kernel Configuration

To enable Pinctrl, ensure the `CONFIG_PINCTRL_SINGLE` option is selected in the kernel configuration.

![Pinctrl Kernel Menuconfig](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/pinctrl-kernel-menuconfig.png)

### Pinctrl DTS Configuration

The device tree definition for the X5 Pinctrl controller is located in the `arch/arm64/boot/dts/hobot/x5.dtsi` file under the SDK package's kernel folder.

<font color="red">Note:</font> The nodes in `x5.dtsi` mainly declare SoC-level features and are not specific to any particular circuit board. In general, these nodes do not need to be modified.

## Pinctrl Usage

### Driver DTS Configuration

Before using Pinctrl interfaces in the driver, the corresponding Pinctrl configuration group needs to be specified in the DTS. When the driver probes, it will configure the Pinctrl settings that correspond to the "default" group into the registers. Other configuration groups must be parsed from the code, and then switched to the appropriate group. For example, for `uart2`:



```c
/* arch/arm64/boot/dts/hobot/x5-evb.dts */
&uart2 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_uart2>;
};
```

The configuration group referenced by `uart2` Pinctrl is located in `arch/arm64/boot/dts/hobot/pinmux-func.dtsi`, with the following content:


```c
/* arch/arm64/boot/dts/hobot/pinmux-func.dtsi */
	pinctrl_uart2: uart2grp {
		horizon,pins = <
			LSIO_UART2_RX  LSIO_PINMUX_3 BIT_OFFSET16  MUX_ALT0 &pconf_drv_pu_mid_3v3
			LSIO_UART2_TX  LSIO_PINMUX_3 BIT_OFFSET18  MUX_ALT0 &pconf_drv_pu_mid_3v3
		>;
	};
};
```
Additionally, GPIO functionality configuration is provided in `arch/arm64/boot/dts/hobot/pinmux-gpio.dtsi`, with the following content:

```c
/* arch/arm64/boot/dts/hobot/pinmux-gpio.dtsi */
	lsio_gpio0_8: lsio_gpio0_8 {
		horizon,pins = <
			LSIO_UART2_RX	LSIO_PINMUX_3 BIT_OFFSET16	MUX_ALT2	&pconf_input_en_3v3
		>;
	};

	lsio_gpio0_9: lsio_gpio0_9 {
		horizon,pins = <
			LSIO_UART2_TX	LSIO_PINMUX_3 BIT_OFFSET18	MUX_ALT2	&pconf_input_en_3v3
		>;
	};
```

The Pinctrl configuration group combines multiple pin configurations into a single set. Each pin's configuration contains five columns:

1. **Pin ID**: The ID for the related domain's pin, defined in header files such as `kernel/include/dt-bindings/pinctrl/horizon-*-pinfunc.h`.
2. **Pin Function Multiplexing Register**: The register for the pin's function multiplexing configuration.
3. **Bit Offset**: The bit offset for the multiplexing configuration in the register.
4. **Pin-mux Value**: The value representing the corresponding function for the pin.
5. **Additional Configurations**: Includes settings for default power-up state, drive strength, pull-up/pull-down, Schmitt trigger configuration, and voltage domain.

### Pin-mux Configuration

Each pin on the X5 chip supports up to four functions. When configuring a particular function, you can refer to the `X5 PIN SW Reg-V1.1.xlsx` document to check the corresponding Pin-mux value. For example, for pin LSIO_UART7_RX:

- When the Pin is configured with `MUX_ALT0`, the function is `uart rx`.
- When the Pin is configured with `MUX_ALT2`, the function is `gpio`.


```c
/* The macro defines the mux mode of horizon pinctrl */
#define MUX_ALT0 0x0
#define MUX_ALT1 0x1
#define MUX_ALT2 0x2
#define MUX_ALT3 0x3
```

### Drive Strength Configuration

Each pin on the X5 chip supports configuring the maximum output current. The default configuration can be set in `arch/arm64/boot/dts/hobot/pinmux-func.dtsi`, and this configuration is then referenced when configuring individual pins. 

X5 pins are categorized into two types: **GPIO type** and **SDIO type**. The same bit configuration represents different drive current levels depending on the pin type. For detailed information on the types and the drive current corresponding to the drive bits, please consult with Horizon Robotics support.


```c
pconf_drv_pu_mid_3v3: pconf-dev-pu-mid-3v3 {
    bias-pull-up;
    power-source = <HORIZON_IO_PAD_VOLTAGE_3V3>;
    drive-strength = <7>;
};
```

<font color="red">Note:</font> Configuring excessive drive strength may cause damage to the chip's pins! Please configure with caution!

### Pull-up/Pull-down Configuration

Each pin on the X5 chip also supports pull-up and pull-down configurations. The default settings can be specified in `arch/arm64/boot/dts/hobot/pinmux-func.dtsi`, and these configurations are referenced when setting up individual pins.


```c
pconf_drv_pu_mid_3v3: pconf-dev-pu-mid-3v3 {
    bias-pull-up;
    power-source = <HORIZON_IO_PAD_VOLTAGE_3V3>;
    drive-strength = <7>;
};
```
Supported Pull-up/Pull-down Configurations:
```c
bias-pull-up
bias-disable
bias-pull-down
```

### Schmitt Trigger Configuration

Each pin on the X5 chip also supports Schmitt trigger configuration. The default settings for the Schmitt trigger can be specified in the `arch/arm64/boot/dts/hobot/pinmux-func.dtsi` file, and these configurations are applied when setting up individual pins.

```c
pconf_drv_pu_mid_3v3: pconf-dev-pu-mid-3v3 {
    input-schmitt-enable
    power-source = <HORIZON_IO_PAD_VOLTAGE_3V3>;
    drive-strength = <7>;
};
```

### Voltage Domain Configuration

Each pin on the X5 chip also supports voltage domain configuration, allowing for 1.8V and 3.3V voltage domains. The default configuration can be set in the `arch/arm64/boot/dts/hobot/pinmux-func.dtsi` file, and these settings are referenced when configuring individual pins.


```c
#define  HORIZON_IO_PAD_VOLTAGE_3V3 0
#define  HORIZON_IO_PAD_VOLTAGE_1V8 1

pconf_drv_pu_mid_3v3: pconf-dev-pu-mid-3v3 {
    bias-pull-up;
    power-source = <HORIZON_IO_PAD_VOLTAGE_3V3>;
    drive-strength = <7>;
};
```

### Driver Example Code

The driver first uses the `Pinctrl-names` property to look up the corresponding Pinctrl state, and then switches to that state. Here's an example of how to do this:


```c
static int hobot_xxx_probe(struct platform_device *pdev)
{
    ...
    g_xxx_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(g_xxx_dev->pinctrl)) {
        dev_warn(&pdev->dev, "pinctrl get none\n");
        g_xxx_dev->pins_xxxx = NULL;
    }
    ...
        /* Lookup pinctrl-names lookup state */
        g_xxx_dev->pins_xxxx = pinctrl_lookup_state(g_xxx_dev->pinctrl, "xxx_func");
    if (IS_ERR(g_xxx_dev->pins_xxxx)) {
        dev_info(&pdev->dev, "xxx_func get error %ld\n",
                PTR_ERR(g_xxx_dev->pins_xxxx));
        g_xxx_dev->pins_xxxx = NULL;
    }
    ...
}
int xxxx_pinmux_select(void)
{
    if (!g_xxx_dev->pins_xxxx)
        return -ENODEV;
    /* change to correspond  state */
        return pinctrl_select_state(g_xxx_dev->pinctrl, g_xxx_dev->pins_xxxx);
}
```

### User Space Debugging

If the `CONFIG_DEBUG_FS` option is enabled in the kernel configuration and the `debugfs` file system is mounted, the kernel provides a debugfs interface for Pinctrl.

First, check if `debugfs` is already mounted. If the following command outputs non-empty data, it means that `debugfs` is currently mounted:


```shell
mount | grep debugfs
```

If no output is returned, you can mount `debugfs` manually with the following command:


```shell
mount -t debugfs none /sys/kernel/debug
```

Once `debugfs` is successfully mounted, the user can view Pinctrl information under the `debugfs` directory in user space. 
For example, for the LSIO subsystem, the debugfs node path is：`/sys/kernel/debug/pinctrl/34180000.lsio_iomuxc/`

The nodes under this directory can view the Pinctrl information of the lsio domain pins (the following command output is for reference only; specific nodes are subject to the Linux kernel code):
```shell
ls /sys/kernel/debug/pinctrl/34180000.lsio_iomuxc/
gpio-ranges  pinconf-groups  pinconf-pins  pingroups  pinmux-functions  pinmux-pins  pinmux-select  pins
```

Taking "pinmux-pins" as an example, this node shows which pins are configured as configuration groups. (The following command output is for reference only, and the actual output may vary depending on the DTS configuration.)


```shell
cat /sys/kernel/debug/pinctrl/34180000.lsio_iomuxc/pinmux-pins
pin 14 (lsio_spi0_sclk): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 36 (lsio_spi0_ssn): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 37 (lsio_spi0_miso): vdd08_gpu_reg@3 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio1_5
pin 38 (lsio_spi0_mosi): soc:cam:vcon@3 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio1_6
pin 15 (lsio_spi1_ssn_1): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 16 (lsio_spi1_sclk): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 17 (lsio_spi1_ssn): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 18 (lsio_spi1_miso): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 19 (lsio_spi1_mosi): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 20 (lsio_spi2_sclk): 34020000.spi (GPIO UNCLAIMED) function lsio_iomuxc group spi2grp
pin 21 (lsio_spi2_ssn): 34020000.spi (GPIO UNCLAIMED) function lsio_iomuxc group spi2grp
pin 22 (lsio_spi2_miso): 34020000.spi (GPIO UNCLAIMED) function lsio_iomuxc group spi2grp
pin 23 (lsio_spi2_mosi): 34020000.spi (GPIO UNCLAIMED) function lsio_iomuxc group spi2grp
pin 24 (lsio_spi3_sclk): 3d060000.mipi_host (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_24
pin 25 (lsio_spi3_ssn): 34110000.lpwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_lpwm1_1
pin 26 (lsio_spi3_miso): 3d080000.mipi_host (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_26
pin 27 (lsio_spi3_mosi): 3d090000.mipi_host (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_27
pin 28 (lsio_spi4_sclk): 341a0000.serial (GPIO UNCLAIMED) function lsio_iomuxc group uart5grp
pin 29 (lsio_spi4_ssn): 341a0000.serial (GPIO UNCLAIMED) function lsio_iomuxc group uart5grp
pin 30 (lsio_spi4_miso): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 31 (lsio_spi4_mosi): 35010000.gmac-tsn (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_31
pin 32 (lsio_spi5_sclk): 34100000.lpwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_lpwm0_0
pin 33 (lsio_spi5_ssn): 34100000.lpwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_lpwm0_1
pin 34 (lsio_spi5_miso): 34100000.lpwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_lpwm0_2
pin 35 (lsio_spi5_mosi): 34100000.lpwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_lpwm0_3
pin 39 (lsio_i2c0_scl): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 40 (lsio_i2c0_sda): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 41 (lsio_i2c1_scl): 34170000.pwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_pwm3_0
pin 42 (lsio_i2c1_sda): 34170000.pwm (GPIO UNCLAIMED) function lsio_iomuxc group pinctrl_pwm3_1
pin 43 (lsio_i2c2_scl): 340d0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c2grp
pin 44 (lsio_i2c2_sda): 340d0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c2grp
pin 45 (lsio_i2c3_scl): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 46 (lsio_i2c3_sda): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 47 (lsio_i2c4_scl): 340f0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c4grp
pin 48 (lsio_i2c4_sda): 340f0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c4grp
pin 0 (lsio_uart7_rx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_0
pin 1 (lsio_uart7_tx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_1
pin 3 (lsio_uart7_rts): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_3
pin 2 (lsio_uart7_cts): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_2
pin 4 (lsio_uart1_rx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_4
pin 5 (lsio_uart1_tx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_5
pin 7 (lsio_uart1_rts): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_7
pin 6 (lsio_uart1_cts): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_6
pin 8 (lsio_uart2_rx): 34080000.serial (GPIO UNCLAIMED) function lsio_iomuxc group uart2grp
pin 9 (lsio_uart2_tx): 34080000.serial (GPIO UNCLAIMED) function lsio_iomuxc group uart2grp
pin 10 (lsio_uart3_rx): 341c0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c5grp
pin 11 (lsio_uart3_tx): 341c0000.i2c (GPIO UNCLAIMED) function lsio_iomuxc group i2c5grp
pin 12 (lsio_uart4_rx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_12
pin 13 (lsio_uart4_tx): gpiochip8 (GPIO UNCLAIMED) function lsio_iomuxc group lsio_gpio0_13
```
