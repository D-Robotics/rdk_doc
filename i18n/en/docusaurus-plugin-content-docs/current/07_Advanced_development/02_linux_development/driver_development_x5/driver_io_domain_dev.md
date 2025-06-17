---
sidebar_position: 6
---

# IO-DOMAIN Debugging Guide

IO-Domain is used to configure the voltage domains for the X5 module. Taking the RGMII interface as an example, if the external voltage domain is designed to be 3.3V, the IO-DOMAIN for the RGMII module needs to be set to 3.3V. If the external voltage domain is 1.8V, the configuration should be 1.8V. It is important to note the following:

- If the external voltage domain is 3.3V but the corresponding IO-DOMAIN is configured to 1.8V, it may cause damage to the chip.
- If the external voltage domain is 1.8V but the corresponding IO-DOMAIN is configured to 3.3V, the module may fail to function properly.

## Driver Code

### Code Location


```bash
drivers/pinctrl/hobot/ # Directory containing the source files for the pinctrl driver
include/linux/platform_data/pinctrl-single.h # Header file for the pinctrl driver
```

### IO-DOMAIN DTS

The Pinctrl function-related definitions for the X5 are located in the `pinmux-func.dtsi` file under the `arch/arm64/boot/dts/hobot/` folder within the SDK package kernel directory.

Since IO-Domain is implemented within the pinctrl-single framework, its DTS configuration is similar to that of pinctrl. In the IO-Domain DTS, all the 1.8V and 3.3V configuration groups for the modules are already listed. Typically, customers do not need to modify this. During development, they can simply choose the appropriate configuration based on the actual requirements.

A special case is the configuration for SD and SDIO IO-Domain groups. Since the voltage domain for SD and SDIO are configured simultaneously on the chip’s PAD, we use the MMC controller for SD and SDIO to independently control the voltage domain of the SD/SDIO pins. The configuration option is the custom "power-source" macro, as shown below:


```c
...
	pconf_sd_sdio_ds12_padctrl_3v3: pconf-sd-sdio-ds12-padctrl-3v3 {
		bias-pull-up;
		power-source = <HORIZON_IO_PAD_CTRL_VOLTAGE_3V3>;
		drive-strength = <12>;
	};

	pconf_sd_sdio_pu_ds12_padctrl_1v8: pconf-sd-sdio-ds12-padctrl-1v8 {
		bias-pull-up;
		power-source = <HORIZON_IO_PAD_CTRL_VOLTAGE_1V8>;
		drive-strength = <12>;
	};
...

	pinctrl_sd: sdgrp {
		horizon,pins = <
			HSIO_SD_WP		HSIO_PINMUX_2	BIT_OFFSET24	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_CLK		HSIO_PINMUX_2	BIT_OFFSET22	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_CMD		HSIO_PINMUX_2	BIT_OFFSET20	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_CDN		HSIO_PINMUX_2	BIT_OFFSET18	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_DATA0	HSIO_PINMUX_2	BIT_OFFSET16	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_DATA1	HSIO_PINMUX_2	BIT_OFFSET14	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_DATA2	HSIO_PINMUX_2	BIT_OFFSET12	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SD_DATA3	HSIO_PINMUX_2	BIT_OFFSET10	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
		>;
	};

	pinctrl_sdio: sdiogrp {
		horizon,pins = <
			HSIO_SDIO_WP	HSIO_PINMUX_2	BIT_OFFSET8	MUX_ALT0		&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_CLK	HSIO_PINMUX_0	BIT_OFFSET30	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_CMD	HSIO_PINMUX_0	BIT_OFFSET28	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_DATA0	HSIO_PINMUX_1	BIT_OFFSET0		MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_DATA1	HSIO_PINMUX_2	BIT_OFFSET30	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_DATA2	HSIO_PINMUX_2	BIT_OFFSET28	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
			HSIO_SDIO_DATA3	HSIO_PINMUX_2	BIT_OFFSET26	MUX_ALT0	&pconf_sd_sdio_pu_ds12_ipctrl
		>;
	};
...
```
### DTS Configuration for Driver Call

First, define the special voltage domain configuration for the current hardware in the DTS. For example, to configure PWM0 channel 1 to 1.8V:


```c
&lsio_iomuxc {
    pinctrl_pwm0_1_1v8: pinctrl-pwm0-1_1v8 {
        horizon,pins = <
            LSIO_SPI2_SSN	LSIO_PINMUX_1	BIT_OFFSET18	MUX_ALT3	&pconf_pwm_1v8
        >;
    };
}
```

Similar to the usage of pinctrl, the driver references the IO-Domain that needs to be configured in its own DTS. For example, for the PWM driver, the configuration is as follows:
：

```c
&pwm0 {
    ...
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_pwm0_1_1v8>; /* Configure pwm0 Ch1 to 1.8V voltage domain, PWM0 function */

    ...
};
```

### Driver Call Example Code

Similar to the Pinctrl invocation method, the driver first searches for the corresponding pinctrl state using Pinctrl-names, and then switches to the appropriate state.


```c
static int hobot_xxx_probe(struct platform_device *pdev)
{
    ...
    g_xxx_dev->pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(g_xxx_dev->pinctrl)) {
        dev_warn(&pdev->dev, "pinctrl get none\n");
        g_xxx_dev->pins_voltage = NULL;
    }
    ...
        

        /* Lookup state according to pinctrl-names */
        g_xxx_dev->pins_voltage = pinctrl_lookup_state(g_xxx_dev->pinctrl,
                "xxx_voltage_func");
    if (IS_ERR(g_xxx_dev->pins_voltage)) {
        dev_info(&pdev->dev, "xxx_voltage_func get error %ld\n",
                PTR_ERR(g_xxx_dev->pins_voltage));
        g_xxx_dev->pins_voltage = NULL;
    }
    ...
        /* select state */
        if (g_xxx_dev->pins_voltage) {
            ret = pinctrl_select_state(g_xxx_dev->pinctrl, g_xxx_dev->pins_voltage);
            if (ret) {
                dev_info(&pdev->dev, "xxx_voltage_func set error %d\n", ret);
            }
        }
    ...
}
```

## Modifying Voltage Domain in U-Boot

The Pinctrl driver is already implemented in X5's U-Boot. The usage is consistent with the kernel, and the specific properties of the pins are defined through DTS.
