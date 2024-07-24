---
sidebar_position: 6
---
# IO-DOMAIN Debugging Guide

The IO-Domain is used to configure the voltage domain of certain modules in X3J3. Taking the RGMII interface as an example, if the external voltage domain is 3.3V in the circuit design, the IO-DOMAIN of the RGMII module needs to be configured as 3.3V. If the external voltage domain is 1.8V, it needs to be configured as 1.8V. It is important to note that:

- Configuring the IO-DOMAIN as 1.8V when the external voltage domain is 3.3V may cause damage to the chip.
- Configuring the IO-DOMAIN as 3.3V when the external voltage domain is 1.8V may result in the module not working properly.

## Pin Query

The pin multiplexing and configuration of IO pins can be found in the "PL-2500-3-X3 PIN SW Reg-V1.2.xls" and "RM-2500-5-X3M Register Reference Manual-GPIO&PIN-V1.1.pdf" in the [datasheets](http://sunrise.horizon.cc/downloads/datasheets/) section.

In the "PL-2500-3-X3 PIN SW Reg-V1.2.xls", you can intuitively query the power-on default status, multiplexing, drive strength, pull-up/down, and Schmitt trigger configurations of the pins.

In the "RM-2500-5-X3M Register Reference Manual-GPIO&PIN-V1.1.pdf", refer to the SD_MODE_CTRL and IO_MODE_CTRL registers to determine the voltage domain configuration.

## Driver Code

### Code Location

```bash
drivers/pinctrl/pinctrl-single.c # pinctrl driver source file
include/linux/platform_data/pinctrl-single.h # pinctrl driver header file
```

### IO-DOMAIN DTS

```c
/* arch/arm64/boot/dts/hobot/hobot-pinctrl-xj3.dtsi */
/* pinctrl_voltage used to config X/J3 pin mode, for example,
* when SD2 external power supply is 3.3v, we need config pin-mode to
* 3.3v, otherwise X/J3 chip will be damaged.
* when SD2 external power supply is 1.8v, we need config pin-mode to
* 1.8v, otherwise SD2 will not work.
*/
pinctrl_voltage: pinctrl_voltag@0xA6003000 {
    compatible = "pinctrl-single";
    reg = <0x0 0xA6003170 0x0 0x8>;
    #pinctrl-cells = <2>;
    #gpio-range-cells = <0x3>;
    pinctrl-single,bit-per-mux;
    pinctrl-single,register-width = <32>;
    pinctrl-single,function-mask = <0x1>;
    status = "okay";
    /* rgmii 1.8v func */
        rgmii_1_8v_func: rgmii_1_8v_func {
            pinctrl-single,bits = <
                0x4 MODE_1_8V RGMII_MODE_P1
                0x4 MODE_1_8V RGMII_MODE_P0
                >;
        };
    /*rgmii 3.3v func */
        rgmii_3_3v_func: rgmii_3_3v_func {
            pinctrl-single,bits = <
                0x4 MODE_3_3V RGMII_MODE_P1
                0x4 MODE_3_3V RGMII_MODE_P0
                >;
        };
    ...
};
```

Since the IO-DOMAIN is implemented under the framework of Pinctrl-single, its DTS is similar to Pinctrl. In the DTS of IO-DOMAIN, all configuration groups for the 1.8V and 3.3V modules have been listed. Customers generally do not need to modify them. During specific development, they can choose to use them according to the actual situation.

### DTS Configuration for Driver Invocation

Similar to the usage of Pinctrl, the driver references the IO-DOMAIN that needs to be configured in its own DTS. Taking the bt1120 driver as an example, the configuration is as follows:

```c
xxx: xxx@0xA6000000 {
    ...
    pinctrl-names = "default", "xxx_voltage_func", ;
    pinctrl-0 = <&xxx_func>;
    pinctrl-1 = <&xxx_1_8v_func>; // pinctrl-3 is the configuration for 1.8V IO-DOMAIN
    ...
};
```

### Example Code for Driver Invocation

Similar to Pinctrl, the driver first looks up the corresponding pinctrl state through the Pinctrl-names and then switches to the corresponding state.

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
        /* Look up state according to pinctrl-names */
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

## Modify voltage domain under uboot

In the uboot source code, in the file board/hobot/xj3/xj3.c, the init_io_vol interface is called to configure the voltage domain based on the actual hardware voltage conditions. If the power domain of the pins on the hardware is 1.8V, then the bit corresponding to the pins is set to 1. If it is 3.3V, then the bit corresponding to the pins is set to 0. Finally, the hex value value is written to base+0x170 and base+0x174 (base: 0xA6003000). For detailed register information, please refer to "RM-2500-5-X3 Register Reference Manual-GPIO&PIN-V1.1.pdf".

```c
int init_io_vol(void)
{
    uint32_t value = 0;
    uint32_t base_board_id = 0;
    struct hb_info_hdr *bootinfo = (struct hb_info_hdr*)HB_BOOTINFO_ADDR;

    hb_board_id = bootinfo->board_id;
    /* work around solution for xj3 bring up ethernet,
     * all io to v1.8 except bt1120
     * BIFSPI and I2C2 is 3.3v in J3DVB, the other is 1.8v
     */
    /*
     * 1'b0=3.3v mode;  1'b1=1.8v mode
     * 0x170 bit[3]       sd2
     *       bit[2]       sd1
     *       bit[1:0]     sd0
     *
     * 0x174 bit[11:10]   rgmii
     *       bit[9]       i2c2
     *       bit[8]       i2c0
     *       bit[7]       reserved
     *       bit[6:4]     bt1120
     *       bit[3:2]     bifsd
     *       bit[1]       bifspi
     *       bit[0]       jtag
     */
    value = 0xF0F;
    base_board_id = hb_base_board_type_get();
    if (base_board_id == BASE_BOARD_J3_DVB) {
        value = 0xD0D;writel(value, GPIO_BASE + 0x174);
writel(0xF, GPIO_BASE + 0x170);
return 0;