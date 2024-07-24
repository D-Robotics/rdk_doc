---
sidebar_position: 6
---

# IO-DOMAIN调试指南

IO-Domain用来配置X3J3部分模块的电压域，以RGMII接口为例，如果电路设计时外接电压域为3.3V，则需要配置RGMII模块的IO-DOMAIN为3.3V，如果电路设计时外接电压域为1.8V，则需要配置为1.8v，需要注意的是：

-   外接电压域为3.3V而对应的IO-DOMAIN配置为1.8V时，可能会对芯片有损伤；
-   外接电压域为1.8V而对应的IO-DOMAIN配置为3.3V时，相应的模块可能无法正常工作；

## 管脚查询

IO管脚的复用和配置可以在 [datasheets](https://sunrise.horizon.cc/downloads/datasheets/) 查阅《PL-2500-3-X3 PIN SW Reg-V1.2.xls》 和《RM-2500-5-X3M Register Reference Manual-GPIO&PIN-V1.1.pdf》。

在 《PL-2500-3-X3 PIN SW Reg-V1.2.xls》可以比较直观的查询到管脚的上电默认状态、复用、驱动能力、上下拉、施密特触发配置。

在 《RM-2500-5-X3M Register Reference Manual-GPIO&PIN-V1.1.pdf》文档中查阅 SD_MODE_CTRL 和 IO_MODE_CTRL 两个寄存器来确定电压域配置。

## 驱动代码

### 代码位置

```bash
drivers/pinctrl/pinctrl-single.c # pinctrl 驱动代码源文件
include/linux/platform_data/pinctrl-single.h # pinctrl 驱动代码头文件
```

### IO-DOMAIN的DTS

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

由于IO-DOMAIN在Pinctrl-single的框架下实现，因此其DTS和Pinctrl的类似，在IO-DOMAIN的DTS里已经列出了所有模块1.8V和3.3V的配置组，客户一般不需要修改，在具体开发时根据实际情况选择使用即可。

### 驱动调用时DTS配置

和Pinctrl的使用方法类似，驱动在自己的DTS中引用需要配置的IO-DOMAIN，以bt1120驱动为例，配置如下：

```c
xxx: xxx@0xA6000000 {
    ...
    pinctrl-names = "default", "xxx_voltage_func", ;
    pinctrl-0 = <&xxx_func>;
    pinctrl-1 = <&xxx_1_8v_func>; // pinctrl-3为1.8v的IO-DOMAIN配置
    ...
};
```

### 驱动调用示例代码

和Pinctrl调用方法一致，驱动先通过Pinctrl-names查找对应的pinctrl state，然后再切换到对应的state。

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
        /* 按照pinctrl-names lookup state */
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

## uboot下修改电压域

在uboot源码 board/hobot/xj3/xj3.c 文件中，根据硬件实际电压情况，调用init_io_vol接口配置电压域，如果硬件上面管脚的电源域是1.8v那么改管脚对应的位是1，如果是3.3v则该管脚对应的bit是0，最后面把拼成的16进制值value写入base+0x170和base+0x174中（base： 0xA6003000），寄存器详细说明可以查阅《RM-2500-5-X3 Register Reference Manual-GPIO&PIN-V1.1.pdf》

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
        value = 0xD0D;
    }
    writel(value, GPIO_BASE + 0x174);
    writel(0xF, GPIO_BASE + 0x170);
    return 0;
}
```
