---
sidebar_position: 5
---

# Pinctrl Debugging Guide

## Pinctrl Usage

The S100 chip contains three sys blocks with software-controllable pins. Users can configure pin multiplexing, pin attributes, and voltage domains according to their requirements.

### Pinmux Functionality

Pins in the SoC can be configured for different functions. Through pinmux settings, users can assign the desired function to each pin. Each pin supports four alternative functions.

### Pinconf Functionality

Pins in the SoC can be configured into different states, including the following attributes:

- Pull-up/pull-down configuration  
- Drive strength configuration  
- Voltage mode configuration  
- Input enable/disable configuration  
- Schmitt trigger configuration  
- Slew rate selection  

### IO-domain Functionality

The S100 supports two voltage domains: 1.8V and 3.3V. Different pins must operate within their appropriate voltage domains. The reference manual groups pins into power groups, and each power group controls a set of pins.

## Source Code Description

The file `pinctrl-hobot.c` contains the main logic for all S100-series pinctrl drivers. It registers the S100 pinctrl device with the pinctrl subsystem and provides registration support for the S100 pinctrl device.

``` {.shell}
/* Paths */
hobot-drivers/pinctrl/pinctrl-hobot.c     # Pinctrl driver source file
hobot-drivers/pinctrl/pinctrl-hobot.h     # Pinctrl driver header file
```

The file `pinctrl-hobot-s100.c` contains all pin definitions, pin groups, and pin function information for the S100.

``` {.shell}
/* Path */
hobot-drivers/pinctrl/pinctrl-hobot-s100.c     # S100 driver data source file
```

## Pinctrl Driver Usage Instructions

### Pinmux and Pinconf Configuration

Pinmux and pinconf are typically configured in the device DTS as follows:

``` C
&pinctrl_peri {
    pinctrl_test_default: pinctrl_test_default {
        /* Pinmux configuration: multiplex pins peri_i2c5_scl and peri_i2c5_sda to the peri_i2c5 function */
        pinmux {
            function = "peri_i2c5";
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
        };
        /* Pinconf configuration: set electrical attributes (drive strength, pull-up/down, input enable, voltage mode, slew rate, Schmitt trigger, etc.) for pins peri_i2c5_scl and peri_i2c5_sda */
        pinconf {
            /* Affected pins */
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
            /* Drive strength setting */
            drive-strength = <7>;
            /* Configure as pull-down */
            bias-pull-down;
            /* Disable input */
            input-disable;
            /* Disable low-voltage mode */
            low-power-disable;
            /* Set slew rate to 1 */
            slew-rate = <1>;
            /* Disable Schmitt trigger on input */
            input-schmitt-disable;
        };
    };

    pinctrl_test_sleep: pinctrl_test_sleep {
        pinmux {
            function = "peri_i2c5";
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
        };
        pinconf {
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
            drive-strength = <1>;
            bias-pull-up;
            input-enable;
            low-power-enable;
            slew-rate = <0>;
            input-schmitt-enable;
        };
    };
};
```

### Pinconf Supported Attributes and Corresponding Keywords in Device Tree

| Keyword | Attribute |
| --- | --- |
| bias-pull-up | Pull-up configuration |
| bias-pull-down | Pull-down configuration |
| bias-disable | Floating (no pull) configuration |
| drive-strength | Drive strength |
| low-power-disable | High-voltage mode (3.3V) |
| low-power-enable | Low-voltage mode (1.8V) |
| input-disable | Input disabled |
| input-enable | Input enabled |
| input-schmitt-enable | Schmitt trigger enabled |
| input-schmitt-disable | Schmitt trigger disabled |

### Voltage Domain Configuration

Incorrect IO-domain configuration for pins can directly affect normal module operation, and in severe cases, may even damage components and render the board inoperable.

Based on prior bring-up experience, the following practical steps are recommended to ensure consistency between hardware voltage levels and software IO-domain settings:

1. Hardware and software owners of each module should align and confirm the applicable IO-domain range for relevant pins, ensuring consistent requirements. They should also provide reference values in a document titled “Pin IO-domain Configuration Manual” (document name for reference only).
2. Based on the confirmed information from step 1, configure the corresponding IO-domain settings in the device tree.
3. After completing step 2, measure the actual IO-domain values of all pins on the prototype board and record these measured values in the “Pin IO-domain Configuration Manual” under the “Actual Values” section to identify any discrepancies (gaps).
4. Since one power group in the S100 controls multiple pins, configuring one pin may affect others in the same group. Therefore, hold alignment meetings to discuss and resolve any gaps identified in step 3 until all discrepancies are eliminated.

### Reference Pinctrl Node Configuration

The pinctrl device tree file path for the S100 acore is: `hobot-drivers/kernel-dts/drobot-s100-pinctrl.dtsi`.

In the pinctrl node, a group of pins is bundled together. Each node configuration includes multiple child nodes.

The first child node, `pinmux`, contains pin multiplexing information: `function` specifies the functional role assigned to the pins, and `pins` lists the pins used for this function.

The second child node, `pinconf`, contains pin configuration details (this node can be omitted if default values are acceptable). `pins` specifies the pins to be configured, and `pinconf_*` properties define the configuration parameters, such as drive strength, pull-up/down, voltage mode, etc.

``` C
&pinctrl_peri {
     pinctrl_test_default: pinctrl_test_default {
         pinmux {
             function = "peri_i2c5";
             pins = "peri_i2c5_scl", "peri_i2c5_sda";
         };
         pinconf {
             pins = "peri_i2c5_scl", "peri_i2c5_sda";
             drive-strength = <7>;
             bias-pull-down;
             input-disable;
             low-power-disable;
             slew-rate = <1>;
             input-schmitt-disable;
         };
     };

     pinctrl_test_sleep: pinctrl_test_sleep {
         pinmux {
             function = "peri_i2c5";
             pins = "peri_i2c5_scl", "peri_i2c5_sda";
         };
         pinconf {
             pins = "peri_i2c5_scl", "peri_i2c5_sda";
             drive-strength = <1>;
             bias-pull-up;
             input-enable;
             low-power-enable;
             slew-rate = <0>;
             input-schmitt-enable;
         };
     };
 };
```

### Node Reference

Before a driver uses pinctrl interfaces, the corresponding pinctrl node must be configured in the DTS. During driver probe, the configuration associated with the "default" state is applied to hardware registers. Other state configurations must be explicitly switched in code.

```dts
pinctrl_test: pinctrl_test {
        compatible = "horizon,s100-pinctrl-test";
        pinctrl-names = "default", "sleep";
        pinctrl-0 = <&pinctrl_test_default>;
        pinctrl-1 = <&pinctrl_test_sleep>;
        #address-cells = <1>;
        #size-cells = <0>;
};
```

To explicitly switch pinctrl states, call the `pinctrl_select_state` interface.

```C
/**
 * pinctrl_select_state() - select/activate/program a pinctrl state to HW
 * @p: the pinctrl handle for the device that requests configuration
 * @state: the state handle to select/activate/program
 */
int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state)
```

## Pinctrl debugfs Usage Instructions

:::info
Note: Non-root users need to prepend commands with "sudo".
:::

### Pin Information Query

#### /sys/kernel/debug/pinctrl/pinctrl-devicesCheck which pinctrl devices exist in the system and whether they use the pinmux and pinconf modules.

```shell
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/pinctrl-devices
name [pinmux] [pinconf]
peri yes yes
cam yes yes
video yes yes
```
#### /sys/kernel/debug/pinctrl/pinctrl-handles

View pin configuration mappings that have already been requested.

```shell
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/pinctrl-handles
 Requested pin control handlers their pinmux maps:
 device: 39420000.i2c current state: default
 state: default
     type: MUX_GROUP controller cam group: cam_i2c0_scl (8) function: cam_i2c0 (6)
     type: MUX_GROUP controller cam group: cam_i2c0_sda (9) function: cam_i2c0 (6)
     type: CONFIGS_GROUP controller cam group cam_i2c0_scl (8)config 00000109
     type: CONFIGS_GROUP controller cam group cam_i2c0_sda (9)config 00000109
 device: 39430000.i2c current state: default
 state: default
     type: MUX_GROUP controller cam group: cam_i2c1_scl (10) function: cam_i2c1 (7)
     type: MUX_GROUP controller cam group: cam_i2c1_sda (11) function: cam_i2c1 (7)
     type: CONFIGS_GROUP controller cam group cam_i2c1_scl (10)config 00000109
     type: CONFIGS_GROUP controller cam group cam_i2c1_sda (11)config 00000109
 device: 39440000.i2c current state: default
 state: default
     type: MUX_GROUP controller cam group: cam_i2c2_scl (12) function: cam_i2c2 (8)
     type: MUX_GROUP controller cam group: cam_i2c2_sda (13) function: cam_i2c2 (8)
     type: CONFIGS_GROUP controller cam group cam_i2c2_scl (12)config 00000109
     type: CONFIGS_GROUP controller cam group cam_i2c2_sda (13)config 00000109
 device: 39450000.i2c current state: default
 state: default
     type: MUX_GROUP controller cam group: cam_i2c3_scl (14) function: cam_i2c3 (9)
     type: MUX_GROUP controller cam group: cam_i2c3_sda (15) function: cam_i2c3 (9)
     type: CONFIGS_GROUP controller cam group cam_i2c3_scl (14)config 00000109
     type: CONFIGS_GROUP controller cam group cam_i2c3_sda (15)config 00000109

 ...

 root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/pinctrl-maps

View the pin maps in use, including the controller each used pin belongs to, which devices are using them, under which pinctrl state, which pin group they belong to, and what function they are multiplexed to.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/pinctrl-maps
Pinctrl maps:
device 39420000.i2c
state default
type MUX_GROUP (2)
controlling device 370f3000.pinctrl
group cam_i2c0_scl
function cam_i2c0

device 39420000.i2c
state default
type MUX_GROUP (2)
controlling device 370f3000.pinctrl
group cam_i2c0_sda
function cam_i2c0

device 39420000.i2c
state default
type CONFIGS_GROUP (4)
controlling device 370f3000.pinctrl
group cam_i2c0_scl
config 00000109

...

root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/gpio-ranges

View the mapping relationship between pin numbers in the pinctrl subsystem and GPIO numbers in the GPIO subsystem. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` for illustration.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/gpio-ranges
GPIO ranges handled:
0: 394f0000.gpio GPIOS [480 - 511] PINS [2 - 33]
0: 39500000.gpio GPIOS [474 - 479] PINS [34 - 39]
root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pinconf-pins

View information for each pin, including register addresses and values for pinmux and pinconf, whether voltage level shifting is supported, and the voltage value. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` for illustration.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pinconf-pins
Pin config settings per pin
Format: pin (name): configs
pin 0 (UFS_REF_CLK): type 2, pwr 0, funs:(0,20,20,20,), mux(39ff5060: 0), cfg(39ff6010: 31), pwr(39ff5040: 1)
pin 1 (UFS_RSTO): type 2, pwr 0, funs:(0,20,20,20,), mux(39ff5064: 0), cfg(39ff6014: 31), pwr(39ff5040: 1)
pin 2 (EMAC_MDC_HSI0): type 2, pwr 0, funs:(1,2,20,3,), mux(39ff5068: 0), cfg(39ff6018: 31), pwr(39ff5040: 1)
pin 3 (EMAC_MDIO_HSI0): type 3, pwr 0, funs:(1,20,20,3,), mux(39ff506c: 0), cfg(39ff601c: 32), pwr(39ff5040: 1)
pin 4 (SD_CLK): type 0, pwr 1, funs:(4,20,20,3,), mux(39ff5070: 0), cfg(39ff6020: 58), pwr(39ff5040: 1)
pin 5 (SD_CMD): type 0, pwr 1, funs:(4,20,20,3,), mux(39ff5074: 0), cfg(39ff6024: 58), pwr(39ff5040: 1)
pin 6 (SD_DATA0): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5078: 0), cfg(39ff6028: 58), pwr(39ff5040: 1)
pin 7 (SD_DATA1): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff507c: 0), cfg(39ff602c: 58), pwr(39ff5040: 1)
pin 8 (SD_DATA2): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5080: 0), cfg(39ff6030: 58), pwr(39ff5040: 1)
pin 9 (SD_DATA3): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5084: 0), cfg(39ff6034: 58), pwr(39ff5040: 1)
pin 10 (SD_DATA4): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5088: 0), cfg(39ff6038: 58), pwr(39ff5040: 1)
pin 11 (SD_DATA5): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff508c: 0), cfg(39ff603c: 58), pwr(39ff5040: 1)
pin 12 (SD_DATA6): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5090: 0), cfg(39ff6040: 58), pwr(39ff5040: 1)
pin 13 (SD_DATA7): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5094: 0), cfg(39ff6044: 58), pwr(39ff5040: 1)
pin 14 (SD_DATA_STRB): type 4, pwr 1, funs:(4,20,20,3,), mux(39ff5098: 0), cfg(39ff6048: 18), pwr(39ff5040: 1)
pin 15 (SD_DET_N): type 2, pwr 0, funs:(4,5,20,3,), mux(39ff509c: 3), cfg(39ff604c: 50), pwr(39ff5040: 1)
pin 16 (SD_WPROT): type 3, pwr 0, funs:(4,6,20,3,), mux(39ff50a0: 0), cfg(39ff6050: 50), pwr(39ff5040: 1)
pin 17 (I2C5_SCL): type 2, pwr 0, funs:(7,8,20,3,), mux(39ff50a4: 0), cfg(39ff6054: 31), pwr(39ff5040: 1)
pin 18 (I2C5_SDA): type 3, pwr 0, funs:(7,9,20,3,), mux(39ff50a8: 0), cfg(39ff6058: 32), pwr(39ff5040: 1)
pin 19 (SPI0_CSN0): type 2, pwr 0, funs:(10,20,20,3,), mux(39ff50ac: 0), cfg(39ff605c: 31), pwr(39ff5040: 1)
pin 20 (SPI0_CSN1): type 3, pwr 0, funs:(10,11,20,3,), mux(39ff50b0: 0), cfg(39ff6060: 32), pwr(39ff5040: 1)
pin 21 (SPI0_MOSI): type 3, pwr 0, funs:(10,20,20,3,), mux(39ff50b4: 0), cfg(39ff6064: 32), pwr(39ff5040: 1)
pin 22 (SPI0_MISO): type 3, pwr 0, funs:(10,20,20,3,), mux(39ff50b8: 0), cfg(39ff6068: 32), pwr(39ff5040: 1)
pin 23 (SPI0_SCLK): type 2, pwr 0, funs:(10,20,20,3,), mux(39ff50bc: 0), cfg(39ff606c: 31), pwr(39ff5040: 1)
pin 24 (SPI1_CSN0): type 3, pwr 0, funs:(12,20,20,3,), mux(39ff50c0: 0), cfg(39ff6070: 32), pwr(39ff5040: 1)
pin 25 (SPI1_CSN1): type 3, pwr 0, funs:(12,13,20,3,), mux(39ff50c4: 0), cfg(39ff6074: 32), pwr(39ff5040: 1)
pin 26 (SPI1_MOSI): type 3, pwr 0, funs:(12,20,20,3,), mux(39ff50c8: 0), cfg(39ff6078: 32), pwr(39ff5040: 1)
pin 27 (SPI1_MISO): type 3, pwr 0, funs:(12,20,20,3,), mux(39ff50cc: 0), cfg(39ff607c: 32), pwr(39ff5040: 1)
pin 28 (SPI1_SCLK): type 3, pwr 0, funs:(12,20,20,3,), mux(39ff50d0: 0), cfg(39ff6080: 32), pwr(39ff5040: 1)
pin 29 (UART0_TXD): type 3, pwr 0, funs:(14,20,20,3,), mux(39ff50d4: 0), cfg(39ff6084: 32), pwr(39ff5040: 1)
pin 30 (UART0_RXD): type 3, pwr 0, funs:(14,20,20,3,), mux(39ff50d8: 0), cfg(39ff6088: 32), pwr(39ff5040: 1)
pin 31 (UART0_RTSN): type 2, pwr 0, funs:(14,15,20,3,), mux(39ff50dc: 0), cfg(39ff608c: 31), pwr(39ff5040: 1)
pin 32 (UART0_CTSN): type 3, pwr 0, funs:(14,20,20,3,), mux(39ff50e0: 0), cfg(39ff6090: 32), pwr(39ff5040: 1)
pin 33 (UART1_TXD): type 3, pwr 0, funs:(16,17,20,3,), mux(39ff50e4: 0), cfg(39ff6094: 32), pwr(39ff5040: 1)
pin 34 (UART1_RXD): type 3, pwr 0, funs:(16,18,20,3,), mux(39ff50e8: 0), cfg(39ff6098: 32), pwr(39ff5040: 1)
pin 35 (PCM0_MCLK): type 2, pwr 0, funs:(19,20,20,3,), mux(39ff50ec: 3), cfg(39ff609c: 32), pwr(39ff5040: 1)
pin 36 (PCM0_BCLK): type 3, pwr 0, funs:(19,20,20,3,), mux(39ff50f0: 3), cfg(39ff60a0: 32), pwr(39ff5040: 1)
pin 37 (PCM0_FSYNC): type 3, pwr 0, funs:(19,20,20,3,), mux(39ff50f4: 3), cfg(39ff60a4: 32), pwr(39ff5040: 1)
pin 38 (PCM0_DATA0): type 3, pwr 0, funs:(19,20,20,3,), mux(39ff50f8: 3), cfg(39ff60a8: 32), pwr(39ff5040: 1)
pin 39 (PCM0_DATA1): type 2, pwr 0, funs:(19,20,20,3,), mux(39ff50fc: 3), cfg(39ff60ac: 32), pwr(39ff5040: 1)
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pinconf-groups

View electrical configuration attributes for each pin, such as pull-up/pull-down resistors, input enable, Schmitt trigger, slew rate, etc. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` for illustration.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pinconf-groups
Pin config settings per pin group
Format: group (name): configs
0 (peri_ufs_ref_clk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
1 (peri_ufs_rsto): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
2 (peri_emac_mdc_hsi0): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
3 (peri_emac_mdio_hsi0): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
4 (peri_sd_clk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (0 mA), input enabled, input schmitt enabled, pin low power (1 mode), slew rate (0)
5 (peri_sd_cmd): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (0 mA), input enabled, input schmitt enabled, pin low power (1 mode), slew rate (0)
6 (peri_sd_data0): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
7 (peri_sd_data1): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
8 (peri_sd_data2): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
9 (peri_sd_data3): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
10 (peri_sd_data4): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
11 (peri_sd_data5): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
12 (peri_sd_data6): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
13 (peri_sd_data7): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
14 (peri_sd_data_strb): input bias disabled, input bias pull down (0 ohms), input bias pull up (0 ohms), input enabled, pin low power (1 mode)
15 (peri_sd_det_n): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), output drive strength (0 mA), input enabled, input schmitt enabled, slew rate (0)
16 (peri_sd_wprot): input bias disabled, input bias pull down (1 ohms), input bias pull up (0 ohms), input enabled
17 (peri_i2c5_scl): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
18 (peri_i2c5_sda): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
19 (peri_spi0_csn0): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
20 (peri_spi0_csn1): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
21 (peri_spi0_mosi): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
22 (peri_spi0_miso): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
23 (peri_spi0_sclk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
24 (peri_spi1_csn0): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
25 (peri_spi1_csn1): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
26 (peri_spi1_mosi): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
27 (peri_spi1_miso): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
28 (peri_spi1_sclk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
29 (peri_uart0_txd): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
30 (peri_uart0_rxd): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
31 (peri_uart0_rtsn): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (1 mA), input enabled, input schmitt enabled, slew rate (0)
32 (peri_uart0_ctsn): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
33 (peri_uart1_txd): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
34 (peri_uart1_rxd): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
35 (peri_pcm0_mclk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (2 mA), input enabled, input schmitt enabled, slew rate (0)
36 (peri_pcm0_bclk): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
37 (peri_pcm0_fsync): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
38 (peri_pcm0_data0): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), input enabled
39 (peri_pcm0_data1): input bias disabled, input bias pull down (0 ohms), input bias pull up (1 ohms), output drive strength (2 mA), input enabled, input schmitt enabled, slew rate (0)
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pingroups

View pin configurations within each pin group. On the S100, each individual pin forms its own group. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` for illustration.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pingroups
registered pin groups:
group: peri_ufs_ref_clk
```pin 0 (UFS_REF_CLK)

group: peri_ufs_rsto
pin 1 (UFS_RSTO)

group: peri_emac_mdc_hsi0
pin 2 (EMAC_MDC_HSI0)

group: peri_emac_mdio_hsi0
pin 3 (EMAC_MDIO_HSI0)

group: peri_sd_clk
pin 4 (SD_CLK)

group: peri_sd_cmd
pin 5 (SD_CMD)

group: peri_sd_data0
pin 6 (SD_DATA0)

group: peri_sd_data1
pin 7 (SD_DATA1)

group: peri_sd_data2
pin 8 (SD_DATA2)

group: peri_sd_data3
pin 9 (SD_DATA3)

group: peri_sd_data4
pin 10 (SD_DATA4)

group: peri_sd_data5
pin 11 (SD_DATA5)

group: peri_sd_data6
pin 12 (SD_DATA6)

group: peri_sd_data7
pin 13 (SD_DATA7)

group: peri_sd_data_strb
pin 14 (SD_DATA_STRB)

group: peri_sd_det_n
pin 15 (SD_DET_N)

group: peri_sd_wprot
pin 16 (SD_WPROT)

group: peri_i2c5_scl
pin 17 (I2C5_SCL)

group: peri_i2c5_sda
pin 18 (I2C5_SDA)

group: peri_spi0_csn0
pin 19 (SPI0_CSN0)

group: peri_spi0_csn1
pin 20 (SPI0_CSN1)

group: peri_spi0_mosi
pin 21 (SPI0_MOSI)

group: peri_spi0_miso
pin 22 (SPI0_MISO)

group: peri_spi0_sclk
pin 23 (SPI0_SCLK)

group: peri_spi1_csn0
pin 24 (SPI1_CSN0)

group: peri_spi1_csn1
pin 25 (SPI1_CSN1)

group: peri_spi1_mosi
pin 26 (SPI1_MOSI)

group: peri_spi1_miso
pin 27 (SPI1_MISO)

group: peri_spi1_sclk
pin 28 (SPI1_SCLK)

group: peri_uart0_txd
pin 29 (UART0_TXD)

group: peri_uart0_rxd
pin 30 (UART0_RXD)

group: peri_uart0_rtsn
pin 31 (UART0_RTSN)

group: peri_uart0_ctsn
pin 32 (UART0_CTSN)

group: peri_uart1_txd
pin 33 (UART1_TXD)

group: peri_uart1_rxd
pin 34 (UART1_RXD)

group: peri_pcm0_mclk
pin 35 (PCM0_MCLK)

group: peri_pcm0_bclk
pin 36 (PCM0_BCLK)

group: peri_pcm0_fsync
pin 37 (PCM0_FSYNC)

group: peri_pcm0_data0
pin 38 (PCM0_DATA0)

group: peri_pcm0_data1
pin 39 (PCM0_DATA1)

root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pinmux-functions

Pins are grouped by function. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` as an example.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pinmux-functions
function 0: peri_ufs, groups = [ peri_ufs_ref_clk peri_ufs_rsto ]
function 1: peri_emac, groups = [ peri_emac_mdc_hsi0 peri_emac_mdio_hsi0 ]
function 2: peri_clkout, groups = [ peri_emac_mdc_hsi0 ]
function 3: peri_gpio, groups = [ peri_emac_mdc_hsi0 peri_emac_mdio_hsi0 peri_sd_clk peri_sd_cmd peri_sd_data0 peri_sd_data1 peri_sd_data2 peri_sd_data3 peri_sd_data4 peri_sd_data5 peri_sd_data6 peri_sd_data7 peri_sd_data_strb peri_sd_det_n peri_sd_wprot peri_i2c5_scl peri_i2c5_sda peri_spi0_csn0 peri_spi0_csn1 peri_spi0_mosi peri_spi0_miso peri_spi0_sclk peri_spi1_csn0 peri_spi1_csn1 peri_spi1_mosi peri_spi1_miso peri_spi1_sclk peri_uart0_txd peri_uart0_rxd peri_uart0_rtsn peri_uart0_ctsn peri_uart1_txd peri_uart1_rxd peri_pcm0_mclk peri_pcm0_bclk peri_pcm0_fsync peri_pcm0_data0 peri_pcm0_data1 ]
function 4: peri_sd, groups = [ peri_sd_clk peri_sd_cmd peri_sd_data0 peri_sd_data1 peri_sd_data2 peri_sd_data3 peri_sd_data4 peri_sd_data5 peri_sd_data6 peri_sd_data7 peri_sd_data_strb peri_sd_det_n peri_sd_wprot ]
function 5: peri_pcm1_bclk, groups = [ peri_sd_det_n ]
function 6: peri_pcm1_fsync, groups = [ peri_sd_wprot ]
function 7: peri_i2c5, groups = [ peri_i2c5_scl peri_i2c5_sda ]
function 8: peri_uart2_rxd, groups = [ peri_i2c5_scl ]
function 9: peri_uart2_txd, groups = [ peri_i2c5_sda ]
function 10: peri_spi0, groups = [ peri_spi0_csn0 peri_spi0_csn1 peri_spi0_mosi peri_spi0_miso peri_spi0_sclk ]
function 11: peri_uart3_rxd, groups = [ peri_spi0_csn1 ]
function 12: peri_spi1, groups = [ peri_spi1_csn0 peri_spi1_csn1 peri_spi1_mosi peri_spi1_miso peri_spi1_sclk ]
function 13: peri_uart3_txd, groups = [ peri_spi1_csn1 ]
function 14: peri_uart0, groups = [ peri_uart0_txd peri_uart0_rxd peri_uart0_rtsn peri_uart0_ctsn ]
function 15: peri_pcm1_mclk, groups = [ peri_uart0_rtsn ]
function 16: peri_uart1, groups = [ peri_uart1_txd peri_uart1_rxd ]
function 17: peri_pcm1_data1, groups = [ peri_uart1_txd ]
function 18: peri_pcm1_data0, groups = [ peri_uart1_rxd ]
function 19: peri_pcm0, groups = [ peri_pcm0_mclk peri_pcm0_bclk peri_pcm0_fsync peri_pcm0_data0 peri_pcm0_data1 ]
root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pinmux-pins

Check which pins are occupied and whether they are claimed by MUX or GPIO. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` as an example.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pinmux-pins
Pinmux settings per pin
Format: pin (name): mux_owner gpio_owner hog?
pin 0 (UFS_REF_CLK): 39410000.ufs (GPIO UNCLAIMED) function peri_ufs group peri_ufs_ref_clk
pin 1 (UFS_RSTO): 39410000.ufs (GPIO UNCLAIMED) function peri_ufs group peri_ufs_rsto
pin 2 (EMAC_MDC_HSI0): 330f0000.ethernet (GPIO UNCLAIMED) function peri_emac group peri_emac_mdc_hsi0
pin 3 (EMAC_MDIO_HSI0): 330f0000.ethernet (GPIO UNCLAIMED) function peri_emac group peri_emac_mdio_hsi0
pin 4 (SD_CLK): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 5 (SD_CMD): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 6 (SD_DATA0): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 7 (SD_DATA1): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 8 (SD_DATA2): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 9 (SD_DATA3): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 10 (SD_DATA4): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 11 (SD_DATA5): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 12 (SD_DATA6): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 13 (SD_DATA7): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 14 (SD_DATA_STRB): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 15 (SD_DET_N): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 16 (SD_WPROT): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 17 (I2C5_SCL): 39470000.i2c (GPIO UNCLAIMED) function peri_i2c5 group peri_i2c5_scl
pin 18 (I2C5_SDA): 39470000.i2c (GPIO UNCLAIMED) function peri_i2c5 group peri_i2c5_sda
pin 19 (SPI0_CSN0): 39800000.spi (GPIO UNCLAIMED) function peri_spi0 group peri_spi0_csn0
pin 20 (SPI0_CSN1): 39800000.spi (GPIO UNCLAIMED) function peri_spi0 group peri_spi0_csn1
pin 21 (SPI0_MOSI): 39800000.spi (GPIO UNCLAIMED) function peri_spi0 group peri_spi0_mosi
pin 22 (SPI0_MISO): 39800000.spi (GPIO UNCLAIMED) function peri_spi0 group peri_spi0_miso
pin 23 (SPI0_SCLK): 39800000.spi (GPIO UNCLAIMED) function peri_spi0 group peri_spi0_sclk
pin 24 (SPI1_CSN0): 39810000.spi (GPIO UNCLAIMED) function peri_spi1 group peri_spi1_csn0
pin 25 (SPI1_CSN1): 39810000.spi 394f0000.gpio:503 function peri_spi1 group peri_spi1_csn1
pin 26 (SPI1_MOSI): 39810000.spi (GPIO UNCLAIMED) function peri_spi1 group peri_spi1_mosi
pin 27 (SPI1_MISO): 39810000.spi (GPIO UNCLAIMED) function peri_spi1 group peri_spi1_miso
pin 28 (SPI1_SCLK): 39810000.spi (GPIO UNCLAIMED) function peri_spi1 group peri_spi1_sclk
pin 29 (UART0_TXD): 394a0000.uart (GPIO UNCLAIMED) function peri_uart0 group peri_uart0_txd
pin 30 (UART0_RXD): 394a0000.uart (GPIO UNCLAIMED) function peri_uart0 group peri_uart0_rxd
pin 31 (UART0_RTSN): 394a0000.uart (GPIO UNCLAIMED) function peri_uart0 group peri_uart0_rtsn
pin 32 (UART0_CTSN): 394a0000.uart (GPIO UNCLAIMED) function peri_uart0 group peri_uart0_ctsn
pin 33 (UART1_TXD): 394b0000.uart0 (GPIO UNCLAIMED) function peri_uart1 group peri_uart1_txd
pin 34 (UART1_RXD): 394b0000.uart0 (GPIO UNCLAIMED) function peri_uart1 group peri_uart1_rxd
pin 35 (PCM0_MCLK): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 36 (PCM0_BCLK): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 37 (PCM0_FSYNC): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 38 (PCM0_DATA0): (MUX UNCLAIMED) (GPIO UNCLAIMED)
pin 39 (PCM0_DATA1): (MUX UNCLAIMED) (GPIO UNCLAIMED)
```#### /sys/kernel/debug/\<pinctrl_dev>/pinmux-select

You can query the required group and function for each pin via the `pinmux-functions` node,  
and configure pinmux via the `pinmux-select` node using the parameters `group name` and `func name`.  
The following commands use `<pinctrl_dev>` as `39ff5000.pinctrl-peri` as an example.

``` {.shell}
cd /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/
echo  "peri_spi0_csn0 peri_gpio"  > pinmux-select
echo  "peri_i2c5_scl peri_i2c5"  > pinmux-select
echo  "peri_i2c5_sda peri_i2c5"  > pinmux-select
```

#### /sys/kernel/debug/\<pinctrl_dev>/pins

View information for each pin, including the pin name and its corresponding GPIO pin. The following command uses `<pinctrl_dev>` as `39ff5000.pinctrl-peri` as an example.

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pins
registered pins: 40
pin 0 (UFS_REF_CLK) 0:?
pin 1 (UFS_RSTO) 0:?
pin 2 (EMAC_MDC_HSI0) 0:394f0000.gpio
pin 3 (EMAC_MDIO_HSI0) 1:394f0000.gpio
pin 4 (SD_CLK) 2:394f0000.gpio
pin 5 (SD_CMD) 3:394f0000.gpio
pin 6 (SD_DATA0) 4:394f0000.gpio
pin 7 (SD_DATA1) 5:394f0000.gpio
pin 8 (SD_DATA2) 6:394f0000.gpio
pin 9 (SD_DATA3) 7:394f0000.gpio
pin 10 (SD_DATA4) 8:394f0000.gpio
pin 11 (SD_DATA5) 9:394f0000.gpio
pin 12 (SD_DATA6) 10:394f0000.gpio
pin 13 (SD_DATA7) 11:394f0000.gpio
pin 14 (SD_DATA_STRB) 12:394f0000.gpio
pin 15 (SD_DET_N) 13:394f0000.gpio
pin 16 (SD_WPROT) 14:394f0000.gpio
pin 17 (I2C5_SCL) 15:394f0000.gpio
pin 18 (I2C5_SDA) 16:394f0000.gpio
pin 19 (SPI0_CSN0) 17:394f0000.gpio
pin 20 (SPI0_CSN1) 18:394f0000.gpio
pin 21 (SPI0_MOSI) 19:394f0000.gpio
pin 22 (SPI0_MISO) 20:394f0000.gpio
pin 23 (SPI0_SCLK) 21:394f0000.gpio
pin 24 (SPI1_CSN0) 22:394f0000.gpio
pin 25 (SPI1_CSN1) 23:394f0000.gpio
pin 26 (SPI1_MOSI) 24:394f0000.gpio
pin 27 (SPI1_MISO) 25:394f0000.gpio
pin 28 (SPI1_SCLK) 26:394f0000.gpio
pin 29 (UART0_TXD) 27:394f0000.gpio
pin 30 (UART0_RXD) 28:394f0000.gpio
pin 31 (UART0_RTSN) 29:394f0000.gpio
pin 32 (UART0_CTSN) 30:394f0000.gpio
pin 33 (UART1_TXD) 31:394f0000.gpio
pin 34 (UART1_RXD) 0:39500000.gpio
pin 35 (PCM0_MCLK) 1:39500000.gpio
pin 36 (PCM0_BCLK) 2:39500000.gpio
pin 37 (PCM0_FSYNC) 3:39500000.gpio
pin 38 (PCM0_DATA0) 4:39500000.gpio
pin 39 (PCM0_DATA1) 5:39500000.gpio
root@ubuntu:~#
```