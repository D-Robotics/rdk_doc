---
sidebar_position: 5
---

# Pinctrl调试指南

## Pinctrl使用

S100芯片中3个sys包含可软件控制的pin脚，根据用户的需要，可以对pin脚进行功能复用，属性设置和电压域设置。

### Pinmux功能

soc中的pin脚可以被设置为不同的功能，通过pinmux设置可以根据用户需要配置为相应的功能，每个引脚有四种复用功能可选。

### Pinconf功能

soc中的pin脚可以被设置为不同的状态，共包含以下几种属性：

- 上下拉设置
- 驱动强度设置
- 电压模式设置
- 输入使能设置
- 施密特设置
- 压摆率选择


### io-domain功能

S100中共存在两种电压域，分别为1.8v和3.3v，不同的pin脚需要工作在合适的电压域中。手册中将pin脚划分入power group中，在手册中可以看到每个power group控制着一组pin脚。

## 源码说明

pinctrl-hobot.c文件中包含了所有S100系列pinctrl的主逻辑代码，向pinctrl注册S100的pinctrl设备，并向下提供S100 pinctrl设备的注册。

``` {.shell}
/* 路径 */
hobot-drivers/pinctrl/pinctrl-hobot.c     #pinctrl 驱动代码源文件
hobot-drivers/pinctrl/pinctrl-hobot.h     #pinctrl 驱动代码头文件
```

pinctrl-hobot-s100.c文件中包含了所有s100的pin，pin group，pin
function信息。

``` {.shell}
/* 路径 */
hobot-drivers/pinctrl/pinctrl-hobot-s100.c     # S100 驱动数据代码
```

## Pinctrl 驱动使用说明
### Pinmux和pinconf的设置

pinmux和pinconf一般是在设备dts里配置，具体如下：

``` C
&pinctrl_peri {
    pinctrl_test_default: pinctrl_test_default {
        /* pinmux 的设置， 将peri_i2c5_scl和peri_i2c5_sda两个引脚复用为peri_i2c5的功能*/
        pinmux {
            function = "peri_i2c5";
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
        };
        /* pinconf 的设置， 为peri_i2c5_scl和peri_i2c5_sda两个引脚配置驱动电流，上下拉，输入使能，电压模式，压摆率，输入施密特等电器属性配置*/
        pinconf {
            /* 两个引脚 */
            pins = "peri_i2c5_scl", "peri_i2c5_sda";
            /* 驱动电流配置 */
            drive-strength = <7>;
            /* 配置为下拉 */
            bias-pull-down;
            /* 禁用输入 */
            input-disable;
            /* 禁用低电压模式 */
            low-power-disable;
            /* 压摆率设置为1 */
            slew-rate = <1>;
            /* 禁用输入施密特 */
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

### Pinconf支持设置的属性与设置树中对应关键字
| 关键字 | 属性 |
| --- | --- |
| bias-pull-up | 上拉设置 |
| bias-pull-down | 下拉设置 |
| bias-disable | 悬空设置 |
| drive-strength | 驱动强度 |
| low-power-disable | 高电压模式（3.3v） |
| low-power-enable | 低电压模式（1.8v） |
| input-disable | 输入不使能 |
| input-enable | 输入使能 |
| input-schmitt-enable | 施密特触发使能 |
| input-schmitt-disable | 施密特触发不使能 |

### 电压域设置

pin脚io-domain的设置，会直接影响到模块功能的正常运行，甚至电压域的设置错误，造成元器件损坏，使电路板无法工作。

因此结合之前bringup支持经验，给出如下硬件电压值和软件io-domain设置一致性确认的实践经验：

1.  首先各个模块的软硬件负责人进行拉通，确认模块中相关pin脚适用的io-domain范围，保证需求一致性，并且输出《pin脚io-domain设置手册》中的参考值部分；（文档名仅供参考）
2.  然后根据第一步中确认的信息，在设备树中进行相应的io-domain设置；
3.  在完成第二步操作后，在样板上抓取所有pin脚的io-domain值，并且将获得的值填入《pin脚io-domain设置手册》中的实际值部分，识别存在的gap；
4.  由于S100中一个group power控制一组pin脚，一个pin脚的设置可能会影响同组中其它的pin。因此拉通对齐会议，根据第三步中识别出来的gap信息，进行讨论和修复，直到gap消除。

### Pinctrl节点参考设置
S100 acore中pinctrl设备树文件路径为: hobot-drivers/kernel/dts/hobot-s100-pinctrl.dtsi。

pinctrl节点中将一组pin组合到一起，每个节点的配置包含多个子节点.

第一个子节点"pinmux"中包括引脚复用的信息，其中"function"表示设备复用引脚的功能，"pins"表示该复用功能下所需要的引脚。

第二个子节点"pinconf"包括了引脚的配置信息（使用默认值的情况下可以不加pinconf节点），"pins"表示需要进行配置的引脚，"pinconf_*"即为配置的内容，主要包括了驱动强度，上下拉，电压模式等。

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

### 节点引用

驱动在使用pinctrl的接口前，需要在DTS里配置相应的pinctrl节点，当驱动probe的时候，会将"default"对应的这组pinctrl配置到寄存器里面，而其它组的配置需要在代码里面显示调用，进行切换使用。

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

显示切换pinctrl state需要调用pinctrl_select_state接口。

```C
/**
 * pinctrl_select_state() - select/activate/program a pinctrl state to HW
 * @p: the pinctrl handle for the device that requests configuration
 * @state: the state handle to select/activate/program
 */
int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state)
```
## Pinctrl debugfs使用说明
:::info
注意：非root用户需要在命令前添加“sudo”
:::
### Pin信息查询

#### /sys/kernel/debug/pinctrl/pinctrl-devices

查看系统中都有哪些pinctrl设备，是否使用pinmux和pinconf模块。

```shell
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/pinctrl-devices
name [pinmux] [pinconf]
peri yes yes
cam yes yes
video yes yes
```
#### /sys/kernel/debug/pinctrl/pinctrl-handles

查看已经被申请过的引脚配置映射。

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

查看被使用的引脚maps，包含每个被使用的引脚所在的控制器，哪些设备在使用，处于哪个pinctrl状态下，处于那一组引脚，被复用为什么功能。

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

查看pinctrl系统中引脚序号和gpio子系统中引脚的映射关系。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/gpio-ranges
GPIO ranges handled:
0: 394f0000.gpio GPIOS [480 - 511] PINS [2 - 33]
0: 39500000.gpio GPIOS [474 - 479] PINS [34 - 39]
root@ubuntu:~#
```

#### /sys/kernel/debug/pinctrl/\<pinctrl_dev>/pinconf-pins

查看每个引脚的信息，包含pinmux对应的寄存器地址和值和pinconf对应的寄存器地址和值，是否支持电压转换以及电压值。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

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

查看每个引脚的电气属性配置，上下拉，输入使能，施密特，压摆率等等。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

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

查看每个引脚组内引脚的配置，S100是每个引脚是一个group。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

``` {.shell}
root@ubuntu:~# cat /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/pingroups
registered pin groups:
group: peri_ufs_ref_clk
pin 0 (UFS_REF_CLK)

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

引脚按照function进行分组。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

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

查看哪些引脚被占用了，是MUX占用还是GPIO占用。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

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
```

#### /sys/kernel/debug/\<pinctrl_dev>/pinmux-select

可以通过节点pinmux-functions来查询需要每个pin的group和function，
通过pinmux-select节点设置pinmux， 参数group name和func name。
以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。
``` {.shell}
cd /sys/kernel/debug/pinctrl/39ff5000.pinctrl-peri/
echo  "peri_spi0_csn0 peri_gpio"  > pinmux-select
echo  "peri_i2c5_scl peri_i2c5"  > pinmux-select
echo  "peri_i2c5_sda peri_i2c5"  > pinmux-select
```

#### /sys/kernel/debug/\<pinctrl_dev>/pins

查看每个引脚的信息，包含引脚名称和对应的gpio管脚。以下命令以`<pinctrl_dev>`为`39ff5000.pinctrl-peri`为例。

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
