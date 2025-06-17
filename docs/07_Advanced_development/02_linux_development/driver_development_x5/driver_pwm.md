---
sidebar_position: 8
---

# PWM 驱动调试指南

X5有两类控制器：一类是标准PWM，有4组，每组2路输出，共8个PWM输出，另一类是LPWM，共两组，每组4路PWM输出，主要用于支持Sensor的同步曝光。

- PWM 默认支持频率范围是0.05Hz到100MHz，占空比寄存器RATIO精度为16bit。周期有效时间为10ns到21s；占空比有效时间为10ns到21s；
- LPWM 默认支持频率范围是1Hz到500KHz，没有占空比寄存器，只有一个高电平持续时间HIGH，HIGH寄存器配置单位是us，占空比有效时间为1us；4ms。
- LPWM是为了Sensor 同步设计的，不是一个通用的PWM，**单纯PWM功能建议使用PWM。**

## 驱动代码

### 代码路径

PWM代码路径
```c
drivers/pwm/pwm-hobot.c
```

LPWM代码路径
```c
kernel/drivers/media/platform/horizon/camsys/lpwm/
```

### 内核配置

```bash
/* arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig */
...
CONFIG_HOBOT_LPWM=m
...
CONFIG_PWM_DROBOT=y
...
```

### DTS节点配置

X5 PWM及LPWM控制器设备树定义位于SDK包的kernel文件夹下的arch/arm64/boot/dts/hobot/x5.dtsi文件内。

:::note
x5.dtsi中的节点主要声明SoC共有特性，和具体电路板无关，一般情况下不用修改。
:::

当需要使能特定PWM端口输出的时候，可以到对应的板级文件修改，这里以x5-rdk-v1p0.dts为例，使能lpwm1_0, lpwm1_1, pwm0_0, pwm0_1, pwm1_0, pwm1_1, pwm2_0, pwm2_1, pwm3_0, pwm3_1。

```c
&lpwm1 {
	status = "okay";
	pinctrl-names = "default";
	/** for display backlight **/
	pinctrl-0 = <&pinctrl_lpwm1_0 &pinctrl_lpwm1_1>;
};

&pwm0 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm0_0 &pinctrl_pwm0_1>;
};

&pwm1 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm1_0 &pinctrl_pwm1_1>;
};

&pwm2 {
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm2_0 &pinctrl_pwm2_1>;
};

&pwm3 {
	/* LSIO_PWM_OUT6 and LSIO_PWM_OUT7 */
	status = "okay";
	pinctrl-names = "default";
	pinctrl-0 = <&pinctrl_pwm3_0 &pinctrl_pwm3_1>;
};
```

### DTS中pwm和pwmchip对应关系查询方法

尽管pwm和lpwm都属于pwmchip，但PWM/LPWM下含的设备数量不一致，所以无法通过aliases固定序号，因此在板端操作pwm时，需要cat pwmchip下的device/uevent，查看pwm地址是否与目标pwm地址是否一致。以pwm0为例，在板端使用以下命令查看pwmchip的uevent

```
root@ubuntu:~# cat /sys/class/pwm/pwmchip0/device/uevent
DRIVER=drobot-pwm
OF_NAME=pwm
OF_FULLNAME=/soc/a55_apb0/pwm@34140000
OF_COMPATIBLE_0=d-robotics,pwm
OF_COMPATIBLE_N=1
OF_ALIAS_0=pwm0
MODALIAS=of:NpwmT(null)Cd-robotics,pwm
```

### 40pin中pwm对应关系表
| 40pin引脚序号 | pwm引脚序号 | pwmchip序号 | srpi-config 中 pwm 序号 | 该引脚在设备树中的配置标签 | 该引脚默认功能 |
| ------- | ---------- | ----------- | ----------------------- | ----------------------- | -------------- |
| 29脚 | pwm0 | pwm0 | pwm0 | pinctrl_pwm0_0 | SPI2_SCLK |
| 31脚 | pwm1 | pwm0 | pwm0 | pinctrl_pwm0_1 | SPI2_SSN |
| 37脚 | pwm2 | pwm1 | pwm1 | pinctrl_pwm1_0 | SPI2_MISO |
| 18脚 | pwm3 | pwm1 | pwm1 | pinctrl_pwm1_1 | SPI2_MOSI |
| 28脚 | pwm4 | pwm2 | pwm2 | pinctrl_pwm2_0 | SCL0 |
| 27脚 | pwm5 | pwm2 | pwm2 | pinctrl_pwm2_1 | SDA0 |
| 32脚 | pwm6 | pwm3 | pwm3 | pinctrl_pwm3_0 | SCL1 |
| 33脚 | pwm7 | pwm3 | pwm3 | pinctrl_pwm3_1 | SDA1 |



除了上述手动查表的方法来确认对应关系以外，我们还可以直接通过 `hb_gpioinfo  | grep pwm` 命令，直接查询到已经使用了的 pwm 相关引脚。其中 403、404、356 等等是该引脚的 gpio 编号，和 40pin 的序号不是一个概念。
```
root@ubuntu:~# hb_gpioinfo  | grep pwm
        line 24:        unnamed input                             LSIO_SPI3_SCLK        403      pinctrl_lpwm1_0
        line 25:        unnamed input                             LSIO_SPI3_SSN         404      pinctrl_lpwm1_1
        line  9:        unnamed input                             LSIO_I2C1_SCL         356      pinctrl_pwm3_0
        line 10:        unnamed input                             LSIO_I2C1_SDA         357      pinctrl_pwm3_1
root@ubuntu:~#
```



## 测试

用户可以参考以下命令进行pwm功能测试，并进行信号测量，验证pwm工作是否正常。具体测量的硬件引脚请用户参考使用的具体硬件提供的说明。
以下命令以验证PWM0 ch0为例。

```shell
cd /sys/class/pwm/pwmchip0/
echo 0 > export
cd pwm0

# 配置周期为100us
echo 100000 > period
# 配置占空比为 50% = 100us * 0.5 = 50us
echo 50000 > duty_cycle
# 使能PWM输出
echo 1 > enable

#以下是进行寄存器读取
echo "Regs of PWM 3:"
echo "PWM_EN       `devmem 0x34170000 32`"
echo "PWM_INT_CTRL `devmem 0x34170004 32`"
echo "PWM0_CTRL    `devmem 0x34170008 32`"
echo "PWM0_CLK     `devmem 0x34170010 32`"
echo "PWM0_PERIOD  `devmem 0x34170020 32`"
echo "PWM0_STATUS  `devmem 0x34170028 32`"
echo "PWM1_CTRL    `devmem 0x34170030 32`"
echo "PWM1_CLK     `devmem 0x34170034 32`"
echo "PWM1_PERIOD  `devmem 0x34170040 32`"
echo "PWM1_STATUS  `devmem 0x34170048 32`"
```
