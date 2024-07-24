---
sidebar_position: 8
---

# PWM 驱动调试指南

X3有两类控制器：一类是标准PWM，有3组，每组三个，共9个，另一类是LPWM，主要用于支持Sensor的同步曝光。

- PWM 默认支持频率范围是192MHz到46.8KHz，每组PWM的占空比寄存器RATIO精度为8bit。
- LPWM 默认支持频率范围是100KHz到24.4Hz，没有占空比寄存器，只有一个高电平持续时间HIGH，HIGH寄存器配置单位是us，最大支持设置高电平为160us，所以LPWM的占空比与频率有关。
- LPWM是为了Sensor 同步设计的，不是一个通用的PWM，**单纯PWM功能建议使用PWM。**

## 驱动代码

### 代码路径

```c
drivers/pwm/pwm-hobot.c
```

### 内核配置

```bash
Device Drivers
    ->  Pulse-Width Modulation (PWM) Support
        ->  Hobot PWM controller support
        ->  Hobot lite PWM controller support
```

### DTS节点配置

在`hobot-xj3.dtsi`这个文件里面有`pwm`和`lpwm`的配置，一般来讲不需要做任何修改。

```c
/* arch/arm64/boot/dts/hobot/hobot-xj3.dtsi */
lpwm: lpwm@0xA500D000 {
    compatible = "hobot,hobot-lpwm";
    reg = <0 0xA5018000 0 0x1000>;
    interrupt-parent = <&gic>;
    interrupts = <0 68 4>;
    pinctrl-names = "lpwm0", "lpwm1","lpwm2","lpwm3", "lpwm_pps";
    pinctrl-0 = <&lpwm0_func>;
    pinctrl-1 = <&lpwm1_func>;
    pinctrl-2 = <&lpwm2_func>;
    pinctrl-3 = <&lpwm3_func>;
    pinctrl-4 = <&lpwm_pps>;
    clocks = <&lpwm_mclk>;
    clock-names = "lpwm_mclk";
    status = "disabled";
};

pwm_c0: pwm@0xA500D000 {
    compatible = "hobot,hobot-pwm";
    #pwm-cells = <3>;
    reg = <0 0xA500D000 0 0x1000>;
    interrupt-parent = <&gic>;
    interrupts = <0 44 4>;
    pinctrl-names = "pwm0", "pwm1","pwm2";
    pinctrl-0 = <&pwm0_func>;
    pinctrl-1 = <&pwm1_func>;
    pinctrl-2 = <&pwm2_func>;
    clocks = <&pwm0_mclk>;
    clock-names = "pwm_mclk";
    status = "disabled";
};
...
```

当需要使能对应串口的时候，可以到对应的板级文件修改，这里以`hobot-x3-sdb_v4.dts`为例，使能`pwm0-2`、`pwm3-5`。

```c
/* arch/arm64/boot/dts/hobot/hobot-x3-sdb_v4.dts */
...
&pwm_c0 {
	status = "okay";
	pinctrl-0 = <&pwm0_func>;
	pinctrl-1 = <>;
	pinctrl-2 = <>;
};
&pwm_c1 {
	status = "okay";
	pinctrl-0 = <>;
	pinctrl-1 = <&pwm4_func>;
	pinctrl-2 = <>;
};
...
```

## 测试

用户可以使用如下脚本进行`pwm`功能测试，并进行信号测量，验证`pwm`工作是否正常。

```shell
echo 8 8 8 8  > /proc/sys/kernel/printk
for i in 0 3
do
        cd /sys/class/pwm/pwmchip${i}
        echo 0 > export
        echo 1 > export
        echo 2 > export
 
        cd pwm0
        echo 10000 > period
        echo 3000  > duty_cycle
        echo 1 > enable
  
        cd ../pwm1
        echo 10000 > period
        echo 1000  > duty_cycle
        echo 1 > enable
 
        cd ../pwm2
        echo 10000 > period
        echo 1000  > duty_cycle
        echo 1 > enable
done
#以下是进行寄存器读取
echo "pwm0 pinctrl:`devmem 0xa6004010 32`"
echo "pwm1 pinctrl:`devmem 0xa6004058 32`"
echo "pwm2 pinctrl:`devmem 0xa600405C 32`"
echo "pwm3 pinctrl:`devmem 0xa6004060 32`"
echo "pwm4 pinctrl:`devmem 0xa6004064 32`"
echo "pwm5 pinctrl:`devmem 0xa6004048 32`"
echo "pwm6 pinctrl:`devmem 0xa600404C 32`"
echo "pwm7 pinctrl:`devmem 0xa6004030 32`"
echo "pwm8 pinctrl:`devmem 0xa6004034 32`"
 
echo "Regs of PWM 0 1 2:"
echo "PWM_EN      `devmem 0xA500d000 32`"
echo "PWM_SLICE   `devmem 0xA500d004 32`"
echo "PWM_FREQ    `devmem 0xA500d008 32`"
echo "PWM_FREQ1   `devmem 0xA500d00C 32`"
echo "PWM_RATIO   `devmem 0xA500d014 32`"
echo "PWM_SRCPND  `devmem 0xA500d01C 32`"
echo "PWM_INTMASK `devmem 0xA500d020 32`"
 
echo "Regs of PWM 3 4 5:"
echo "PWM_EN      `devmem 0xA500e000 32`"
echo "PWM_SLICE   `devmem 0xA500e004 32`"
echo "PWM_FREQ    `devmem 0xA500e008 32`"
echo "PWM_FREQ1   `devmem 0xA500e00C 32`"
echo "PWM_RATIO   `devmem 0xA500e014 32`"
echo "PWM_SRCPND  `devmem 0xA500e01C 32`"
echo "PWM_INTMASK `devmem 0xA500e020 32`"
```
