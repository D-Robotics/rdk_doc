---
sidebar_position: 8
---
# PWM Drive Debugging Guide

X3 has two types of controllers: one is standard PWM, with 3 groups of 3 each, totaling 9; the other is LPWM, mainly used to support synchronous exposure of sensors.

- PWM default supports a frequency range of 192MHz to 46.8KHz, with RATIO register precision of 8 bits for each group of PWM.
- LPWM default supports a frequency range of 100KHz to 24.4Hz, without a duty cycle register, only a high-level duration HIGH. The HIGH register is configured in units of microseconds (us) and supports a maximum high-level duration of 160us, so the duty cycle of LPWM is related to the frequency.
- LPWM is designed for sensor synchronization and is not a general-purpose PWM. **It is recommended to use PWM for pure PWM functionality.**

## Drive Code

### Code Path

```c
drivers/pwm/pwm-hobot.c
```

### Kernel Configuration

```bash
Device Drivers
    -> Pulse-Width Modulation (PWM) Support
        -> Hobot PWM controller support
        -> Hobot lite PWM controller support
```

### DTS Node Configuration

In the `hobot-xj3.dtsi` file, there are configurations for `pwm` and `lpwm`, which generally do not require any modifications.

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

When you need to enable the corresponding serial port, you can modify the corresponding board-level file. Here, take `hobot-x3-sdb_v4.dts` as an example to enable `pwm0-2` and `pwm3-5`.

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

## Test

Users can use the following script to test the `pwm` function and measure the signal to verify if the `pwm` is working properly.

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
# The following is for register reading
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