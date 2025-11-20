---
sidebar_position: 2
---

# PWM Driver Debugging Guide

The X5 has two types of controllers: one is the standard PWM, with 4 groups, each having 2 output channels, for a total of 8 PWM outputs. The other is LPWM, with 2 groups, each having 4 PWM outputs, primarily used for supporting synchronized exposure of sensors.

- PWM supports a default frequency range of 0.05 Hz to 1 MHz.The duty-cycle register RATIO has 16-bit precision.The effective period range is 1 μs to 20 s, and the effective duty-cycle width is 10 ns to 20 s.
- LPWM supports a default frequency range of 1 Hz to 1 MHz,with an output pulse-width range of 1 μs to 4 ms.
- LPWM is designed for sensor synchronization and is not a general-purpose PWM. **For pure PWM functionality, it is recommended to use PWM.**

## Driver Code

### Code Path

The PWM driver code is located at:

```c
drivers/pwm/pwm-hobot.c
```

Path of LPWM code
```c
kernel/drivers/media/platform/horizon/camsys/lpwm/
```

### Kernel Configuration

```bash
/* arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig */
...
CONFIG_HOBOT_LPWM=m
...
CONFIG_PWM_DROBOT=y
...
```

### DTS Node Configuration

The device tree definitions for X5 PWM and LPWM controllers are located in the `x5.dtsi` file under the `arch/arm64/boot/dts/hobot/` folder in the SDK package.

:::note
The nodes in `x5.dtsi` primarily declare SoC common features and are not related to a specific circuit board. In most cases, modifications are not required.
:::

When you need to enable a specific PWM port output, you can modify the corresponding board-level file. Here, as an example, we enable `lpwm1_0`, `lpwm1_1`, `pwm0_0`, `pwm0_1`, `pwm1_0`, `pwm1_1`, `pwm2_0`, `pwm2_1`, `pwm3_0`, and `pwm3_1` in the `x5-rdk-v1p0.dts` file.


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

### PWM and PWMCHIP Correspondence in DTS

Although both PWM and LPWM belong to `pwmchip`, the number of devices under PWM/LPWM is not consistent. Therefore, fixed numbering through aliases is not possible. When operating PWM on the board, you need to use the `cat` command to view the `device/uevent` under `pwmchip` to check if the PWM address matches the target PWM address. 

For example, to check the `pwmchip` uevent for `pwm0`, use the following command on the board:


```
cat /sys/class/pwm/pwmchip0/device/uevent
DRIVER=drobot-pwm
OF_NAME=pwm
OF_FULLNAME=/soc/a55_apb0/pwm@34140000
OF_COMPATIBLE_0=d-robotics,pwm
OF_COMPATIBLE_N=1
MODALIAS=of:NpwmT(null)Cd-robotics,pwm
```

## Testing

Users can refer to the following commands to test the PWM functionality and perform signal measurements to verify whether the PWM is working correctly. For specific hardware pins to be measured, users should refer to the documentation provided for the hardware they are using.

The following commands are provided as an example to verify PWM0 channel 0.


```shell
cd /sys/class/pwm/pwmchip0/
echo 0 > export
cd pwm0

# Configure Period to 100us
echo 100000 > period

# Configure Duty Cycle to 50% = 100us * 0.5 = 50us
echo 50000 > duty_cycle

# Enable PWM Output
echo 1 > enable

# The following is for reading register values

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
