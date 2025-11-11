---
sidebar_position: 1
---

# 3.3.1 Pin Definitions and Applications

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

The development board features an expansion pin header to facilitate peripheral expansion for users. Please refer to this section for interface definitions.

## Pin Multiplexing Configuration (Deprecated)

To be updated

## Expansion Pin Header Definition {#40pin_define}

The RDKS100 provides a 40-pin header for user peripheral expansion, with digital I/O pins designed for 3.3V logic levels. The pinout is defined as follows:

![image-rdk_100_mainboard_40pin](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_mainboard_40pin.png)

:::info
Pins labeled as `40PIN_GPIO[x]_3V3` in the pinout are GPIOs provided by an I2C-expander IC. These pins are **not managed** by the SoC's Pinctrl controller, **cannot be multiplexed** to other functions, **do not require Pinmux configuration** in the device tree source (DTS), and can **only be used as GPIOs**.
:::

## GPIO Read/Write Operation Example

:::tip
The pins mentioned below are for illustrative purposes only. Port numbers may vary across different platforms—please verify against your actual hardware. Alternatively, you can directly use the code under the `/app/40pin_samples/` directory, which has been validated on the actual board.
:::

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=16

The development board includes pre-built functional test scripts for various pins under `/app/40pin_samples/`, covering GPIO input/output, PWM, I2C, SPI, UART, and more. All test programs are written in Python; refer to other sections of this chapter for detailed information.

Take `/app/40pin_samples/button_led.py` as an example: this script configures pin `24` as an input and pin `23` as an output, then controls the output state of pin `23` based on the input state of pin `24`.

## Environment Setup

Use a Dupont jumper wire to connect pin `24` to either 3.3V or GND to control its logic level (high or low).

## Execution Method

Run the `button_led.py` script to start the GPIO read/write program:

```bash
root@ubuntu:~# cd /app/40pin_samples/
root@ubuntu:/app/40pin_samples# sudo python3 ./button_led.py
```

## Expected Behavior

By toggling the logic level of pin `24`, you can change the output level of pin `23`.

```bash
root@ubuntu:/app/40pin_samples# ./button_led.py
Starting demo now! Press CTRL+C to exit
Outputting 1 to Pin 23
Outputting 0 to Pin 23
Outputting 1 to Pin 23
```