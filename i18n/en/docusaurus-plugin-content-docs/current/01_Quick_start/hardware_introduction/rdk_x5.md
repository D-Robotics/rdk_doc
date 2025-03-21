---
sidebar_position: 2
---
# 1.1.2 RDK X5

## Interface Overview

RDK X5 provides various functional interfaces, including Ethernet, USB, camera, LCD, HDMI, CAN FD, and 40PIN, enabling users to develop and test applications such as image multimedia and deep learning algorithms. The layout of the development board's interfaces is shown below:

![RDK_X5_interface](../../../../../../static/img/01_Quick_start/image/hardware_interface/RDK_X5_interface.jpg)

| No. | Function                       | No. | Function                  | No. | Function                  |
| --- | ------------------------------ | --- | ------------------------- | --- | ------------------------- |
| 1   | Power Interface (USB Type C)   | 2   | RTC Battery Interface     | 3   | Easy Connect Port (USB Type C) |
| 4   | Debug Serial Port (Micro USB)  | 5   | Dual MIPI Camera Ports    | 6   | Gigabit Ethernet Port with PoE |
| 7   | 4 USB 3.0 Type A Ports         | 8   | High-Speed CAN FD Interface | 9   | 40PIN Interface           |
| 10  | HDMI Display Interface         | 11  | Multi-standard Headphone Jack | 12 | Onboard Wi-Fi Antenna     |
| 13  | TF Card Interface (Bottom)     | 14  | LCD Display Interface (MIPI DSI) |     |                          |

:::caution
When the RTC is powered by a battery, the voltage and discharge current requirements for the battery are: 2~3.3V and >2.5uA.
After the device is powered on, if the PMIC detects that the RTC voltage is low and reaches the charging voltage, it will automatically charge the RTC. The battery requirements are: the maximum chargeable voltage must be ≥3.3V, and the maximum allowable charging current must be >1mA.
Additionally, an RTC that is not being charged must not be used for power supply.
:::

---

## Power Interface

The development board provides a USB Type C interface (No. 1) as the power interface. It requires a **5V/5A** power adapter for supplying power to the board. Once the adapter is connected, the **green power indicator** and the **orange indicator** will light up, indicating normal power supply.

:::caution
Do not use a computer USB port to power the board. Insufficient power may cause **abnormal shutdown or repeated reboots**.
:::

---

## Debug Serial Port {#debug_uart}

The development board includes a debug serial port (No. 4) for serial login and debugging functions. Configure the parameters in the serial tool on your computer as follows:

- **Baud rate**: 115200  
- **Data bits**: 8  
- **Parity**: None  
- **Stop bits**: 1  
- **Flow control**: None  

To connect, use a Micro USB cable to link the board's interface No. 4 to your PC.  
For first-time use, you may need to install the CH340 driver on your computer. Search for `CH340串口驱动` to download and install it.

---

## Ethernet Port

The development board features a Gigabit Ethernet port (No. 6) supporting 1000BASE-T and 100BASE-T standards. By default, it uses a static IP configuration with the address `192.168.127.10`.  
To verify the board's IP address, log in via the serial port and use the `ifconfig` command to check the configuration of the `eth0` interface.

Additionally, this port supports PoE (Power over Ethernet), allowing simultaneous data and power transmission via a single Ethernet cable for easier installation.

---

## HDMI Display Interface {#hdmi_interface}

The development board includes an HDMI display interface (No. 10) that supports a maximum resolution of 1080P. Using the HDMI interface, the board can output the Ubuntu system desktop (on the Ubuntu Server version, it displays the logo).  
The HDMI interface also supports real-time display of camera and network stream images.

---

## USB Interfaces

The development board supports multiple USB interface extensions to accommodate various USB devices. Details are as follows:

| Interface Type     | Interface No. | Quantity | Description                                                     |
| ------------------ | ------------- | -------- | --------------------------------------------------------------- |
| USB 2.0 Type C     | No. 3         | 1 port   | USB Device mode for ADB, Fastboot, system flashing, etc.        |
| USB 3.0 Type A     | No. 7         | 4 ports  | USB Host mode for connecting USB 3.0 peripherals，expand 4 USB ports through HUB. |

### Connecting USB Flash Drives

The USB Type A ports (No. 7) support USB flash drives, which will be automatically detected and mounted. The default mount directory is `/media/sda1`.

### Connecting USB-to-Serial Adapters

The USB Type A ports (No. 7) support USB-to-serial adapters, which will be automatically detected and create device nodes such as `/dev/ttyUSB*` or `/dev/ttyACM*` (where the asterisk represents a number starting from 0). Refer to the [使用串口](../../03_Basic_Application/03_40pin_user_guide/uart.md#40pin_uart_usage) section for details.

### Connecting USB Cameras

The USB Type A ports support USB cameras, which will be automatically detected and create device nodes such as `/dev/video0`.

---

## MIPI Camera Interface {#mipi_port}

The development board provides two MIPI CSI interfaces (No. 5) for connecting up to two MIPI cameras, including stereo cameras. Compatible camera modules and specifications are as follows:

| No. | Sensor  | Resolution | FOV | I2C Device Address |
| --- | ------- | ---------- | --- | ------------------ |
| 1   | IMX219  | 8 MP       |     |                    |
| 2   | OV5647  | 5 MP       |     |                    |

Connect the camera module to the board using an FPC cable with the blue side facing upwards.  
After installation, use the `i2cdetect` command to check if the I2C address of the module can be detected.

:::caution
Important: Do not connect or disconnect the camera while the board is powered on, as this may damage the camera module.
:::

---

## LCD Display Interface

The RDK X5 provides an LCD display interface (MIPI DSI, No. 14) that supports LCD screens. This interface is 22-pin,can use DSI-Cable-12cm to compatible with several Raspberry Pi LCD displays.

---

## Micro SD Interface

The development board includes a Micro SD card interface (No. 13). It is recommended to use a card with at least 16GB of storage to meet the installation requirements of Ubuntu and related packages.

:::caution
Do not hot-swap the TF card during use, as it may cause system abnormalities or file system corruption.
:::

---

## Wi-Fi Antenna Interface

The board supports both onboard and external antennas for wireless networking. The onboard antenna is sufficient for most scenarios. If the board is enclosed in a metal casing, connect an external antenna to the port near interface No. 12 to enhance signal strength.

---

## CAN FD Interface

The RDK X5 provides a CAN FD interface for CAN and CAN FD communication. Refer to the [CAN使用](../../07_Advanced_development/01_hardware_development/rdk_x5/can.md) section for details.

---

## 40PIN Interface

The development board includes a 40PIN interface with IO signals designed at 3.3V. The pin definition is compatible with Raspberry Pi and similar products. For detailed pin definitions and multiplexing information, refer to the hardware development section.

---


