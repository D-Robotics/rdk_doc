---
sidebar_position: 3
---

# 1.1.3 RDK Ultra

RDK Ultra development kit provides various peripheral interfaces, including Ethernet, USB, HDMI, MIPI CSI, and 40PIN, enabling users to experience features, develop, and test the RDK Ultra kit. The interface layout is as follows:

![image-carrier-board1](../../../../../../static/img/01_Quick_start/image/hardware_interface/image-rdk-ultra-interface1.jpg)
![image-carrier-board2](../../../../../../static/img/01_Quick_start/image/hardware_interface/image-rdk-ultra-interface2.jpg)

| No.  | Interface Function       | No.  | Interface Function        | No.  | Interface Function          |
| ---- | ------------------------ | ---- | ------------------------- | ---- | --------------------------- |
| 1    | Power Interface          | 7    | 40PIN Header              | 13   | CAM3 Interface, 24PIN, 4lane |
| 2    | HDMI Display Interface   | 8    | PWM Fan Interface         | 14   | CAM1 Interface, 24PIN, 4lane |
| 3    | 4 USB3.0 Ports           | 9    | RTC Battery Interface     | 15   | Wireless Network Interface, PCIe M.2-E |
| 4    | Gigabit Ethernet Port    | 10   | Function Control Interface | 16   | SSD Interface, PCIe M.2-M    |
| 5    | Debug Interface          | 11   | CAM2 Interface, 15PIN, 2lane | 17 | SSD Interface, PCIe M.2-M    |
| 6    | Status Indicator Light   | 12   | CAM0 Interface, 15PIN, 2lane |     |                             |

---

## Core Module Interface

The RDK Ultra Module core board hardware interfaces are compatible with Jetson Orin series development boards, facilitating rapid integration and product deployment.

![rdk_ultra_module](../../../../../../static/img/01_Quick_start/image/hardware_interface/rdk_ultra_module.png)

---

## Power Interface

The RDK Ultra development board is powered via a DC interface. It is recommended to use the power adapter included in the kit or a power adapter with at least **12V/5A** output. After connecting the power supply, the red power indicator light (No. 6) will turn on, indicating normal power supply.

:::caution
Do not use a computer USB port to power the board. Insufficient power may cause **abnormal shutdown or repeated reboots**.
:::

---

## Debug Serial Port {#debug_uart}

The RDK Ultra development board provides a debug interface (No. 5), which uses the `CH340` chip to convert the core module's debug serial port into a USB interface. This interface can be used for various debugging tasks. Configure your serial tool with the following parameters:

- **Baud rate**: 921600  
- **Data bits**: 8  
- **Parity**: None  
- **Stop bits**: 1  
- **Flow control**: None  

For first-time use, install the CH340 driver on your computer. Search for `CH340串口驱动` to download and install it.

---

## Ethernet Port

The development board provides a Gigabit Ethernet port (No. 4) supporting 1000BASE-T and 100BASE-T standards. By default, the static IP address is `192.168.1.10`.  
To verify the board's IP address, log in via the serial port and use the `ifconfig` command to check the configuration of the `eth0` interface.

---

## HDMI Interface {#hdmi_interface}

The RDK Ultra development board provides an HDMI display interface (No. 2) supporting a maximum resolution of 1080P. After powering on the board, the Ubuntu graphical interface will output via the HDMI interface. This interface also supports previewing camera or video stream images with specific demo programs.

Currently, the HDMI interface supports only the 1080p60 display mode. Additional modes will be supported in future software updates.

---

## USB Interface

The RDK Ultra development board provides four USB3.0 standard ports (No. 3), allowing up to four USB peripherals to be used simultaneously. Note that the USB ports of RDK Ultra support only Host mode.

---

## USB Cameras

Video: [https://www.bilibili.com/video/BV1rm4y1E73q/?p=6](https://www.bilibili.com/video/BV1rm4y1E73q/?p=6)

The USB Type A ports of the development board support USB cameras. When a USB camera is connected, it will be automatically detected and create a device node such as `/dev/video8`.

---

## MIPI CSI {#mipi_port}

The RDK Ultra development board provides four camera interfaces (`CAM 0~3`) to support simultaneous connections for four MIPI camera modules. The usage notes are as follows:

1. **CAM 0/2** (Interfaces 11/12): Use 15PIN FPC connectors, compatible with Raspberry Pi OV5647, IMX219, IMX477, and other camera modules.  
2. **CAM 1/3** (Interfaces 13/14): Use 24PIN FPC connectors, compatible with F37, GC4663, IMX415, and other camera modules.

Specifications of supported camera modules:

| No.  | Sensor  | Resolution | FOV              | I2C Device Address |
| ---- | ------- | ---------- | ---------------- | ------------------ |
| 1    | GC4663  | 4 MP       | H:104 V:70 D:113 | 0x29               |
| 2    | JXF37   | 2 MP       | H:62  V:37 D:68  | 0x40               |
| 3    | IMX219  | 8 MP       | H:62  V:37 D:68  | 0x10               |
| 4    | IMX477  | 12 MP      | H:62  V:37 D:68  | 0x1a               |
| 5    | OV5647  | 5 MP       | H:62  V:37 D:68  | 0x36               |

For purchasing camera modules, refer to the community accessories page: [Purchase link](../../07_Advanced_development/01_hardware_development/rdk_x3/accessory.md).

:::caution
Important: Do not connect or disconnect the camera while the board is powered on, as this may damage the camera module.
:::

---

## Wi-Fi Antenna Interface

The development board comes with a pre-installed wireless network card module and antenna (No. 15).

---

## 40PIN Header Interface

The RDK Ultra development board provides a 40PIN header interface (No. 7) supporting GPIO, UART, I2C, SPI, I2S, and other interfaces. The detailed pin definitions and multiplexing relations are as follows:

![image-40pin-header](../../../../../../static/img/01_Quick_start/image/hardware_interface/image-interface-40pin.jpg)

---

## Function Control Interface

The RDK Ultra development board provides a function control interface (No. 10) to control the functional modes of the core module. The pin definitions are as follows:

| Pin No. | Pin Name | Function Description          | Usage Method                           |
| ------- | -------- | ---------------------------- | -------------------------------------- |
| 1       | WAKE_UP  | Wakes up the development board | Short circuit to GND using a jumper cap |
| 2       | GND      | Ground Signal                | GND                                    |
| 3       | FC_REC   | Forces the board into recovery mode | Short circuit to GND using a jumper cap |
| 4       | GND      | Ground Signal                | GND                                    |
| 5       | SYS_RST  | Resets the system            | Short circuit to GND, then disconnect to reset |
| 6       | GND      | Ground Signal                | GND                                    |
| 7       | PWR_EN   | Power enable signal          | Short circuit to GND to disable power supply |
| 8       | GND      | Ground Signal                | GND                                    |

![image-carrier-board-control-pin1](../../../../../../static/img/01_Quick_start/image/hardware_interface/image-rdk-ultra-interface-control.jpg)
