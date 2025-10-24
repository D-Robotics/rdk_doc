---
sidebar_position: 1
---

# 1.5.1 RDK X5

RDK X5 supports two display output methods: HDMI and MIPI DSI. They cannot be used simultaneously. The system uses HDMI output by default.

## HDMI

RDK X5 provides one HDMI interface, supporting a maximum resolution of 1080P60.

### HDMI Switching

The system uses HDMI output by default and requires no additional configuration. If you have already switched to MIPI DSI output, you can use the `srpi-config` tool to switch back to HDMI output. The changes take effect after rebooting the device.

`2 Display Options` > `D1 Display Choice` > `2 HDMI`

![image-hdmi-choice.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/image-hdmi-choice.png)

If you prefer not to use the srpi-config tool, you can configure it using the following command line method.

```bash
mv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable
mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf
```

### Default Resolution

The default resolution can be set by modifying the `/etc/X11/xorg.conf.d/1-resolution.conf` file:

```bash
Section "Screen"
    Identifier "Screen0"
    Device "Device0"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1280x720"
    EndSubSection
EndSection
```

## MIPI DSI

### 2.8inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Installation result:

![screenshot-20250916-165057](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-165057.png)

#### Software Configuration

Use the `srpi-config` tool to select `2.8inch DSI LCD`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `2.8inch DSI LCD`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-170803](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-170803.png)

### 3.4inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Connect 5V power supply through the 4-pin pogo connector, including 5V and GND. On the RDK X5 board's 40-pin interface, pin 4 is 5V and pin 6 is GND.

Installation result:

![screenshot-20250916-170117](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-170117.png)

![screenshot-20250916-170258](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-170258.png)

#### Software Configuration

Use the `srpi-config` tool to select `3.4inch DSI LCD (C)`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `3.4inch DSI LCD (C)`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-174413](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-174413.png)

### 4.3inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Installation result:

![screenshot-20250916-175405](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-175405.png)

#### Software Configuration

Use the `srpi-config` tool to select `4.3inch DSI LCD`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `4.3inch DSI LCD`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-175718](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-175718.png)

### 7inchC DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Connect 5V power supply through the 4-pin pogo connector, including 5V and GND. On the RDK X5 board's 40-pin interface, pin 4 is 5V and pin 6 is GND.

Connect I2C communication through the 4-pin pogo connector to the I2C5 interface on the X5 RDK board's 40-pin connector. On the RDK X5 board's 40-pin interface, pin 3 is SDA5 and pin 5 is SCL5.

Installation result:

![screenshot-20250916-180903](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-180903.png)
![screenshot-20250916-180943](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-180943.png)

#### Software Configuration

Use the `srpi-config` tool to select `7inch DSI LCD (C)`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `7inch DSI LCD (C)`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-181324](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-181324.png)

### 7.9inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Use a 5V/3A Type-C power supply to power the screen.

Installation result:

![screenshot-20250916-191016](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-191016.png)

#### Software Configuration

Use the `srpi-config` tool to select `7.9inch DSI LCD`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `7.9inch DSI LCD`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-191422](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-191422.png)

### 8inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Use a 5V/3A Type-C power supply to power the screen.

Installation result:

![screenshot-20250916-192525](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-192525.png)

#### Software Configuration

Use the `srpi-config` tool to select `8inch DSI LCD (C)`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `8inch DSI LCD (C)`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-192539](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-192539.png)

### 10.1inch DSI LCD

#### Hardware Connection

Use the `DSI-Cable-12cm` cable to connect the display's DSI interface to the 22-pin DSI interface on the X5 RDK board. Use the interface latch to secure the non-contact side.

Use a 5V/3A Type-C power supply to power the screen.

Installation result:

![screenshot-20250916-192555](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-192555.png)

#### Software Configuration

Use the `srpi-config` tool to select `10.1inch DSI LCD (C)`. The changes take effect after rebooting.

`2 Display Options` > `D3 MIPI LCD Choice` > `10.1inch DSI LCD (C)`

![screenshot-20250916-155629](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-155629.png)

#### Demo

![screenshot-20250916-192612](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-192612.png)

### Legacy Method

If the srpi-config tool version on your system is outdated and does not have the `D3 MIPI LCD Choice` option, you can use the following method to upgrade the tool.

![screenshot-20250916-162038](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/display/screenshot-20250916-162038.png)

Alternatively, if you prefer not to use the srpi-config tool, you can configure it using the following command line method.

1. Switch to MIPI DSI display mode with the following command:
```bash
mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable
mv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf
```

2. Open the `/boot/config.txt` file. Taking `2.8inch DSI LCD` as an example, add the following code at the end of config.txt, save, exit, and restart the system.

```bash
dtoverlay=dsi-waveshare-panel-overlay-2_8_inch
```

The configuration method is the same for all display models. Simply modify the `dtoverlay` parameter. Refer to the table below:

| Screen Type | dtoverlay Parameter |
| --- | --------- |
| 2.8inch DSI LCD | dsi-waveshare-panel-overlay-2_8_inch |
| 3.4inch DSI LCD (C) | dsi-waveshare-panel-overlay-3_4_inch |
| 4.3inch DSI LCD | dsi-waveshare-panel-overlay-4_3_inch |
| 7inch DSI LCD (C) | dsi-waveshare-panel-overlay-7_0_inchC |
| 7.9inch DSI LCD | dsi-waveshare-panel-overlay-7_9_inch |
| 8inch DSI LCD | dsi-waveshare-panel-overlay-8_0_inch |
| 10.1inch DSI LCD | dsi-waveshare-panel-overlay-10_1_inch |
