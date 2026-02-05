---
sidebar_position: 4
---

# 1.2.4.4 Using the Rufus Tool

## SD Card Flashing

### Hardware Connection

Insert the SD card into the card reader, and plug the card reader into the appropriate port on your PC.

### System Flashing

1. Open the Rufus tool and select the corresponding Micro SD card as the target device from the "Device" dropdown menu.

    ![image-rufus-select-device](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device-en.png)

2. Click the "SELECT" button and choose the extracted `rdk-x5-ubuntu-preinstalled-desktop-3.0.1-arm64.img` file as the flashing image.

    ![image-rufus-select-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus-x5-install-en.png)

3. Keep all other settings at their defaults, click the "START" button, and wait for the flashing process to complete.

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x5_install_finish-en.png)

4. After flashing is complete, close Rufus and remove the storage card.

## On-Board Flashing

### SD Card On-Board

1. Insert the SD card into the development board, connect the USB Type-C cable to your PC, press and hold the Sleep button (Pin 23), power on the development board, and wait for 5 seconds. The board will enter flashing mode, and the SD card will appear as a USB drive on your PC.
2. Follow the instructions in the [System Flashing](#system-flashing) section to complete system flashing.

### eMMC Flashing

1. Connect the USB Type-C cable to your PC, press and hold the Sleep button (Pin 23), power on the development board, and wait for 5 seconds. The board will enter flashing mode, and the built-in eMMC on the core board will appear as a USB drive on your PC.

2. Follow the instructions in the [System Flashing](#system-flashing) section to complete system flashing.

## Booting the System

**Default Behavior**

The system boots from eMMC by default.

**Configuring Boot Mode**

Use `srpi-config` to configure the boot mode. For details, refer to [srpi-config Advanced Options Boot Order](../../../02_System_configuration/02_srpi-config.md#advanced-options).

After the Ubuntu Desktop system finishes booting, the desktop environment will be displayed on the monitor via the Display interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)