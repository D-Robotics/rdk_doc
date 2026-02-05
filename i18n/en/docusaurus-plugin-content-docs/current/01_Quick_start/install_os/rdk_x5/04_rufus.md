---
sidebar_position: 4
---

# 1.2.3.4 Using the Rufus Tool

## Device Connection

Insert the SD card into the card reader, then plug the card reader into the PC.

## System Flashing

### SD Card Flashing

1. Launch the Rufus tool and select the corresponding Micro SD card as the target device from the "Device" dropdown menu.

    ![image-rufus-select-device](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device-en.png)

2. Click the "SELECT" button and choose the extracted image file `rdk-x5-ubuntu-preinstalled-desktop-3.0.1-arm64.img` as the flashing image.

    ![image-rufus-select-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus-x5-install-en.png)

3. Keep all other settings at their defaults and click the "START" button. Wait until the flashing process completes.

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x5_install_finish-en.png)

4. Once flashing is complete, close Rufus and remove the SD card.

### On-Board SD Card Flashing

Insert the SD card into the development board, connect the USB Type-C cable to your PC, press and hold the Sleep button (located next to the headphone jack), power on the development board, and wait for 5 seconds until the board enters flashing mode.

The PC will recognize the SD card as a USB drive. Then, follow the instructions in the [System Flashing](#system-flashing) section above to complete the system flashing process.

## Booting the System

First, ensure the development board is powered off. Insert the prepared SD card into the Micro SD slot on the development board, connect the board to a monitor using an HDMI cable, and finally power on the development board.

During the first boot, the system performs default environment configuration, which takes approximately 45 seconds. Once completed, the Ubuntu desktop will appear on the monitor.

:::tip Development Board LED Indicator

* <font color='Green'>Green LED</font> lit: indicates the system is booting; turned off or blinking indicates the system has finished booting.
:::

If there is no display output after powering on the development board for an extended period (more than 2 minutes), it indicates an abnormal boot. In this case, use a serial debug cable to inspect whether the board is functioning properly.

After the Ubuntu Desktop system finishes booting, the desktop interface will be displayed on the monitor via the Display output interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)