---
sidebar_position: 4
---

# 1.2.1.4 Using the Rufus Tool

## Hardware Connection

Insert the SD card into the card reader, then plug the card reader into the appropriate port on your PC.

## System Flashing

1. Launch the Rufus tool and select the corresponding Micro SD card as the target device from the "Device" dropdown menu.

    ![image-rufus-select-device](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device-en.png)

2. Click the "SELECT" button and choose the extracted file `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img` as the flashing image.

    ![image-rufus-select-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus-x3-install-en.png)

3. Keep all other settings at their defaults and click the "START" button. Wait until the flashing process completes.

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x3_install_finish-en.png)

4. Once flashing is complete, close Rufus and safely remove the storage card.

## Booting the System

First, ensure the development board is powered off. Insert the prepared SD card into the Micro SD slot on the development board, connect the board to a monitor using an HDMI cable, and finally power on the development board.

During the first boot, the system performs initial environment configuration, which takes approximately 45 seconds. Once this setup finishes, the Ubuntu desktop will appear on the connected display.

:::tip Development Board LED Indicators

When powering on the development board:

* <font color='Red'>Red LED</font> lit: Indicates normal hardware power-up.
* <font color='Green'>Green LED</font> lit: Indicates the system is booting; turning off or blinking indicates the system has finished booting.

If there is no display output for an extended period after powering on (more than 2 minutes), the board may have failed to boot properly. Use the LEDs to verify the system status:

* <font color='Green'>Green LED</font> constantly lit: Indicates system boot failure. Check whether your power adapter meets the required specifications, and try re-flashing the system image.
* <font color='Green'>Green LED</font> off or blinking: Indicates successful system boot, but failure of the display service. Verify that your connected monitor meets the supported specifications.

:::

After the Ubuntu Desktop system boots successfully, the desktop interface will be displayed on the monitor via the Display output, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)