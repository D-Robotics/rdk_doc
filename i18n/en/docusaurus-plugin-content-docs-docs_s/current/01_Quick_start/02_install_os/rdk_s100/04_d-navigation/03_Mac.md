---
sidebar_position: 1
---

# Mac Platform



## Hardware Connection

Connect the PC's USB port to the development board's Type-C port using a Type-C data cable.




## System Flashing

### U-Boot Mode Flashing

1. Run `xattr -cr D-navigation.app` in the terminal to remove quarantine attributes.
2. Double-click to open D-Navigation and configure as follows:

   - Select Product Model: S100
   - Download Mode: uboot; Storage Medium: ufs; Type: secure
   - Click "Browse" to select the firmware's product folder
   - Choose the serial port connected to the RDK S600 with a baud rate of 921600
   - Click "Start Upgrade" (If a 'Need manual reset' prompt appears during the upgrade, power cycle the device)
   
      ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download-en.jpg)
   
3. Power cycle the device after the upgrade completes
   
   
### USB Mode Flashing
   
   :::tip
   
   For explanations of SW1, SW2, etc., refer to [Section 1.1.1: Switches, Buttons, and LED Indicators](../../../../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#switches-buttons-and-led-indicators)
   :::
   
   1. Switch the device into DFU mode by following these steps:
   
      1. Set SW1 to ↑ and turn off power
      2. Set SW2 to ↑ to enter Download mode
      3. Set SW1 to ▽ and turn on power
      4. If the `DOWNLOAD` LED lights up, the device has entered DFU mode; otherwise, press `K1` to reset the system.
         ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1-en.jpg)
   
   2. Open the Digua Chip Tool D-Navigation and perform the following operations:
   
      - Select Product Model: S100
      - Download Mode: usb; Storage Medium: emmc; Type: secure
      - Click "Browse" to select the firmware's product folder
      - Power cycle the device, click "Start Upgrade", and wait for the process to complete
   
      ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download_dfu-en.png)
   
   3. After the upgrade completes, power off the device, switch the flashing switches downward (to exit DFU mode), and then power it back on.
   
   
## Miniboot and File System Upgrade

The D-Navigation tool supports updating the [Miniboot image](/rdk_s/Advanced_development/rdk_gen#765-custom-partition-description) for S100. When customers need to preserve modifications to the root file system (e.g., manually installed Python/Deb packages), they can first perform a file system upgrade on the board using `sudo apt update && sudo apt upgrade`, then use the D-Navigation tool to upgrade the Miniboot image.

The overall Miniboot flashing procedure is consistent with [Full System Flashing](#system-flashing), with the following additional configuration steps:

4. Click the rightmost arrow under "Other Settings";
5. Click and select "Partition Selection";
6. Uncheck "emmc";

   - U-Boot flashing example is shown below:
     
         ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_uboot_miniboot.png)
   
   - USB flashing example configuration is shown below:
     
         ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_dfu_miniboot.png)
   
### Booting the System

First, ensure the development board is powered off, connect it to a monitor via an HDMI cable, and then power on the board.

During the first boot, the system performs default environment setup, which takes approximately 45 seconds. Upon completion, the Ubuntu desktop will appear on the monitor.

:::tip Development Board LED Indicators

- **<font color='Green'>Green</font>** LED: Illuminated indicates normal hardware power-up

:::

If there is no display output after powering on the board for an extended period (over 2 minutes), the board has likely failed to boot properly. In this case, use a serial cable for debugging to verify whether the board is functioning correctly.

After successfully booting the Ubuntu Desktop version, the system desktop will be displayed on the monitor via the Display interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)