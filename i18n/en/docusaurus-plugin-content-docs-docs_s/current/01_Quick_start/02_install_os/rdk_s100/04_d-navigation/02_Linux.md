---
sidebar_position: 1
---

# Linux Platform


## Hardware Connection

Connect the PC's USB port to the development board's Type-C port using a Type-C data cable.


## System Flashing

### U-Boot Mode Flashing

1. Launch D-Navigation:
   
    xhost +
    sudo ./D-Navigation --no-sandbox

2. Select product model: S100  
   - Download mode: uboot; Storage medium: ufs; Type: secure  
   - Click "Browse" to select the firmware's product folder  
   - Choose the serial port connected to the RDK S600 with a baud rate of 921600  
   - Click "Start Upgrade" (During the upgrade, if a 'Need manual reset' prompt appears, power-cycle the device)

   ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download-en.jpg)

3. Power-cycle the device after the upgrade completes


### USB Mode Flashing

:::tip

For explanations of SW1, SW2, etc., refer to [Section 1.1.1: Switches, Buttons, and LED Indicators](../../../../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#switches-buttons-and-led-indicators)
:::

1. Switch the device into DFU mode by following these steps:

   1. Set SW1 switch to ↑, then turn off power  
   2. Set SW2 switch to ↑ to enter Download mode  
   3. Set SW1 switch to ▽, then turn on power  
   4. If the `DOWNLOAD` LED lights up, the device has entered DFU mode; otherwise, press `K1` to reset the system.  
      ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1-en.jpg)

3. Open the Digua Chip Tool D-Navigation and perform the following operations:

   - Select product model: S100  
   - Download mode: usb; Storage medium: emmc; Type: secure  
   - Click "Browse" to select the firmware's product folder  
   - Power-cycle the device, click "Start Upgrade", and wait for the upgrade to complete  

   ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download_dfu-en.png)

4. After the upgrade completes, power off the device, toggle the flashing switch downward (to exit DFU mode), then power it back on.


## Miniboot and File System Upgrade

The D-Navigation tool supports updating the [Miniboot image](/rdk_s/Advanced_development/rdk_gen#765-custom-partition-description) for S100. When users need to preserve modifications to the root file system (e.g., custom-installed Python or .deb packages), they can first upgrade the file system on the board using `sudo apt update && sudo apt upgrade`, then use the D-Navigation tool to update the Miniboot image.

The overall Miniboot flashing procedure is consistent with the [full system flashing](#system-flashing) process, with the following additional configuration required:

1. Click the rightmost arrow in "Other Settings";  
2. Click and select "Partition Selection";  
3. Uncheck "emmc";

- U-Boot flashing example is shown below:
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_uboot_miniboot.png)

- USB flashing example configuration is shown below:
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_dfu_miniboot.png)

### Booting the System

First, ensure the development board is powered off. Connect the board to a monitor using an HDMI cable, then power on the development board.

During the first boot, the system performs default environment setup, which takes approximately 45 seconds. Once completed, the Ubuntu desktop will appear on the monitor.

:::tip Development Board LED Indicators

- **<font color='Green'>Green</font>** LED: Illuminated indicates normal hardware power-up



:::

If there is no display output after powering on the development board for an extended period (over 2 minutes), the board likely failed to boot properly. In this case, connect a serial cable for debugging to check whether the board is functioning normally.

After the Ubuntu Desktop system boots successfully, the desktop interface will be displayed on the monitor via the Display interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)