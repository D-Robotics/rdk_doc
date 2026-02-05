---
sidebar_position: 1
---

# Windows Platform

## Hardware Connection

Connect the PC's USB port to the development board's Type-C port using a Type-C data cable.

:::warning Note

Please ensure the Type-C data cable is of high quality:
1. Shielded  
2. As short as possible  
3. High data transmission quality to guarantee flashing stability

:::

## Driver Download and Installation

Before using the flashing tool, Windows users need to confirm whether the driver has been installed.

1. Put the development board into Fastboot mode so the computer can recognize the device:
    
    1. Connect the development board to the PC using a serial cable (refer to the [Hardware Introduction - Serial Login](../../../01_hardware_introduction/01_rdk_s100.md) section).
    2. Power on the development board and immediately press and hold the spacebar to enter the U-Boot command line.
    3. Enter `fastboot 0` in the command line and press Enter:
    
        ```bash
        Warning: eth1 (eth0) using random MAC address - 9a:07:de:92:a2:c5
        eth0: eth1
        system_slot: 0 adc_boardinfo: 6a84
        strap_pin = 0x45bc0 bootinfo = 0x0 bootcount = 0x1
        boot_block_device [1]
        flash boot
        success!
        Hit any key to stop autoboot:  0
        Hobot$
        Hobot$
        Hobot$ fastboot 0
        ```

2. Open **Device Manager** on your PC to check and install the driver.


     * If the driver is not installed: an unknown device named `USB download gadget` will appear (as shown below). In this case, you need to install the driver.

         ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usb-driver1.png)

     * Driver download and installation:
          1. [Click here to download](https://archive.d-robotics.cc/downloads/software_tools/winusb_drivers/) the `sunrise5_winusb.zip` archive.
          2. Extract the archive, navigate to the directory, right-click `install_driver.bat`, and select **Run as administrator**.

     * Successful installation: After successful driver installation, Device Manager will display an `Android Device` (as shown below).

         ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-ok-en.jpg)
       


## System Flashing

:::warning Note

- Currently, **SW3 must be switched to the ↑ position** to boot from the onboard eMMC; booting from an M.2 NVMe SSD is not supported yet.
- D-Navigation on Windows PC can only be used after successful [driver installation](#driver-download-and-installation). Please ensure the driver is properly installed before proceeding.

:::

### U-Boot Mode Flashing

1. Power on the development board.

   :::tip

   The U-Boot method requires exclusive access to the serial port. Ensure the serial port is not occupied by other devices or applications.
   :::

2. Double-click to launch D-Navigation and configure as follows:

   - Select Product Model: S100
   - Download Mode: uboot; Storage Medium: ufs; Type: secure
   - Click Browse to select the firmware’s product folder
   - Choose the serial port connected to the RDK S600 with a baud rate of 921600
   - Click Start Upgrade (If a 'Need manual reset' prompt appears during the upgrade, please power cycle the board)

   ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download-en.jpg)

3. After the upgrade completes, power cycle the board.


### USB Mode Flashing

:::tip

For details about SW1, SW2, etc., refer to [Section 1.1.1: Switches, Buttons, and LED Indicators](../../../../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#switches-buttons-and-led-indicators)
:::

1. Switch the device to DFU mode by following these steps:

   1. Set SW1 to ↑ and turn off power
   2. Set SW2 to ↑ to enter Download mode
   3. Set SW1 to ▽ and turn on power
   4. If the `DOWNLOAD` LED lights up, the board has entered DFU mode; otherwise, press the `K1` button to reset the system.
      ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1-en.jpg)

3. Launch the Digua Chip Tool D-Navigation and perform the following:

   - Select Product Model: S100
   - Download Mode: usb; Storage Medium: emmc; Type: secure
   - Click Browse to select the firmware’s product folder
   - Power cycle the device, click Start Upgrade, and wait for the process to complete

   ![image-S100-download](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download_dfu-en.png)

4. After the upgrade finishes, power off the board, switch the flashing jumper downward (to exit DFU mode), and then power it back on.


## Miniboot and File System Upgrade

The D-Navigation tool supports updating the [Miniboot image](/rdk_s/Advanced_development/rdk_gen#765-custom-partition-description) for S100. When customers need to retain modifications to the root file system (e.g., custom-installed Python/Deb packages), they can first run `sudo apt update && sudo apt upgrade` on the board to upgrade the file system, then use D-Navigation to update the Miniboot image.

The overall Miniboot flashing procedure is consistent with the [full system flashing](#system-flashing) process, with the following additional configuration:

1. Click the rightmost arrow under “Other Settings”;
2. Click and select “Partition Selection”;
3. Uncheck “emmc”.

- Example configuration for U-Boot flashing:
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_uboot_miniboot.png)

- Example configuration for USB flashing:
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_dfu_miniboot.png)

### Booting the System

First, ensure the development board is powered off. Connect the board to a monitor using an HDMI cable, then power on the board.

During the first boot, the system performs default environment setup, which takes approximately 45 seconds. Once completed, the Ubuntu desktop will appear on the monitor.

:::tip Development Board LED Indicator

- **<font color='Green'>Green</font>** LED: Illuminated indicates normal hardware power-up.

:::

If there is no display output after powering on the board for more than 2 minutes, the board likely failed to boot properly. In this case, connect via serial cable for debugging to verify the board’s status.

After successfully booting the Ubuntu Desktop version, the system desktop will be displayed on the monitor via the Display interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)