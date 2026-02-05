---
sidebar_position: 4
---

# 1.2.2.4 Using the Rufus Tool

## SD Card Flashing
### Hardware Connection

Insert the SD card into a card reader, and connect the card reader to the appropriate port on your PC.

### System Flashing

To flash the system onto the SD card (instead of booting from eMMC mode), follow the same steps as described in [RDK X3 System Flashing Procedure](../rdk_x3/04_rufus.md#system-flashing).

## eMMC Flashing

### Hardware Connection

1. Use a jumper cap to switch the RDK X3 carrier board to 3.3V power supply.
       
       ![image-X3MD-3v3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-3v3.PNG)  

2. Connect the Micro USB port (debug serial port) on the carrier board to your PC using a USB cable. Refer to the image below for the port location.
   
       ![image-X3MD-MicroUSB](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-MicroUSB.PNG)   

3. Connect the Micro USB port (flashing port) on the carrier board to your PC using a USB cable. Refer to the image below for the port location.
   
       ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)

### System Flashing

1. Connect to the device via a serial terminal tool on your PC, setting the baud rate to 921600. While powering on the device, hold down the spacebar to enter the U-Boot command-line interface.
   
       ![imagex3md-ums1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums1.PNG)  

2. In U-Boot, execute `watchdog off` to disable the watchdog timer and prevent unexpected reboots. Then run `ums 0 mmc 0` to expose the onboard eMMC (device number 0) as a USB Mass Storage device via USB OTG interface 0. This allows the host PC to recognize it as a standard USB drive for direct read/write or flashing operations.

          ```shell
            watchdog off
            ums 0 mmc 0
          ```

           ![imagex3md-ums2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums2.png) 

3. The PC will recognize a standard USB drive corresponding to the eMMC partition of the RDK X3 Module.
   
       ![imagex3md-ums3](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/portable.png) 

4. Launch the Rufus tool, select the corresponding drive letter in the "Device" dropdown menu as the target device, and proceed with the remaining steps identical to those in [RDK X3 flashing](../rdk_x3/04_rufus.md#system-flashing) to complete the image flashing.
   
       ![image-rufus-select-device](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device-en.png)

    :::warning Note

    If the flashing process is interrupted, please repeat the steps above from the beginning.
    :::

## Booting the System

### No System Flashed to eMMC

Insert the SD card containing the prepared system image into the carrier board to boot directly from the SD card.

### System Already Flashed to eMMC

**Default Behavior**

By default, the system boots from eMMC.

**Switching to Boot from SD Card**

1. To disable eMMC boot and switch to SD card boot, log into the system and execute the following commands to remove the boot flag from the second partition of the eMMC, then reboot for the change to take effect:
    
      ```
      sudo parted /dev/mmcblk0 set 2 boot off
      sudo reboot
      ```

2. During U-Boot initialization, the system will detect that the eMMC lacks a bootable partition and will instead search for a boot partition on the SD card, loading the OS from there. After logging into the system, running the `mount` command will show the root filesystem mounted on the second partition of the SD card (`/dev/mmcblk2p2`), and the config partition mounted from the first partition of the SD card (`/dev/mmcblk2p1`).

      ```
      /dev/mmcblk2p2 on / type ext4 (rw,relatime,data=ordered) 
      /dev/mmcblk2p1 on /boot/config type vfat
      ```

**Switching Back from SD Card Boot to eMMC Boot**

When currently booting from an SD card and the eMMC already contains a flashed system, run the following commands to restore eMMC booting. The change will take effect after rebooting.

      ```
      sudo parted /dev/mmcblk0 set 2 boot on
      sudo reboot
      ```
        ![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)