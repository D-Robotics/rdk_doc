---
sidebar_position: 4
---

# 1.2.2.4 Using the hbupdate Tool

## Installing USB Drivers

For Windows users, please follow the steps below to verify whether the fastboot driver has been installed before flashing:

1. Use a jumper cap to connect the `Boot` pin on the RDK X3 carrier board to ground. Refer to the figure below for the pin location.
   
       ![image-carrier-board-bootstrap](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-bootstrap.png)

2. Connect the carrier board's Micro USB port to your computer using a USB cable. Refer to the figure below for the port location.
   
       ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)

3. Power on the device and check the Device Manager on your computer. If an unknown device named `USB download gadget` appears, you need to update the device driver; otherwise, you can skip the following steps.
   
       ![image-usb-driver1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-usb-driver1.png)

4. Download and extract the driver package `android_hobot.zip` from the link: [android_hobot](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip).

5. Navigate to the extracted directory and run `5-runasadmin_register-CA-cer.cmd` as administrator to register the driver.

6. Double-click the unknown `USB download gadget` device, select the extracted driver directory, and click Next.
   
       ![image-usb-driver2](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/driver.png)

7. After successful driver installation, the Device Manager will display a fastboot device named `Android Device`.
   
       ![image-usb-driver3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-usb-driver3.png)

## Hardware Connection

1. Use a jumper cap to connect the `BOOT` pin to ground. Refer to the [Function Control Interface](../../hardware_introduction/rdk_x3.md#function-control-interface) for pin locations.

2. Connect the Micro USB port to your computer. The `Android Device` should appear in your computer's Device Manager.

## Flashing the System {#flash_system}

1. Run `hbupdate.exe` to launch the flashing tool and follow the steps below:

    ![image-flash-system1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system1.png)

2. Select your development board model (required):

      - **RDK_X3_2GB**: RDK X3 (Sunrise X3 Pi), 2GB RAM version, supports only minimal system image flashing.

      - **RDK_X3_4GB**: RDK X3 (Sunrise X3 Pi), 4GB RAM version, supports only minimal system image flashing.

      - **RDK_X3_MD_2GB**: RDK X3 Module, 2GB RAM version.

      - **RDK_X3_MD_4GB**: RDK X3 Module, 4GB RAM version.

        ![image-flash-system2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system2.png)

3. Click the `Browse` button to select the image file to be flashed (required).

        ![image-flash-system3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system3.png)

4. Click the `Start` button to begin flashing:

        ![image-flash-system4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system4.png)

5. After flashing is complete, disconnect power and the USB cable, remove the BOOT pin jumper cap, and then power on the device again.

    If booting succeeds normally, the `ACT LED` on the hardware will blink twice quickly followed by one slow blink.

6. Verify the upgrade result:

   - When flashing succeeds, the tool displays the following message:

        ![image-flash-system6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system6.png)

   - When flashing fails, the tool displays the following message. In this case, confirm whether the `Android Device` appears in your PC’s Device Manager.

        ![image-flash-system7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system7.png)

## Booting the System

## Booting the System

### eMMC Without a Flashed System

Insert an SD card with a prepared system image into the carrier board to boot from the SD card.

### eMMC With a Flashed System

**Default Behavior**

By default, the system boots from eMMC.

**Switching to Boot from SD Card**

1. To disable eMMC boot and switch to booting from the SD card, log into the system and run the following commands to remove the boot flag from the second partition of eMMC, then reboot to apply the change:
    
      ```
      sudo parted /dev/mmcblk0 set 2 boot off
      sudo reboot
      ```

2. During U-Boot initialization, the system will detect that eMMC lacks a bootable partition and will instead search for the SD card’s boot partition. The system will then load from the SD card. After logging in, running the `mount` command will show the root filesystem mounted on the SD card’s second partition, and the config partition on the SD card’s first partition:

      ```
      /dev/mmcblk2p2 on / type ext4 (rw,relatime,data=ordered) 
      /dev/mmcblk2p1 on /boot/config type vfat
      ```

**Switching Back from SD Card Boot to eMMC Boot**

When currently booting from an SD card and the eMMC already contains a flashed system, run the following commands to restore booting from eMMC. The change takes effect after rebooting:

      ```
      sudo parted /dev/mmcblk0 set 2 boot on
      sudo reboot
      ```
        ![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)