---
sidebar_position: 3
---

# 1.2.2.3 Using RDK Studio Tool

## SD Card Flashing

The RDK Studio tool provides system flashing functionality, allows device connection and management, and supports usage on Windows, Linux, and Mac platforms. For detailed steps, refer to [Flashing System Using RDK Studio](../../09_RDK_Studio/04_flashing.md).

## eMMC Flashing

### Hardware Connection

1. Use a jumper cap to switch the RDK X3 carrier board to 3.3V power supply.
       
       ![image-X3MD-3v3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-3v3.PNG)  

2. Connect the Micro USB interface (debug serial port) on the carrier board to your computer using a USB cable. Refer to the figure below for the interface location. 
   
       ![image-X3MD-MicroUSB](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-MicroUSB.PNG)   

3. Connect the Micro USB interface (flashing port) on the carrier board to your computer using a USB cable. Refer to the figure below for the interface location.  
   
       ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)


### System Flashing

1. Connect to the device via a serial terminal tool on your PC, setting the baud rate to 921600. When powering on the device, press and hold the spacebar to enter the U-Boot command-line interface.
   
       ![imagex3md-ums1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums1.PNG)  

2. In U-Boot, execute `watchdog off` to disable the watchdog timer and prevent unexpected reboots. Then run `ums 0 mmc 0` to expose the onboard eMMC device (device number 0) as a USB Mass Storage device via USB OTG port 0. This allows the host system to recognize it as a standard USB drive for direct read/write or flashing operations.

          ```shell
            watchdog off
            ums 0 mmc 0
          ```

           ![imagex3md-ums2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums2.png) 

3. The standard USB drive recognized by your PC corresponds to the eMMC partitions of the RDK X3 Module.
   
       ![imagex3md-ums3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums3.png) 

4. For flashing instructions, refer to [Flashing System Using RDK Studio](../../09_RDK_Studio/04_flashing.md).