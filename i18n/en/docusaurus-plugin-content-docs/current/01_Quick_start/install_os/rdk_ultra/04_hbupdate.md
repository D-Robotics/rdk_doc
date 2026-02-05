---
sidebar_position: 4
---

# 1.2.5.3 Using the hbupdate Tool

## Hardware Connection

1. Connect the RDK Ultra directly to the PC using an Ethernet cable and ensure the network connection is pingable.

2. Short the `FC_REC` and `GND` signals on the functional control interface (Interface 10).

        ![image-ultra-fc-rec](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-ultra-fc-rec.jpg)

## System Flashing

1. Launch the `hbupdate` main program, open the flashing tool, and select the development board model as `RDK_ULTRA` (mandatory).

        ![image-flash-system1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system1.jpg)

4. Click the `Browse` button to select the image file to be flashed (mandatory).

        ![image-flash-system2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system2.jpg)

5. Click the `Start` button to begin flashing. After confirming the operation is correct based on the prompt, click the `OK` button:

        ![image-flash-system3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-system-download3.jpg)

6. When the tool displays output similar to the following, it indicates that the flashing process has started. The duration depends on the network transfer speed—please wait patiently.

        ![image-flash-system4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system4.jpg)

7. Wait until the flashing completes and verify the result:

   - If the image is successfully flashed, the tool will display the following message:

        ![image-flash-system6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system6.png)

   - If the tool shows the following error, please verify whether steps 1–3 were performed correctly.

        ![image-flash-system7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system7.png)

   - If the tool displays the following error, it indicates the network transfer speed is too slow. We recommend using a higher-performance PC and retrying the upgrade.
          ![image-flash-system8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system8.jpg)




    :::warning Note
    
    If the flashing process is interrupted, please repeat the steps above.
    :::

## Booting the System

First, ensure the development board is powered off. Remove the shorting wire between the `FC_REC` and `GND` signals on the functional control interface (Interface 10), connect the development board to a monitor via an HDMI cable, and then power on the board.

During the first boot, the system performs default environment configuration, which takes approximately 45 seconds. Once completed, the Ubuntu desktop will appear on the monitor.

After the Ubuntu Desktop system finishes booting, the desktop interface will be displayed on the monitor via the HDMI port, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)