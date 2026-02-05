---
sidebar_position: 1
---

# Linux Platform

## Installing and Launching XBurn

1. Run the command `sudo dpkg -i XBurn-gui_1.1.5_amd64.deb` in the installation package directory and wait for the installation to complete. An example of the installation process is as follows:

        ```shell
        (base) hobot@hobot-ThinkPad-T14-Gen-1:~/tools$ sudo dpkg -i XBurn-gui_1.1.5_amd64.deb 
        [sudo] password for hobot: 
        Selecting previously unselected package XBurn-gui.
        (Reading database ... 494898 files and directories currently installed.)
        Preparing to unpack XBurn-gui_1.1.5_amd64.deb ...
        Unpacking XBurn-gui (1.1.5) ...
        Setting up XBurn-gui (1.1.5) ...
        Udev rules installed and activated
        User nobody added to plugdev group
        User hobot added to plugdev group
        User snapd-range-524288-root added to plugdev group
        User snap_daemon added to plugdev group
        User xpj added to plugdev group
        Processing triggers for mailcap (3.70+nmu1ubuntu1) ...
        Processing triggers for gnome-menus (3.36.0-1ubuntu3) ...
        Processing triggers for desktop-file-utils (0.26-1ubuntu3) ...
        Processing triggers for hicolor-icon-theme (0.17-2) ...
        ```
        
2. Execute the command `sudo XBurn-gui`, or click the `XBurn-gui` icon in the application menu (a password prompt will appear). Enter your password to open the flashing tool interface.

## Checking Drivers

On first use, verify that the drivers are correctly installed. Open the tool and switch to the driver interface. If a message indicates that the driver is not installed, click the **Install Driver** button and follow the prompts to install it.

![Xburn Checking Drivers](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-bf4b-43f0-8ae0-25f3c6ddbd3c-en.png)

This interface allows you to view and manage driver status:
- **Driver Name**: Lists installed drivers (e.g., USB Driver (ADB, Fastboot, DFU) and USB to Serial Driver (CH341)).
- **Current Version**: Displays the current version of each driver.
- **Actions**: Provides buttons to install or uninstall drivers.
- **Scan Drivers**: Offers a button to scan for and install new drivers.

## Hardware Connections

- Connect the serial port to PC: Micro-USB  
- Connect the flashing port to PC: USB Type-C  
- Connect the power cable: USB Type-C, using a power adapter that supports 5V/5A  

## Firmware Flashing

![6443f0bb-da94-4a52-8abb-480bcea2bdd9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/6443f0bb-da94-4a52-8abb-480bcea2bdd9-en.png)

1. **Product Type**: Select `X5`  
2. **Connection Type**: Select `Serial+USB`  
3. **Download Mode**: Select `xmodem_fastboot`  
4. **Image Directory**: Specify the directory containing the firmware image to be flashed.  
5. **Batch Flashing Quantity**: Set the number of devices to flash simultaneously. Adjust this number based on your computer’s performance and the bandwidth of your hardware connection. It is recommended to flash no more than 8 devices at once.  
6. **Baud Rate**: Choose `115200` for `RDK X5`, and `921600` for `RDK X5 Module`.  

7. Click **Start Upgrade**, then plug/unplug the power supply when prompted.

        If the serial port is lost after plugging/unplugging power, keep the board unpowered initially; power it on only after the prompt appears.
        
        ![d785a399-9e2e-40c5-a0c8-222a515f35f0](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/d785a399-9e2e-40c5-a0c8-222a515f35f0-en.png)
        
8. Start upgrading

        ![267d637b-f67e-42a7-981f-2e45278bd877](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/267d637b-f67e-42a7-981f-2e45278bd877-en.png)

9. Upgrade completed

        ![078e4c6a-fca1-467b-bc93-c5a7ca73f8b7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/078e4c6a-fca1-467b-bc93-c5a7ca73f8b7-en.png)