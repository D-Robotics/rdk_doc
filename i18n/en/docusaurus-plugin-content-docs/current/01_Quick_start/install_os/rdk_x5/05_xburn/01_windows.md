---
sidebar_position: 1
---

# Windows Platform


## Installing and Launching XBurn

1. Double-click the installer package “XBurn-gui_1.1.5_x64-setup.exe” to begin installation.        
2. After installation completes, the “XBurn-gui” interface will open automatically:

        ![image-202510311956](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-202510311956-en.png)

3. Afterwards, you can launch the software directly by double-clicking the “XBurn-gui” icon on your desktop.

## Checking Drivers

On first use, you need to verify whether drivers are correctly installed. Open the tool and switch to the driver interface. If a message indicates that drivers are not installed, click the “Install Driver” button and follow the prompts to install them.

![Xburn Checking Drivers](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-bf4b-43f0-8ae0-25f3c6ddbd3c-en.png)

This interface is used to view and manage driver status.
- **Driver Name**: Lists the names of installed drivers (e.g., USB Driver (ADB, Fastboot, DFU) and USB to Serial Driver (CH341)).
- **Current Version**: Displays the current version of each driver.
- **Actions**: Provides buttons to install or uninstall drivers.
- **Scan Drivers**: Offers a button to scan for and detect new drivers for installation.

## Hardware Connections

- Connect serial port to PC: Micro-USB  
- Connect flashing port to PC: USB Type-C  
- Connect power cable: USB Type-C, using a power adapter that supports 5V/5A


## Firmware Flashing

![6443f0bb-da94-4a52-8abb-480bcea2bdd9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/6443f0bb-da94-4a52-8abb-480bcea2bdd9-en.png)

1. **Product Type**: Select `X5`  
2. **Connection Type**: Select `Serial+USB`  
3. **Download Mode**: Select `xmodem_fastboot`  
4. **Image File Directory**: Choose the directory containing the firmware image files to be flashed.  
5. **Batch Flashing Quantity**: Set the number of devices to flash simultaneously. Adjust this number reasonably based on your computer’s performance, bandwidth of the hardware connection type, etc. We recommend flashing no more than 8 devices at once.  
6. **Baud Rate**: Select `115200` for `RDK X5`, and `921600` for `RDK X5 Module`.

7. Click **Start Upgrade**, then unplug and re-plug the power as prompted.

        If the serial port disappears after reconnecting power, keep the board powered off initially; apply power only after the prompt appears.
        
        ![d785a399-9e2e-40c5-a0c8-222a515f35f0](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/d785a399-9e2e-40c5-a0c8-222a515f35f0-en.png)

8. Upgrade in Progress

        ![267d637b-f67e-42a7-981f-2e45278bd877](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/267d637b-f67e-42a7-981f-2e45278bd877-en.png)                        

9. Upgrade Completed

        ![078e4c6a-fca1-467b-bc93-c5a7ca73f8b7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/078e4c6a-fca1-467b-bc93-c5a7ca73f8b7-en.png)