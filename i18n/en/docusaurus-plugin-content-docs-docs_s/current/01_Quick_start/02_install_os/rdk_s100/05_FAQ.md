---
sidebar_position: 5
---

# 1.2.1.4 FAQ

## Issues Encountered When Using Ubuntu Laptop

Q1. **Serial port garbled when connecting the development board to an Ubuntu laptop**

   1. Download the official serial driver [CH340N Driver](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)
   2. Modify `ch341_tty_driver->name = "ttyUSB";`
   3. Recompile and install the driver

Q2. **Driver installation required for Ubuntu 24.04**

   1. Run the following script

   ```bash
   #!/bin/bash

   set -e

   echo "[INFO] Updating APT package list..."
   sudo apt update

   echo "[INFO] Installing required packages..."
   sudo apt install -y dfu-util libusb-1.0-0-dev

   echo "[INFO] Writing udev rules to /etc/udev/rules.d/99-drobotics.rules..."

   sudo tee /etc/udev/rules.d/99-drobotics.rules > /dev/null <<EOF
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6610", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6615", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6620", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6625", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", ATTR{idProduct}=="6631", MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
   EOF

   echo "[INFO] Reloading and triggering udev rules..."
   sudo udevadm control --reload
   sudo udevadm trigger

   echo "[INFO] Setup complete. Please replug your devices or reboot if necessary."
   ```

   2. Or execute the following commands sequentially

   ```bash
   # Update APT sources
   sudo apt update

   # Install DFU tools and libusb
   sudo apt install -y dfu-util libusb-1.0-0-dev

   # Set permissions for the development board interface
   sudo tee /etc/udev/rules.d/99-drobotics.rules > /dev/null <<EOF
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6610", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6615", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6620", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6625", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", ATTR{idProduct}=="6631", MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
   EOF

   # Reload udev
   sudo udevadm control --reload
   sudo udevadm trigger
   ```

   3. Use a Type-C cable to connect the computer to the Type-C port on the development board (near the DC power connector)
   4. Click to [download](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) the latest `Xburn` tool
   5. Install and launch the `Xburn` flashing tool.

## Serial Port Garbled When Using MacOS Laptop

Taking MacOS version 15.0 (M3 chip) as an example, the default serial driver on MacOS may cause garbled output when connecting to CH340N at a baud rate of 921600. The latest CH340N driver needs to be installed. Follow these steps:

1. The default CH340N driver displays the connected device as `tty.usbserial*`, indicating the default MacOS serial driver is in use. An update is required:
   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttyusb.png)

2. Installation process: (The following steps are based on the README.md document on the [CH340N latest driver release page](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file))
   1. Click to download the compressed package on the [CH340N latest driver release page](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install1.png)
   2. Unzip and install the driver using the pkg package
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install2.png)
   3. Click Continue
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install3-en.png)
   4. Click Install and enter the password
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install4-en.png)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install5-en.png)
   5. Click Install and open System Settings
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install6-en.png)
   6. Authorize permission and enter the password
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install7-en.png)
   7. A pop-up window shows successful installation
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install8-en.png)
   8. **<font color='red'>Restart the computer</font>**
   9. Check if the installation was successful. Recognizing `tty.wch*` indicates the driver was installed successfully
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttywch.png)
3. Connect the device to verify
   :::warning Note

   The latest official CH340N driver still does not support communication at 921600 baud rate using the MacOS built-in `screen` tool. Please use the `minicom` tool.

   :::

   1. Using the image above as an example, generally the smaller number corresponds to the ACore serial port, and the larger number corresponds to the MCU serial port. The image shows `/dev/tty.wchusbserial1220` is the ACore serial port, and `/dev/tty.wchusbserial1230` is the MCU serial port. The command to connect to the ACore serial port is: `minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`; the command to connect to the MCU serial port is: `minicom -D /dev/tty.wchusbserial1230 -b 921600 -8`. Replace the device path in the command according to your actual device number **/dev/tty.wchusbserial**.
   2. Command to connect to the ACore serial port using `minicom` (`minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom.png)
   3. Verify by connecting to the development board
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom-success.png)

4. If the serial port still shows garbled characters after installing or using the CH340N driver downloaded from the official website.

   A: If the driver has been installed from the official website but the device still shows as `tty.usbserial*`, move CH34xVCPDriverApp to the Trash, empty the Trash, **<font color='red'>restart the computer</font>**, and reinstall according to [the steps above](#serial-port-garbled-when-using-macos-laptop).


:::tip

For more issues, please refer to the [FAQ](../../../08_FAQ/01_hardware_and_system.md) section. You can also visit the [D-Robotics Developer Official Forum](https://developer.d-robotics.cc/forum) for assistance.

:::