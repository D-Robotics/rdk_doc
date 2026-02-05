---
sidebar_position: 4
---

# 1.2.1.4 FAQ

:::tip

For solutions to more issues, please refer to the [FAQ](../../../08_FAQ/01_hardware_and_system.md) section, and you can also visit the [D-Robotics Developer Official Forum](https://developer.d-robotics.cc/forum) for assistance.

:::

## **Issues encountered when using Ubuntu-based laptops**

1. **Garbled characters appear on the serial port after connecting the development board to an Ubuntu laptop**

   1. Download the official serial driver: [CH340N Driver](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)
   2. Modify `ch341_tty_driver->name = "ttyUSB";`
   3. Recompile and reinstall the driver

2. **Ubuntu 24.04 requires driver installation**

   1. Run the following script:

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

   2. Alternatively, execute the following commands one by one:

   ```bash
   # Update APT sources
   sudo apt update

   # Install DFU tools and libusb
   sudo apt install -y dfu-util libusb-1.0-0-dev

   # Set permissions for development board interfaces
   sudo tee /etc/udev/rules.d/99-drobotics.rules > /dev/null <<EOF
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6610", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6615", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6620", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6625", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", ATTR{idProduct}=="6631", MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
   EOF

   # Reload udev rules
   sudo udevadm control --reload
   sudo udevadm trigger
   ```

   3. Connect your computer to the development board using a Type-C cable (plug into the Type-C port near the DC power jack).
   4. Click [here](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) to download the latest D-Navigation, e.g., `D-navigation-linux-x64-v2.4.tar.gz`.
   5. Extract the archive and open a `Terminal` inside the extracted folder.
   6. Run `sudo ./D-navigation --no-sandbox` to launch the flashing tool.

## **Serial Port Garbled Characters on macOS Laptops**

Taking macOS version 15.0 (M3 chip) as an example: the default macOS serial driver connects to CH340N at 921600 baud rate, resulting in garbled output. You need to install the latest CH340N driver as follows:

1. With the default CH340N driver, plugging in the device shows it as `tty.usbserial*`, indicating that the system is using macOS’s built-in serial driver, which needs updating:
   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttyusb.png)

2. Installation procedure (based on the README.md from the [latest CH340N driver release page](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file)):
   1. Download the compressed package from the [CH340N latest driver release page](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install1.png)
   2. Unzip and install the driver using the `.pkg` installer
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install2.png)
   3. Click Continue
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install3.png)
   4. Click Install and enter your password
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install4.png)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install5.png)
   5. Click Install, then open System Settings
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install6.png)
   6. Authorize and enter your password
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install7.png)
   7. A pop-up confirms successful installation
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install8.png)
   8. **<font color='red'>Restart your computer</font>**
   9. Verify successful installation: if the device appears as `tty.wch*`, the driver has been installed correctly
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttywch.png)

3. Connect and verify the device
   :::warning Note

   The latest official CH340N driver still does **not** support macOS’s built-in `screen` tool for communication at 921600 baud rate. You must use the `minicom` tool instead.

   :::

   1. In the example above, typically the smaller-numbered port corresponds to the ACore serial port, while the larger-numbered one corresponds to the MCU serial port. For instance, `/dev/tty.wchusbserial1220` is the ACore serial port and `/dev/tty.wchusbserial1230` is the MCU serial port. To connect to the ACore serial port, run:  
      `minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`;  
      to connect to the MCU serial port, run:  
      `minicom -D /dev/tty.wchusbserial1230 -b 921600 -8`.  
      Please replace the device path (`/dev/tty.wchusbserial...`) with your actual device identifier.
   2. Example command to connect to the ACore serial port using `minicom` (`minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`):
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom.png)
   3. Verification after connecting to the development board:
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom-success.png)

4. FAQ

   1. Q1: I previously installed or used the CH340N driver downloaded from the official website, but the serial output is still garbled.  
      - A: If you have already installed the driver from the official site but the device still appears as `tty.usbserial*`, move the `CH34xVCPDriverApp` to Trash, empty the Trash, **<font color='red'>restart your computer</font>**, and reinstall the driver following the steps outlined [above](#using-macos-system-laptops-serial-port-garbled-characters-issue).