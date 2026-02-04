---
sidebar_position: 4
---

# 1.2.1.4 FAQ

:::tip

更多问题的处理，可以查阅 [常见问题](../../../08_FAQ/01_hardware_and_system.md) 章节，同时可以访问 [D-Robotics 开发者官方论坛](https://developer.d-robotics.cc/forum) 获得帮助。

:::

## **使用 ubuntu 系统笔记本遇到的问题**

1. **ubuntu 系统笔记本连接开发板后，串口出现乱码**

   1. 下载官方串口驱动 [CH340N 驱动](https://www.wch.cn/downloads/CH341SER_LINUX_ZIP.html)
   2. 修改`ch341_tty_driver->name = "ttyUSB";`
   3. 重新编译并安装驱动

2. **ubuntu 24.04 系统需要安装驱动**

   1. 执行如下脚本

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

   2. 或者依次执行如下命令

   ```bash
   # 更新APT源
   sudo apt update

   # 安装DFU工具和libusb
   sudo apt install -y dfu-util libusb-1.0-0-dev

   # 设定开发板接口权限
   sudo tee /etc/udev/rules.d/99-drobotics.rules > /dev/null <<EOF
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6610", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6615", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6620", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="3652", ATTR{idProduct}=="6625", MODE="0666"
   SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", ATTR{idProduct}=="6631", MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
   EOF

   # 重载udev
   sudo udevadm control --reload
   sudo udevadm trigger
   ```

   3. 使用 Type-C 线连接电脑和开发板的 Type-C 口（靠近 DC 电源接头位置）
   4. 点击[下载](https://archive.d-robotics.cc/downloads/software_tools/download_tools/)最新的 D-Navigation，例如`D-navigation-linux-x64-v2.4.tar.gz`
   5. 解压缩，并在解压缩后的文件夹中开启`Terminal`
   6. 执行`sudo ./D-navigation --no-sandbox`，启动烧录工具。

## **使用 MacOS 系统笔记本串口乱码问题**

以 MacOS 版本 15.0(芯片 M3)为例，MacOS 系统默认串口驱动以 921600 波特率连接 CH340N 会出现乱码，需要安装最新的 CH340N 的驱动，操作如下：

1. 默认的 CH340N 驱动插上设备显示为`tty.usbserial*`，说明此时为 MacOS 默认串口驱动，需要更新：
   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttyusb.png)

2. 安装流程：(以下安装流程参考[CH340N 最新驱动发布页面](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file)下的 README.md 文档编写)
   1. 在[CH340N 最新驱动发布页面](https://github.com/WCHSoftGroup/ch34xser_macos?tab=readme-ov-file)点击下载压缩包
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install1.png)
   2. 解压并使用 pkg 包进行驱动安装
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install2.png)
   3. 点击继续
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install3.png)
   4. 点击安装并输入密码
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install4.png)
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install5.png)
   5. 点击安装，打开系统设置
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install6.png)
   6. 授权允许并输入密码
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install7.png)
   7. 弹框显示安装成功
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-install8.png)
   8. **<font color='red'>重启电脑</font>**
   9. 检查是否安装成功，识别到 tty.wch\* 表示驱动安装成功
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-ttywch.png)
3. 连接设备验证
   :::warning 注意

   CH340N 最新官方驱动仍不支持 MacOS 系统自带的 screen 工具以 921600 波特率通讯，需使用`minicom`工具。

   :::

   1. 以上图为例，一般编号小的为 ACore 串口，编号大的为 MCU 串口，如上图显示`/dev/tty.wchusbserial1220`是 ACore 串口，`/dev/tty.wchusbserial1230`是 MCU 串口，连接 ACore 串口命令是：`minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`; 连接 MCU 串口命令是：`minicom -D /dev/tty.wchusbserial1230 -b 921600 -8`，请根据实际设备编号 **/dev/tty.wchusbserial** 替换命令中的设备路径
   2. `minicom`连接 ACore 串口命令如下(`minicom -D /dev/tty.wchusbserial1220 -b 921600 -8`）
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom.png)
   3. 连接开发板验证
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom-success.png)

4. FAQ

   1. Q1：之前已安装或使用官网下载的 CH340N 驱动后，串口查看还是乱码。
      - A：如果已经从官网安装驱动，但是查看到设备仍为`tty.usbserial*`，需要把 CH34xVCPDriverApp 放到废纸篓里，清空废纸篓，**<font color='red'>重启电脑</font>**，根据[上面的步骤](#使用MacOS系统笔记本串口乱码问题)重新安装。
