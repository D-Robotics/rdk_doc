---
sidebar_position: 4
---

# RDK S100 系列

:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI 和网线之外的任何设备
- 选用正规品牌的电源适配器，否则会出现供电异常，导致系统异常断电的问题
- 建议使用板载 POWER ON/OFF 按键实现主板上下电，并在适配器断电状态下对 DC 头进行插拔。

:::

## 烧录准备

### 供电

RDK S100 开发板通过 DC 接口供电，推荐使用套件中自带的电源适配器。

### 存储

RDK S100 采用 eMMC 作为系统启动介质。

### 显示

RDK S100 开发板支持 HDMI 显示接口。通过对应的线缆将开发板与显示器相连接，可实现图形化桌面显示。

### 网络连接

RDK S100 开发板支持以太网、Wi-Fi 两种网络接口，用户可通过任意接口实现网络连接功能。

### 驱动下载

**安装 USB 驱动（仅 Windows）**

在使用烧录工具前，Windows 用户需要确认驱动是否已安装。

**1. 进入 Fastboot 模式**

首先需要让开发板进入 Fastboot 模式，以便电脑识别设备：

1. 使用串口线连接开发板与电脑（参考 [硬件介绍-串口登录](../01_hardware_introduction/01_rdk_s100.md) 章节）。
2. 给开发板上电，立刻长按空格键进入 Uboot 命令行。
3. 在命令行输入 `fastboot 0` 并回车：

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

**2. 检查与安装驱动**

此时打开电脑的**设备管理器**：

* **未安装驱动时**：会提示存在 `USB download gadget` 的未知设备（如下图）。此时需要安装驱动。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-no.png)

* **驱动下载与安装**：
    1. [点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/winusb_drivers/) `sunrise5_winusb.zip` 压缩包。
    2. 解压压缩包，进入目录右键点击 `install_driver.bat`，选择**以管理员身份运行**。

* **安装成功**：驱动安装完成后，设备管理器会显示 `Android Device` 设备（如下图）。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-ok.png)

## 系统烧录

RDK S100 套件目前提供 Ubuntu 22.04 系统镜像，可支持 Desktop 桌面图形化交互。

:::info 注意

**RDK S100** 出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。
:::

### 镜像下载

参考[1.6 资源汇总](../../01_Quick_start/download.md)章节。

### 整机系统烧录

:::info 注意

目前**需要将 SW3 拨至 ↑ 位置**，使用板载 eMMC 来启动，暂时不支持从 M.2 NVMe 固态硬盘启动。

:::

RDK S100 开发套件可借助 PC 端工具 D-Navigation 来完成 Ubuntu 系统的烧录工作。当前，该烧录过程支持两种 USB 下载模式，用户可在烧录工具的 “下载选取” 界面里的 “下载模式” 选项处进行选择。这两种模式的具体区别如下：

- **U-Boot 烧录方式：** 此模式依赖 RDK S100 进入 U-Boot 的烧录模式（即 fastboot 模式），在日常的烧录场景中使用较为频繁，能满足大多数常规的系统烧录需求。
- **USB 烧录方式：** 该模式基于 DFU 协议，当 RDK S100 遇到无法进入 U-Boot 模式，或者系统损坏导致设备变砖等特殊情况时，使用此模式帮助恢复系统。

下面给出使用 PC 工具 D-Navigation 烧录的具体烧录步骤。

:::info 注意

windows pc 上 D-Navigation 需要在[驱动安装](#驱动下载)成功后才能使用，使用前请确保驱动安装成功。

:::

:::tip

在烧录 Ubuntu 系统镜像前，需要做如下准备：

- 准备一根 Type-C 数据线，数据线的一端与板子的 Type-C 接口相连接，另一端与 PC 相连接。
- 请确保 Type-C 数据线为高质量数据线：1. 带有屏蔽层 2. 长度越短越好 3. 数据传输质量高，以确保烧录的稳定性。
- 下载镜像烧录工具 D-Navigation（可[点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/download_tools/)）,根据系统不同，启动地瓜芯片工具 D-Navigation 方式分为三种：

  - Windows 版本启动：

        双击打开 D-Navigation.exe

  - Ubuntu 版本启动：

        xhost +
        sudo ./D-Navigation --no-sandbox

  - MacOS 版本启动(目前支持 M 芯片)：

         xattr -cr D-navigation.app # App 解除隔离，在终端执行
         双击打开 D-Navigation

    :::

#### Uboot 烧录

1. 准备 RDK S100 镜像包

   1. 从网页上[下载](#镜像下载)镜像包
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/download_web.png)
   2. 解压后得到 product 文件夹，结构如下所示，确保同一个文件夹内有`img_packages`文件夹和`xmodem_tools`文件
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)

2. 开发板上电

   :::tip

   U-Boot 方式需要占用串口，须保证串口没有被其它设备或应用占用。
   :::

3. 打开地瓜芯片工具 D-Navigation，完成如下操作：

   - 选择产品型号：S100
   - 下载模式：uboot；介质存储 emmc；类型：secure
   - 点击浏览选择固件所在 product 文件夹
   - 选择与 RDK S100 连接的串口，波特率 921600
   - 点击开始升级(升级过程中，如有'Need manual reset'提示，请重新上电)

   ![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download.png)

4. 待升级完成后重新上电

#### USB 烧录(空片烧录或烧挂重新烧录)

:::tip

SW1、SW2 等说明可查看[1.1.1 章节开关、按键和灯光说明内容](../../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#开关按键和灯光说明)
:::

1. 准备 RDK S100 镜像包

   1. 从网页上[下载](#镜像下载)镜像包
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/download_web.png)
   2. 解压后得到 product 文件夹，结构如下所示，确保同一个文件夹内有`img_packages`文件夹和`xmodem_tools`文件
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)

2. 将设备切换到 DFU 模式，具体步骤：

   1. 将 SW1 拨码至 ↑，关闭电源
   2. 将 SW2 拨码至 ↑，进入 Download 模式
   3. 将 SW1 拨码至 ▽，开启电源
   4. 如果`DOWNLOAD`灯亮，则进入 DFU 模式，否则按下`K1`复位系统。
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1.png)

3. 打开地瓜芯片工具 D-Navigation，完成如下操作

   - 选择产品型号：S100
   - 下载模式：usb；介质存储 emmc；类型：secure
   - 点击浏览选择固件所在 product 文件夹
   - 设备断电重启，点击开始升级，等待升级完成

   ![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download_dfu.jpg)

4. 升级完成后，关闭电源，将烧录开关向下拨动后(从 DFU 模式退出)，重新上电。

### Miniboot 及文件系统升级

D-Navigation 工具支持对 S100 进行[Miniboot 镜像](/rdk_s/Advanced_development/rdk_gen#765-自定义分区说明)更新，在客户需要保留根文件系统修改（例如自行安装的 python/deb 包）时，可以在板端使用`sudo apt update && sudo apt upgrade`进行文件系统升级后，使用 D-Navigation 工具进行 Miniboot 镜像升级。

Miniboot 系统烧录整体流程与[整机系统烧录](#整机系统烧录)一致，需要额外配置：

1. 点击“其他配置”最右边的箭头；
2. 点击并选中“分区选择”；
3. 去掉勾选“emmc”；

- Uboot 烧录示例如下图：
  ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_uboot_miniboot.png)

- USB 烧录示例配置如下图：
  ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_dfu_miniboot.png)

### 启动系统

首先保持开发板断电，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

- **<font color='Green'>绿色</font>** 指示灯：点亮代表硬件上电正常

如果开发板上电后长时间没有显示输出（2 分钟以上），说明开发板启动异常。需要通过串口线进行调试，查看开发板是否正常。

:::

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)

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

## **常见问题**

:::tip

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节，同时可以访问 [D-Robotics 开发者官方论坛](https://developer.d-robotics.cc/forum) 获得帮助。

:::