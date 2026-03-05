---
sidebar_position: 4
---

# 1.2.2 Xburn 烧录工具

Xburn 提供界面化烧录升级功能，如下


## **烧录工具界面简介**

**烧录界面**

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-xburn-menu.png)

说明如下
1. 产品类型： 请选择您要烧录设备的对应产品类型。

- 每种产品类型对应一种特定的硬件型号或配置（例如 X5 、 RDKS100 等）。

- 请确保选择与设备实际型号一致的选项，以确保刷入的固件与设备兼容，从而避免刷机失败。

2. 连接类型： 请选择设备与主机通信的连接方式，支持串口、 USB 和网口三种方式。

- 可以组合使用不同的连接方式，例如串口 +USB，以满足不同的刷机需求。

- 连接类型的选择会直接影响刷机过程中的下载模式。

- 每种连接方式对应的协议和操作方法可能不同，请根据目标设备的兼容性和使用场景选择合适的连接方式。

3. 下载模式： 根据所选设备的连接类型，系统会自动匹配适用的下载模式。常用的下载模式如下：

- fastboot（推荐）：

  - 连接类型： USB

  - 适用于非空板设备，能够实现快速烧录，速度最快。

- dfu_fastboot（空板烧录）：

  - 连接类型： USB

  - 适用于空板设备，进行整机烧录。需手动拨码使设备进入 DFU 引导模式，速度较快。

4. 镜像文件目录：
- 请选择要烧录的镜像文件所在的目录。

## **工具下载**

Xburn 下载 （可[点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/download_tools/)）

Xburn 已提供 Windows，Ubuntu，MacOS 平台安装包，用户根据自己使用平台选择下载，以 Xburn-1.x.x 为例
- xburn-gui_1.x.x_x64-setup.exe
- xburn-gui_1.x.x_amd64.deb
- xburn-gui_1.x.x_universal.dmg

:::info 注意

1.x.x 是 Xburn 工具版本号，请以服务器更新版本为准

:::


## **驱动 & 依赖下载**

以下分别以 Windows，Ubuntu，MacOS 平台为例

#### Windows 平台工具安装

:::info 注意

由于 Windows 平台 Xburn 工具已集成 dfu-util，fastboot，adb，用户无需额外安装，仅需安装 windows 平台驱动。

:::

**USB 驱动下载与安装**

usb 驱动下载（可[点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/winusb_drivers/)）

下载`sunrise5_winusb.zip`压缩包，进行驱动安装，步骤如下：

1、解压`sunrise5_winusb.zip`。

2、进入`sunrise5_winusb`，右键点击`install_driver.bat`，选择以管理员身份运行。

**验证驱动安装**

在开发板的 uboot 命令行下输入 fastboot 0，让开发板进入 fastboot 模式：

```bash
Hit any key to stop autoboot:  0
Hobot$
Hobot$
Hobot$ fastboot 0
```
成功安装驱动后，设备管理器会显示 Android Device 设备，如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-ok.png)

如果未成功安装驱动时，设备管理器会提示存在 USB download gadget 的未知设备，如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-no.png)


#### Ubuntu 平台工具安装

Ubuntu 平台用户可通过以下命令安装工具

```
sudo apt update
sudo apt install android-tools-adb android-tools-fastboot
sudo apt install dfu-util
```


#### MacOS 平台工具安装

Ubuntu 平台用户可通过以下命令安装工具

```
brew update
brew install android-platform-tools
brew install dfu-util
```


