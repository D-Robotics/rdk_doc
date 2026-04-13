---
sidebar_position: 2
---

# 1.2.2.2 Bootloader 固件烧录

## Bootloader 固件烧录概述

Bootloader 固件烧录是指将启动相关的底层固件（如 Miniboot、U-Boot 等 Bootloader）写入开发板板载 NAND Flash 的过程。该固件负责设备上电后的初始化与启动流程，包括硬件初始化和引导加载操作系统，是系统能够正常启动的前提。一般在设备无法启动、启动固件损坏或需要升级底层引导程序时进行，用于决定设备“能否启动”。

:::warning 固件烧录说明

- RDK 最小系统存储于 NAND Flash 中，包含 Bootloader（Miniboot、U-Boot）等关键启动组件。
- 设备出厂时已预装与硬件匹配的最新 NAND 固件。
- 为确保兼容性与设备稳定性，严禁降级刷入旧版本固件，否则可能导致设备无法正常启动。
- 若您已遇到设备无法启动的情况，请重新烧录 NAND 固件。

:::

## 烧录工具

RDK X3 Module 可借助 PC 端工具 **hbupdate** 来完成 Bootloader 固件的烧录工作。

### hbupdate

- 支持使用“本地已有镜像”。
- 支持 Windows 系统。
- 通过 fastboot 方式烧录 Bootloader 固件。

#### 下载地址

[[点击此处]](https://archive.d-robotics.cc/downloads/hbupdate/) 进入下载地址，根据使用平台、OS 和固件版本选择工具安装包的版本。

#### 安装方法

**Windows 系统**

1. 解压 `hbupdate` 压缩包。

    :::warning 注意

    解压工具压缩包时，解压路径中不要包含空格、中文、特殊字符等内容。

    :::

2. 双击 `.exe` 应用程序文件即可打开 hbupdate 工具。

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_exe.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>


## 固件下载

[[点击此处]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x3/) 下载固件，进入下载目录，根据产品容量选择对应的 `.img` 文件。

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/miniboot_download.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

## 固件烧录步骤


### 驱动检查

在使用烧录工具前，Windows 用户需要按照以下步骤确认驱动是否已安装。

1. 将载板的 Micro USB 接口（烧录口）通过 USB 线与 PC 连接，接口位置参考下图。

    <img
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/flashing_port.png"
    style={{ width: '100%', height: 'auto', align: 'center' }}
    />

2. 给设备上电，观察电脑设备管理器端口状态，如出现 `USB download gadget` 未知设备时，需要更新设备驱动。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/usb_download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

### 驱动下载与安装 {#x3md-android-driver}

1. [[点击此处]](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) 下载并解压驱动包 `android_hobot.zip`。


2. 进入解压后的目录，以管理员身份运行 `5-runasadmin_register-CA-cer.cmd`，完成驱动程序的注册。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/usb_driver_administrator.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. 双击 `USB download gadget` 未知设备，选择驱动包解压目录，然后点击下一步。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/usb_download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

4. 驱动安装完成后，设备管理器会显示 fastboot 设备 `Android Device`。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/android_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />


### 烧录固件

运行 `hbupdate.exe` 打开烧录工具，并按照以下步骤进行烧录：

1. **选择开发板型号**（必选项）：

    - **RDK_X3_2GB**：RDK X3（旭日 X3 派），2GB 内存版本，仅支持烧写最小系统镜像。
    - **RDK_X3_4GB**：RDK X3（旭日 X3 派），4GB 内存版本，仅支持烧写最小系统镜像。
    - **RDK_X3_MD_2GB**：RDK X3 Module，2GB 内存版本。
    - **RDK_X3_MD_4GB**：RDK X3 Module，4GB 内存版本。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_boardname.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

2. 点击 `Browse` 按钮选择将要烧录的镜像文件（必选项）。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_select_image.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. 点击 `Start` 按钮开始烧录：

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_start.png"
      style={{ width: '80%', height: 'auto', align: 'center' }}
    />

4. 烧录完毕断开电源，断开和电脑的连接线，将 BOOT 管脚跳线帽拔下，重新上电即可。

若启动正常，硬件上的 ACT LED 会进入两次快闪、一次慢闪的状态。

### 检查升级结果

- 镜像烧录成功时，工具提示如下：

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_success.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

- 镜像烧录失败时，工具提示如下，此时需要确认 PC 设备管理器是否存在 `Android Device` 设备：

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_fail.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

