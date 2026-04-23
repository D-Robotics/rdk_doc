---
sidebar_position: 1
---

# 1.2.2.1 全镜像烧录

全镜像烧录是指将完整的操作系统镜像写入存储介质（通常为 SD 卡或 eMMC）的过程。该过程主要用于为开发板提供运行环境，包括用户空间应用、驱动程序及基础软件服务。

:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备。
- RDK X3 Module 通过载板上的 DC 接口供电，推荐使用[配件清单](../../../07_Advanced_development/01_hardware_development/rdk_x3_module/accessory.md)中推荐的 12V/2A 适配器。
- 请不要使用电脑 USB 接口为开发板供电，否则会因供电不足造成开发板异常断电、反复重启等异常情况。

:::

## 系统启动介质

RDK X3 Module 板载 eMMC，支持 Micro SD 存储卡和 eMMC 作为系统启动介质，烧录时可选择将系统烧录进 Micro SD 存储卡或 eMMC。

- 准备至少 8GB 容量的 Micro SD 存储卡，以便满足 Ubuntu 系统、应用功能软件对存储空间的需求。
- SD 读卡器。

## 烧录工具

RDK X3 Module 可借助 PC 端工具 **RDK Studio** 和 **Rufus** 来完成 Ubuntu 系统的烧录工作。

### RDK Studio 工具

- 支持使用 “本地已有镜像” 和 “烧录时在线下载镜像” 两种方式。
- 支持 Windows、Mac 系统。
- 支持 Micro SD 存储卡烧录。
- 支持 eMMC 烧录，采用 UMS 方式。

#### RDK Studio 下载链接
- [点此下载 windows 版本](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22%20Setup.exe)
- [点此下载 macOS 版本](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22-arm64.dmg)

#### 安装方法

**Windows 系统**

双击下载的 `.exe` 安装包文件即可自动安装完成并打开应用。



**Mac 系统**

双击安装包，随即会出现一个弹窗，单击并按住图标拖到 Applications 图标中。

<img 
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_install_mac.JPEG" 
    style={{ width: '100%', height: 'auto', align:'center'}}
/>

### Rufus 工具

- 支持使用 “本地已有镜像”。
- 支持 Windows 系统。
- 支持 Micro SD 存储卡烧录。
- 支持 eMMC 烧录，采用 UMS 方式。

#### 下载地址

[[点击此处]](https://rufus.ie/zh/) 进入 Rufus 官网，根据使用平台选择工具版本。

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_install.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

#### 安装方法

双击下载的 `.exe` 安装包文件即可自动安装完成并打开应用。

## 镜像下载

1. [[点击此处]](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/) 下载镜像，选择 RDK X3 镜像。

    :::info 镜像说明

    - RDK X3 目前提供 Ubuntu 20.04/22.04 系统镜像，支持无桌面 Server 版本系统和带有桌面的 desktop 版本系统:

      - desktop 版本：带有桌面的 Ubuntu 系统，可以外接屏幕、鼠标操作。
      - server 版本：无桌面的 Ubuntu 系统，可以通过串口、网络远程连接操作。

    - RDK X3 出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，建议参考本文档完成最新版本系统镜像的烧写。

      - 3.X.X 版本（Ubuntu 22.04）：基于 RDK Linux 开源代码包制作，支持 RDK X3、RDK X3 Module 开发套件。
      - 2.X.X 版本（Ubuntu 20.04）：基于 RDK Linux 开源代码包制作，支持 RDK X3、RDK X3 Module 开发套件。
      - 1.X.X 版本：RDK X3 历史版本，仅支持 RDK X3，解压后系统镜像名为 `system_sdcard.img`。

    :::

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. 进入所选版本目录，以下载 3.0.3 版本的系统镜像为例，选择 “server” 版本镜像或 “desktop” 版本镜像，点击下载。

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download1.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. 下载完成后，解压出 Ubuntu 系统镜像文件，如 `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`。

## 系统烧录步骤

### 烧录 SD 卡

#### 使用 RDK Studio 工具

RDK Studio 工具提供烧录系统功能，并且可以连接设备并进行管理，支持在 Windows、Linux、Mac 平台上使用，详细步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

#### 使用 Rufus 工具

烧录步骤与 [RDK X3 系统烧录步骤](../rdk_x3/01_system_burn.md#使用-rufus-工具) 相同。


### 烧录 eMMC {#x3md-emmc-system-burn}

#### 驱动检查

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

#### 驱动下载与安装 {#x3md-android-driver}

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


#### 烧录系统 {#x3md-emmc-ums}


1. 使用跳线帽将 RDK X3 Module 载板切换到 3.3V 供电。

    <img
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/3.3v_power.png"
          style={{ width: '100%', height: 'auto', align: 'center' }}
      />

2. 第一次使用载板的 Micro USB 接口（调试串口）时请 [[点击此处]](https://archive.d-robotics.cc/downloads/software_tools/serial_to_usb_drivers/) 下载安装 CH340 驱动，将载板的 Micro USB 接口（调试串口）通过 USB 线与 PC 连接，接口位置参考下图。

      <img
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/debug_port.png"
            style={{ width: '100%', height: 'auto', align: 'center' }}
        />

3. 将载板的 Micro USB 接口（烧录口）通过 USB 线与 PC 连接。

      <img
        src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/flashing_port.png"
        style={{ width: '100%', height: 'auto', align: 'center' }}
      />

4. 下载远程连接工具 [MobaXterm](https://mobaxterm.mobatek.net/download.html)。
5. 打开 MobaXterm 工具，点击 `Session`，然后选择 `Serial`，配置端口号（例如 `COM3`，实际使用的串口号以 PC 识别到的串口号为准），设置完成后点击 `OK`。

    串口配置参数如下：

    | 配置项 | 参数值 |
    | ------ | ------ |
    | 波特率（Baud rate） | 921600 |
    | 数据位（Data bits） | 8 |
    | 奇偶校验（Parity） | None |
    | 停止位（Stop bits） | 1 |
    | 流控（Flow Control） | 无 |

    <img
        src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/serial_parameter.png"
        style={{ width: '100%', height: 'auto', align: 'center' }}
      />

3. 开发板上电后立刻长按空格键，进入 U-Boot 命令行模式。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/enter_uboot.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

4. 在 U-Boot 中执行 `watchdog off` 关闭看门狗，防止设备重启；执行 `ums 0 mmc 0`，将板载 eMMC 设备（设备号 0）通过 USB OTG 接口 0 映射为 USB Mass Storage 设备，使得主机系统可以将其识别为标准 U 盘，从而进行直接读写或烧录操作。

    ```bash
    watchdog off
    ums 0 mmc 0
    ```

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/watch_dog_off.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

5. PC 识别到标准 U 盘即 RDK X3 Module 的 eMMC 分区。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/portable_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />



##### 使用 RDK Studio 工具

烧录时，在 “选择存储设备” 步骤中选择识别到的标准 U 盘，烧录步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

<img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/studio_select_storage_refresh.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

##### 使用 Rufus 工具

打开 Rufus 工具，在 “设备” 下拉框中选择对应的盘符作为目标设备，其余步骤与 [RDK X3 使用 Rufus 烧录](../rdk_x3/01_system_burn.md#使用-rufus-工具) 一致，完成镜像烧录。

![imagex3md-ums4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums4.png)

:::warning 注意

如烧录过程发生中断，请按照上述步骤重新进行。

:::


