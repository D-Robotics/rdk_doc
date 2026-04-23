---
sidebar_position: 1
---

# 1.2.1.1 全镜像烧录


全镜像烧录是指将完整的操作系统镜像写入存储介质（通常为 SD 卡或 eMMC）的过程。该过程主要用于为开发板提供运行环境，包括用户空间应用、驱动程序及基础软件服务。

:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备。
- RDK X3 的 Type-C USB 接口仅用作供电。
- RDK X3 开发板需要使用支持 5V/3A 的电源适配器为开发板供电，推荐使用[基础配件清单](../../../07_Advanced_development/01_hardware_development/rdk_x3/accessory.md)中推荐的电源适配器型号。
- 请不要使用电脑 USB 接口为开发板供电，否则会因供电不足造成开发板异常断电、反复重启等异常情况。

:::

## 系统启动介质

RDK X3 采用 Micro SD 存储卡作为系统启动介质，烧录时将系统烧录进 Micro SD 卡。

- 准备至少 8GB 容量的 Micro SD 存储卡，以便满足 Ubuntu 系统、应用功能软件对存储空间的需求。
- SD 读卡器。

## 烧录工具

RDK X3 可借助 PC 端工具 **RDK Studio** 和 **Rufus** 来完成 Ubuntu 系统的烧录工作。

### RDK Studio 工具

- 支持使用 “本地已有镜像” 和 “烧录时在线下载镜像” 两种方式。
- 支持 Windows、Mac 系统。

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

- 支持使用 “本地已有镜像”
- 支持 Windows 系统

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

### 使用 RDK Studio 工具

RDK Studio 工具提供烧录系统功能，并且可以连接设备并进行管理，支持在 Windows、Linux、Mac 平台上使用，详细步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

### 使用 Rufus 工具

#### 硬件连接

将 Micro SD 卡插入读卡器，将读卡器插入 PC 相应接口。

#### 烧录方法

1. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的 Micro SD 存储卡作为目标设备。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

3. 点击 “选择” 按钮，选择解压出来的 `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img` 文件作为烧录镜像。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/rufus_select_image.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

3. 其他参数保持默认，点击 “开始” 按钮，等待烧录完成。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/rufus_success.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

4. 烧录完成后，关闭 Rufus 并取出存储卡。
