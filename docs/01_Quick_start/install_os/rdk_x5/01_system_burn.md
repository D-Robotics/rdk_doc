---
sidebar_position: 1
---

# 1.2.3.1 全镜像烧录


全镜像烧录是指将完整的操作系统镜像写入存储介质（通常为 SD 卡或 eMMC）的过程。该过程主要用于为开发板提供运行环境，包括用户空间应用、驱动程序及基础软件服务。

:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备。
- RDK X5 的 Type-C USB 接口仅用作供电。
- 选用正规品牌的 USB Type-C 口供电线，否则会出现供电异常，导致系统异常断电的问题。
- RDK X5 开发板通过 USB Type-C 接口供电，需要使用支持 5V/5A 的电源适配器为开发板供电，请不要使用电脑 USB 接口为开发板供电，否则会因供电不足造成开发板异常断电、反复重启等异常情况。
- 更多供电方式参见 [PoE 供电使用](../../../07_Advanced_development/01_hardware_development/rdk_x5/POE.md)。

:::

## 系统启动介质

RDK X5 采用 Micro SD 存储卡作为系统启动介质，烧录时将系统烧录进 SD 卡。

- 准备至少 16GB 容量的 Micro SD 存储卡，以便满足 Ubuntu 系统、应用功能软件对存储空间的需求。
- SD 读卡器。

## 烧录工具下载

RDK X5 支持 SD 卡单独烧录和 SD 卡在板烧录两种方式，可借助 PC 端工具 **RDK Studio** 和 **Rufus** 来完成 Ubuntu 系统的烧录工作。

### RDK Studio 工具

- 支持使用 “本地已有镜像” 和 “烧录时在线下载镜像” 两种方式。
- 支持 Windows、Linux、Mac 系统。
- 支持 SD 卡单独烧录和 SD 卡在板烧录。

#### - RDK Studio 下载链接：
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
- 支持 SD 卡单独烧录和 SD 卡在板烧录。

#### 下载地址

[[点击此处]](https://rufus.ie/zh/) 进入 Rufus 官网，根据使用平台选择工具版本。

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_install.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

#### 安装方法

双击下载的 `.exe` 安装包文件即可自动安装完成并打开应用。

## 镜像下载

1. [[点击此处]](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/) 进入 RDK X5 镜像下载目录，选择 RDK X5 镜像版本。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/x5_os_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. 进入所选版本目录，以下载 3.3.3 版本的系统镜像为例，选择 “server” 版本镜像或 “desktop” 版本镜像，点击下载。

    :::info 镜像说明

    RDK X5 目前提供 Ubuntu 22.04 系统镜像，支持无桌面 Server 版本系统和带有桌面的 desktop 版本系统:

    - **desktop 版本**：带有桌面的 Ubuntu 系统，可以外接屏幕、鼠标操作。
    - **server 版本**：无桌面的 Ubuntu 系统，可以通过串口、网络远程连接操作。

    :::

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/x5_os_download_type.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. 下载完成后，解压出 Ubuntu 系统镜像文件，如 `rdk-x5-ubuntu22-preinstalled-desktop-3.3.3-arm64.img`。


## 系统烧录步骤

### 使用 RDK Studio 工具

#### SD 卡单独烧录

RDK Studio 工具提供烧录系统功能，并且可以连接设备并进行管理，支持在 Windows、Linux、Mac 平台上使用，详细步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

#### SD 卡在板烧录

1. SD 卡插入开发板，USB Type-C 接到 PC 端，长按 Sleep 按键（位于耳机接口旁），开发板上电，等待 5s，开发板进入烧录模式。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/sleep_key.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. PC 可以发现映射成 U 盘的 SD 卡，烧录时，在 “选择存储设备” 步骤中选择识别到的标准 U 盘，烧录步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

        <img
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/studio_select_storage_refresh.png"
            style={{ width: '100%', height: 'auto', align: 'center' }}
            />

### 使用 Rufus 工具


#### SD 卡单独烧录 {#rufus-sd-card-flash}

1. 将 SD 卡插入读卡器，将读卡器插入 PC。

2. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的 Micro SD 存储卡作为目标设备。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

3. 点击 “选择” 按钮，选择解压出来的 `rdk-x5-ubuntu22-preinstalled-desktop-3.3.3-arm64.img` 文件作为烧录镜像。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_select_image.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

4. 其他参数保持默认，点击 “开始” 按钮，等待烧录完成。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_start.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

5. 烧录完成后，关闭 Rufus 并取出存储卡。

#### SD 卡在板烧录

1. SD 卡插入开发板，USB Type-C 接到 PC 端，长按 Sleep 按键（位于耳机接口旁），开发板上电，等待 5s，开发板进入烧录模式。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/sleep_key.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的盘符作为目标设备，其余步骤与 [SD 卡单独烧录](#rufus-sd-card-flash) 一致，完成镜像烧录。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />
