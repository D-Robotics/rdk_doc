---
sidebar_position: 2
---

# 1.2.2.2 烧录准备

## 存储设备

- 准备至少 8GB 容量的存储卡，以便满足 Ubuntu 系统、应用功能软件对存储空间的需求

- SD 读卡器


## 镜像下载

:::warning 烧录提示

**RDK X3 Module**出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。
:::

1. 点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/)，选择 RDK X3 镜像版本。

        :::info 版本说明

        RDK X3 套件目前提供 Ubuntu 20.04/22.04 系统镜像。
        
        - 3.0 版本：<font color = 'red'>**补充：基于 RDK Linux 开源代码包制作，支持 RDK X3 派、X3 模组等全系列硬件**</font>
        - 2.0 版本：基于 RDK Linux 开源代码包制作，支持 RDK X3 派、X3 模组等全系列硬件
        - 1.0 版本：旭日 X3 派历史版本，仅支持旭日 X3 派硬件，系统镜像名为 `system_sdcard.img`
        
        :::

        ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download.png)


  2. 进入所选版本目录，选择 “server” 版本镜像或 “desktop” 版本镜像，点击下载。
   
          :::info 镜像说明
        
            支持无桌面 server 系统 和 desktop 桌面图形化交互。
            - desktop：带有桌面的 Ubuntu 系统，可以外接屏幕、鼠标操作
            - server：无桌面的 Ubuntu 系统，可以通过串口、网络远程连接操作
            
          :::

          ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download1.png)


  3. 下载完成后，解压出 Ubuntu 系统镜像文件，如`rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`

  
## 烧录工具下载

### RDK  Studio 工具下载

[[点击此处]](https://developer.d-robotics.cc/rdkstudio) 进入下载地址，根据使用平台选择安装包版本，建议选用 User Installer。

![RDK Studio下载界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rdkStudio_download.png)

### Rufus 工具下载

[[点击此处]](https://rufus.ie/) 进入官网，根据使用平台选择工具版本。





