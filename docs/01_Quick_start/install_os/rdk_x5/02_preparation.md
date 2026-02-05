---
sidebar_position: 2
---

# 1.2.3.2 烧录准备

## 系统烧录准备

### 存储设备

- 准备至少 16GB 容量的 Micro SD 存储卡，以便满足 Ubuntu 系统、应用功能软件对存储空间的需求

- SD 读卡器


### 镜像下载


1. 点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/)，选择 RDK X5 镜像版本。


        ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download.png)


2. 进入所选版本目录，选择 “server” 版本镜像或 “desktop” 版本镜像，点击下载。
   
          :::info 镜像说明
        
            支持无桌面 server 系统 和 desktop 桌面图形化交互。
            
            - desktop：带有桌面的 Ubuntu 系统，可以外接屏幕、鼠标操作
            - server：无桌面的 Ubuntu 系统，可以通过串口、网络远程连接操作
            
          :::

          ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download1.png)


  3. 下载完成后，解压出 Ubuntu 系统镜像文件，如`rdk-x5-ubuntu22-preinstalled-desktop-3.0.1-arm64.img`

  
### 烧录工具下载


#### RDK  Studio 工具下载

[[点击此处]](https://developer.d-robotics.cc/rdkstudio) 进入下载地址，根据使用平台选择安装包版本，建议选用 User Installer。

![RDK Studio下载界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rdkStudio_download.png)

#### Rufus 工具下载

[[点击此处]](https://rufus.ie/) 进入官网，根据使用平台选择工具版本。

## NAND 固件烧录准备

### 固件下载

1. [[点击此处]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/)下载最新的 `product_发布日期.zip`。

        ![image-20251031-170821](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-170821.png)

2. 解压后得到 `product` 文件夹，作为后续烧录的镜像所在目录。

### 烧录工具下载

#### XBurn 工具下载

[[点击此处]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/)下载镜像烧录工具 XBurn ：

        ![image-20251031-194712](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-194712.png)
        
        - windows 系统下载 `XBurn-gui_版本号_x64-setup.exe`。
        
        - Ubuntu 系统下载 `XBurn-gui_版本号_x64-setup.deb`。
        
        - MAC 系统下载 `XBurn-gui_版本号_x64-setup.dmg`。
        




