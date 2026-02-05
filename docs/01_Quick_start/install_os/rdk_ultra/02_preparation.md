---
sidebar_position: 2
---

# 1.2.5.2 烧录准备


## 镜像下载


1. 点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images/rdk_ultra/)。


        ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/ultra_os_image_download.png)


2. 进入所选版本目录，选择 “server” 版本镜像或 “desktop” 版本镜像，点击下载。
   
          :::info 镜像说明
        
            支持无桌面 server 系统 和 desktop 桌面图形化交互。
            - desktop：带有桌面的 Ubuntu 系统，可以外接屏幕、鼠标操作
            - server：无桌面的 Ubuntu 系统，可以通过串口、网络远程连接操作
            
          :::

          ![镜像版本选择界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/ultra_os_image_download1.png)


3. 下载完成后，解压出 Ubuntu 系统镜像文件，如`ubuntu-preinstalled-desktop-arm64-rdkultra.img`
  
## 烧录工具下载
RDK Ultra开发套件烧录Ubuntu系统时，需要使用D-Robotics hbupdate烧录工具。目前工具支持Windows、Linux两种版本，分别以 hbupdate_win64、 hbupdate_linux 开头。

[[点击此处]](https://archive.d-robotics.cc/downloads/hbupdate/) 进入下载页面，根据使用平台选择工具版本。

:::warning 注意事项

  - 解压工具压缩包，注意解压路径中不要包含**空格、中文、特殊字符**等内容。
  - 工具通过网口跟 RDK Ultra 通讯，为保证烧录速度，请确保**PC支持千兆网口，并采用直连方式**。
  - PC端网络需要提前配置为**静态IP方式**，具体如下：
    - IP：192.168.1.195
    - netmask：255.255.255.0
    - gateway：192.168.1.1
:::





