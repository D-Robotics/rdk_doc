---
sidebar_position: 2
---

# 1.2.1.2 烧录准备


## 镜像下载

:::warning 注意

**RDK S100**出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。

:::

RDK S100 套件目前提供 Ubuntu 22.04 系统镜像，可支持 Desktop 桌面图形化交互。
1. [[点击此处]](../../download.md#系统镜像与工具)下载镜像。
   
    ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/download_web.png)

2. 解压后得到 product 文件夹，结构如下所示，确保同一个文件夹内有 `img_packages` 文件夹和 `xmodem_tools` 文件
    ![product文件夹界面](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)


## 烧录工具下载

[[点击此处]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) 进入烧录工具下载地址，RDK S100 开发套件可借助 PC 端工具 D-Navigation 完成 Ubuntu 系统的烧录工作，根据使用平台选择工具和版本:

以 D-navigation-v2.4  为例：
   
   - Windows 系统下载：D-navigation-win32-x64_v2.4.zip 
   - Linux 系统下载：D-navigation-linux-x64-v2.4.tar.gz 
   - Mac 系统下载：D-navigation-darwin-arm64_v2.4.zip



