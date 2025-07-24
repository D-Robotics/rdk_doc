---
sidebar_position: 2
---

# 1.2.2 RDK X5

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

在使用RDK X5开发板前，需要做下述准备工作。

## 烧录准备

### **供电**

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

RDK X5开发板通过USB Type C接口供电，需要使用支持**5V/5A**的电源适配器为开发板供电。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module通过载板上USB Type C接口供电，需要使用支持**5V/5A**的电源适配器为开发板供电。

</TabItem>
</Tabs>

:::caution

请不要使用电脑USB接口为开发板供电，否则会因供电不足造成开发板**异常断电、反复重启**等异常情况。

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节。

:::


### **存储** 

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

RDK X5开发板采用Micro SD存储卡作为系统启动介质，推荐至少16GB容量的存储卡，以便满足Ubuntu系统、应用功能软件对存储空间的需求。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module 核心板自带emmc，载版上有sd卡插槽，支持eMMC和SD卡两种模式启动系统。

优先从eMMC启动，可以通过srpi-config配置启动模式，详情参考[srpi-config Advanced Options Boot Order](../../System_configuration/srpi-config#advanced-options)

使用Micro SD存储卡作为系统启动介质时，推荐至少16GB容量的存储卡，以便满足Ubuntu系统、应用功能软件对存储空间的需求。

</TabItem>
</Tabs>


### **显示** 

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

RDK X5开发板支持HDMI显示接口，通过HDMI线缆连接开发板和显示器，支持图形化桌面显示。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module开发板支持HDMI显示接口，通过HDMI线缆连接开发板和显示器，支持图形化桌面显示。

</TabItem>
</Tabs>

### **网络连接**

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

RDK X5开发板支持以太网、Wi-Fi两种网络接口，用户可通过任意接口实现网络连接功能。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module开发板支持以太网、Wi-Fi两种网络接口，用户可通过任意接口实现网络连接功能。

</TabItem>
</Tabs>

## 系统烧录

RDK套件目前提供Ubuntu 22.04系统镜像，有desktop，server两个版本可供选择；

desktop：带有桌面的Ubuntu系统，可以外接屏幕、鼠标操作；

server：无桌面的Ubuntu系统，可以通过串口、网络远程连接操作。

### 镜像下载 {#img_download}

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images/rdk_x5)，进入版本选择页面，选择最新版本目录，进入系统下载页。

下载完成后，解压出Ubuntu系统镜像文件，如`ubuntu-preinstalled-desktop-arm64.img`

:::caution

1.使用balenaEtcher 验证过程可能会报错，但是可以正常进入系统；

2.官方十分建议使用rufus烧录软件进行镜像安装，相对较为稳定；

:::

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

:::info 注意

**RDK X5 Module**出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。
:::

点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images/rdk_x5)，进入版本选择页面，选择最新版本目录，进入系统下载页。

下载完成后，解压出Ubuntu系统镜像文件，如`ubuntu-preinstalled-desktop-arm64.img`

:::caution

1.使用balenaEtcher 验证过程可能会报错，但是可以正常进入系统；

2.官方十分建议使用rufus烧录软件进行镜像安装，相对较为稳定；

3.RDK X5 Module 只能使用3.2.0 及其后续版本的系统；

:::

</TabItem>
</Tabs>

### 系统烧录

:::tip

在烧录Ubuntu系统镜像前，需要做如下准备：
- 准备至少16GB容量的Micro SD卡
- SD 读卡器
- 下载镜像烧录工具Rufus（可[点击此处下载](https://rufus.ie/)）
:::

Rufus是一款支持Windows平台的启动盘制作工具，使用Rufus制作SD启动卡流程如下：
1. 打开Rufus工具，在“设备”下拉框中选择对应的Micro SD存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击“选择”按钮，选择解压出来的`rdk-x5-ubuntu-preinstalled-desktop-arm64.img`文件作为烧录镜像。

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. 其他参数保持默认，点击“开始”按钮，等待烧录完成。烧录完成后，可以关闭Rufus并取出存储卡。

    ![image-rufus-flash](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-flash.png)

### 在板烧录

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

SD卡插入开发板，usb2.0 type c接到PC端，长按Sleep按键（位于耳机接口旁），开发板上电，等待5s，开发板进入烧录模式。

PC可以发现映射成U盘的SD卡，然后依照系统烧录一节，完成系统烧录。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

SD卡插入开发板，usb2.0 type c接到PC端，长按Sleep按键（接口23），开发板上电，等待5s，开发板进入烧录模式。

PC可以发现映射成U盘的SD卡；

SD卡不插入开发板，核心板自带的emmc会被映射成U盘；

然后依照系统烧录一节，完成系统烧录。

</TabItem>
</Tabs>

![img-2025-1451](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/img-2025-1451.png)


### 启动系统


首先保持开发板断电，然后将制作好的存储卡插入开发板的Micro SD卡槽，并通过HDMI线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续45秒左右，配置结束后会在显示器输出Ubuntu系统桌面。



:::tip 开发板指示灯说明



* **<font color='Green'>绿色</font>** 指示灯：点亮代表硬件上电正常



如果开发板上电后长时间没有显示输出（2分钟以上），说明开发板启动异常。需要通过串口线进行调试，查看开发板是否正常。

:::



Ubuntu Desktop 版本系统启动完成后，会通过 HDMI 接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)

## **常见问题**  

首次使用开发板时的常见问题如下：

- **<font color='Blue'>上电不开机</font>** ：请确保使用[供电](#供电)中推荐的适配器供电；请确保开发板的Micro SD卡已经烧录过Ubuntu系统镜像
- **<font color='Blue'>使用中热插拔存储卡</font>** ：开发板不支持热插拔Micro SD存储卡，如发生误操作请重启开发板



### **注意事项**

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备
- RDK X5 的 Type C USB 接口仅用作供电 
- 选用正规品牌的USB Type C 口供电线，否则会出现供电异常，导致系统异常断电的问题



:::tip

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节，同时可以访问 [D-Robotics 开发者官方论坛](https://developer.d-robotics.cc/forum) 获得帮助。

:::