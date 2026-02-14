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
<TabItem value="x5" label="RDK X5">

RDK X5开发板通过USB Type C接口供电，需要使用支持**5V/5A**的电源适配器为开发板供电。

更多供电方式参见[PoE 供电使用](https://developer.d-robotics.cc/rdk_doc/Advanced_development/hardware_development/rdk_x5/POE?_highlight=poe)。

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
- 准备至少 16GB 容量的Micro SD卡
- SD 读卡器
- 下载地瓜提供的烧录工具 RDK Studio（可[点击此处下载](https://developer.d-robotics.cc/rdkstudio)）或镜像烧录工具 Rufus（可[点击此处下载](https://rufus.ie/)）
:::

<Tabs groupId="flashing-type">

<TabItem value="RDK Studio" label="RDK Studio 工具">

使用 RDK Studio 工具烧录系统后可以添加设备进行管理，建议使用 RDK Studio 工具，详细步骤参见 [使用 RDK Studio 烧录系统](../09_RDK_Studio/04_flashing.md)。


</TabItem>
<TabItem value="Rufus" label="Rufus 工具">

Rufus是一款支持Windows平台的启动盘制作工具，使用Rufus制作SD启动卡流程如下：
1. 打开Rufus工具，在“设备”下拉框中选择对应的Micro SD存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击“选择”按钮，选择解压出来的`rdk-x5-ubuntu-preinstalled-desktop-arm64.img`文件作为烧录镜像。

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. 其他参数保持默认，点击“开始”按钮，等待烧录完成。烧录完成后，可以关闭Rufus并取出存储卡。

    ![image-rufus-flash](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-flash.png)

</TabItem>

<TabItem value="board" label="在板烧录">

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



![img-2025-1451](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/img-2025-1451.png)
</TabItem>
</Tabs>
</TabItem>


</Tabs>

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

##  NAND 固件烧录

RDK 最小系统存储于 `NAND Flash` 中，包含 `Bootloader（Miniboot、U-Boot）` 等关键启动组件。

设备出厂时已预装与硬件匹配的最新 NAND 固件。

为确保兼容性与设备稳定性，严禁降级刷入旧版本固件，否则可能导致设备无法正常启动。

若您已遇到设备无法启动的情况，请按照以下步骤重新烧录 NAND 固件。

### 下载 NAND 固件

下载最新的 `product_发布日期.zip`，解压后得到 `product` 文件夹，作为后续烧录的镜像所在目录。

-下载地址：https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/

![image-20251031-170821](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-170821.png)

### 下载安装烧录工具

`XBurn`是官方提供的镜像烧录工具，可以用于烧录RDK X5 NAND固件。它提供了直观的图形界面，用户只需点击几步即可完成镜像烧录，非常便捷。

- 下载地址：https://archive.d-robotics.cc/downloads/software_tools/download_tools/

![image-20251031-194712](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-194712.png)

windows 系统下载 `XBurn-gui_版本号_x64-setup.exe`。

Ubuntu 系统下载 `XBurn-gui_版本号_x64-setup.deb`。

MAC 系统下载 `XBurn-gui_版本号_x64-setup.dmg`。

#### windows 系统下安装与启动 XBurn

1. 双击安装包 `xburn-gui_1.1.5_x64-setup.exe` 即可进行安装
2. 安装完成后，`XBurn-gui`界面随后会自动打开：

![image-202510311956](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-202510311956.png)

3. 后续可以直接双击桌面上的 `XBurn.exe` 图标即可启动软件。

#### Ubuntu 系统下安装与启动 XBurn

在安装包目录中执行 `sudo dpkg -i xburn-gui_1.1.5_amd64.deb` 命令即可等待完成安装，安装示例如下：

```shell
(base) hobot@hobot-ThinkPad-T14-Gen-1:~/tools$ sudo dpkg -i xburn-gui_1.1.5_amd64.deb 
[sudo] hobot 的密码： 
正在选中未选择的软件包 xburn-gui。
(正在读取数据库 ... 系统当前共安装有 494898 个文件和目录。)
准备解压 xburn-gui_1.1.5_amd64.deb  ...
正在解压 xburn-gui (1.1.5) ...
正在设置 xburn-gui (1.1.5) ...
Udev rules installed and activated
User nobody added to plugdev group
User hobot added to plugdev group
User snapd-range-524288-root added to plugdev group
User snap_daemon added to plugdev group
User xpj added to plugdev group
正在处理用于 mailcap (3.70+nmu1ubuntu1) 的触发器 ...
正在处理用于 gnome-menus (3.36.0-1ubuntu3) 的触发器 ...
正在处理用于 desktop-file-utils (0.26-1ubuntu3) 的触发器 ...
正在处理用于 hicolor-icon-theme (0.17-2) 的触发器 ...
```

然后执行 `sudo xburn-gui` 命令或者在应用菜单中点击 `xburn-gui` 图标（会弹出输入密码的提示），输入密码即可打开烧录工具界面。

#### MAC 系统下安装与启动 XBurn

双击安装包 `xburn-gui_1.1.5_universal.dmg` 即可进行安装，具体安装步骤如下：

1. 随即会出现一个弹窗，单击并按住 `xburn-gui` 图标拖到 `Applications` 图标中：

![XBurn_mac_install_1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/XBurn_mac_install_1.png)

2. `xburn-gui` 便装完成，此时可以双击 `xburn-gui` 的图标来打开该程序：

3. 安装过程中若遇到提示缺少一些依赖，需要先安装相应的依赖。

### 检查驱动

首次使用，需要检查驱动是否安装正确。打开工具，切换到驱动界面，如果提示驱动未安装，则需点击“安装驱动”按钮，根据提示安装驱动。

![image-bf4b-43f0-8ae0-25f3c6ddbd3c](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-bf4b-43f0-8ae0-25f3c6ddbd3c.png)

该界面用于查看和管理驱动程序的状态。
- 驱动名称：列出已安装的驱动程序名称（例如 USB Driver (ADB, Fastboot, DFU) 和 USB to Serial Driver (CH341)）。
- 当前版本：显示驱动程序的当前版本。
- 操作：提供安装和卸载驱动程序的按钮。
- 扫描驱动：提供扫描驱动程序的按钮，用于检测和安装新的驱动程序。

### 连接设备

连接串口到PC，Micro-USB；

连接烧录口到PC，USB Type C；

连接电源线，USB Type C,使用支持5V/5A的电源适配器；

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X5">

![1d9a837c-c3a9-400d-a74b-23ee20f5ec44](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/1d9a837c-c3a9-400d-a74b-23ee20f5ec44.png)

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

![image_2025-10-31_201701_994](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image_2025-10-31_201701_994.png)

</TabItem>
</Tabs>

### 烧录

![6443f0bb-da94-4a52-8abb-480bcea2bdd9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/6443f0bb-da94-4a52-8abb-480bcea2bdd9.png)

1. 产品类型： 选择 `X5` 
2. 连接类型： 选择 `Serial+USB`
3. 下载模式： 选择 `xmodem_fastboot`
4. 镜像文件目录：请选择要烧录的镜像文件所在的目录。
5. 批量烧录数量：设置同时烧录的设备数量。根据电脑性能、硬件连接类型的带宽等因素，合理设置烧录设备数量。建议最多同时烧录 8 台设备。
6. 波特率： `RDK X5` 选择`115200`，`RDK X5 Module` 选择 `921600`。

点击开始升级，看到提示后插拔电源；

如果插拔电源后，串口丢失，可以先不上电，看到提示后给板卡上电；

![d785a399-9e2e-40c5-a0c8-222a515f35f0](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/d785a399-9e2e-40c5-a0c8-222a515f35f0.png)

开始升级

![267d637b-f67e-42a7-981f-2e45278bd877](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/267d637b-f67e-42a7-981f-2e45278bd877.png)

升级结束

![078e4c6a-fca1-467b-bc93-c5a7ca73f8b7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/078e4c6a-fca1-467b-bc93-c5a7ca73f8b7.png)