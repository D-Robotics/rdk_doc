---
sidebar_position: 4
---

# 1.2.1 RDK S100

:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI 和网线之外的任何设备
- 选用正规品牌的电源适配器，否则会出现供电异常，导致系统异常断电的问题
- 建议使用板载 POWER ON/OFF 按键实现主板上下电，并在适配器断电状态下对 DC 头进行插拔。

:::

## 烧录准备

### **供电**

RDK S100 开发板通过 DC 接口供电，推荐使用套件中自带的电源适配器。

### **驱动安装**

RDK S100 开发板通过 USB Type-C 接口与 PC 连接，烧录系统镜像前需在 PC 上安装 fastboot 驱动程序。

驱动下载地址：[winusb 驱动程序](https://archive.d-robotics.cc/downloads/software_tools/winusb_drivers/)

Sunrise5 USB 驱动支持 bl1/bl2/bl33 DFU、fastboot 和 adb 模式，安装步骤如下：

1. 下载并解压 `sunrise5_winusb.zip`
2. 右键以管理员身份运行 `install_driver.bat`


### **存储**

RDK S100 采用 eMMC 作为系统启动介质。

### **显示**

RDKS100 开发板支持 HDMI 显示接口。通过对应的线缆将开发板与显示器相连接，可实现图形化桌面显示。

### **网络连接**

RDK S100 开发板支持以太网、Wi-Fi 两种网络接口，用户可通过任意接口实现网络连接功能。

### **驱动下载**

安装USB驱动
对于Windows操作系统，在使用adb和fastboot功能前，需要首先确认是否安装对应的驱动程序。

在开发板的uboot命令行下输入fastboot 0，让开发板进入fastboot模式：

```bash
Warning: eth1 (eth0) using random MAC address - 9a:07:de:92:a2:c5
eth0: eth1
system_slot: 0 adc_boardinfo: 6a84
strap_pin = 0x45bc0 bootinfo = 0x0 bootcount = 0x1
boot_block_device [1]
flash boot
success!
Hit any key to stop autoboot:  0
Hobot$
Hobot$
Hobot$ fastboot 0
```

此时，设备管理器会提示存在USB download gadget的未知设备。

未安装驱动时，设备管理器会提示存在USB download gadget的未知设备，如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-no.png)

usb驱动下载（可[点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/winusb_drivers/)）

下载sunrise5_winusb.zip压缩包，进行驱动安装，步骤如下：

1、解压 sunrise5_winusb.zip。

2、进入sunrise5_winusb，右键点击install_driver.bat，选择以管理员身份运行。

成功安装驱动后，设备管理器会显示Android Device设备，如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-usbdriver-ok.png)

## 系统烧录

RDK S100 套件目前提供 Ubuntu 22.04 系统镜像，可支持 Desktop 桌面图形化交互。

:::info 注意

**RDK S100**出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。
:::

### 镜像下载 {#img_download}

参考[1.6 资源汇总](../../01_Quick_start/download.md)章节。

### 系统烧录

RDK S100 开发套件可借助 PC 端工具 D-Navigation 来完成 Ubuntu 系统的烧录工作。当前，该烧录过程支持两种 USB 下载模式，用户可在烧录工具的 “下载选取” 界面里的 “下载模式” 选项处进行选择。这两种模式的具体区别如下：

- **U-Boot 烧录方式：** 此模式依赖 RDK S100 进入 U-Boot 的烧录模式（即 fastboot 模式），在日常的烧录场景中使用较为频繁，能满足大多数常规的系统烧录需求。
- **USB 烧录方式：** 该模式基于 DFU 协议，当 RDK S100 遇到无法进入 U-Boot 模式，或者系统损坏导致设备变砖等特殊情况时，使用此模式帮助恢复系统。

下面给出使用 PC 工具 D-Navigation 烧录的具体烧录步骤。

:::tip

在烧录 Ubuntu 系统镜像前，需要做如下准备：

- 准备一根 Type-C 数据线，数据线的一端与板子的 Type-C 接口相连接，另一端与 PC 相连接。
- 下载镜像烧录工具 D-Navigation（可[点击此处下载](https://archive.d-robotics.cc/downloads/software_tools/download_tools/)）,根据系统不同，启动地瓜芯片工具 D-Navigation 方式分为三种：

  - Windows 版本启动：

        双击打开D-Navigation.exe

  - Ubuntu 版本启动：

        xhost +
        sudo ./D-Navigation --no-sandbox

  - MacOS 版本启动(目前支持 M 芯片)：

            双击打开D-Navigation.exe

    :::

#### uboot 烧录

1. 准备 RDKS100 镜像包，结构如下所示

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)

2. 开发板上电

:::tip

U-Boot 方式需要占用串口，须保证串口没有被其它设备或应用占用。
:::

3. 打开地瓜芯片工具 D-Navigation，完成如下操作：

- 选择产品型号：S100
- 下载模式：uboot；介质存储 emmc；类型：secure
- 点击浏览选择固件所在 product 文件夹
- 选择与 RDK S100 连接的串口，波特率 921600
- 点击开始升级(升级过程中，如有'Need manual reset'提示，请重新上电)

![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download.png)

4. 待升级完成后重新上电

#### USB 烧录(空片烧录或烧挂重新烧录)

1. 准备 RDKS100 镜像包，结构如下所示

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)

2. 将设备切换到 DFU 模式，具体步骤：

   1. 将 SW1 拨码至 ↑，关闭电源
   2. 将 SW2 拨码至 ↑，进入 Download 模式
   3. 将 SW1 拨码至 ▽，开启电源
   4. 如果`DOWNLOAD`灯亮，则进入 DFU 模式，否则按下`K1`复位系统。
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1.png)

3. 打开地瓜芯片工具 D-Navigation，完成如下操作

- 选择产品型号：S100
- 下载模式：usb；介质存储 emmc；类型：secure
- 点击浏览选择固件所在 product 文件夹
- 设备断电重启，点击开始升级，等待升级完成

![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download_dfu.jpg)

4. 升级完成后，关闭电源，将烧录开关向下拨动后(从 DFU 模式退出)，重新上电。

### 启动系统

首先保持开发板断电，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

- **<font color='Green'>绿色</font>** 指示灯：点亮代表硬件上电正常

如果开发板上电后长时间没有显示输出（2 分钟以上），说明开发板启动异常。需要通过串口线进行调试，查看开发板是否正常。

:::

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)

## **常见问题**

:::tip

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节，同时可以访问 [D-Robotics 开发者官方论坛](https://developer.d-robotics.cc/forum) 获得帮助。

:::
