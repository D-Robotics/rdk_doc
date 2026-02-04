---
sidebar_position: 1
---

# Linux 平台


## 硬件连接

使用 Type-C 数据线将 PC 的 USB 接口和开发板的 Type-C 接口相连接。


## 系统烧录

### U-Boot 模式烧录

1. 启动 D-Navigation：
   
    xhost +
    sudo ./D-Navigation --no-sandbox

2. 选择产品型号：S100
   - 下载模式：uboot；介质存储 ufs；类型：secure
   - 点击浏览选择固件所在 product 文件夹
   - 选择与 RDK S600 连接的串口，波特率 921600
   - 点击开始升级(升级过程中，如有'Need manual reset'提示，请重新上电)

   ![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-download.png)

3. 待升级完成后重新上电


### USB 模式烧录

:::tip

SW1、SW2 等说明可查看[1.1.1 章节开关、按键和灯光说明内容](../../../../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#开关按键和灯光说明)
:::

1. 将设备切换到 DFU 模式，具体步骤：

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


## Miniboot 及文件系统升级

D-Navigation 工具支持对 S100 进行[Miniboot 镜像](/rdk_s/Advanced_development/rdk_gen#765-自定义分区说明)更新，在客户需要保留根文件系统修改（例如自行安装的 python/deb 包）时，可以在板端使用`sudo apt update && sudo apt upgrade`进行文件系统升级后，使用 D-Navigation 工具进行 Miniboot 镜像升级。

Miniboot 系统烧录整体流程与[整机系统烧录](#系统烧录)一致，需要额外配置：

1. 点击“其他配置”最右边的箭头；
2. 点击并选中“分区选择”；
3. 去掉勾选“emmc”；

- Uboot 烧录示例如下图：
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_uboot_miniboot.png)

- USB 烧录示例配置如下图：
  
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-s100-download_dfu_miniboot.png)

### 启动系统

首先保持开发板断电，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

- **<font color='Green'>绿色</font>** 指示灯：点亮代表硬件上电正常



:::

如果开发板上电后长时间没有显示输出（2 分钟以上），说明开发板启动异常。需要通过串口线进行调试，查看开发板是否正常。

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)


