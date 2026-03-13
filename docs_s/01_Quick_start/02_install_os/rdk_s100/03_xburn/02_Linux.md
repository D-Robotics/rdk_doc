---
sidebar_position: 2
---

# Linux 平台烧录步骤


## 硬件连接

使用 Type-C 数据线将 PC 的 USB 接口和开发板的 Type-C 接口相连接。


## 安装依赖


Ubuntu 平台用户可通过以下命令安装工具

```
sudo apt update
sudo apt install android-tools-adb android-tools-fastboot
sudo apt install dfu-util
```


## 系统烧录

### 全镜像烧录

#### DFU-Fastboot 烧录

:::warning 注意

- 目前**需要将 SW3 拨至 ↑ 位置**，使用板载 eMMC 来启动，暂时不支持从 M.2 NVMe 固态硬盘启动。
- windows pc 上 Xburn 工具 需要在[驱动安装](#驱动下载与安装)成功后才能使用，使用前请确保驱动安装成功。

:::

**使 RDK S100 进入 DFU 启动模式**

   1. 将 SW1 拨码至 ↑，关闭电源
   2. 将 SW2 拨码至 ↑，进入 Download 模式
   3. 将 SW1 拨码至 ▽，开启电源
   4. 如果`DOWNLOAD`灯亮，则进入 DFU 模式，否则按下`K1`复位系统。
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1.png)



**使用 Xburn 进行烧录**

设置方法如下：

   - 选择产品型号: `RDKS100`
   - 连接模式: `usb`, 下载模式: `DFU+Fastboot`
   - 介质存储: `emmc`, 固件类型: `secure`
   - 镜像所在目录：点击浏览选择固件所在 product 文件夹

      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-xburn-download_dfu.png)


   - 点击开始升级，设备上电并等待升级完成

      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S600-xburn-burn_progress.png)


- 升级完成后，关闭电源，将烧录开关向下拨动后(从 DFU 模式退出)，重新上电。



#### Fastboot 烧录

:::info 注意

- Fastboot 使用正常启动模式。

- 需保证系统 U-boot 正常启动并进入 Fastboot。

:::

**使 RDK S100 进入 Fastboot 模式**

可以通过两种方式进入 Fastboot

- 自动进入 Fastboot : 系统启动后自动生成 ADB 设备，Xburn 检测 ADB 设备并下发命令让板端进入 Fastboot
- 手动进入 Fastboot : 板端启动进入 uboot，输入 `fastboot 0` 进入 Fastboot


**使用 Xburn 进行 Fastboot 烧录**

设置方法如下：

   - 选择产品型号: `RDKS100`
   - 连接模式: `usb`, 下载模式: `Fastboot`
   - 介质存储: `emmc`, 类型: `secure`
   - 镜像所在目录：点击浏览选择固件所在 product 文件夹
   
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-xburn-download_fastboot.png)


   - 点击开始升级，设备进入 Fastboot 模式并等待升级完成
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S600-xburn-burn_progress.png)


- 升级完成后重新上电。

### 指定区域烧录

#### 烧录区域说明

RDK S100 支持通过 Xburn 烧录指定区域，支持的烧录区域如下

|     区域      |     存储介质     | <center> 固件内容 </center>|  <center> 镜像 </center>  |
| :-------------: | :--------------: | ----------  | -------------------------|
| miniboot_flash |  Norflash  | Norflash 上的基础启动镜像，包括 HSM/MCU0 等系统组件的镜像   | img_packages/disk/miniboot_flash.img |
| miniboot_emmc   |  eMMC  |  eMMC 上的基础启动镜像，包括BL31/Uboot等系统组件的镜像   | img_packages/disk/miniboot_emmc.img |
| emmc            |  eMMC  |  eMMC 完整镜像，已包含 miniboot_emmc  | img_packages/disk/emmc_disk.img  |


#### 使用 Xburn 指定区域烧录

以指定烧录 `miniboot_flash` 和 `miniboot_emmc` 为例

设置方法如下：

   - 选择产品型号: `RDKS100`
   - 连接模式: `usb`, 下载模式: `DFU+Fastboot`
   - 介质存储: `emmc`, 类型: `secure`
   - 镜像所在目录：点击浏览选择固件所在 product 文件夹
   - 高级配置: 勾选 `烧录指定区域`，勾选 `miniboot_flash` 和 `miniboot_emmc`
   
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-xburn-download_partition.png)


   - 点击开始升级，设备上电并等待升级完成


### 指定区域备份

#### 备份区域说明

RDK S100 支持通过 Xburn 备份指定区域，支持的备份区域如下

|     区域      |     存储介质     | <center> 固件内容 </center>|  <center> 备份镜像路径 </center>  |
| :-------------: | :--------------: | ----------  | -------------------------|
| miniboot_flash |  Norflash  | Norflash 完整镜像   | img_packages/disk/miniboot_flash_backup.img |
| emmc            |  eMMC  |  eMMC 完整镜像  | img_packages/disk/emmc_disk_backup.img  |


#### 使用 Xburn 指定区域备份

以指定烧录 `miniboot_flash` 为例

设置方法如下：

   - 选择产品型号: `RDKS100`
   - 连接模式: `usb`, 下载模式: `DFU+Fastboot`
   - 介质存储: `emmc`, 类型: `secure`
   - 镜像所在目录：点击浏览选择备份镜像所在 product 文件夹
   - 高级配置: 勾选 `备份指定区域`，勾选 `miniboot_flash`


      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-xburn-backup_partition.png)


   - 点击开始升级，设备上电并等待操作完成

   - 操作完成后，打开 `img_packages/disk/`，查看备份镜像文件 `miniboot_flash_backup.img`
      ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-S100-xburn-backup_partition_image.png)


   :::warning 注意

   对于整个存储介质数据备份，耗时较长，请耐心等待备份结束。

   :::


### 启动系统

首先保持开发板断电，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

- **<font color='Green'>绿色</font>** 指示灯：点亮代表硬件上电正常



:::

如果开发板上电后长时间没有显示输出（2 分钟以上），说明开发板启动异常。需要通过串口线进行调试，查看开发板是否正常。

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display_s100.jpg)



