---
sidebar_position: 2
---

# 1.2.1.2 Bootloader 固件烧录

Bootloader 固件烧录是指将启动相关的底层固件（如 Miniboot、U-Boot 等 Bootloader）写入开发板板载 NAND Flash 的过程。该固件负责设备上电后的初始化与启动流程，包括硬件初始化和引导加载操作系统，是系统能够正常启动的前提。一般在设备无法启动、启动固件损坏或需要升级底层引导程序时进行，用于决定设备“能否启动”。

:::warning 固件烧录说明

- RDK 最小系统存储于 NAND Flash 中，包含 Bootloader（Miniboot、U-Boot）等关键启动组件。
- 设备出厂时已预装与硬件匹配的最新 NAND 固件。
- 为确保兼容性与设备稳定性，严禁降级刷入旧版本固件，否则可能导致设备无法正常启动。
- 若您已遇到设备无法启动的情况，请重新烧录 NAND 固件。

:::

## 烧录工具

RDK X3 可借助 PC 端工具 **hbupdate** 来完成 Bootloader 固件的烧录工作。

### hbupdate

- 支持使用 “本地已有镜像”。
- 支持 Windows 系统。
- 支持 eMMC 烧录，采用 UMS 方式。

#### 下载地址

[[点击此处]](https://archive.d-robotics.cc/downloads/hbupdate/) 进入下载地址，根据使用平台、OS 和固件版本选择工具安装包的版本。

#### 安装方法

**Windows 平台**

1. 解压 `hbupdate` 压缩包。

    :::warning 注意

    解压工具压缩包时，解压路径中不要包含空格、中文、特殊字符等内容。

    :::


2. 双击 `.exe` 应用程序文件即可打开 hbupdate 工具。

  <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_exe.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

## 固件下载

[[点击此处]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x3/) 下载固件，进入下载目录后，根据产品容量选择对应的 `.img` 文件。

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/miniboot_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## 固件烧录步骤

### 驱动检查

1. 连接串口：将杜邦线接入开发板接口 3，串口 USB 转接板接入电脑。连接完成后，请确认接线正确。

  <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
  />

2. 首次使用串口需要下载安装 CH340 串口驱动，可 [[点击此处]](https://archive.d-robotics.cc/downloads/software_tools/serial_to_usb_drivers/) 下载。

3. 驱动安装完成后，设备管理器可正常识别串口板端口。

    <img 
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/usb_serial.png" 
            style={{ width: '100%', height: 'auto', align:'center'}}
    />

4. 下载远程连接工具 [MobaXterm](https://mobaxterm.mobatek.net/download.html)。
5. 打开 MobaXterm 工具，点击 `Session`，然后选择 `Serial`，配置端口号（例如 `COM9`，以 PC 实际识别到的串口号为准），设置完成后点击 `OK`。

  

    串口配置参数如下：

    | 配置项 | 参数值 |
    | ------ | ------ |
    | 波特率（Baud rate） | 921600 |
    | 数据位（Data bits） | 8 |
    | 奇偶校验（Parity） | None |
    | 停止位（Stop bits） | 1 |
    | 流控（Flow Control） | 无 |
    <img 
              src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/serial_parameter.png" 
              style={{ width: '100%', height: 'auto', align:'center'}}
      />

6. 开发板上电后立刻长按空格键，进入 U-Boot 命令行模式，输入 `fastboot 0`，让开发板进入 fastboot 模式。

    ```bash
    U-Boot 2022.10-gca2c6582a0 (Mar 13 2024 - 19:04:15 +0800)

    ... (省略) ...
    Hit any key to stop autoboot:  0
    Hobot>
    Hobot>fastboot 0
    ```

7. **未安装驱动时**，设备管理器会提示存在 `USB download gadget` 的未知设备。

    <img 
              src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/download_gedget.png" 
              style={{ width: '100%', height: 'auto', align:'center'}}
      />

### 驱动下载与安装 {#x3md-android-driver}

1. [[点击此处]](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) 下载并解压驱动包 `android_hobot.zip`。


2. 进入解压后的目录，以管理员身份运行 `5-runasadmin_register-CA-cer.cmd`，完成驱动程序的注册。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/usb_driver_administrator.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. 驱动安装完成后，设备管理器会显示 fastboot 设备 `Android Device`。

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/android_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />



### 烧录固件

运行 `hbupdate.exe` 打开烧录工具，并按照以下步骤进行烧录。

1. 选择开发板型号，必选项。

    - **RDK_X3_2GB**：RDK X3（旭日 X3 派），2GB 内存版本。
    - **RDK_X3_4GB**：RDK X3（旭日 X3 派），4GB 内存版本。
    - **RDK_X3_MD_2GB**：RDK X3 Module，2GB 内存版本。
    - **RDK_X3_MD_4GB**：RDK X3 Module，4GB 内存版本。

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_boardname.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. 点击 `Browse` 按钮选择将要烧录的镜像文件，必选项。

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_select_image.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

4. 点击 `Start` 按钮开始烧录：

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_start.png" 
      style={{ width: '80%', height: 'auto', align:'center'}}
    />

5. 烧录完毕断开电源，断开和电脑的连接线，重新上电即可。

### 检查升级结果

- 镜像烧录成功时，工具提示如下：

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_success.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

- 镜像烧录失败时，工具提示如下，此时需要确认 PC 设备管理器是否存在 `Android Device` 设备：

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_fail.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />
