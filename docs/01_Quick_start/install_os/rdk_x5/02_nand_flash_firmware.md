---
sidebar_position: 2
---

# 1.2.3.2 Bootloader 固件烧录


Bootloader 固件烧录是指将启动相关的底层固件（如 Miniboot、U-Boot 等 Bootloader）写入开发板板载 NAND Flash 的过程。该固件负责设备上电后的初始化与启动流程，包括硬件初始化和引导加载操作系统，是系统能够正常启动的前提。一般在设备无法启动、启动固件损坏或需要升级底层引导程序时进行，用于决定设备“能否启动”。

:::warning 固件烧录说明

- RDK 最小系统存储于 NAND Flash 中，包含 Bootloader（Miniboot、U-Boot）等关键启动组件。
- 设备出厂时已预装与硬件匹配的最新 NAND 固件。
- 为确保兼容性与设备稳定性，严禁降级刷入旧版本固件，否则可能导致设备无法正常启动。
- 若您已遇到设备无法启动的情况，请重新烧录 NAND 固件。

:::

## 固件下载


### 下载地址

[[点击此处]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/) 下载最新的 `product_发布日期.zip`，具体文件名以所选版本目录内文件为准。

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/miniboot_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## 烧录工具下载

RDK X5 可借助 PC 端工具 **XBurn** 来完成 Bootloader 固件的烧录工作。

### XBurn

- 用于 Bootloader 固件烧录。
- 支持 Windows、Linux、Mac 系统


#### 下载地址

[[点击此处]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) 进入软件工具下载区获取镜像烧录工具 XBurn：
- Windows 系统下载 `XBurn-gui_版本号_x64-setup.exe`。
- Ubuntu 系统下载 `XBurn-gui_版本号_x64-setup.deb`。
- Mac 系统下载 `XBurn-gui_版本号_x64-setup.dmg`。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

#### 安装方法

**Windows 平台**

1. 双击安装包 “XBurn-gui_1.1.9_x64-setup.exe” 进行安装，点击 `next`。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_01.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

2. 选择 `Uninstall before installing/Do not uninstall`，然后点击 `Next`：

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_02.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

3. 选择卸载路径。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_03.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

4. 点击 `Browse` 选择安装路径。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_04.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

5. 点击 `Install` 开始安装。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_05.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

6. 安装完成后，出现 `Setup was completed successfully` 即说明安装成功，此时点击 `Next`。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_06.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

7. 勾选 `Run xburn-gui` 以及 `Create desktop shortcut`，然后点击 Finish 完成安装。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_07.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

8. XBurn-gui 界面随后会自动打开，后续可以直接双击桌面上的 “ XBurn-gui ” 图标即可启动软件。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_08.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

**Linux 平台**

1. 在安装包目录中的终端执行 `sudo dpkg -i XBurn-gui_1.1.9_amd64.deb` 命令即可等待完成安装，安装示例如下：

```bash
(base) hobot@hobot-ThinkPad-T14-Gen-1:~/tools$ sudo dpkg -i XBurn-gui_1.1.5_amd64.deb 
[sudo] hobot 的密码： 
正在选中未选择的软件包 XBurn-gui。
(正在读取数据库 ... 系统当前共安装有 494898 个文件和目录。)
准备解压 XBurn-gui_1.1.5_amd64.deb  ...
正在解压 XBurn-gui (1.1.5) ...
正在设置 XBurn-gui (1.1.5) ...
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

2. 执行 `sudo XBurn-gui` 命令或者在应用菜单中点击 XBurn-gui 图标（会弹出输入密码的提示），输入密码即可打开烧录工具界面。

**Mac 平台**

双击安装包 `XBurn-gui_1.1.5_x64-setup.dmg`，随即会出现一个弹窗，单击并按住 XBurn-gui 图标拖到 Applications 图标中，XBurn-gui 便安装完成，此时可以双击 XBurn-gui 的图标来打开该程序。

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_mac.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## 固件烧录步骤

### 检查驱动

首次使用，需要检查驱动是否安装正确。打开工具，切换到驱动界面，该界面用于查看和管理驱动程序的状态。

- **驱动名称**：列出已安装的驱动程序名称（例如 USB Driver (ADB, Fastboot, DFU) 和 USB to Serial Driver (CH341)）。
- **当前版本**：显示驱动程序的当前版本。
- **操作**：提供安装和卸载驱动程序的按钮。
- **扫描驱动**：提供扫描驱动程序的按钮，用于检测和安装新的驱动程序。如果提示驱动未安装，则需点击 “安装驱动” 按钮，根据提示安装驱动。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_check_driver.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

### 硬件连接

- 连接串口到 PC：Micro-USB
- 连接烧录口到 PC：USB Type-C
- 连接电源线：USB Type-C，使用支持 5V/5A 的电源适配器。

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_hardware_connect.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

### 固件烧录

1. 配置烧录参数。

    - **产品类型**：选择 X5
    - **连接类型**：选择 Serial+USB
    - **下载模式**：选择 `xmodem_fastboot`
    - **镜像文件目录**：请选择要烧录的镜像文件所在的目录。
    - **批量烧录数量**：设置同时烧录的设备数量。根据电脑性能、硬件连接类型的带宽等因素，合理设置烧录设备数量。建议最多同时烧录 8 台设备。
    - **串口**：选择 PC 识别到的串口。
    - **波特率**：RDK X5 选择 115200。

        <img 
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_parameter_config.png" 
            style={{ width: '100%', height: 'auto', align:'center'}}
        />

2. 点击开始升级，看到提示后插拔电源，如果插拔电源后，串口丢失，可以先不上电，看到提示后再给板卡上电。

        <img 
                src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_power.png" 
                style={{ width: '100%', height: 'auto', align:'center'}}
        />

3. 上电后开始升级。

        <img 
                src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_start.png" 
                style={{ width: '100%', height: 'auto', align:'center'}}
        />

4. 升级成功。

        <img 
                    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_success.png" 
                    style={{ width: '100%', height: 'auto', align:'center'}}
        />


