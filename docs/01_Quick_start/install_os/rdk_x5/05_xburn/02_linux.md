---
sidebar_position: 1
---

# Linux 平台

## 安装和启动 XBurn

1. 在安装包目录中执行 `sudo dpkg -i XBurn-gui_1.1.5_amd64.deb` 命令即可等待完成安装，安装示例如下：

        ```shell
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
        
2. 执行 `sudo XBurn-gui` 命令或者在应用菜单中点击 `XBurn-gui` 图标（会弹出输入密码的提示），输入密码即可打开烧录工具界面。

## 检查驱动

首次使用，需要检查驱动是否安装正确。打开工具，切换到驱动界面，如果提示驱动未安装，则需点击 “安装驱动” 按钮，根据提示安装驱动。

![image-bf4b-43f0-8ae0-25f3c6ddbd3c](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-bf4b-43f0-8ae0-25f3c6ddbd3c.png)

该界面用于查看和管理驱动程序的状态。
- 驱动名称：列出已安装的驱动程序名称（例如 USB Driver (ADB, Fastboot, DFU) 和 USB to Serial Driver (CH341)）。
- 当前版本：显示驱动程序的当前版本。
- 操作：提供安装和卸载驱动程序的按钮。
- 扫描驱动：提供扫描驱动程序的按钮，用于检测和安装新的驱动程序。

## 硬件连接

连接串口到 PC：Micro-USB；

连接烧录口到 PC：USB Type C；

连接电源线：USB Type C,使用支持 5V/5A 的电源适配器；


## 固件烧录

![6443f0bb-da94-4a52-8abb-480bcea2bdd9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/6443f0bb-da94-4a52-8abb-480bcea2bdd9.png)

1. 产品类型： 选择 `X5` 
2. 连接类型： 选择 `Serial+USB`
3. 下载模式： 选择 `xmodem_fastboot`
4. 镜像文件目录：请选择要烧录的镜像文件所在的目录。
5. 批量烧录数量：设置同时烧录的设备数量。根据电脑性能、硬件连接类型的带宽等因素，合理设置烧录设备数量。建议最多同时烧录 8 台设备。
6. 波特率： `RDK X5` 选择`115200`，`RDK X5 Module` 选择 `921600`。

7. 点击开始升级，看到提示后插拔电源；

        如果插拔电源后，串口丢失，可以先不上电，看到提示后给板卡上电；
        
        ![d785a399-9e2e-40c5-a0c8-222a515f35f0](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/d785a399-9e2e-40c5-a0c8-222a515f35f0.png)
        
8. 开始升级

        ![267d637b-f67e-42a7-981f-2e45278bd877](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/267d637b-f67e-42a7-981f-2e45278bd877.png)

9. 升级结束

        ![078e4c6a-fca1-467b-bc93-c5a7ca73f8b7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/078e4c6a-fca1-467b-bc93-c5a7ca73f8b7.png)