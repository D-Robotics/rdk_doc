---
sidebar_position: 1
---

# Mac 平台

## 安装和启动 XBurn

1. 双击安装包 `XBurn-gui_1.1.5_x64-setup.exe` ，随即会出现一个弹窗，单击并按住 `XBurn-gui` 图标拖到 `Applications` 图标中：

        ![XBurn_mac_install_1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/XBurn_mac_install_1.png)

2. `XBurn-gui` 便装完成，此时可以双击 `XBurn-gui` 的图标来打开该程序。
    :::warning 注意

    安装过程中若遇到提示缺少一些依赖，需要先安装相应的依赖。

    :::

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