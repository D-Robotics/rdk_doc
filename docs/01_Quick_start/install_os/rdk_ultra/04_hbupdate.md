---
sidebar_position: 4
---

# 1.2.5.3 使用 hbupdate 工具

## 硬件连接

1. 通过网线将RDK Ultra和PC机直连，并确保网络可以ping通。

2. 将功能控制接口（接口10）的`FC_REC`和`GND`信号短接。

        ![image-ultra-fc-rec](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-ultra-fc-rec.jpg)

## 系统烧录

1. 运行`hbupdate`主程序，打开下载工具并选择开发板型号为`RDK_ULTRA`，必选项。

        ![image-flash-system1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system1.jpg)

4. 点击`Browse`按钮选择将要烧录的镜像文件，必选项。

        ![image-flash-system2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system2.jpg)

5. 点击`Start`按钮开始刷机，根据提示信息确认操作无误后，点击`OK`按钮：

        ![image-flash-system3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-system-download3.jpg)

6. 当工具显示如下打印时，说明进入烧写过程，该过程耗时依赖于网络传输速度，请耐心等待。

        ![image-flash-system4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system4.jpg)

7. 等待工具烧录完成，并检查烧录结果：

   - 镜像烧录成功时，工具提示如下：

        ![image-flash-system6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system6.png)

   - 工具提示如下错误时，请确认步骤1~3是否操作正确。

        ![image-flash-system7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system7.png)

   - 工具提示如下错误时，说明网络传输速度过慢，建议更换性能更好的PC后重新升级。
          ![image-flash-system8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system8.jpg)




    :::warning 注意
    
    如烧录过程发生中断，请按照上述步骤重新进行。
    :::

## 启动系统

首先保持开发板断电，去除功能控制接口（接口10）的FC_REC和GND信号短接线，通过HDMI线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续45秒左右，配置结束后会在显示器输出Ubuntu系统桌面。

Ubuntu Desktop版本系统启动完成后，会通过HDMI接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)