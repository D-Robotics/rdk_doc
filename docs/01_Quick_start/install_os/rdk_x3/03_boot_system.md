---
sidebar_position: 3
---

# 1.2.1.3 启动系统

首先保持开发板断电，然后将制作好的存储卡插入开发板的 Micro SD 卡槽，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

开发板上电：

- <font color='Red'>红灯</font>点亮：代表硬件上电正常。
- <font color='Green'>绿灯</font>点亮：代表系统启动中，熄灭或闪烁代表系统启动完成。

如果开发板上电后长时间没有显示输出（2 分钟以上），说明开发板启动异常，可通过指示灯确认系统状态：

- <font color='Green'>绿灯</font>常亮：说明系统启动失败，可检查使用的电源适配器是否满足要求，可尝试重新制作系统镜像。
- <font color='Green'>绿灯</font>熄灭或闪烁：说明系统启动成功，但显示服务启动失败，请确认连接的显示器符合支持列表规格。

:::

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)
