---
sidebar_position: 4
---

# 1.2.1.4 使用 Rufus 工具

## 硬件连接

将 SD 卡插入读卡器，将读卡器插入 PC 相应接口。

## 系统烧录

1. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的 Micro SD 存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击 “选择” 按钮，选择解压出来的 `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img` 文件作为烧录镜像。

    ![image-rufus-select-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus-x3-install.png)

3. 其他参数保持默认，点击 “开始” 按钮，等待烧录完成。

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x3_install_finish.png)

4. 烧录完成后，关闭 Rufus 并取出存储卡。

## 启动系统


首先保持开发板断电，然后将制作好的存储卡插入开发板的 Micro SD 卡槽，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

开发板上电：

* <font color='Red'>红灯</font>点亮：代表硬件上电正常
* <font color='Green'>绿灯</font>点亮：代表系统启动中，熄灭或闪烁代表系统启动完成

如果开发板上电后长时间没有显示输出（2分钟以上），说明开发板启动异常，可通过指示灯确认系统状态：

* <font color='Green'>绿灯</font>常亮：说明系统启动失败，可检查使用的电源适配器是否满足要求，可尝试重新制作系统镜像
* <font color='Green'>绿灯</font>熄灭或闪烁：说明系统启动成功，但显示服务启动失败，请确认连接的显示器符合支持列表规格

:::

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)