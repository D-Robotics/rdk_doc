---
sidebar_position: 4
---

# 1.2.3.4 使用 Rufus 工具

## 设备连接

将 SD 卡插入读卡器，将读卡器插入 PC。

## 系统烧录

### SD 卡烧录

1. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的 Micro SD 存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击 “选择” 按钮，选择解压出来的 `rdk-x5-ubuntu-preinstalled-desktop-3.0.1-arm64.img` 文件作为烧录镜像。

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. 其他参数保持默认，点击 “开始” 按钮，等待烧录完成。

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x5_install_finish.png)

4. 烧录完成后，关闭 Rufus 并取出存储卡。

### SD 卡在板烧录

SD 卡插入开发板，USB  Type-C 接到PC端，长按 Sleep 按键（位于耳机接口旁），开发板上电，等待5s，开发板进入烧录模式。

PC 可以发现映射成 U 盘的 SD 卡，然后依照[系统烧录](#系统烧录)一节，完成系统烧录。

## 启动系统


首先保持开发板断电，然后将制作好的存储卡插入开发板的 Micro SD 卡槽，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续 45 秒左右，配置结束后会在显示器输出 Ubuntu 系统桌面。

:::tip 开发板指示灯说明

* <font color='Green'>绿灯</font>点亮：代表系统启动中，熄灭或闪烁代表系统启动完成
:::

如果开发板上电后长时间没有显示输出（2分钟以上），说明开发板启动异常，需要通过串口线进行调试，查看开发板是否正常。


Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)

