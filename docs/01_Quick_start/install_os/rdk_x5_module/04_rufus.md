---
sidebar_position: 4
---

# 1.2.4.4 使用 Rufus 工具

## SD 卡烧录

### 硬件连接

将 SD 卡插入读卡器，将读卡器插入 PC 相应接口。


### 系统烧录

1. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的 Micro SD 存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击 “选择” 按钮，选择解压出来的 `rdk-x5-ubuntu-preinstalled-desktop-3.0.1-arm64.img` 文件作为烧录镜像。

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. 其他参数保持默认，点击 “开始” 按钮，等待烧录完成。

    ![image-rufus-flash](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rufus_x5_install_finish.png)

4. 烧录完成后，关闭 Rufus 并取出存储卡。
   
## 在板烧录

### SD 卡在板

1. SD 卡插入开发板，USB Type-C 接到 PC 端，长按 Sleep 按键（接口23），开发板上电，等待 5s，开发板进入烧录模式，PC 可以发现映射成 U 盘的 SD 卡。
2. 依照[系统烧录](#系统烧录)一节，完成系统烧录。


### eMMC 烧录


1. USB Type-C 接到 PC 端，长按Sleep按键（接口23），开发板上电，等待5s，开发板进入烧录模式，核心板自带的 eMMC 会被映射成 U 盘。

2. 依照[系统烧录](#系统烧录)一节，完成系统烧录。

## 启动系统

**默认情况**

从 eMMC 启动系统

**配置启动模式**

通过srpi-config配置启动模式，详情参考[srpi-config Advanced Options Boot Order](../../../02_System_configuration/02_srpi-config.md#advanced-options)

Ubuntu Desktop 版本系统启动完成后，会通过 Display 传输接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)