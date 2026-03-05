---
sidebar_position: 2
---

# 2.2 srpi-config 工具配置

## 简介

`srpi-config`是一个系统配置工具，要打开配置工具，请在命令行中键入以下命令：

```
sudo srpi-config
```

> sudo 是提权管理命令，这里必须输入，这样就可以 root 权限来进行配置管理，默认的 sunrise 账号不具备系统文件修改权限。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-home_s100.png)


如果您使用的是桌面系统，那么您可以在菜单中找到`RDK Configuration`应用来进行配置。同样会打开如上图一样的配置终端。背景颜色的不同与开启终端的环境变量`TERM`相关。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/desktop_rdk_configuration_s100.png)

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-home_s100.png)

## System Options

系统选项菜单，允许对 Wi-Fi网络、用户密码、主机名、系统登录模式、浏览器选择等各个部分进行配置更改，以及一些系统级别的更改。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-system_s100.png)

- **Wireless LAN**

  设置无线网络的`SSID`和`密码`。

- **Password**

  更改“默认”用户密码，默认账号为 `sunrise`。

- **Hostname**

  设置当前设备在网络上的可见名称。

- **Boot / Auto login**

  选择是引导到控制台还是桌面，以及是否需要自动登录。如果选择自动登录，会使用系统默认账号 `sunrise` 的身份进行登录。

- **Power LED**

  如果 RDK 的型号允许，可以在这个选项中更改电源 LED 的行为。默认是熄灭或者闪烁。

- **Browser**

  如果使用的桌面系统，可以设置默认的浏览器。不配置的情况下默认使用 `firefox`， 用户可以通过命令 `sudo apt install chromium`安装`chromium`浏览器。

- **Update Miniboot**

  如果需要进行Miniboot相关分区的升级可在此选项中进行操作，具体升级的原理以及升级涉及的分区请参考：[miniboot升级](../07_Advanced_development/02_linux_development/06_OTA/02_ota_miniboot.md)。

## Interface Options

接口选项菜单，有以下选项可启用/禁用：SSH，外设等功能。

![image-20231123103322961](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-intf_s100.png)

- **SSH**

  使用 SSH 启用/禁用对`RDK`的远程登录。默认情况下系统是启用SSH选项的。

- **VNC**
  S100正在对VNC进行适配；

- 外设配置
  建议参考[config.txt 文件配置](./config_txt)进行外设的配置；


## Performance Options

性能选项，包括CPU运行模式和定频设置、调整ION内存大小等功能。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-performance_s100.png)

- **ION memory**

  可以通过此选项配置ION内存的常用大小。

> ION 内存是预留出来给BPU和图像、视频多媒体功能的物理内存空间。默认配置大小为672MB，如果需要运行比较大的算法模型、同时编解码多路视频时，请根据具体需要调整该内存大小。

## Localisation Options

本地化选项，为您提供以下选项供您选择：本地语言、时区、键盘布局。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-localisation_s100.png)

- **Locale**

  选择一个语言环境，例如配置中文环境`zh_CN.UTF-8`，重启生效。

- **Time Zone**

  选择您当地的时区，从地区开始，例如亚洲，然后选择一个城市，例如上海。键入一个字母以将列表向下跳到字母表中的该点。

- **Keyboard**

  读取所有键盘类型需要很长时间才能显示。更改通常会立即生效，但可能需要重新启动。

## Advanced Options

高级选项，可以对磁盘扩展、网络代理等选项进行设置。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-advanced_s100.png)

- **Expand Filesystem**

  此选项将扩展文件系统以填满整个储存介质（S100默认为eMMC），提供更多空间用于文件系统。

- **Network Proxy Settings**

  配置网络的代理设置。

## Update

将`srpi-config`工具更新到最新版本。

## About srpi-config

关于 `srpi-config` 的信息

选择此选项会显示以下信息：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-about_s100.png)

## Finish 按钮

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/srpi-config/srpi-config-gui-finish_s100.png)

完成更改后选中 `Finish` 按钮。`srpi-config`提供的系统配置，依赖重启生效的选项将会询问您是否要重新启动，如果不重新启动，配置将无法正常生效，请用户根据使用情况决定重启时机。
