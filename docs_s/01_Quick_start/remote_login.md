---
sidebar_position: 4
---

# 1.4 远程登录

本章节旨在向需要通过个人电脑(PC)远程访问开发板的用户介绍如何通过串口、网络(SSH)方式进行远程登录。

:::tip
通过网络方式远程登录前，开发板需要通过有线以太网或者无线 WiFi 方式接入网络，配置好开发板 IP 地址。对于两种连接方式下的 IP 地址信息可参考如下描述：

- 有线以太网：
  - 开发板 eth1 接口默认采用静态 IP 模式，IP 地址为`192.168.127.10`，掩码`255.255.255.0`，网关 `192.168.127.1`
  - 开发板 eth0 接口默认采用dhcp模式，IP 地址一版由路由器分配，可在设备命令行中通过`ifconfig`命令查看 eth0 网络的 IP 地址
- 无线 WiFi：开发板 IP 地址一般由路由器分配，可在设备命令行中通过`ifconfig`命令查看 wlan0 网络的 IP 地址

:::

## 串口登录{#login_uart}

### **win连接串口**

参考视频: https://www.bilibili.com/video/BV1rm4y1E73q/?p=2

在使用串口登录前，需要确认开发板串口线跟电脑正确连接，连接方法可参考对应开发板的调试串口章节：

- [rdk_s100 调试串口章节](../01_Quick_start/01_hardware_introduction/01_rdk_s100.md#debug_uart)

串口登录需要借助 PC 终端工具，目前常用的工具有`Putty`、`MobaXterm`等，用户可根据自身使用习惯来选择。不同工具的端口配置流程基本类似，下面以`MobaXterm`为例，介绍新建串口连接过程：

- 当串口 USB 转接板首次插入电脑时，需要安装串口驱动。驱动程序可从资源中心的[工具子栏目](https://developer.d-robotics.cc/resource)获取。驱动安装完成后，设备管理器可正常识别串口板端口，如下图：

![image-20220416105939067](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-20220416105939067.png)

- 打开`MobaXterm`工具，点击`Session`，然后选择`Serial`

- 配置端口号，例如`COM3`，实际使用的串口号以 PC 识别到的串口号为准

- 设置串口配置参数，如下：

  | 配置项               | 参数值 |
  | -------------------- | ------ |
  | 波特率（Baud rate）  | 921600 |
  | 数据位（Data bits）  | 8      |
  | 奇偶校验（Parity）   | None   |
  | 停止位（Stop bits）  | 1      |
  | 流控（Flow Control） | 无     |

- 点击`OK`，输入用户名：`root`、密码：`root`登录设备
  ![image-Uart-Login](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-Uart-Login.gif)

此时，可使用`ifconfig -a`命令查询开发板 IP 地址，其中 eth0/eth1、wlan0 分别代表有线、无线网络：

```bash
eth0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether c8:30:76:63:2d:93  txqueuelen 1000  (Ethernet)
        RX packets 7547  bytes 2230733 (2.2 MB)
        RX errors 0  dropped 2  overruns 0  frame 0
        TX packets 1126  bytes 108615 (108.6 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 93

eth1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.127.10  netmask 255.255.255.0  broadcast 192.168.127.255
        inet6 fe80::e0b2:71ff:fea0:6ba7  prefixlen 64  scopeid 0x20<link>
        ether e2:b2:71:a0:6b:a7  txqueuelen 1000  (Ethernet)
        RX packets 43  bytes 3882 (3.8 KB)
        RX errors 0  dropped 1  overruns 0  frame 0
        TX packets 46  bytes 6234 (6.2 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 99

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 46  bytes 6342 (6.3 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 46  bytes 6342 (6.3 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 28:d0:43:83:63:57  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```
### **mac连接串口**

macos系统下，使用minicom工具连接串口，步骤如下：
1. 使用minicom命令连接串口验证(`minicom -D /dev/tty.wchusbserial* -b 921600 -8`）
      ```bash
      minicom  # 启动 minicom 终端工具，用于串口通信
      -D       # 指定要使用的串口设备（device）
      -b       # 设置串口波特率（baud rate）
      -8       # 设置 数据位数为 8 位（data bits）
      ```
      ![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom.png)
2. 连接开发板验证
   ![image-S100-download](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-mac-usb-driver-minicom-success.png)

:::tip

使用minicom连接出现乱码，请查看[使用macos系统笔记本串口乱码](../01_Quick_start/02_install_os/rdk_s100.md#使用macos系统笔记本串口乱码问题)
:::

## 网络状态确认{#network_config}

参考: https://www.bilibili.com/video/BV1rm4y1E73q/?p=3

在使用远程登录前，需要确保电脑、开发板网络通信正常，如无法`ping`通，需按如下步骤进行确认：

- 确认开发板、电脑 IP 地址配置，一般前三段需要是一样的，例如开发板：`192.168.127.10` 电脑：`192.168.127.100`
- 确认开发板、电脑的子网掩码、网关配置是否一致
- 确认电脑网络防火墙是否处于关闭状态

开发板靠外的有线以太网口（eth1）默认采用静态 IP 模式，IP 地址为`192.168.127.10`。对于开发板、电脑网络直连的情况，只需要将电脑配置为静态 IP，保证跟开发板处于同一网段即可。以 WIN10 系统为例，电脑静态 IP 修改方法如下：

- 在网络连接中找到对应的以太网设备并双击打开
- 找到 Internet 协议版本 4 选项并双击打开
- 在下图红框位置填入对应的网络参数，点击确定

![image-20220416110242445](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-s100-pc-static-ip.png)

如需将开发板有线网络配置为动态获取 DHCP 模式，可参考[有线网络](../02_System_configuration/01_network_bluetooth.md)章节进行配置。

## SSH 登录{#ssh}
下面分别介绍终端软件、终端命令行两种方法的创建步骤。

### 终端软件

目前常用终端工具有`Putty`、`MobaXterm`等，用户可根据自身使用习惯来选择。不同工具的端口配置流程基本类似，下面以`MobaXterm`为例，介绍新建 SSH 连接过程：

1. 打开`MobaXterm`工具，点击`Session`，然后选择`SSH`
2. 输入开发板 IP 地址，例如`192.168.127.10`
3. 选中`specify username`，输入`sunrise`
4. 点击 OK 后，输入用户名（sunrise）、密码（sunrise）即可完成登录

![image-Network-Login](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-Network-Login.gif)

### 电脑命令行

用户也可通过命令行方式进行 SSH 登录，步骤如下：

1. 打开终端窗口，输入 SSH 登录命令，例如`ssh sunrise@192.168.127.10`
2. 弹出连接确认提示，输入 YES
3. 输入密码（sunrise）即可完成登录

![image-Cmdline-Linux](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/linux_login_01.gif)


## NoMachine登陆

:::tip
NoMachine功能需要S100端的软件包支持，配置指南见[NoMachine配置](./03_configuration_wizard/configuration_wizard_s100.md#nomachine-配置)
:::

本章节面向使用Ubuntu Desktop系统版本的用户，介绍如何通过`NoMachine`实现远程桌面登录功能。

**连接开发板**

1. 打开`NoMachine`客户端，点击`Add`增加主机配置

![nomachine_login01](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-S100-nomachine_login01.jpg)

2. 在跳出来的界面中填写`RDK100`的主机信息，完成后点击`Add`

![nomachine_login02](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-S100-nomachine_login02.jpg)

3. 此时返回主界面，双击刚才生成的主机

![nomachine_login03](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-S100-nomachine_login03.jpg)

4. 弹出登录界面，输入用户名、密码点击OK即可完成远程登陆

![nomachine_login04](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-S100-nomachine_login04.jpg)

![nomachine_login05](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/remote_login/image-S100-nomachine_login05.jpg)
