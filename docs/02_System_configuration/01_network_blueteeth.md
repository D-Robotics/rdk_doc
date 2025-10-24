---
sidebar_position: 1
---

# 2.1 网络与蓝牙配置

本章节主要介绍开发板有线、无线网络配置的修改方法。

## 有线网络：RDK X5（>= 3.3.0）RDK X3（>= 3.0.2）

开发板有线网络默认采用静态IP配置，初始IP地址为`192.168.127.10`。用户可通过如下方法实现静态、DHCP模式的切换。

### [shell]修改静态IP配置
开发板静态网络配置保存在`/etc/NetworkManager/system-connections/netplan-eth0.nmconnection`文件中，通过修改`address1`字段，可完成对静态IP配置的修改，`route-metri`是网络优先级配置，设置为`700`是为了让有线网络的优先级更低，当有线和无线网络同时使能时优先会使用无线网络。

```shell
sudo vim /etc/NetworkManager/system-connections/netplan-eth0.nmconnection
```

```shell
[connection]
id=netplan-eth0
uuid=f6f8b5a7-9e23-49b2-a792-dc589b3d3e88
type=ethernet
interface-name=eth0
timestamp=1754294545

[ethernet]
wake-on-lan=0

[ipv4]
address1=192.168.127.10/24,192.168.127.1
dns=8.8.8.8;8.8.4.4;
method=manual
route-metric=700

[ipv6]
addr-gen-mode=eui64
method=ignore

[proxy]
```

修改完成后，命令行输入`sudo restart_network`命令让配置生效。

### [shell]修改DHCP配置

修改`[ipv4]`字段，只保留`method=auto`和`route-metric=700`

```shell
[ipv4]
method=auto
route-metric=700
```

修改完成后，命令行输入sudo restart_network命令让配置生效。

### [shell]修改MAC地址配置

修改`[ethernet]`字段，添加`cloned-mac-address=12:34:56:78:9A:BA`

```shell
[ethernet]
cloned-mac-address=12:34:56:78:9A:BA
wake-on-lan=0
```

修改完成后，reboot重启让配置生效。

### [桌面]修改静态IP配置

![image-edid](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-edid.png)

![image-edid2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-edid2.png)

![image-setip](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-setip.png)

### [桌面]修改DHCP配置

![image-dhcp](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-dhcp.png)

### [桌面]修改MAC地址配置

![image-mac](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-mac.png)

### [桌面]配置生效

点选`netplan-eth0`，配置生效。

![image-enable](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-enable.png)

## 有线网络：RDK X5（< 3.3.0）RDK X3（< 3.0.2）{#config_ethnet}

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=11

开发板有线网络默认采用静态IP配置，初始IP地址为`192.168.127.10`。用户可通过如下方法实现静态、DHCP模式的切换。

### 修改静态IP配置 
开发板静态网络配置保存在`/etc/network/interfaces`文件中，通过修改`address`、`netmask`、`gateway`等字段，可完成对静态IP配置的修改，`metric`是网络优先级配置，设置为`700`是为了让有线网络的优先级更低，当有线和无线网络同时使能时优先会使用无线网络，例如：

```shell
sudo vim /etc/network/interfaces
```

```shell
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
auto eth0
iface eth0 inet static
    pre-up /etc/set_mac_address.sh
    address 192.168.127.10
    netmask 255.255.255.0
    gateway 192.168.127.1
    metric 700
```

修改完成后，命令行输入`sudo restart_network`命令让配置生效。

### 修改DHCP配置
DHCP(Dynamic Host Configuration Protocol，动态主机配置协议)通常被应用在局域网络环境中，主要作用是集中的管理、分配IP地址，使网络环境中的主机动态的获得IP地址、Gateway地址、DNS服务器地址等信息，并能够提升地址的使用率。

开发板的DHCP网络配置保存在`/etc/network/interfaces`文件，通过修改eth0相关配置，可完成对DHCP模式的修改，例如：

```shell
sudo vim /etc/network/interfaces
```

```shell
source-directory /etc/network/interfaces.d
auto lo
iface lo inet loopback
auto eth0
iface eth0 inet dhcp
    metric 700
```

修改完成后，命令行输入`sudo restart_network`命令让配置生效。

### 修改MAC地址配置
如需修改开发板默认MAC地址，可通过在`/etc/network/interfaces`文件中增加`pre-up`配置信息，指定用户需要的MAC地址，例如：

```shell
sudo vim /etc/network/interfaces
```

```shell
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
auto eth0
iface eth0 inet static
    pre-up /etc/set_mac_address.sh
    address 192.168.127.10
    netmask 255.255.255.0
    gateway 192.168.127.1
    metric 700
    pre-up ifconfig eth0 hw ether 00:11:22:9f:51:27
```

修改完成后，`reboot`重启让配置生效。

## 无线网络

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=12

开发板集成了2.4GHz无线WiFi模块，支持Soft AP和Station两种模式，默认运行在Station模式下。下面介绍两种模式的使用方法。

### Station模式
Station模式下，开发板作为客户端，接入路由器无线热点进行联网。

- 对于使用Ubuntu Desktop版本系统的用户，可点击桌面右上角Wi-Fi图标，选择对应热点并输入密码以完成网络配置，如下图：  
![image-wifi-config](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-wifi-config.jpeg)



- 对于使用Ubuntu Server版本系统的用户，可通过命令行完成无线网络配置，步骤如下：

1. 使用`sudo nmcli device wifi rescan`命令扫描热点。如返回如下信息，说明扫描过于频繁，需要稍后再试
    ```shell
    root@ubuntu:~# sudo nmcli device wifi rescan
    Error: Scanning not allowed immediately following previous scan.
    ```
2. 使用`sudo nmcli device wifi list`命令列出扫描到的热点
3. 使用 `sudo wifi_connect "SSID" "PASSWD"`命令连接热点，返回如下信息，说明网络连接成功
    ```shell
    root@ubuntu:~# sudo wifi_connect "WiFi-Test" "12345678" 
    Device 'wlan0' successfully activated with 'd7468833-4195-45aa-aa33-3d43da86e1a7'.
    ```
    :::tip
    如果连接热点后，返回如下信息，说明热点没有找到，可以执行`sudo nmcli device wifi rescan`命令重新扫描后再次连接
    
    ```shell
    root@ubuntu:~# sudo wifi_connect "WiFi-Test" "12345678" 
    Error: No network with SSID 'WiFi-Test' found.
    ```
    :::

### Soft AP模式

开发板无线网络默认运行在Station模式下，如需使用Soft AP模式，请按照以下步骤进行配置。

1. 安装`hostapd` 和 `isc-dhcp-server`

    ```shell
    sudo apt update
    sudo apt install hostapd
    sudo apt install isc-dhcp-server
    ```

2. 运行 `sudo vim /etc/hostapd.conf`命令来配置`hostapd.conf`，主要关注下面几个字段:

    ```shell
    interface=wlan0 #作为AP热点的网卡
    ssid=Sunrise #WiFi名字
    wpa=2 #0为WPA 2为WPA2 一般为2
    wpa_key_mgmt=WPA-PSK #加密算法 一般为WPA-PSK
    wpa_passphrase=12345678 #密码
    wpa_pairwise=CCMP #加密协议，一般为CCMP
    ```

      - 无密码的热点配置，请在`hostapd.conf`文件添加以下内容：

    ```shell
    interface=wlan0
    driver=nl80211
    ctrl_interface=/var/run/hostapd
    ssid=Sunrise
    channel=6
    ieee80211n=1
    hw_mode=g
    ignore_broadcast_ssid=0
    ```

      - 有密码的热点配置，请在`hostapd.conf`文件添加以下内容：

    ```shell
    interface=wlan0
    driver=nl80211
    ctrl_interface=/var/run/hostapd
    ssid=Sunrise
    channel=6
    ieee80211n=1
    hw_mode=g
    ignore_broadcast_ssid=0
    wpa=2
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=CCMP
    wpa_passphrase=12345678
    ```

      - RDK X5 可以配置5G的热点，请在`hostapd.conf`文件修改`hw_mode`和`channel`字段：

    ```shell
    channel=36
    hw_mode=a
    ```

3. 配置`isc-dhcp-server`文件，步骤如下：

    - 执行 `sudo vim /etc/default/isc-dhcp-server`修改`isc-dhcp-server`文件，添加如下定义的网络接口：
    ```shell
    INTERFACESv4="wlan0"
    ```
    -  执行 `sudo vim /etc/dhcp/dhcpd.conf`修改`dhcpd.conf`文件， 取消以下字段的注释：
    ```shell
      authoritative;
    ```
    - 然后在 `/etc/dhcp/dhcpd.conf`文件末尾增加以下配置：
    ```shell
      subnet 10.5.5.0 netmask 255.255.255.0 { #网段和子网掩码
      range 10.5.5.100 10.5.5.254;#可获取的IP范围
      option subnet-mask 255.255.255.0; #子网掩码
      option routers 10.5.5.1;#默认网关
      option broadcast-address 10.5.5.31;#广播地址
      default-lease-time 600;#默认租约期限，单位秒
      max-lease-time 7200;#最长租约期限，单位秒
    }
    ```

4. 停止 `wpa_supplicant` 服务，并重启 `wlan0`

    ```bash
    systemctl mask wpa_supplicant
    systemctl stop wpa_supplicant

    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up
    ```

5. 按如下步骤启动 `hostapd`服务
   - 执行`sudo hostapd -B /etc/hostapd.conf`命令
   ```bash
    root@ubuntu:~# sudo hostapd -B /etc/hostapd.conf
   
    Configuration file: /etc/hostapd.conf
    Using interface wlan0 with hwaddr 08:e9:f6:af:18:26 and ssid "sunrise"
    wlan0: interface state UNINITIALIZED->ENABLED
    wlan0: AP-ENABLED
   ```
   - 通过`ifconfig`命令，配置无线接口`wlan0`的IP和网段，注意要跟第三步的配置保持一致
    ```bash
    sudo ifconfig wlan0 10.5.5.1 netmask 255.255.255.0
    ```
   - 最后开启`dhcp`服务器，连上热点会从`10.5.5.100`到`10.5.5.255`之间分配一个ip地址给客户端
    ```bash
    sudo ifconfig wlan0 10.5.5.1 netmask 255.255.255.0
    sudo systemctl start isc-dhcp-server
    sudo systemctl enable isc-dhcp-server
    ```

6. 连接开发板热点，例如 `sunrise`   
![image-20220601203025803](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-20220601203025803.png)  

7. 如需切换回`Station`模式，可按如下方式进行：

    [RDK X5]

    ```bash
    # 停止 hostapd
    killall -9 hostapd
    
    # 清除 wlan0 的地址
    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up
    
    # 重启 wpa_supplicant
    systemctl unmask wpa_supplicant
    systemctl restart wpa_supplicant

    #重装wifi驱动
    rmmod aic8800_fdrv 
    modprobe aic8800_fdrv

    # 连接热点,，具体操作可以查看上一章节 “无线网络”
    wifi_connect "WiFi-Test" "12345678"
    ```

    [Other]

    ```bash
    # 停止 hostapd
    killall -9 hostapd
    
    # 清除 wlan0 的地址
    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up
    
    # 重启 wpa_supplicant
    systemctl unmask wpa_supplicant
    systemctl restart wpa_supplicant
    
    # 连接热点,，具体操作可以查看上一章节 “无线网络”
    wifi_connect "WiFi-Test" "12345678"
    ```

### Soft AP模式（NetworkManager）：RDK X5（>= 3.3.0）RDK X3（>= 3.0.2）

新版本系统也可以使用NetworkManager来建立您的wifi热点。

点击桌面右上角的无线网络图标，选择`Edit Connections...`

![image-wifi1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-wifi1.png)

点击左下角+号，Connection Type选择`Wi-Fi`

![image-wifi2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-wifi2.png)

`Wi-Fi`标签下，填写SSID Mode Band

SSID填写您希望的热点名称

Mode选择`Hotspot`

Band 可以选择`Automatic`,`A(5 GHz)`或 `B/G（2.4 GHz)`

![image-wifi3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-wifi3.png)

`Wi-Fi Security`标签下，选择加密方式，填写密码

![image-wifi4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-wifi4.png)

重启板卡，或者`restart_network`让配置生效

## DNS服务

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=13

DNS(Domain Name Server)是进行域名(domain name)和与之相对应的IP地址转换的服务器。

开发板DNS配置通过`/etc/systemd/resolved.conf`文件管理，用户可通过修改该文件完成DNS相关配置，步骤如下：
1. 修改`resolved.conf`文件，添加DNS服务器地址，例如：

    ```bash
    DNS=8.8.8.8 114.114.114.114
    ```

2. 通过如下命令，使能DNS配置：

    ```bash
    sudo systemctl restart systemd-resolved
    sudo systemctl enable systemd-resolved
    sudo mv /etc/resolv.conf  /etc/resolv.conf.bak
    sudo ln -s /run/systemd/resolve/resolv.conf /etc/
    ```

##  系统更新
出于系统安全、稳定性的考虑，推荐用户安装完系统后，通过`apt`命令对系统进行更新。

在`/etc/apt/source.list`文件中，保存了`apt`命令的软件源列表，在安装软件前，需要先通过`apt`命令更新package列表。

首先打开终端命令行，输入如下命令：
```bash
sudo apt update
```
其次，升级所有已安装的软件包到最新版本，命令如下：
```bash
sudo apt full-upgrade
```

:::tip
推荐使用`full-upgrade`而不是`upgrade`选项，这样当相关依赖发生变动时，也会同步更新依赖包。

当运行`sudo apt full-upgrade`命令时，系统会提示数据下载和磁盘占用大小，但是`apt`不会检查磁盘空间是否充足，建议用户通过`df -h`命令手动检查。此外，升级过程中下载的deb文件会保存在`/var/cache/apt/archives`目录中，用户可以通过`sudo apt clean`命令删除缓存文件以释放磁盘空间。
:::

执行`apt full-upgrade`命令后，可能会重新安装驱动、内核文件和部分系统软件，建议用户手动重启设备使更新生效，命令如下：

```bash
sudo reboot
```

## 蓝牙配置

:::info 注意
从 3.0.0 系统开始，蓝牙默认随系统启动，无需手动重复初始化。在 RDK X3 和 RDK X5 平台上均保持一致。
:::

### 初始化

如果开发板蓝牙功能默认没有开启，`hciconfig`查看不到设备，可以执行 `/usr/bin/startbt.sh`脚本进行初始化，该脚本完成以下工作：

- 完成蓝牙初始化
- 执行 `hciconfig hci0 up`  完成蓝牙的Link Up
- 执行 `hciconfig hci0 piscan` 进行蓝牙扫描

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=9

脚本执行成功后的log如下：

![image-20220601172145987](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172145987.png)

此外，用户可以使用命令查询蓝牙进程是否正常，命令如下：

```bash
ps ax | grep "/usr/bin/dbus-daemon\|/usr/lib/bluetooth/bluetoothd"
/usr/bin/dbus-daemon

/usr/lib/bluetooth/bluetoothd
```

### 通信接口

为充分发挥开发板的扩展能力，当前硬件设计已集成多样化的接口与外设资源。

受限于接口布局及硬件资源分配，开发板未能完整复现蓝牙模组的所有通信接口。

目前仅提供 `BT_RX` `BT_TX` 双线模式，可满足无实时性要求的 AT 指令交互与数据传输等基础功能。

基于 UART 接口的蓝牙模组，不同接口连接方式及对应功能如下：

- ‌基础通信模式（UART Only）‌
- - 接口引脚：`BT_RX` `BT_TX`
- - 功能特性：基于UART的异步串行数据通信（如AT指令交互、低速率数据传输），无流控机制，在波特率超载或持续大数据量传输时，存在数据包丢失及缓冲区溢出风险。
- ‌增强型传输模式（增加硬件流控）‌
- - 接口引脚：`BT_RX` `BT_TX` `BT_CTS` `BT_RTS`
- - 功能特性：可有效避免数据包丢失及缓冲区溢出风险，支持A2DP高保真单向音频流传输。
- 语音通信模式（PCM同步接口）
- - 接口引脚：`PCM_SYNC` `PCM_DIN` `PCM_CLK` `PCM_DOUT`
- - 功能特性：支持基于SCO链路的实时双向音频传输，比如HPF/HSP

### USB蓝牙

如需深入使用蓝牙功能，例如在 `SPP（蓝牙虚拟串口）` 模式下实现高速稳定通信、在 `PAN（蓝牙虚拟网卡）` 模式下保证带宽质量，或在 `A2DP（高保真单向音频流）` 模式下避免音频中断，建议优先使用 `USB 接口蓝牙模块`。

开发板已集成 `USB2.0-BT`、`CSR8510 A10` 等常见蓝牙驱动，可直接支持大部分免固件的 USB 蓝牙模组。

![image-USB2.0-BT](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-USB2.0-BT.png)

![image-CSR8510](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-CSR8510.png)

![image-hci1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-hci1.png)

对于 `Realtek` 系列蓝牙模组，则需要额外的`固件`支持。请向模组厂家获取适配 Linux 平台 的固件，并将其放置到指定目录后，方可正常使用。

### 配网连接

执行`sudo bluetoothctl`进入交互模式下的蓝牙配置界面，出现了类似下图的设备信息表示蓝牙被识别到了，然后用`show`来查看蓝牙信息，留意蓝牙的`powered`和`discoverable`状态。

![image-20220601172604051](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172604051.png)

执行 `power on` 使能蓝牙，如下图所示：

![image-20220601172501882](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172501882.png)

为了能够使蓝牙被附近的设备发现，需要执行`discoverable on`使能蓝牙并打开蓝牙可发现属性，如下图所示：

![image-20220601172648853](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172648853.png)

此时使用手机或者电脑扫描蓝牙就可以发现 `ubuntu` 这个名称的蓝牙设备：

![image-20220601175322650](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601175322650.png)

接下来测试蓝牙的主动扫描功能，在`bluetoothctl`的交互界面输入`scan on`即可打开主动扫描，它会周期性地打印附近的设备，可以看到已经发现了我的手机设备，`scan off`关闭扫描功能并汇总打印扫描到的蓝牙设备：

![image-20220601154131158](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154131158.png)

![image-20220601154253947](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154253947.png)

然后就是和其他蓝牙的配对：

- 配对命令：`pair [targetMAC] `，输入该命令后，根据提示输入`yes`，对端蓝牙设备选择`配对`选项完成配对。

- 配对成功后可以使用`trust [targetMAC]`来让下次自动连接

![image-20220601154414717](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154414717.png)

经过以上操作后，蓝牙的扫描、配对的基本功能就完成了，如需使用更多功能，可查阅 `BlueZ`的官方帮助说明。
