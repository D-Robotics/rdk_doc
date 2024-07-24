---
sidebar_position: 1
---

# 1.1 环境准备

TogetheROS.Bot支持在地平线RDK和X86平台的Ubuntu 20.04/Ubuntu 22.04系统上安装。使用Ubuntu系统通过DEB包安装的方式简单快捷，建议初期体验的用户尽量采用该方式进行安装。

接下来分别介绍地平线RDK和X86平台环境准备详情。

## 地平线RDK平台

### 系统安装

安装tros.b之前，建议用户将地平线RDK系统镜像升级到最新版本，Ubuntu 20.04/Ubuntu 22.04镜像烧录方法：[Ubuntu镜像烧录方法](https://developer.horizon.cc/documents_rdk/installation/install_os)

如果已经安装镜像，可以通过命令`sudo apt update`和`sudo apt upgrade`完成升级。

:::caution **注意**
- **如果您安装的是1.x版本系统，需要将系统升级到2.x版本。**
- **系统版本号查看方法以及详细说明，请查看[FAQs](../FAQs/hardware_and_system.md)。**
:::

### 系统配置

镜像成功烧写后，需要配置地平线RDK IP地址，方便日常使用。登录用户名：root 密码：root。

:::caution **注意**
为方便后续顺利安装和使用tros.b，请使用**root**账户进行登录。
:::

体验和开发过程中经常需要使用scp/ssh等命令通过IP地址访问地平线RDK，因此这里推荐使用动态配置，参考[网络配置](https://developer.horizon.cc/documents_rdk/configuration/network)。

尝试ping百度服务器

```shell
root@ubuntu:~# ping www.baidu.com
PING www.a.shifen.com (180.101.49.11) 56(84) bytes of data.
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=1 ttl=52 time=4.10 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=2 ttl=52 time=4.34 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=3 ttl=52 time=4.28 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=4 ttl=52 time=4.21 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=5 ttl=52 time=4.19 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=6 ttl=52 time=4.98 ms
^C
--- www.a.shifen.com ping statistics ---
6 packets transmitted, 6 received, 0% packet loss, time 5008ms
rtt min/avg/max/mdev = 4.100/4.348/4.978/0.291 ms

```

ping命令正常返回说明互联网访问以及DNS配置均正确

升级系统镜像以及源信息`sudo apt update` `sudo apt upgrade`

测试ssh，`ssh root@地平线RDK IP地址` 这里地平线RDK IP地址为10.64.61.228，因此输入`ssh root@10.64.61.228`，第一次ssh登陆会有如下提示

```shell
 ssh root@10.64.61.241
The authenticity of host '10.64.61.241 (10.64.61.241)' can't be established.
ECDSA key fingerprint is SHA256:5NQuzIkfNYZftPkxrzCugbQs5Gy5CEC5U3Nhtu+sJs8.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
```

输入`yes`回车，输入密码：root，即可正常访问地平线RDK

```dotnetcli
ssh root@10.64.61.241
The authenticity of host '10.64.61.241 (10.64.61.241)' can't be established.
ECDSA key fingerprint is SHA256:5NQuzIkfNYZftPkxrzCugbQs5Gy5CEC5U3Nhtu+sJs8.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added '10.64.61.241' (ECDSA) to the list of known hosts.
root@10.64.61.241's password:
Permission denied, please try again.
root@10.64.61.241's password:
Welcome to Ubuntu 20.04.4 LTS (GNU/Linux 4.14.87 aarch64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage
Last login: Sat Apr  2 05:57:05 2022 from 10.64.37.219
root@ubuntu:~#
```

## X86平台

使用X86平台物理机安装Ubuntu 20.04 64位系统，并配置好网络环境。也可使用虚拟机安装或docker，但是运行效率可能会较低。
