---
sidebar_position: 1
---
# 5.1.1 Environment Setup

TogetheROS.Bot supports installation on the RDK with  Ubuntu 20.04/22.04 system. Installing through DEB packages on Ubuntu system is simple and recommended for users who want to experience it initially.

Next, we will introduce the environment setup details for RDK.

## RDK

### System Installation

Before installing tros.b, it is recommended to upgrade the RDK system image to the latest version. Here is the method of burning  Ubuntu 20.04/22.04 image: [Ubuntu Image Burning Method](/i18n/en/docusaurus-plugin-content-docs/current/01_Quick_start/install_os.md)

If the image has already been installed, you can upgrade it by running the commands `sudo apt update` and `sudo apt upgrade`.

### Configuration

After successfully burning the image, you need to configure the IP address of the RDK. The login username is "root" and the password is "root".

:::caution **Note**
To facilitate the smooth installation and use of tros.b later, please log in with the **root** account.
:::

In the experience and development process, it is often necessary to access the RDK through commands such as scp/ssh using the IP address. Therefore, it is recommended to use dynamic configuration, as described in [Network Configuration](/i18n/en/docusaurus-plugin-content-docs/current/02_System_configuration/01_network_blueteeth.md)

Try to ping Baidu server:

```shell
root@ubuntu:~# ping www.baidu.com
PING www.a.shifen.com (180.101.49.11) 56(84) bytes of data.
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=1 ttl=52 time=4.10 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=2 ttl=52 time=4.34 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=3 ttl=52 time=4.28 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=4 ttl=52 time=4.21 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=5 ttl=52 time=4.19 ms
64 bytes from 180.101.49.11 (180.101.49.11): icmp_seq=6 ttl=52 time=4.98 ms

--- www.a.shifen.com ping statistics ---
6 packets transmitted, 6 received, 0% packet loss, time 5008ms
rtt min/avg/max/mdev = 4.100/4.348/4.978/0.291 ms
```
The normal return of the "ping" command indicates that both internet access and DNS configuration are correct.

Upgrade the system image and source information: `sudo apt update` `sudo apt upgrade`

Test SSH: `ssh root@RDK IP address`. Here, the RDK IP address is 10.64.61.228, so enter `ssh root@10.64.61.228`. There will be the following prompt for the first SSH login:

```shell
 ssh root@10.64.61.241
The authenticity of host '10.64.61.241 (10.64.61.241)' can't be established.
ECDSA key fingerprint is SHA256:5NQuzIkfNYZftPkxrzCugbQs5Gy5CEC5U3Nhtu+sJs8.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
```

Enter `yes` and press Enter, then enter the password: root, to access RDK normally.

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