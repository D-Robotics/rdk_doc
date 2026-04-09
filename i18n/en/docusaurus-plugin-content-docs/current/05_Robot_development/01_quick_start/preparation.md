---
sidebar_position: 1
---



```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

TogetheROS.Bot supports installation on Ubuntu 20.04/Ubuntu 22.04 systems running on both RDK and x86 platforms. Installing via DEB packages on Ubuntu is simple and fast, and we recommend this method for users who are trying it out for the first time.

The following sections describe environment preparation details for the RDK and x86 platforms, respectively.

## RDK Platform

### System Installation

Before installing tros.b, we recommend upgrading your RDK system image to the latest version. Instructions for flashing Ubuntu 20.04/Ubuntu 22.04 images are as follows:

<Tabs groupId="tros-distro">
<TabItem value="RDK X3/X5/Ultra" label="RDK X3/X5/Ultra">

[Ubuntu Image Flashing Guide](/install_os/)

:::caution **Note**
- **If you are using an RDK X3 with a 1.x system version installed, you must upgrade to version 2.x.**
- **For instructions on checking your system version number and further details, please refer to the [FAQs](../../08_FAQ/03_applications_and_examples.md).**
:::

</TabItem>
<TabItem value="RDK S100" label="RDK S100">

[Ubuntu Image Flashing Guide](/rdk_s/02_install_os/)

</TabItem>
</Tabs>

If the image is already installed, you can perform an upgrade by running `sudo apt update` and `sudo apt upgrade`.

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, please refer to the section `Q10: How to handle failures or errors when running apt update?` in [Common Issues](../../08_FAQ/01_hardware_and_system.md) for solutions.**
:::

### System Configuration

After successfully flashing the image, you need to configure the RDK’s IP address for convenient daily usage. Login credentials: username `root`, password `root`.

:::caution **Note**
To ensure smooth installation and use of tros.b later on, please log in using the **root** account.
:::

During development and testing, you will frequently need to access the RDK via commands like `scp`/`ssh` using its IP address. Therefore, we recommend configuring the network dynamically, as described below:

<Tabs groupId="tros-distro">
<TabItem value="RDK X3/X5/Ultra" label="RDK X3/X5/Ultra">

[Network Configuration](../../02_System_configuration/01_network_blueteeth.md)

</TabItem>
<TabItem value="RDK S100" label="RDK S100">

[Network Configuration](/rdk_s/System_configuration/network_bluetooth)

</TabItem>
</Tabs>

Try pinging Baidu's server:

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

A successful ping response indicates that internet access and DNS configuration are working correctly.

Update the system image and package sources by running `sudo apt update` and `sudo apt upgrade`.

Test SSH connectivity by running `ssh root@<RDK_IP_ADDRESS>`. In this example, the RDK’s IP address is `10.64.61.228`, so you would run `ssh root@10.64.61.228`. The first SSH login attempt will display the following prompt:

```shell
 ssh root@10.64.61.241
The authenticity of host '10.64.61.241 (10.64.61.241)' can't be established.
ECDSA key fingerprint is SHA256:5NQuzIkfNYZftPkxrzCugbQs5Gy5CEC5U3Nhtu+sJs8.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
```

Type `yes` and press Enter, then enter the password `root` to successfully access the RDK:

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

## x86 Platform

Install a 64-bit Ubuntu 20.04 system on a physical x86 machine and configure the network environment properly. Alternatively, you may use a virtual machine or Docker container, though performance may be lower.