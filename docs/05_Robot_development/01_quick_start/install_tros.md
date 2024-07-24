---
sidebar_position: 2
---

# 1.2 apt安装与升级

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

本章节介绍地平线RDK和X86平台如何使用apt安装TogetheROS.Bot。

## 地平线RDK平台

前提

- 已完成1.1章节环境准备工作
- 地平线RDK已安装2.x版本系统
- 地平线RDK能够正常访问互联网
- 地平线RDK能够远程ssh

:::caution**注意**
- **2.x版本tros.b仅支持2.x版本系统镜像，[1.x版本tros.b](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html)仅支持1.x版本系统。**
- **如果您使用的是1.x版本系统镜像，需要将[系统升级](./preparation)到2.x版本。**
- **系统和tros.b版本号查看方法以及详细说明，请查看[FAQs](../FAQs/hardware_and_system.md)。**

| 依赖关系    | 1.x tros.b  | 2.x tros.b |
| -----------| ------------| ------------|
| 1.x系统镜像 |       √     |       x     |
| 2.x系统镜像 |       x     |       √     |
:::

### 安装tros.b

**注意：这里使用的地平线RDK IP为10.64.61.241，安装时需要根据自己的地平线RDK IP进行替换**

登录地平线RDK：

```shell
ssh root@10.64.61.241
```

安装tros.b功能包：

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
sudo apt update
sudo apt install tros
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
sudo apt update
sudo apt install tros-humble
```

</TabItem>
</Tabs>

**注意：如果您运行安装命令后提示`E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).`，先执行`apt --fix-broken install`命令安装相关依赖后再安装tros.b。**

安装完成后，查看/opt目录下文件

```bash
root@ubuntu:/userdata# ls /opt/
hobot  tros
```

可以看到tros.b已安装在/opt目录下

### 升级tros.b

以地平线RDK安装为例，X86 Ubuntu升级方法和地平线RDK一致。

登录地平线RDK

```shell
ssh root@10.64.61.241
```

升级tros.b deb包

```shell
sudo apt update
sudo apt upgrade
```

查看当前tros.b版本


<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
root@ubuntu:~# apt show tros
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@horizon.ai>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, tros-audio-control, tros-audio-msg, tros-audio-tracking, tros-body-tracking, tros-dnn-benchmark-example, tros-dnn-node, tros-dnn-node-example, tros-dnn-node-sample, tros-elevation-net, tros-gesture-control, tros-hand-gesture-detection, tros-hand-lmk-detection, tros-hbm-img-msgs, tros-hobot-app-xrrobot-body-tracking, tros-hobot-app-xrrobot-gesture-control, tros-hobot-codec, tros-hobot-cv, tros-hobot-falldown-detection, tros-hobot-hdmi, tros-hobot-image-publisher, tros-hobot-mot, tros-hobot-usb-cam, tros-image-subscribe-example, tros-img-msgs, tros-imu-sensor, tros-line-follower-model, tros-line-follower-perception, tros-mipi-cam, tros-mono2d-body-detection, tros-mono2d-trash-detection, tros-mono3d-indoor-detection, tros-parking-perception, tros-parking-search, tros-rgbd-sensor, tros-websocket, tros-xrrobot, tros-xrrobot-msgs
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://sunrise.horizon.cc/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot

```

可以看到当前tros.b版本已升级为2.0.0版本

</TabItem>
<TabItem value="humble" label="Humble">

```bash
root@ubuntu:~# apt show tros-humble
Package: tros-humble
Version: 2.2.0-jammy.20240410.221258
Priority: optional
Section: misc
Maintainer: zhuo <zhuo.wang@horizon.cc>
Installed-Size: 44.0 kB
Depends: hobot-models-basic, tros-humble-ai-msgs, tros-humble-audio-control, tros-humble-audio-msg, tros-humble-   audio-tracking, tros-humble-base, tros-humble-body-tracking, tros-humble-dnn-benchmark-example, tros-humble-dnn-   node, tros-humble-dnn-node-example, tros-humble-dnn-node-sample, tros-humble-elevation-net, tros-humble-gesture-   control, tros-humble-hand-gesture-detection, tros-humble-hand-lmk-detection, tros-humble-hbm-img-msgs, tros-humb   le-hobot-audio, tros-humble-hobot-chatbot, tros-humble-hobot-codec, tros-humble-hobot-cv, tros-humble-hobot-fall   down-detection, tros-humble-hobot-hdmi, tros-humble-hobot-image-publisher, tros-humble-hobot-llm, tros-humble-ho   bot-mot, tros-humble-hobot-shm, tros-humble-hobot-tts, tros-humble-hobot-usb-cam, tros-humble-hobot-vio, tros-hu   mble-hobot-visualization, tros-humble-img-msgs, tros-humble-imu-sensor, tros-humble-line-follower-model, tros-hu   mble-line-follower-perception, tros-humble-mipi-cam, tros-humble-mono2d-body-detection, tros-humble-mono2d-trash   -detection, tros-humble-mono3d-indoor-detection, tros-humble-parking-perception, tros-humble-parking-search, tro   s-humble-rgbd-sensor, tros-humble-websocket, tros-humble-ros-workspace
Download-Size: 5,546 B
APT-Manual-Installed: yes
APT-Sources: http://sunrise.horizon.cc/ubuntu-rdk jammy/main arm64 Packages
Description: TogetheROS Bot

```

可以看到当前tros.b版本已升级为2.2.0版本

</TabItem>
</Tabs>

## X86平台

前提：

- 已完成2.1章节环境准备工作
- Ubuntu系统为Ubuntu 20.04，且能够正常访问互联网

1. 设置locale和启用universe软件源

   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   sudo apt install software-properties-common
   sudo add-apt-repository universe
   ```

2. 下载gpg密钥文件并添加源列表：

   ```bash
   sudo apt update && sudo apt install curl

   sudo curl -sSL http://sunrise.horizon.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
   echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://sunrise.horizon.cc/ubuntu-rdk-sim focal main" | sudo    tee /etc/apt/sources.list.d/sunrise.list > /dev/null

   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. 更新源信息，安装tros.b：

   ```bash
   sudo apt update
   sudo apt install tros
   ```

:::caution**注意**
- **如果您的X86平台已安装1.x版本tros.b，请先使用命令`sudo apt remove tros`删除后再安装2.x版本tros.b**。
- **关于如何查看tros.b版本号，请查看[FAQs](../FAQs/hardware_and_system.md)**。
:::
