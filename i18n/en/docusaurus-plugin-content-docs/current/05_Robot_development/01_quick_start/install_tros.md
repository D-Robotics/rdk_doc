---
sidebar_position: 2
---
# 5.1.2 apt installation and upgrade

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

This section introduces how to use apt to install TogetheROS.Bot on RDK.

## RDK

Prerequisites

- The environment preparation in section 1.1 has been completed.
- The RDK system has been installed.
- The RDK can access the internet normally.
- The RDK can be accessed remotely via SSH.

### Installation

**Note: The IP address of the RDK used here is 10.64.61.241. Replace it with your IP address during installation.**

Login to the RDK

```shell
ssh root@10.64.61.241
```

Install the tros.b package:

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

**Note: If you encounter the error `E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).' after running the installation command, please execute the command `apt --fix-broken install` to install the related dependencies before installing tros.b.**

After the installation is complete, check the files in the /opt directory

```bash
root@ubuntu:/userdata# ls /opt/
hobot  tros
```
The tros.b is installed in the /opt directory.

### Upgrade tros.b

Login to RDK:

```shell
ssh root@10.64.61.241
```

Upgrade tros.b deb package:

```shell
sudo apt update
sudo apt upgrade
```

### Check the current version of tros.b

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
root@ubuntu:~# apt show tros
Package: tros
Version: 2.0.0-20230523223852
Maintainer: kairui.wang <kairui.wang@d-robotics.cc>
Installed-Size: unknown
Depends: hobot-models-basic, tros-ros-base, tros-ai-msgs, tros-audio-control, tros-audio-msg, tros-audio-tracking, tros-body-tracking, tros-dnn-benchmark-example, tros-dnn-node, tros-dnn-node-example, tros-dnn-node-sample, tros-elevation-net, tros-gesture-control, tros-hand-gesture-detection, tros-hand-lmk-detection, tros-hbm-img-msgs, tros-hobot-app-xrrobot-body-tracking, tros-hobot-app-xrrobot-gesture-control, tros-hobot-codec, tros-hobot-cv, tros-hobot-falldown-detection, tros-hobot-hdmi, tros-hobot-image-publisher, tros-hobot-mot, tros-hobot-usb-cam, tros-image-subscribe-example, tros-img-msgs, tros-imu-sensor, tros-line-follower-model, tros-line-follower-perception, tros-mipi-cam, tros-mono2d-body-detection, tros-mono2d-trash-detection, tros-mono3d-indoor-detection, tros-parking-perception, tros-parking-search, tros-rgbd-sensor, tros-websocket, tros-xrrobot, tros-xrrobot-msgs
Download-Size: 980 B
APT-Manual-Installed: yes
APT-Sources: http://archive.d-robotics.cc/ubuntu-rdk focal/main arm64 Packages
Description: TogetheROS Bot

```

It can be seen that the current version of tros.b has been upgraded to version 2.0.0.





</TabItem>
<TabItem value="humble" label="Humble">

```bash
root@ubuntu:~# apt show tros-humble
Package: tros-humble
Version: 2.2.0-jammy.20240410.221258
Priority: optional
Section: misc
Maintainer: zhuo <zhuo.wang@d-robotics.cc>
Installed-Size: 44.0 kB
Depends: hobot-models-basic, tros-humble-ai-msgs, tros-humble-audio-control, tros-humble-audio-msg, tros-humble-   audio-tracking, tros-humble-base, tros-humble-body-tracking, tros-humble-dnn-benchmark-example, tros-humble-dnn-   node, tros-humble-dnn-node-example, tros-humble-dnn-node-sample, tros-humble-elevation-net, tros-humble-gesture-   control, tros-humble-hand-gesture-detection, tros-humble-hand-lmk-detection, tros-humble-hbm-img-msgs, tros-humb   le-hobot-audio, tros-humble-hobot-chatbot, tros-humble-hobot-codec, tros-humble-hobot-cv, tros-humble-hobot-fall   down-detection, tros-humble-hobot-hdmi, tros-humble-hobot-image-publisher, tros-humble-hobot-llm, tros-humble-ho   bot-mot, tros-humble-hobot-shm, tros-humble-hobot-tts, tros-humble-hobot-usb-cam, tros-humble-hobot-vio, tros-hu   mble-hobot-visualization, tros-humble-img-msgs, tros-humble-imu-sensor, tros-humble-line-follower-model, tros-hu   mble-line-follower-perception, tros-humble-mipi-cam, tros-humble-mono2d-body-detection, tros-humble-mono2d-trash   -detection, tros-humble-mono3d-indoor-detection, tros-humble-parking-perception, tros-humble-parking-search, tro   s-humble-rgbd-sensor, tros-humble-websocket, tros-humble-ros-workspace
Download-Size: 5,546 B
APT-Manual-Installed: yes
APT-Sources: http://archive.d-robotics.cc/ubuntu-rdk jammy/main arm64 Packages
Description: TogetheROS Bot

```
It can be seen that the current version of tros.b has been upgraded to version 2.2.0.

</TabItem>
</Tabs>

## X86 Platform Setup

### Prerequisites:
- Completed the environment preparation as described in Section 2.1.
- The system is Ubuntu 20.04 with active internet access.

---

### 1. Set Locale and Enable Universe Repository
Run the following commands to configure the locale and enable the universe software repository:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
```
### 2. Download GPG Keys and Add Source Lists

Run the following commands to download the required GPG keys and add the appropriate source lists:

```bash
sudo apt update && sudo apt install curl

sudo curl -sSL http://archive.d-robotics.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://archive.d-robotics.cc/ubuntu-rdk-sim focal main" | sudo tee /etc/apt/sources.list.d/sunrise.list > /dev/null

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### 3. Update Package Sources and Install `tros.b`

Run the commands below to update package sources and install the `tros.b` package:

```bash
sudo apt update
sudo apt install tros
```
:::caution
- **If your X86 platform already has `tros.b` version 1.x installed, please use the command `sudo apt remove tros` to uninstall it before installing `tros.b` version 2.x.**  
- **For details on how to check the version of `tros.b`, refer to the [FAQs](/docs/08_FAQ/03_applications_and_examples.md).**

:::

