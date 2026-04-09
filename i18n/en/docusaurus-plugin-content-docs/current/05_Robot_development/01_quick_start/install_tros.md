---
sidebar_position: 2
---
# 5.1.2 apt installation and upgrade

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

This section describes how to install TogetheROS.Bot using `apt` on both RDK and X86 platforms.

## RDK Platform

Prerequisites:

- Completed the steps in the [Environment Preparation](./preparation.md) section
- Ubuntu OS is already installed on the RDK
- The RDK has normal internet access
- The RDK supports remote SSH access

:::info 📋 Confirm System Version and Download

Before installing TogetheROS, confirm your system version and download the latest image. For system image downloads, refer to: [Download Resources Summary](../../01_Quick_start/download.md)

:::


<Tabs groupId="tros-distro">
<TabItem value="RDK X3/X5/Ultra" label="RDK X3/X5/Ultra">

**Note for RDK X3 platform users:**

:::caution Attention
- **tros.b version 2.x only supports system images of version 2.x; [tros.b version 1.x](https://developer.d-robotics.cc/api/v1/fileData/TogetherROS/index.html) only supports system version 1.x.**
- **If you are using a system image of version 1.x, you must [upgrade your system](./preparation) to version 2.x.**
- **For instructions on checking system and tros.b versions and further details, please see the [FAQs](../../08_FAQ/03_applications_and_examples.md).**
:::

| Dependency        | tros.b 1.x | tros.b 2.x |
| ------------------| ---------- | ---------- |
| System Image 1.x  |      √     |      x     |
| System Image 2.x  |      x     |      √     |

</TabItem>
<TabItem value="RDK S100" label="RDK S100">

</TabItem>
</Tabs>

### Installing tros.b

**Note: The RDK IP used here is 10.64.61.241. Replace it with your actual RDK IP during installation.**

Log into the RDK:

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

:::caution **Attention**
- **If the `sudo apt update` command fails or returns an error, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section titled `Q10: How to resolve failures or errors when running apt update?`**
- **If after running the installation command you receive the message `E: Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).`, first run `apt --fix-broken install` to install required dependencies before proceeding with the tros.b installation.**
:::

After installation, check the contents of the `/opt` directory:

```bash
root@ubuntu:/userdata# ls /opt/
hobot  tros
```

You can see that tros.b has been installed under the `/opt` directory.

### Upgrading tros.b

The following example uses the RDK platform; the upgrade procedure for X86 Ubuntu is identical.

Log into the RDK:

```shell
ssh root@10.64.61.241
```

Upgrade the tros.b deb package:

```bash
sudo apt update
sudo apt upgrade
```

:::caution **Attention**
**If the `sudo apt update` command fails or returns an error, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section titled `Q10: How to resolve failures or errors when running apt update?`**
:::

### Checking the Current tros.b Version

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

You can see that the current tros.b version has been upgraded to version 2.0.0.

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
Depends: hobot-models-basic, tros-humble-ai-msgs, tros-humble-audio-control, tros-humble-audio-msg, tros-humble-audio-tracking, tros-humble-base, tros-humble-body-tracking, tros-humble-dnn-benchmark-example, tros-humble-dnn-node, tros-humble-dnn-node-example, tros-humble-dnn-node-sample, tros-humble-elevation-net, tros-humble-gesture-control, tros-humble-hand-gesture-detection, tros-humble-hand-lmk-detection, tros-humble-hbm-img-msgs, tros-humble-hobot-audio, tros-humble-hobot-chatbot, tros-humble-hobot-codec, tros-humble-hobot-cv, tros-humble-hobot-falldown-detection, tros-humble-hobot-hdmi, tros-humble-hobot-image-publisher, tros-humble-hobot-llm, tros-humble-hobot-mot, tros-humble-hobot-shm, tros-humble-hobot-tts, tros-humble-hobot-usb-cam, tros-humble-hobot-vio, tros-humble-hobot-visualization, tros-humble-img-msgs, tros-humble-imu-sensor, tros-humble-line-follower-model, tros-humble-line-follower-perception, tros-humble-mipi-cam, tros-humble-mono2d-body-detection, tros-humble-mono2d-trash-detection, tros-humble-mono3d-indoor-detection, tros-humble-parking-perception, tros-humble-parking-search, tros-humble-rgbd-sensor, tros-humble-websocket, tros-humble-ros-workspace
Download-Size: 5,546 B
APT-Manual-Installed: yes
APT-Sources: http://archive.d-robotics.cc/ubuntu-rdk jammy/main arm64 Packages
Description: TogetheROS Bot

```

You can see that the current tros.b version has been upgraded to version 2.2.0.

:::caution Note
- The displayed `Version` number indicates the actual installed version of `tros.b`. This example illustrates version `2.2.0`.
- For detailed release information about `tros.b`, see the [Release Notes](./changelog).
:::

</TabItem>
</Tabs>

## X86 Platform

Prerequisites:

- Completed the steps in the [Environment Preparation](./preparation.md) section
- Running Ubuntu 20.04 with normal internet access

1. Set locale and enable the universe repository:

   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   sudo apt install software-properties-common
   sudo add-apt-repository universe
   ```

2. Download the GPG key file and add the repository list:

   ```bash
   sudo apt update && sudo apt install curl

   sudo curl -sSL http://archive.d-robotics.cc/keys/sunrise.gpg -o /usr/share/keyrings/sunrise.gpg
   echo "deb [arch=amd64 signed-by=/usr/share/keyrings/sunrise.gpg] http://archive.d-robotics.cc/ubuntu-rdk-sim focal main" | sudo tee /etc/apt/sources.list.d/sunrise.list > /dev/null

   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Update repository information and install tros.b:
```bash
sudo apt update
sudo apt install tros
```

:::caution
- **If you have already installed tros.b version 1.x on your x86 platform, please first remove it using the command `sudo apt remove tros` before installing tros.b version 2.x**.
- **For instructions on how to check the tros.b version number, please refer to the [FAQs](../../08_FAQ/03_applications_and_examples.md)**.
:::