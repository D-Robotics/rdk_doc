---
sidebar_position: 2
---

# 1.3.1 RDK S100 Initial Setup

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Default Login Accounts

Before configuring the system, you need to log in first. The RDK S100 system provides two default accounts:

- **Standard User:** Username `sunrise`, Password `sunrise`
- **Superuser (root):** Username `root`, Password `root`

## Connect to Wi-Fi

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

Follow the Ubuntu 22.04 Wi-Fi connection guide.

</TabItem>

<TabItem value="server" label="Server">

Connect via serial port or SSH using the instructions below:

```bash
# Scan for Wi-Fi networks
sudo nmcli device wifi rescan
sudo nmcli device wifi list # List detected Wi-Fi networks
sudo wifi_connect "SSID" "PASSWD" # Connect to a specific Wi-Fi network
```

After successfully executing the above commands, you will see a message like `successfully xxx`. Use `ifconfig` to obtain the Wi-Fi IP address of the board.

</TabItem>
</Tabs>

## Enable SSH Service

The current system version enables the SSH login service by default. Users can use the following method to start or stop the SSH service.

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

</TabItem>

<TabItem value="server" label="Server">

Use the `systemctl` command to check the current status of the SSH service:

```
sudo systemctl status ssh
```

After running this command, detailed status information about the SSH service will be displayed. If the service is running, the output will show `Active: active (running)`. If the service is not running, it will display `Active: inactive (dead)` or similar information.

Below are the control commands for SSH:

```bash
sudo systemctl start ssh   # Start SSH service
sudo systemctl stop ssh    # Stop SSH service
sudo systemctl enable ssh  # Enable SSH service to start on boot
sudo systemctl disable ssh # Disable SSH service from starting on boot
sudo systemctl restart ssh # Restart SSH service
```

</TabItem>
</Tabs>

For instructions on using SSH, please refer to [Remote Login - SSH Login](../remote_login#ssh).

## Set Login Mode

### Automatic Login on Text Terminal

Modify the `serial-getty@.service` file to enable passwordless login. Follow the steps below:

1. Open `serial-getty@.service`:

```bash
# If logged in as root
vim /lib/systemd/system/serial-getty@.service
# If logged in as sunrise user
sudo vim /lib/systemd/system/serial-getty@.service
```

2. Modify the line containing `ExecStart=-/sbin/agetty` as follows:

```
ExecStart=-/sbin/agetty -a root --keep-baud 921600,115200,38400,9600 %I $TERM
```

**Parameter Explanation:**  
The `-a` parameter specifies the username for automatic login. The option `-o '-p -- \\u'` further customizes the login process by preserving the current environment variables and displaying the username in the login prompt.

3. After rebooting, the user will be automatically logged in.

### Automatic Login on Graphical Terminal

To be updated.

## Configure Chinese Locale

1. Install required packages:

```bash
sudo apt install language-pack-zh-hans language-pack-zh-hans-base fonts-wqy-microhei
```

- `language-pack-zh-hans`: Contains Chinese translation files to display the system interface in Chinese.
- `language-pack-zh-hans-base`: Provides basic Chinese language support.
- `fonts-wqy-microhei`: Installs Chinese fonts.

2. Open a terminal and edit the locale configuration file:

```bash
sudo vim /etc/default/locale
```

Add or modify the following content in the file:

```text
LANG=zh_CN.UTF-8
LANGUAGE=zh_CN:zh
LC_ALL=zh_CN.UTF-8
```

3. Apply the new configuration:

```bash
fc-cache -fv
source /etc/default/locale
```

<!-- 英文文档无需配置中文
## Set Up Chinese Input Method

After installing the Chinese locale, the system’s built-in input method is supported by default. Press the `Super (Windows) key` + `Space` to switch between different input methods. -->

## Configure RDK Studio

To be updated.

## NoMachine Configuration

`NoMachine` is currently not available via `apt` and must be downloaded as a `.deb` package from the official website.

Official NoMachine download page: [NoMachine Download](https://downloads.nomachine.com/download/?id=30&platform=linux&distro=arm)

**Download the Installation Package**

On the official website, locate and download the `ARM64` version suitable for `RDK S100`, then click `Download`.

![image_s100_nomachine_dl](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/configuration_wizard/image_s100_nomachine_dl.PNG)

**Installation**

```shell
sudo apt update; sudo apt upgrade -y   // Ensure all packages are up to date
dpkg -i nomachine_*_arm64.deb
```

**Configuration and Startup**

1. Configure the server to allow remote connections:

    ```shell
    sudo systemctl start nxserver
    ```

2. Enable `NoMachine` to start automatically on boot:

    ```shell
    sudo systemctl enable nxserver
    ```

3. Enable `EGL Capture` (a screen capture feature provided by `NoMachine` to enhance remote desktop experience in certain display server environments):

    ```shell
    sudo /etc/NX/nxserver --eglcapture yes
    ```

    This setting takes effect after a reboot. You can verify it using the following command. If the output shows `EGL Capture has been enabled`, the feature has been successfully configured:

    ```shell
    if [ -f "/usr/lib/systemd/user/org.gnome.Shell@wayland.service" ] && grep -q "nxpreload.sh" "/usr/lib/systemd/user/org.gnome.Shell@wayland.service" && [ -f "/usr/share/applications/org.gnome.Shell.desktop" ] && grep -q "nxpreload.sh" "/usr/share/applications/org.gnome.Shell.desktop" && [ -f "/usr/NX/etc/node.cfg" ] && grep -q "EnableEGLCapture 1" "/usr/NX/etc/node.cfg"; then echo "EGL Capture has been enabled"; else echo "Not enabled"; fi
    ```

4. Restart the `NoMachine` service:

    ```shell
    sudo systemctl restart nxserver
    ```

**Reboot the RDK S100**
Due to a configuration issue with `NXServer`, directly connecting after performing the above steps will result in a black screen; a reboot is required before use.

For instructions on using `NoMachine`, please refer to [Remote Login - NoMachine Login](../remote_login#Nomachine-login).

## User Management

**Change Username**

Taking the new username as `usertest` as an example:

```shell
# Terminate all processes owned by the 'sunrise' user
sudo pkill -u sunrise
# Rename the 'sunrise' user to 'usertest'
sudo usermod -l usertest sunrise
# Change the user's home directory to /home/usertest
sudo usermod -d /home/usertest -m usertest
# Change the user's password
sudo passwd usertest
```

Finally, update the username configured for automatic login in the desktop service:
  - **gdm**: The default desktop service for RDK S100. Change `AutomaticLogin = sunrise` to `AutomaticLogin = usertest` in the file `/etc/gdm3/custom.conf`.
  - **lightdm**: Change `autologin-user=sunrise` to `autologin-user=usertest` in the file `/etc/lightdm/lightdm.conf.d/22-hobot-autologin.conf`.

**Add a New User**

Taking the addition of a new user named `usertest` as an example:

```shell
sudo useradd -U -m -d /home/usertest -k /etc/skel/ -s /bin/bash -G disk,kmem,dialout,sudo,audio,video,render,i2c,lightdm,vpu,gdm,weston-launch,graphics,jpu,ipu,vps,misc,gpio usertest
sudo passwd usertest
sudo cp -aRf /etc/skel/. /home/usertest
sudo chown -R usertest:usertest /home/usertest
```

You may also refer to the username modification steps above to configure the newly added user for automatic login.