---
sidebar_position: 1
---

# 2.1 Network and Bluetooth Configuration

This section mainly introduces how to modify wired and wireless network configurations on the development board.

## Wired Network {#config_ethnet}

### Wired Network Configuration – Netplan Method

:::info Note
Network configuration via Netplan has only been verified and used on the `RDK S100`; other platforms are currently not supported.

The root filesystem of `RDK S100` is built upon Ubuntu 22.04, which by default does not support enabling or disabling network interfaces using the traditional `ifup`/`ifdown` commands.
:::

In Ubuntu systems, static network configuration for the development board is stored in the file `/etc/netplan/01-hobot-net.yaml`. Below are specific configuration instructions:

- **Static IP and Subnet Mask**: To assign a static IP address and subnet mask to a network interface, use the `addresses` field with CIDR notation for the subnet mask.
- **DHCP Configuration**: To enable a specified network interface to automatically obtain an IP address via DHCP (Dynamic Host Configuration Protocol), set the `dhcp4` or `dhcp6` field to `yes`.
- **Custom MAC Address**: Use the `macaddress` field to assign a custom MAC address to a network interface.
- **Custom DNS Servers**: Use the `nameservers` field to specify custom DNS server addresses for a network interface.

An example of network configuration using Netplan is as follows:

```shell
sudo vim /etc/netplan/01-hobot-net.yaml
```

```shell
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: yes
      dhcp6: yes
      nameservers:
        addresses: [10.9.1.2, 8.8.8.8, 8.8.4.4]
    eth1:
      addresses:
        - 192.168.127.10/24
      nameservers:
        addresses: [10.9.1.2, 8.8.8.8, 8.8.4.4]

```

After making changes, run `sudo netplan apply` to apply the new configuration.

For detailed documentation on Netplan configuration files, please refer to: [Ubuntu Manpage: netplan](https://manpages.ubuntu.com/manpages/jammy/man5/netplan.5.html).

:::tip Tip
The desktop version of RDK S100 uses NetworkManager + Netplan as its default network management framework. When users configure network connections via NetworkManager (either through GUI or the `nmcli` command), corresponding configuration files are generated under `/etc/NetworkManager/system-connections`.

During Ubuntu system boot, Netplan configuration files located in `/etc/netplan/` are used to generate corresponding connection profiles under `/run/NetworkManager/system-connections`. By default in Ubuntu, connection profiles under `/run/NetworkManager/system-connections` take **precedence over** those under `/etc/NetworkManager/system-connections`.

Therefore, if you wish configurations made via NetworkManager (GUI or `nmcli`) to take effect, you must first **remove** the corresponding settings from the Netplan configuration files under `/etc/netplan/`.
:::

## Wireless Network

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=12

The development board integrates a 2.4GHz Wi-Fi module, supporting both Soft AP and Station modes, and runs in Station mode by default. The following sections describe how to use both modes.

### Station Mode

In Station mode, the development board acts as a client connecting to a router's Wi-Fi hotspot for internet access.

- For users running the Ubuntu Desktop edition, click the Wi-Fi icon in the top-right corner of the desktop, select the desired hotspot, and enter the password to complete network setup, as shown below:  
  ![image-wifi-config](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-wifi-config.jpeg)

- For users running the Ubuntu Server edition, wireless network configuration can be performed via command line as follows:

1. Use the command `sudo nmcli device wifi rescan` to scan for available hotspots. If you receive the following message, scanning was too frequent—please wait and try again later:
   ```shell
   root@ubuntu:~# sudo nmcli device wifi rescan
   Error: Scanning not allowed immediately following previous scan.
   ```
2. Use the command `sudo nmcli device wifi list` to display the list of discovered hotspots.
3. Connect to a hotspot using the command `sudo wifi_connect "SSID" "PASSWD"`. A successful connection will return output similar to the following:

   ```shell
   root@ubuntu:~# sudo wifi_connect "WiFi-Test" "12345678"
   Device 'wlan0' successfully activated with 'd7468833-4195-45aa-aa33-3d43da86e1a7'.
   ```

   :::tip
   If after attempting to connect you receive the following message, the hotspot was not found. In this case, run `sudo nmcli device wifi rescan` to rescan and then retry the connection:

   ```shell
   root@ubuntu:~# sudo wifi_connect "WiFi-Test" "12345678"
   Error: No network with SSID 'WiFi-Test' found.
   ```

   :::

### Soft AP Mode

:::tip
Wi-Fi AP mode on RDK S100 is currently unavailable.  
Continuously being updated....
:::

<!-- By default, the development board’s wireless network operates in Station mode. To switch to Soft AP mode, follow the steps below.

1. Install `hostapd` and `isc-dhcp-server`:

    ```shell
    sudo apt update
    sudo apt install hostapd
    sudo apt install isc-dhcp-server
    ```

2. Run `sudo vim /etc/hostapd.conf` to configure `hostapd.conf`, focusing primarily on the following fields:

    ```shell
    interface=wlan0 # Network interface acting as AP hotspot
    ssid=Sunrise # Wi-Fi name
    wpa=2 # 0 for WPA, 2 for WPA2 (typically set to 2)
    wpa_key_mgmt=WPA-PSK # Encryption algorithm (usually WPA-PSK)
    wpa_passphrase=12345678 # Password
    wpa_pairwise=CCMP # Encryption protocol (typically CCMP)
    ```

      - For an open (password-free) hotspot, add the following content to `hostapd.conf`:

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

      - For a secured hotspot (with password), add the following content to `hostapd.conf`:

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

3. Configure the `isc-dhcp-server` as follows:

    - Edit `/etc/default/isc-dhcp-server` by running `sudo vim /etc/default/isc-dhcp-server` and add the following defined network interface:
    ```shell
    INTERFACESv4="wlan0"
    ```
    - Edit `/etc/dhcp/dhcpd.conf` by running `sudo vim /etc/dhcp/dhcpd.conf` and uncomment the following line:
    ```shell
      authoritative;
    ```
    - Then append the following configuration block at the end of `/etc/dhcp/dhcpd.conf`:
    ```shell
      subnet 10.5.5.0 netmask 255.255.255.0 { # Subnet and subnet mask
      range 10.5.5.100 10.5.5.254; # IP address range available for assignment
      option subnet-mask 255.255.255.0; # Subnet mask
      option routers 10.5.5.1; # Default gateway
      option broadcast-address 10.5.5.31; # Broadcast address
      default-lease-time 600; # Default lease time in seconds
      max-lease-time 7200; # Maximum lease time in seconds
    }
    ```

4. Stop the `wpa_supplicant` service and restart `wlan0`:

    ```bash
    systemctl mask wpa_supplicant
    systemctl stop wpa_supplicant

    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up
    ```

5. Start the `hostapd` service as follows:
   - Execute the command `sudo hostapd -B /etc/hostapd.conf`:
   ```bash
    root@ubuntu:~# sudo hostapd -B /etc/hostapd.conf

    Configuration file: /etc/hostapd.conf
    Using interface wlan0 with hwaddr 08:e9:f6:af:18:26 and ssid "sunrise"
    wlan0: interface state UNINITIALIZED->ENABLED
   ```wlan0: AP-ENABLED
   ```
   - Use the `ifconfig` command to configure the IP address and subnet for the wireless interface `wlan0`. Ensure this configuration matches the settings from Step 3.
    ```bash
    sudo ifconfig wlan0 10.5.5.1 netmask 255.255.255.0
    ```
   - Finally, start the `dhcp` server. When clients connect to the hotspot, they will be assigned an IP address from the range `10.5.5.100` to `10.5.5.255`.
    ```bash
    sudo ifconfig wlan0 10.5.5.1 netmask 255.255.255.0
    sudo systemctl start isc-dhcp-server
    sudo systemctl enable isc-dhcp-server
    ```

6. Connect to the development board's hotspot, for example, `sunrise`.
![image-20220601203025803](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/network/image-20220601203025803.png)

7. To switch back to `Station` mode, follow these steps:

    [RDK X5]

    ```bash
    # Stop hostapd
    killall -9 hostapd

    # Clear the IP address of wlan0
    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up

    # Restart wpa_supplicant
    systemctl unmask wpa_supplicant
    systemctl restart wpa_supplicant

    # Reinstall the Wi-Fi driver
    rmmod aic8800_fdrv
    modprobe aic8800_fdrv

    # Connect to a hotspot; refer to the previous section "Wireless Network" for detailed instructions.
    wifi_connect "WiFi-Test" "12345678"
    ```

    [Other]

    ```bash
    # Stop hostapd
    killall -9 hostapd

    # Clear the IP address of wlan0
    ip addr flush dev wlan0
    sleep 0.5
    ifconfig wlan0 down
    sleep 1
    ifconfig wlan0 up

    # Restart wpa_supplicant
    systemctl unmask wpa_supplicant
    systemctl restart wpa_supplicant

    # Connect to a hotspot; refer to the previous section "Wireless Network" for detailed instructions.
    wifi_connect "WiFi-Test" "12345678"
    ``` -->

## DNS Service

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=13

DNS (Domain Name Server) is a server that translates domain names into their corresponding IP addresses.

The DNS configuration on the development board is managed via the `/etc/systemd/resolved.conf` file. Users can modify this file to configure DNS settings as follows:

1. Edit the `resolved.conf` file and add DNS server addresses, for example:

   ```bash
   DNS=8.8.8.8 114.114.114.114
   ```

2. Apply the DNS configuration with the following commands:

   ```bash
   sudo systemctl restart systemd-resolved
   sudo systemctl enable systemd-resolved
   sudo mv /etc/resolv.conf  /etc/resolv.conf.bak
   sudo ln -s /run/systemd/resolve/resolv.conf /etc/
   ```

## Proxy Configuration

Proxy configuration refers to setting up a network proxy. In network communication, a proxy server acts as an intermediary between the client and the destination server. The client sends requests to the proxy server, which then forwards them to the target server. Similarly, responses from the target server are returned to the client via the proxy server.

Edit either the `~/.bashrc` or `/etc/environment` file. Edit `~/.bashrc` to configure the proxy for the current user, or edit `/etc/environment` to configure it system-wide for all users.

Add the following lines to the file (using an HTTP proxy as an example):

```
http_proxy=http://proxy_server_address:port
https_proxy=http://proxy_server_address:port
ftp_proxy=http://proxy_server_address:port
no_proxy=localhost,127.0.0.1
```

After saving the file, run the following command to apply the configuration:

```
source ~/.bashrc
```

## System Update

:::warning
Do not execute before the product is officially released.
:::

For system security and stability, it is recommended that users update the system using the `apt` command after installation.

The `/etc/apt/sources.list` file contains the list of software repositories used by the `apt` command. Before installing software, you should first update the package list using `apt`.

Open a terminal and run the following command:

```bash
sudo apt update
```

Next, upgrade all installed packages to their latest versions with:

```bash
sudo apt full-upgrade
```

:::tip
It is recommended to use `full-upgrade` instead of `upgrade`, as `full-upgrade` will also update dependency packages when dependencies change.

When running `sudo apt full-upgrade`, the system will display the amount of data to be downloaded and the disk space required. However, `apt` does not check whether sufficient disk space is available. It is advisable to manually verify available disk space using the `df -h` command. Additionally, downloaded `.deb` files during the upgrade process are stored in `/var/cache/apt/archives`. You can free up disk space by clearing this cache with `sudo apt clean`.
:::

After running `apt full-upgrade`, drivers, kernel files, and some system software may be reinstalled. It is recommended to manually reboot the device to apply all updates:

```bash
sudo reboot
```

## Bluetooth Configuration (Deprecated)

:::tip
Bluetooth configuration for RDK S100 is currently unavailable.
Continuously being updated....
:::

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=9

### Initialization

Bluetooth functionality is disabled by default on the development board. You need to run the `/usr/bin/startbt6212.sh` script to initialize it. This script performs the following tasks:

- Resets the Bluetooth module.
- Creates the `messagebus` user and group, which are required by the `dbus-daemon` process.
- Runs `brcm_patchram_plus` to load the Bluetooth driver and firmware.
- Continuously checks for the existence of the `/sys/class/bluetooth/hci0` directory to confirm that the Bluetooth driver is running properly.
- Displays **Done setting line discipline** when Bluetooth initialization succeeds.
- Executes `hciconfig hci0 up` to bring the Bluetooth interface up.
- Executes `hciconfig hci0 piscan` to enable Bluetooth scanning (this step can be omitted depending on your needs).

The log output after successful script execution is shown below:

![image-20220601172145987](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172145987.png)

Additionally, you can verify whether Bluetooth processes are running normally with the following command:

```bash
ps ax | grep "/usr/bin/dbus-daemon\|/usr/lib/bluetooth/bluetoothd"
/usr/bin/dbus-daemon

/usr/lib/bluetooth/bluetoothd
```

### Network Pairing and Connection

Run `sudo bluetoothctl` to enter the interactive Bluetooth configuration interface. If the device information appears as shown in the image below, the Bluetooth adapter has been recognized. Use the `show` command to view Bluetooth details, paying attention to the `powered` and `discoverable` states.

![image-20220601172604051](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172604051.png)

Execute `power on` to enable Bluetooth, as shown below:

![image-20220601172501882](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172501882.png)

To make the Bluetooth device discoverable by nearby devices, run `discoverable on` to enable Bluetooth and set it to discoverable mode, as illustrated below:

![image-20220601172648853](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601172648853.png)

At this point, scanning with a smartphone or computer will reveal a Bluetooth device named `ubuntu`:

![image-20220601175322650](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601175322650.png)

Next, test active Bluetooth scanning. In the `bluetoothctl` interactive interface, type `scan on` to start active scanning. The system will periodically print nearby devices—in this case, my smartphone has been detected. Use `scan off` to stop scanning and display a summary of discovered Bluetooth devices:

![image-20220601154131158](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154131158.png)

![image-20220601154253947](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154253947.png)Then comes pairing with other Bluetooth devices:

- Pairing command: `pair [targetMAC]`. After entering this command, type `yes` when prompted, and select the `Pair` option on the remote Bluetooth device to complete pairing.

- After successful pairing, you can use `trust [targetMAC]` to enable automatic connection the next time.

![image-20220601154414717](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/hardware_interface/image-20220601154414717.png)

After the above steps, basic Bluetooth scanning and pairing functionality is complete. For more advanced features, please refer to the official `BlueZ` documentation.