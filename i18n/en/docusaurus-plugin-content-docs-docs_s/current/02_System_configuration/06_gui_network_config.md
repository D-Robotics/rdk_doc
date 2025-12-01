---
sidebar_position: 6
---

# 2.6 GUI Network Configuration Process

This section describes how to configure static `IP`, `DNS`, and `Proxy` settings for the ETH network via the GUI interface in the `Ubuntu` system.

## Modifying Static IP and DNS Configuration

1. After entering the desktop, click the bottom-left corner to open the application list and select the `Settings` app. In the pop-up window, choose `Network`.

![image-show-app](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-show-app.jpg)

2. `Ethernet (eth0)` and `Ethernet (eth1)` correspond to configurations for different physical network ports, as shown below:

![image-phy-eth](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-phy-eth.png)

![image-sel-eth](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-sel-eth.png)

3. Taking `Ethernet (eth1)` as an example, click the gear icon next to the edit button. In the pop-up window, select `IPv4`, choose `Manual` for manual configuration, and enter the `IP` address, subnet mask, and gateway in the `Addresses` field below.

![image_set_static_ip](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image_set_static_ip.png)

4. Scroll down and enter the DNS configuration in the DNS field below.

![image_set_static_dns](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image_set_static_dns.png)

5. To assign multiple IP addresses to a single network interface (using `Ethernet (eth1)` as an example), click the plus sign (`+`) on the right. Configure the additional IP address following the same steps as in steps 3 and 4. After completing the configuration, ensure that the `netplan-eth1` option is selected (indicated by a `√`).

![image-sel_ok](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-sel_ok.jpg)

## Modifying Proxy Configuration

Similar to configuring a static `IP`, follow these steps to modify the `Proxy` settings:

1. After entering the desktop, click the bottom-left corner to open the application list and select the `Settings` app. In the pop-up window, choose `Network`.

2. Scroll down and click the gear icon next to `Network Proxy` to access its configuration.

![image-proxy](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-proxy.jpg)

3. Enter the required proxy settings in the pop-up window.

![image-proxy_set](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/02_System_configuration/image/gui_network_config/image-proxy_set.png)