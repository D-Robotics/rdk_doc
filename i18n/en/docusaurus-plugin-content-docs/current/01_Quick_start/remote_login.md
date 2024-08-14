---
sidebar_position: 4
---
# 1.4 Remote Login

This chapter is intended to introduce users who need to remotely access the development board through their personal computer (PC) on how to remotely login via serial port and network (VNC, SSH) methods.

:::tip

Before remote login via network, the development board needs to be connected to the network through wired Ethernet or wireless WiFi and configure the IP address of the development board. For the IP address information under both connection methods, refer to the following descriptions:

- Wired Ethernet: The development board defaults to static IP mode, with an IP address of `192.168.1.10`, subnet mask of `255.255.255.0`, and gateway of `192.168.1.1`.
- Wireless WiFi: The development board's IP address is generally assigned by the router and can be viewed in the device command line using the `ifconfig` command for the wlan0 network.

:::

## Serial Port Login{#login_uart}

<iframe width="560" height="315" src="https://www.youtube.com/embed/dYV5nw_PDMw?si=SzkP6H_QaL1OPft9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Before using serial port login, it is necessary to confirm that the serial port cable of the development board is correctly connected to the computer. The connection method can refer to the [Serial Port Debugging](../01_Quick_start/hardware_introduction.md) chapter. Serial port login requires a PC terminal tool. The commonly used tools are `Putty`, `MobaXterm`, etc. Users can choose according to their own habits. The port configuration process is similar for different tools. Below is an example with `MobaXterm` to explain the process of creating a new serial port connection:

- When the USB-to-Serial adapter is first plugged into the computer, the serial port driver needs to be installed. The driver can be obtained from the [Tools sub-column](https://developer.d-robotics.cc/resource) of the Resource Center. After the driver is installed, the Device Manager can recognize the serial port board port normally, as shown in the figure below:  
![image-20220416105939067](../../../../../static/img/01_Quick_start/image/remote_login/image-20220416105939067.png)

- Open the `MobaXterm` tool, click `Session`, and then select `Serial`.

- Configure the port number, for example, `COM3`. The actual serial port number used depends on the serial port number recognized by the PC.

- Set the serial port configuration parameters as follows:
  
  | Configuration Item    | Parameter Value |
  | --------------------- | -------------- |
  | Baud rate             | 921600         |
  | Data bits             | 8              |
  | Parity                | None           |
  | Stop bits             | 1              |
  | Flow Control          | None           |
  
- Click `OK`, enter the username: `root`, password: `root` to log in to the device  
![image-Uart-Login](../../../../../static/img/01_Quick_start/image/remote_login/image-Uart-Login.gif)

At this point, you can use the `ifconfig` command to query the IP address of the development board, where eth0 and wlan0 represent the wired and wireless networks respectively:
```bash
root@ubuntu:~# ifconfig
eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.1.10  netmask 255.255.255.0  broadcast 192.168.1.255
        inet6 fe80::211:22ff:feaa:7637  prefixlen 64  scopeid 0x20<link>
        ether 00:11:22:aa:76:37  txqueuelen 1000  (Ethernet)
        RX packets 767  bytes 54006 (54.0 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 5766  bytes 246466 (246.4 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 43  base 0xa000  

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 3847  bytes 339115 (339.1 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3847  bytes 339115 (339.1 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 08:e9:f6:ae:f8:8a  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

```

## Network Status Confirmation
<iframe width="560" height="315" src="https://www.youtube.com/embed/Of4mN0MaoiU?si=3wALa0UP2gxlhm26" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Before using remote login, it is necessary to ensure that the communication between the computer and the development board is normal. If it cannot be pinged, please follow the steps below to confirm:

- Confirm the IP address configuration of the development board and the computer. Generally, the first three segments need to be the same. For example, the development board: `192.168.1.10` and the computer: `192.168.1.100`.
- Confirm that the subnet mask and gateway configuration of the development board and the computer are consistent.
- Confirm whether the network firewall of the computer is turned off.

The wired Ethernet of the development board is set to use the static IP mode by default, and the IP address is `192.168.1.10`. For the case where the development board and the computer are directly connected to the network, only need to configure the computer as a static IP to ensure that it is in the same network segment as the development board. Taking the WIN10 system as an example, the method to modify the static IP of the computer is as follows:

- Find the corresponding Ethernet device in the network connection and double-click to open it.
- Double-click to open the Internet Protocol Version 4 option.
- Enter the corresponding network parameters in the red box in the figure below and click OK.

![image-20220416110242445](../../../../../static/img/01_Quick_start/image/remote_login/image-20220416110242445.png)

If you want to configure the wired network of the development board to obtain IP dynamically via DHCP mode, please refer to the [Wired Network](../02_System_configuration/01_network_blueteeth.md) chapter for configuration.

## VNC Login

<iframe width="560" height="315" src="https://www.youtube.com/embed/whwi7O2XBKs?si=dwqJ5RjwjhUqyd8y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

This section is for users using the Ubuntu Desktop system version, and it explains how to use "VNC Viewer" to achieve remote desktop login. "VNC Viewer" is a graphical desktop sharing software that allows you to remotely log in and control the desktop of the device on your computer. With this software, you can preview the system desktop of the development board on your computer screen and use your computer's mouse and keyboard for remote operation. By using VNC Viewer, you can achieve the same effect as local operation on the development board. You can download VNC Viewer from the following link: [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/).

**Connect to the Development Board**  
Currently, VNC supports two connection methods: direct connection and cloud connection. Users can choose according to their own needs. This article recommends using the direct connection method. The connection steps are as follows:- Enter the IP address of the input device, for example: 192.168.1.10  
![image-20220610160658103](../../../../../static/img/01_Quick_start/image/remote_login/image-20220610160658103.png)

- After entering the IP address, press Enter, a prompt for an unencrypted connection will appear, click `Continue`  
![image-20220610160715916](../../../../../static/img/01_Quick_start/image/remote_login/image-20220610160715916.png)

- Enter the password `sunrise`, check `Remember password`, and click `OK` to connect  
![image-20220610160928136](../../../../../static/img/01_Quick_start/image/remote_login/image-20220610160928136.png)

## SSH Login {#ssh}
In addition to VNC login for remote desktop, you can also connect to the development board via SSH. The following steps describe how to create SSH connections using terminal software and terminal command line methods.

### Terminal Software
Commonly used terminal tools include `Putty`, `MobaXterm`, etc. Users can choose according to their own preferences. The configuration process for different tools is similar. The following example shows how to create a new SSH connection using `MobaXterm`:

1. Open the `MobaXterm` tool, click on `Session`, then select `SSH`.
2. Enter the development board IP address, for example: `192.168.1.10`.
3. Select `specify username`, enter `sunrise`.
4. After clicking OK, enter the username (sunrise) and password (sunrise) to complete the login.

![image-Network-Login](../../../../../static/img/01_Quick_start/image/remote_login/image-Network-Login.gif)

### Command Line on PC
Users can also use the command line to log in via SSH. The steps are as follows:

1. Open the terminal window and enter the SSH login command, for example: `ssh sunrise@192.168.1.10`.
2. A connection confirmation prompt will appear, enter YES.
3. Enter the password (sunrise) to complete the login.

![image-Cmdline-Linux](../../../../../static/img/01_Quick_start/image/remote_login/linux_login_01.gif)