---
sidebar_position: 9
---

# USB 总线速率测试

## 测试原理

USB 速率测试的基本原理是通过测量 USB 设备与主机之间的数据传输速率，计算数据传输时间，以此来测量实际的总线速率。

### 测试内容

USB 总线速率测试分别使用了 USB 虚拟网口和 dd 命令对 U 盘进行读写操作，这两种测试方法用于评估 USB 2.0 和 USB 3.0 的传输速率。

#### dd 命令原理说明

使用 `dd` 命令测试 USB 总线速率的原理涉及到 **磁盘 I/O 性能测试**，虽然它并不直接测量 USB 总线本身的速度，但通过测试与 USB 设备进行数据读写时的吞吐量，可以间接推测出 USB 总线的性能瓶颈。

```shell
dd if=/dev/zero of=/mnt/usb/testfile bs=1M count=1024 conv=sync oflag=direct
```

- `if=/dev/zero`：输入文件是 /dev/zero，它会生成连续的零数据。
- `of=/mnt/usb/testfile`：输出文件为 USB 存储设备的挂载点下的 testfile，即将数据写入 U 盘。
- `bs=1M`：块大小为 1MB。每次操作读取或写入 1MB 数据。
- `count=1024` ：共写入 1024 个块，即 1GB 的数据。
- `conv=sync`：这个参数用于处理块对齐。如果输入的数据块大小不足 bs（在此示例中是 4KB）， conv=sync 会将缺少的部分用零填充，以确保每个块的大小都是 4KB。
- `oflag=direct`：避免缓存的影响，直接写入 U 盘。

```shell
dd if=/path/to/source_file of=/dev/null bs=4K iflag=direct
```

- `if=/path/to/source_file`: if( 输入文件 ) 指定输入文件的路径，/path/to/source_file 是要从中读取数据的文件。
- `of=/dev/null`: of（输出文件）：指定输出文件的位置，这里将数据写入 /dev/null。
- `bs=4K`: bs（块大小）：设置每次读取和写入的数据块大小为 4KB（ 4096 字节）。
- `iflag=direct`: direct 标志确保数据直接从 U 盘读取，而不经过操作系统的缓存。

通过执行 dd 读写命令，系统会把零数据写入到 U 盘，测试写入过程中的吞吐量。命令执行后，系统会显示写入的字节数和所需时间，从而计算出写入速度， dd 读命令同理。

#### usb 虚拟网口原理说明

设置 USB 口工作在 device 模式，使能 rndis 驱动，连接到个人电脑后，会在电脑上生成一个远程网卡。配置好板端和电脑上的网络 IP 地址后，使用 iperf3 工具进行速率带宽的测试。

## 准备工作

### U 盘读写准备方法

**1.** 将 U 盘插入 RDK_S100 开发板 USB3.0 接口，可输入命令查看 U 盘接口类型：

```shell
root@ubuntu:~# lsusb
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 002: ID 24a9:205a          MoveSpeed YD 3.1
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

lsblk 识别到了 /dev/sda1 u盘设备

```shell
root@ubuntu:~# lsblk
NAME         MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
sda            8:0    1 117.2G  0 disk
`-sda1         8:1    1 117.2G  0 part
mmcblk0      179:0    0  58.2G  0 disk
|-mmcblk0p1  179:1    0     1M  0 part
|-mmcblk0p2  179:2    0     1M  0 part
|-mmcblk0p3  179:3    0     1M  0 part
|-mmcblk0p4  179:4    0     2M  0 part
|-mmcblk0p5  179:5    0     2M  0 part
|-mmcblk0p6  179:6    0     2M  0 part
|-mmcblk0p7  179:7    0     2M  0 part
|-mmcblk0p8  179:8    0     8M  0 part
|-mmcblk0p9  179:9    0     8M  0 part
|-mmcblk0p10 179:10   0     8M  0 part
|-mmcblk0p11 179:11   0     8M  0 part
|-mmcblk0p12 179:12   0    60M  0 part /boot
|-mmcblk0p13 179:13   0    60M  0 part
|-mmcblk0p14 179:14   0     8G  0 part /ota
|-mmcblk0p15 179:15   0     4G  0 part /log
|-mmcblk0p16 179:16   0     2G  0 part /userdata
`-mmcblk0p17 179:17   0  44.1G  0 part /
mmcblk0boot0 179:32   0     4M  1 disk
mmcblk0boot1 179:64   0     4M  1 disk
```

**2.** 挂载 /dev/sda1 设备到 /mnt/usb

```bash
mount  /dev/sda1 /mnt/usb
```

通过 lsusb 命令发现， BUS2 ： ID 1d6b:205a 表示这是一个 USB 3.1 控制器，新插入的 U 盘挂载在 BUS2 上，因此可以确认该 U 盘为 USB 3.1 接口。

## 测试方法

输入命令 lsblk 可以查看 U 盘的挂载路径是否正确。

```bash
root@ubuntu:~# lsblk
NAME         MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
sda            8:0    1 117.2G  0 disk
├─sda1         8:1    1 117.2G  0 part /mnt/usb
mmcblk0      179:0    0  58.2G  0 disk
├─mmcblk0p1   179:1    0     1M  0 part
├─mmcblk0p2   179:2    0     1M  0 part
├─mmcblk0p3   179:3    0     1M  0 part
├─mmcblk0p4   179:4    0     2M  0 part
├─mmcblk0p5   179:5    0     2M  0 part
├─mmcblk0p6   179:6    0     2M  0 part
├─mmcblk0p7   179:7    0     2M  0 part
├─mmcblk0p8   179:8    0     8M  0 part
├─mmcblk0p9   179:9    0     8M  0 part
├─mmcblk0p10  179:10   0     8M  0 part
├─mmcblk0p11  179:11   0     8M  0 part
├─mmcblk0p12  179:12   0    60M  0 part /boot
├─mmcblk0p13  179:13   0    60M  0 part
├─mmcblk0p14  179:14   0     8G  0 part /ota
├─mmcblk0p15  179:15   0     4G  0 part /log
├─mmcblk0p16  179:16   0     2G  0 part /userdata
└─mmcblk0p17  179:17   0  44.1G  0 part /
mmcblk0boot0 179:32   0     4M  1 disk
mmcblk0boot1 179:64   0     4M  1 disk
```

输入 dd 命令测试 usb 总线的写速率：

```shell
root@ubuntu:~# dd if=/dev/zero of=/mnt/usb/myfile bs=4K count=256K conv=sync
262144+0 records in
262144+0 records out
1073741824 bytes (1.1 GB, 1.0 GiB) copied, 4.19208 s, 256 MB/s
```

输入 dd 命令测试 usb 总线的读速率：

```shell
root@ubuntu:~# dd if=/mnt/usb/myfile of=/dev/null bs=4K count=256K
262144+0 records in
262144+0 records out
1073741824 bytes (1.1 GB, 1.0 GiB) copied, 10.5222 s, 102 MB/s
```

## 准备工作

### usb 虚拟网口准备方法

**1.** 将 RDK_S100 开发板的 usb2.0 口连接至 PC 的 USB2.0 ，如图：

![S100_USB2.0](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/S100_USB2.0.png)

**2.** 将 usb2.0 虚拟为网口，命令如下：

```shell
usb-gadget.sh stop adb
usb-gadget.sh start rndis
```

执行输出：

```shell
USB2.0 Gadget
Detecting platform:
 board : D-Robotics RDK S100 V0P5
 udc   : 39820000.dwc3
Stopping the USB gadget
Stoping & Delete usb-gadget g_comp
waiting...
... ( 省略 ) ...
OK
Bind functions...
Bind functions according to .usb-config file
bind gadget rndis...
Creating RNDIS gadget functionality
OK
Pre run userspace daemons(eg. adb)...
0
waiting
.
OK
Binding USB Device Controller
OK
Run some userspace daemons(eg. usb_camera)...
usb-gadget start succeed.
```

在电脑上可以查看网络配置页面，可以看到如下图所示的 `Remote RNIS Compatible Device` 网卡。

![RNIS_Device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/RNIS_Device.png)

**3.** 使用以下命令来配置 usb0 网络接口的 IP 地址，使其与 PC 端远程网卡的 IP 地址网段一致。

```shell
ifconfig usb0 192.168.1.110
```

PC 端远程网卡 ip 配置如图：

![NETWORK_CONFIG](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/NETWORK_CONFIG.png)

## 测试方法

在 pc 端运行 iperf3 启动 Server，通过命令提示符（cmd）运行以下命令启动服务器：

```bash
iperf3 -s -p 5002
```

在 RDK_S100 开发板运行 iperf3 启动客户端，设置 Server 端一样的 ip 地址和 port 号连接 Server 启动测试，执行命令如下：

```shell
root@ubuntu:~# iperf3 -c 192.168.1.111 -i 1 -t 600 -p 5002
Connecting to host 192.168.1.111, port 5002
[  5] local 192.168.5.10 port 57234 connected to 192.168.5.12 port 5002
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec  34.4 MBytes   289 Mbits/sec    0   67.0 KBytes
[  5]   1.00-2.00   sec  33.8 MBytes   284 Mbits/sec    0   67.0 KBytes
[  5]   2.00-3.00   sec  34.0 MBytes   285 Mbits/sec    0   67.0 KBytes
[  5]   3.00-4.00   sec  34.0 MBytes   285 Mbits/sec    0   67.0 KBytes
```

此时客户端如果成功连接， pc 服务端打印如下：

```shell
Accepted connection from 192.168.1.110, port 57220
[  5] local 192.168.1.111 port 5002 connected to 192.168.1.110 port 57234
```

## 测试指标

USB 总线的理论速率（最高）：

- usb 2.0 ： 480Mbps 即 60MB/s
- USB 3.1 Gen 1 (即 USB 3.0)： 5Gbps 即 625MB/s
- USB 3.1 Gen 2：10 Gbps 即 1250MB/s

实际的传输速率通常会低于理论最大值，由于传输过程中存在协议开销、硬件限制、以及设备性能等因素， USB 2.0 的实际传输速率通常在 25 MB/s 到 35 MB/s 之间， USB 3.1 Gen 1的实际传输速率在 40 MB/s 到 200 MB/s 之间，USB 3.1 Gen 2实际传输速度通常在 600 MB/s 到 900 MB/s 之间。

### 常见影响因素

- **USB 版本**： USB 3.1 Gen 1 、 USB 3.1 Gen 2 和 usb 2.0 的带宽差异会直接影响数据传输速率。如果你在 USB 2.0 接口上使用设备，理论最大速度为 480 Mbps，实际速度可能更低。
- **设备类型**： USB 设备（如 U 盘、外接硬盘、 SSD）具有不同的读写性能，尤其是闪存类型的 U 盘速度较慢，而 SSD 性能更好。
- **块大小 (`bs`)**：`dd` 命令的块大小选择对测试结果有很大影响，较大的块大小可能提高吞吐量，而过小的块大小可能导致效率低下，可适当更改 dd 命令参数来达到更好的测试效果。

### 测试结果

通过测试一和测试二的方法，可以有效地测试出 USB 3.1 Gen 1 和 USB 2.0 的实际传输速率，并与 USB 在实际使用中的速率结果相符。
