---
sidebar_position: 6
---

# 以太网性能测试

本章旨在指导如何使用 `iperf3` 工具进行以太网性能测试。可以查阅 [iperf3](https://iperf.fr/iperf-doc.php#3docd) 了解该命令的详细使用说明。

**以太网性能测试的关注点：**

1. **带宽和吞吐量：** 测试以太网的实际可用带宽，以确定以太网在高负载情况下的性能表现。
2. **延迟：** 评估数据在以太网上传输时的延迟，特别是在高负载条件下，延迟的增加可能会对实时应用产生负面影响。
3. **丢包率：** 测试以太网中数据包的丢失率，以确定以太网的稳定性。丢包率过高会影响数据传输的完整性。

## 测试原理

以太网性能测试依赖于客户端和服务器之间的数据传输操作。具体原理如下：
1. **服务器端：** 使用 `iperf3` 监听指定端口，等待客户端连接。服务器通过记录接收的数据量和时间间隔，计算实际带宽和吞吐量。
2. **客户端：** 客户端主动连接服务器，并以指定的速率和间隔发送数据包，模拟网络流量负载。
3. **统计信息：** 测试过程中，`iperf3` 会记录关键数据，包括带宽、延迟、丢包率、重传次数等，帮助分析网络性能。

## 准备工作


 ![Ethernet_usage_diagram.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/images_to_upload/Ethernet_usage_diagram.png)

1. **选择连接方式：** `开发板`- `电脑`直连或者`开发板` - `开发板` 直连。
2. **确定服务器和客户端：** 开发板做客户端和服务端都可以，对端设备可以是个人电脑（PC），也可以是另一块开发板。
3. **确定网段：** 配置为相同网段的 IP 地址。

<div class="note">
<strong> 注意：</strong> 本章节内测试的配置<u>仅供参考</u>，用户需根据其实际硬件情况调整测试配置。
</div>

### 示例配置

在本测试中配置如下：

- **连接方式 ：** `开发板`- `电脑`直连。
- **服务器（PC）：** IP 地址为 `192.168.127.195`
- **客户端（开发板）：** IP 地址为 `192.168.127.10`

执行命令测试 PC 和开发板的以太网连通性：

```bash
ping -I eth0 192.168.127.195
```

执行结果：

```bash
PING 192.168.127.195 (192.168.127.195) from 192.168.127.10 eth0: 56(84) bytes of data.
64 bytes from 192.168.127.195: icmp_seq=1 ttl=128 time=1.54 ms
64 bytes from 192.168.127.195: icmp_seq=2 ttl=128 time=1.28 ms
64 bytes from 192.168.127.195: icmp_seq=3 ttl=128 time=1.57 ms
64 bytes from 192.168.127.195: icmp_seq=4 ttl=128 time=1.40 ms
```

## 测试方法

### 步骤一：启动服务器

在 PC 上使用 `iperf3` 启动服务器。首先，前往[iperf 官网](https://iperf.fr/iperf-download.php) 下载 iperf3 安装包并完成安装。

在 Windows 系统上，通过命令提示窗口（cmd）执行命令：

```bash
iperf3 -s -p 5002
```

启动日志：

```bash
-----------------------------------------------------------
Server listening on 5002
-----------------------------------------------------------
```

客户端连接成功后的日志：

```bash
Accepted connection from 192.168.127.10, port 51592
[  5] local 192.168.127.195 port 5002 connected to 192.168.127.10 port 51598
[ ID] Interval           Transfer     Bitrate
[  5]   0.00-1.00   sec   112 MBytes   937 Mbits/sec
[  5]   1.00-2.01   sec   113 MBytes   940 Mbits/sec
[  5]   2.01-3.01   sec   112 MBytes   941 Mbits/sec
[  5]   3.01-4.01   sec   113 MBytes   942 Mbits/sec
[  5]   4.01-5.01   sec   112 MBytes   941 Mbits/sec
[  5]   5.01-6.01   sec   113 MBytes   942 Mbits/sec
[  5]   6.01-7.01   sec   112 MBytes   942 Mbits/sec
[  5]   7.01-8.01   sec   112 MBytes   941 Mbits/sec
[  5]   8.01-9.00   sec   111 MBytes   941 Mbits/sec
[  5]   8.01-9.00   sec   111 MBytes   941 Mbits/sec
```

### 步骤二：启动客户端

在板端启动 iperf3 客户端，指定要连接的 Server 端的 IP 地址和端口号，然后启动测试。

执行命令：

```bash
iperf3 -c 192.168.127.195 -i 1 -t 600 -p 5002
```

<!-- 命令参数说明： `iperf3 -c [server 端 IP 地址] -i [数据打印间隔时间] -t [总运行时间] -p [对应端口号]`。 -->

命令参数说明：

- `-c `: 指定服务器的 IP 地址。
- `-i `: 数据打印间隔时间（单位：秒）。
- `-t `: 测试总运行时间（单位：秒）。
- `-p `: 指定服务器监听的端口号。

执行日志：

```bash
Connecting to host 192.168.127.195, port 5002
[  5] local 192.168.127.10 port 51598 connected to 192.168.127.195 port 5002
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec   115 MBytes   965 Mbits/sec    0    381 KBytes
[  5]   1.00-2.00   sec   113 MBytes   947 Mbits/sec    0    402 KBytes
[  5]   2.00-3.00   sec   113 MBytes   949 Mbits/sec    0    402 KBytes
[  5]   3.00-4.00   sec   113 MBytes   950 Mbits/sec    0    402 KBytes
[  5]   4.00-5.00   sec   113 MBytes   949 Mbits/sec    0    402 KBytes
[  5]   5.00-6.00   sec   114 MBytes   956 Mbits/sec    0    425 KBytes
[  5]   6.00-7.00   sec   113 MBytes   947 Mbits/sec    0    492 KBytes
[  5]   7.00-8.00   sec   114 MBytes   954 Mbits/sec    0    492 KBytes
```

## 测试标准

### 测试结果

请查阅 [iperf3- 测试结果分析](https://iperf.fr/iperf-doc.php#3doc) 了解输出信息各字段含义。

为获得理想的性能测试结果，请确保以下条件：

- 客户端与服务器通过高质量网线直连。
- 客户端和服务器协商得到 1000M 的速率；
- 测试时，客户端和服务器无其他高负载任务运行。

目前我们测试出来的性能指标数据如下（仅供参考）：

接收带宽：**949Mbits/sec**

发送带宽：**950Mbits/sec**

## 常见问题

### 1. 为什么 `iperf3` 客户端无法连接到服务器？
- **原因分析：**
  1. 客户端和服务器未在同一网段，或 IP 地址配置错误。
  2. 防火墙阻止了 `iperf3` 使用的端口。
  3. 服务器未正确启动或未监听指定端口。

- **解决方法：**
  1. 确保客户端和服务器的 IP 地址处于同一网段，且能通过 `ping` 命令相互通信。
  2. 检查防火墙设置，允许 `iperf3` 使用的端口通信（例如，`5002`）。
  3. 确保服务器端已正确执行 `iperf3 -s` 命令并在监听。

### 2. 为什么测试带宽低于预期值？
- **原因分析：**
  1. 网络硬件如网卡不支持千兆速率，或未协商到千兆速率。
  2. 客户端或服务器运行了其他高负载任务，影响了性能。

- **解决方法：**
  1. 检查设备硬件支持千兆以太网，必要时强制网卡协商为千兆速率。
  2. 测试前，关闭客户端和服务器上可能占用网络或系统资源的其他任务。

### 3. 为什么测试过程中出现高丢包率？
- **原因分析：**
  1. 测试网络环境存在干扰，或网线质量较差。
  2. 测试设备的网络配置（如缓冲区大小）不符合要求。

- **解决方法：**
  1. 使用高质量网线，并确保连接稳固无干扰。
  2. 调整网络配置，例如增大 TCP 缓冲区：
    ```bash
    sysctl net.core.rmem_max #查看接收缓冲区（rmem_max）的最大值
    sysctl net.core.wmem_max #查看发送缓冲区（wmem_max）的最大值
    sysctl -w net.core.rmem_max=2500000 #设置接收缓冲区（rmem_max）的最大值为 2,500,000 字节
    sysctl -w net.core.wmem_max=2500000 #设置发送缓冲区（wmem_max）的最大值为 2,500,000 字节
    ```

### 4. 如何解决 `iperf3` 运行时显示“Address already in use”错误？
- **原因分析：**
  此错误通常是因为服务器端口已经被占用，可能是另一个 `iperf3` 实例未关闭。

- **解决方法：**
  1. 确保服务器端未运行其他 `iperf3` 实例，或使用不同端口启动。
     ```bash
     iperf3 -s -p <新端口号>
     ```
  2. 检查并结束占用端口的进程：
     ```bash
     netstat -tuln | grep 5002
     kill <进程 ID>
     ```
<div class="note">
<strong> 提示：</strong> 若以上方法未能解决问题，请参考 [iperf3 官方文档](https://iperf.fr/) 或相关技术论坛，获取更多支持。
</div>
