---
sidebar_position: 1
---

# ifconfig

**ifconfig** 是一个用于配置和管理网络接口的命令。它允许用户查看和修改网络接口的配置，包括 IP 地址、子网掩码、MAC 地址、MTU、广播地址、点对点地址等。

## 语法说明

```
ifconfig [-a] [-v] [-s] <interface> [[<AF>] <address>]
  [add <address>[/<prefixlen>]]
  [del <address>[/<prefixlen>]]
  [[-]broadcast [<address>]]  [[-]pointopoint [<address>]]
  [netmask <address>]  [dstaddr <address>]  [tunnel <address>]
  [outfill <NN>] [keepalive <NN>]
  [hw <HW> <address>]  [mtu <NN>]
  [[-]trailers]  [[-]arp]  [[-]allmulti]
  [multicast]  [[-]promisc]
  [mem_start <NN>]  [io_addr <NN>]  [irq <NN>]  [media <type>]
  [txqueuelen <NN>]
  [[-]dynamic]
  [up|down] ...
```

## 选项说明

- `ifconfig`：显示所有已配置激活的网络接口及其状态。

- `ifconfig -a`：显示所有网络接口，包括那些没有激活的。
- `ifconfig <interface>`：显示指定网络接口的配置。
- `ifconfig <interface> up`：激活指定网络接口。
- `ifconfig <interface> down`：停用指定网络接口。
- `ifconfig <interface> add <address>`：为指定网络接口添加 IP 地址。
- `ifconfig <interface> del <address>`：从指定网络接口删除 IP 地址。
- `ifconfig <interface> netmask <address>`：设置指定网络接口的子网掩码。
- `ifconfig <interface> broadcast <address>`：设置广播地址。
- `ifconfig <interface> pointopoint <address>`：设置点对点地址。
- `ifconfig <interface> hw <HW> <address>`：设置 MAC 地址。
- `ifconfig <interface> mtu <NN>`：设置 MTU（最大传输单元）。
- `ifconfig <interface> arp`：启用 ARP（地址解析协议）。
- `ifconfig <interface> promisc`：启用混杂模式，接收所有流经网络接口的数据包。
- `ifconfig <interface> multicast`：启用组播模式。
- `ifconfig <interface> dynamic`：启用动态配置。
- `ifconfig -s`：以简洁格式显示网络接口信息。
- `ifconfig -v`：显示详细信息。

## 常用命令

常用命令

```
ifconfig   #处于激活状态的网络接口
ifconfig -a  #所有配置的网络接口，不论其是否激活
ifconfig eth0  #显示eth0的网卡信息
```

启动、关闭指定网卡

```shell
ifconfig eth0 up
ifconfig eth0 down
```

配置IP地址

```shell
ifconfig eth0 192.168.1.10
ifconfig eth0 192.168.1.10 netmask 255.255.255.0
ifconfig eth0 192.168.1.10 netmask 255.255.255.0 broadcast 192.168.1.255
```
