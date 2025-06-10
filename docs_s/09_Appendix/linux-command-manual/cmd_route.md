---
sidebar_position: 1
---

# route

**route命令** 用来显示并设置Linux内核中的网络路由表，route命令设置的路由主要是静态路由。要实现两个不同的子网之间的通信，需要一台连接两个网络的路由器，或者同时位于两个网络的网关来实现。

在Linux系统中设置路由通常是为了解决以下问题：

该Linux系统在一个局域网中，局域网中有一个网关，能够让机器访问Internet，那么就需要将这台机器的ip地址设置为Linux机器的默认路由。要注意的是，直接在命令行下执行route命令来添加路由，不会永久保存，当网卡重启或者机器重启之后，该路由就失效了；可以在`/etc/rc.local`中添加route命令来保证该路由设置永久有效。

## 语法说明

```
route [-nNvee] [-FC] [<AF>]           List kernel routing tables
route [-v] [-FC] {add|del|flush} ...  Modify routing table for AF.
```

- `-A`：设置地址类型。
- `-v, --verbose`：显示详细信息。
- `-n, --numeric`：不执行DNS反向查找，直接显示数字形式的IP地址。
- `-e, --extend`：netstat格式显示路由表。
- `-F, --fib`：显示转发信息库（默认）。
- `-C, --cache`：显示路由缓存，而不是转发信息库。
- `-net`：到一个网络的路由表。
- `-host`：到一个主机的路由表。

## 选项说明

- `add`：用于添加指定的路由记录，将指定的目的网络或目的主机路由到指定的网络接口。
- `del`：用于删除指定的路由记录。
- `target`：指定目的网络或目的主机。
- `gw`：用于设置默认网关。
- `mss`：设置TCP的最大区块长度（MSS）。
- `window`：指定通过路由表的TCP连接的TCP窗口大小。
- `dev`：指定路由记录所表示的网络接口。

## 常用命令

显示当前路由

```
root@ubuntu:~# route
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
default         192.168.0.1     0.0.0.0         UG    600    0        0 wlan0
default         192.168.1.1     0.0.0.0         UG    700    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     600    0        0 wlan0
192.168.1.0     0.0.0.0         255.255.255.0   U     700    0        0 eth0

root@ubuntu:~# route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.1     0.0.0.0         UG    600    0        0 wlan0
0.0.0.0         192.168.1.1     0.0.0.0         UG    700    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     600    0        0 wlan0
192.168.1.0     0.0.0.0         255.255.255.0   U     700    0        0 eth0
```

其中Flags为路由标志，标记当前网络节点的状态，Flags标志说明：

- `U`： Up表示此路由当前为启动状态。
- `H`： Host，表示此网关为一主机。
- `G`：Gateway，表示此网关为一路由器。
- `R`：Reinstate Route，使用动态路由重新初始化的路由。
- `D`： Dynamically,此路由是动态性地写入。
- `M`： Modified，此路由是由路由守护程序或导向器动态修改。
- `!`： 表示此路由当前为关闭状态。

添加网关/设置网关

```
route add -net 192.168.2.0 netmask 255.255.255.0 dev eth0
```

屏蔽一条路由

```shell
route add -net 192.168.2.0 netmask 255.255.255.0 reject
```

删除路由记录

```shell
route del -net 192.168.2.0 netmask 255.255.255.0
route del -net 192.168.2.0 netmask 255.255.255.0 reject
```

删除和添加设置默认网关

```shell
route del default gw 192.168.2.1
route add default gw 192.168.2.1
```

