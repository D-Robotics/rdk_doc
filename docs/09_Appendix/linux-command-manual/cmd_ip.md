---
sidebar_position: 1
---

# ip

**ip** 命令与`ifconfig`命令类似，但比 ifconfig 命令更加强大，主要功能是用于显示或设置网络设备。

**ip** 命令是 Linux 加强版的的网络配置工具，用于代替 ifconfig 命令。

## 语法说明

```
ip [ OPTIONS ] OBJECT { COMMAND | help }
ip [ -force ] -batch filename
```

- **OBJECT**：

  ```shell
  OBJECT := { link | address | addrlabel | route | rule | neigh | ntable |
         tunnel | tuntap | maddress | mroute | mrule | monitor | xfrm |
         netns | l2tp | macsec | tcp_metrics | token }
         
  -V：显示指令版本信息；
  -s：输出更详细的信息；
  -f：强制使用指定的协议族；
  -4：指定使用的网络层协议是IPv4协议；
  -6：指定使用的网络层协议是IPv6协议；
  -0：输出信息每条记录输出一行，即使内容较多也不换行显示；
  -r：显示主机时，不使用IP地址，而使用主机的域名。
  ```

- **OPTIONS**：

  ```shell
  OPTIONS := { -V[ersion] | -s[tatistics] | -d[etails] | -r[esolve] |
          -h[uman-readable] | -iec |
          -f[amily] { inet | inet6 | ipx | dnet | bridge | link } |
          -4 | -6 | -I | -D | -B | -0 |
          -l[oops] { maximum-addr-flush-attempts } |
          -o[neline] | -t[imestamp] | -ts[hort] | -b[atch] [filename] |
          -rc[vbuf] [size] | -n[etns] name | -a[ll] }
          
  网络对象：指定要管理的网络对象；
  具体操作：对指定的网络对象完成具体操作；
  help：显示网络对象支持的操作命令的帮助信息。
  ```

------

## 常用命令

```shell
ip link show                     # 显示网络接口信息
ip link set eth0 up             # 开启网卡
ip link set eth0 down            # 关闭网卡
ip link set eth0 promisc on      # 开启网卡的混合模式
ip link set eth0 promisc offi    # 关闭网卡的混个模式
ip link set eth0 txqueuelen 1200 # 设置网卡队列长度
ip link set eth0 mtu 1400        # 设置网卡最大传输单元

ip addr show     # 显示网卡IP信息
ip addr add 192.168.0.1/24 dev eth0 # 设置eth0网卡IP地址192.168.0.1
ip addr del 192.168.0.1/24 dev eth0 # 删除eth0网卡IP地址

ip route show # 显示系统路由
ip route add default via 192.168.1.254   # 设置系统默认路由
ip route list                 # 查看路由信息
ip route add 192.168.1.0/24  via  192.168.0.254 dev eth0 # 设置192.168.4.0网段的网关为192.168.0.254,数据走eth0接口
ip route add default via  192.168.0.254  dev eth0        # 设置默认网关为192.168.0.254
ip route del 192.168.1.0/24   # 删除192.168.4.0网段的网关
ip route del default          # 删除默认路由
ip route delete 192.168.1.0/24 dev eth0 # 删除路由
```

获取主机所有网络接口

```shell
ip link | grep -E '^[0-9]' | awk -F: '{print $2}'
```
