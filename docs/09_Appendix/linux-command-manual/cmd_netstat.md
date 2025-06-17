---
sidebar_position: 1
---

# netstat

**netstat命令** 用来打印Linux中网络系统的状态信息，可让你得知整个系统的网络情况。

## 语法说明

```
netstat [-vWeenNcCF] [<Af>] -r         
netstat {-V|--version|-h|--help}
netstat [-vWnNcaeol] [<Socket> ...]
netstat { [-vWeenNac] -i | [-cnNe] -M | -s [-6tuw] }
```

## 选项说明

- `-A`： 列出该网络类型连线中的相关地址。
- `-r, --route`：显示路由表，列出系统的路由信息。
- `-i, --interfaces`：显示网络接口信息，包括接口名称、IP 地址和其他相关信息。
- `-g, --groups`：显示组播组成员信息，包括哪些网络组成员在组播组中。
- `-s, --statistics`：显示网络统计信息，类似于 SNMP（Simple Network Management Protocol），提供有关网络活动的详细统计。
- `-M, --masquerade`：显示伪装（masquerade）连接信息，通常用于 Network Address Translation (NAT) 网络。
- `-v, --verbose`：显示详细信息，提供更多的信息以帮助诊断网络问题。
- `-W, --wide`：不截断 IP 地址，以显示完整的 IP 地址信息。
- `-n, --numeric`：不解析主机名或端口名，以显示数字格式的 IP 地址、端口号和用户信息。
- `--numeric-hosts`：不解析主机名。
- `--numeric-ports`：不解析端口名。
- `--numeric-users`：不解析用户名称。
- `-N, --symbolic`：解析硬件名，显示硬件设备的符号名称。
- `-e, --extend`：显示附加信息。使用此选项两次以获取最大的详细信息。
- `-p, --programs`：显示 PID（进程标识符）和程序名称，以显示与套接字相关的进程信息。
- `-o, --timers`：显示计时器信息，包括套接字的计时器状态。
- `-c, --continuous`：使 `netstat` 持续打印所选信息，每秒一次，以进行连续监控。
- `-l, --listening`：只显示正在监听的服务器套接字。
- `-a, --all`：显示所有套接字，包括已连接和未连接的。
- `-F, --fib`：显示转发信息表（FIB）。
- `-C, --cache`：显示路由缓存，而不是转发信息表。
- `-Z, --context`：显示 SELinux 安全上下文，用于显示套接字的 SELinux 安全信息。
- `-v, --verbose`：启用详细输出，向用户提供有关正在进行的操作的更多信息。特别是在处理未配置的地址族时，提供一些有用的信息。
- `-o, --timers`：包括与网络计时器相关的信息。
- `-p, --program`：显示每个套接字所属的程序的 PID 和名称。
- `-l, --listening`：仅显示正在监听的套接字。默认情况下，这些被省略。
- `-a, --all`：显示监听和非监听的套接字。在使用 `--interfaces` 选项时，显示未启用的接口。
- `-F`：从 FIB 打印路由信息（默认）。
- `-C`：从路由缓存中打印路由信息。

## 常用命令

显示详细的网络状况

```
netstat -a	   #列出所有端口
netstat -at    #列出所有tcp端口
netstat -au    #列出所有udp端口   
```

显示当前户籍UDP连接状况

```
netstat -nu
```

显示UDP端口号的使用情况

```
netstat -apu
```

显示网卡列表

```
netstat -i
```

显示组播组的关系

```
netstat -g
```

显示网络统计信息

```
netstat -s   显示所有端口的统计信息
netstat -st   显示TCP端口的统计信息
netstat -su   显示UDP端口的统计信息
```

显示监听的套接口

```
netstat -l        #只显示监听端口
netstat -lt       #只列出所有监听 tcp 端口
netstat -lu       #只列出所有监听 udp 端口
netstat -lx       #只列出所有监听 UNIX 端口
```

在netstat输出中显示 PID 和进程名称

```
netstat -pt
```

`netstat -p`可以与其它开关一起使用，就可以添加“PID/进程名称”到netstat输出中。

持续输出netstat信息

```
netstat -c   #每隔一秒输出网络信息
```

显示核心路由信息

```shell
netstat -r
```

使用`netstat -rn`显示数字格式，不查询主机名称。

找出程序运行的端口

并不是所有的进程都能找到，没有权限的会不显示，使用 root 权限查看所有的信息。

```shell
netstat -ap | grep ssh
```

找出运行在指定端口的进程

```shell
netstat -an | grep ':80'
```

通过端口找进程ID

```bash
netstat -anp|grep 8081 | grep LISTEN|awk '{printf $7}'|cut -d/ -f1
```
