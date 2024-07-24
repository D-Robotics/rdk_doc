---
sidebar_position: 1
---

# ssh

**ssh命令** 是openssh套件中的客户端连接工具，可以给予ssh加密协议实现安全的远程登录服务器。

## 语法说明

```
ssh [-46AaCfGgKkMNnqsTtVvXxYy] [-B bind_interface]
           [-b bind_address] [-c cipher_spec] [-D [bind_address:]port]
           [-E log_file] [-e escape_char] [-F configfile] [-I pkcs11]
           [-i identity_file] [-J [user@]host[:port]] [-L address]
           [-l login_name] [-m mac_spec] [-O ctl_cmd] [-o option] [-p port]
           [-Q query_option] [-R address] [-S ctl_path] [-W host:port]
           [-w local_tun[:remote_tun]] destination [command]
```

- **destination**：指定要连接的远程ssh服务器；
- **command**：要在远程ssh服务器上执行的指令。

## 选项说明

- `-4`：强制使用IPv4地址。
- `-6`：强制使用IPv6地址。
- `-A`：开启认证代理连接转发功能。
- `-a`：关闭认证代理连接转发功能。
- `-B`：在尝试连接到目标主机之前，绑定到`bind_interface`的地址。这在具有多个地址的系统上非常有用。
- `-b`：使用本地指定地址作为对应连接的源IP地址。
- `-C`：请求压缩所有数据。
- `-F`：指定SSH指令的配置文件。
- `-f`：在后台执行SSH指令。
- `-g`：允许远程主机连接主机的转发端口。
- `-i`：指定身份（私钥）文件。
- `-l`：指定连接远程服务器的登录用户名。
- `-N`：不执行远程指令。
- `-o`：指定配置选项。
- `-p`：指定远程服务器上的端口。
- `-q`：静默模式。
- `-X`：开启X11转发功能。
- `-x`：关闭X11转发功能。
- `-y`：开启信任X11转发功能。

## 常用命令

```shell
# ssh 用户名@远程服务器地址
ssh sunrise@192.168.1.10
# 指定端口
ssh -p 2211 sunrise@192.168.1.10

# ssh 大家族
ssh -p 22 user@ip  # 默认用户名为当前用户名，默认端口为 22
ssh-keygen # 为当前用户生成 ssh 公钥 + 私钥
ssh-keygen -f keyfile -i -m key_format -e -m key_format # key_format: RFC4716/SSH2(default) PKCS8 PEM
ssh-copy-id user@ip:port # 将当前用户的公钥复制到需要 ssh 的服务器的 ~/.ssh/authorized_keys，之后可以免密登录
```

连接远程服务器

```shell
ssh username@remote_host
```

连接远程服务器并指定端口

```shell
ssh -p port username@remote_host
```

使用密钥文件连接远程服务器

```shell
ssh -i path/to/private_key username@remote_host
```

在本地执行远程命令

```shell
ssh username@remote_host "command"
```

在本地端口转发到远程服务器

```shell
ssh -L local_port:remote_host:remote_port username@remote_host
```

在远程服务器端口转发到本地

```shell
ssh -R remote_port:local_host:local_port username@remote_host
```
