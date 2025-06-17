---
sidebar_position: 1
---

# scp

Linux scp 命令用于 Linux 之间复制文件和目录。

scp 是 secure copy 的缩写, scp 是 linux 系统下基于 ssh 登陆进行安全的远程文件拷贝命令。

scp 是加密的，[rcp](https://www.runoob.com/linux/linux-comm-rcp.html) 是不加密的，scp 是 rcp 的加强版。

## 语法说明

```
scp [-346BCpqrTv] [-c cipher] [-F ssh_config] [-i identity_file]
            [-J destination] [-l limit] [-o ssh_option] [-P port]
            [-S program] source ... target
```

简易写法:

```
scp [option] file_source file_target 
```

- **file_source**：指定要复制的源文件。
- **file_target**：目标文件。格式为`user@host：filename`（文件名为目标文件的名称）。

## 选项说明

- -3：通过本地主机传输两个远程主机之间的文件。如果不使用此选项，数据将直接在两个远程主机之间传输。请注意，此选项会禁用传输进度显示。
- -4： 强制scp命令只使用IPv4寻址
- -6： 强制scp命令只使用IPv6寻址
- -B： 选择批处理模式（防止询问密码或密码短语）
- -C： 允许压缩。（将-C标志传递给ssh，从而打开压缩功能）
- -p：保留原文件的修改时间，访问时间和访问权限。
- -q： 静默模式：禁用进度表以及来自ssh(1)的警告和诊断消息。
- -r： 递归地复制整个目录。请注意，scp会遵循树遍历中遇到的符号链接。、
- -T：禁用严格的文件名检查。默认情况下，当从远程主机复制文件到本地目录时，scp会检查接收的文件名是否与命令行上请求的文件名匹配，以防止远程端发送意外或不需要的文件。由于不同操作系统和Shell解释文件名通配符的方式不同，这些检查可能会导致希望的文件被拒绝。此选项禁用这些检查，但需要完全信任服务器不会发送意外的文件名。
- -v：详细模式。导致scp和ssh(1)打印关于它们的进展的调试消息。这在调试连接、身份验证和配置问题时很有帮助。
- -c cipher： 选择用于加密数据传输的密码。此选项直接传递给ssh(1)。
- -F ssh_config： 指定用于ssh的替代每用户配置文件。此选项直接传递给ssh(1)。
- -i identity_file： 选择用于公钥身份验证的身份（私钥）文件。此选项直接传递给ssh(1)。
- -l limit： 限制使用的带宽，以Kbit/s为单位。
- -o ssh_option： 可以用于以ssh_config(5)中使用的格式传递选项给ssh。这对于指定没有单独的scp命令行标志的选项非常有用。
- -P port：指定要连接到远程主机的端口。请注意，此选项使用大写的 'P'，因为小写的 '-p' 已经被保留用于保留文件的修改时间和模式。
- -S program： 用于加密连接的程序名称。该程序必须理解ssh(1)选项。

## 常用命令

**从本地复制到远程**

命令格式：

```
scp local_file remote_username@remote_ip:remote_folder 
或者 
scp local_file remote_username@remote_ip:remote_file 
或者 
scp local_file remote_ip:remote_folder 
或者 
scp local_file remote_ip:remote_file 
```

- 第1,2个指定了用户名，命令执行后需要再输入密码，第1个仅指定了远程的目录，文件名字不变，第2个指定了文件名；
- 第3,4个没有指定用户名，命令执行后需要输入用户名和密码，第3个仅指定了远程的目录，文件名字不变，第4个指定了文件名；

应用实例：

```
scp /home/sunrise/test.c root@192.168.1.10:/userdata 
scp /home/sunrise/test.c root@192.168.1.10:/userdata/test_01.c
scp /home/sunrise/test.c 192.168.1.10:/userdata
scp /home/sunrise/test.c 192.168.1.10:/userdata/test_01.c
```

复制目录命令格式：

```
scp -r local_folder remote_username@remote_ip:remote_folder 
或者 
scp -r local_folder remote_ip:remote_folder 
```

- 第1个指定了用户名，命令执行后需要再输入密码；
- 第2个没有指定用户名，命令执行后需要输入用户名和密码；

应用实例：

```
scp -r /home/sunrise/app/ root@192.168.1.10:/userdata/app/ 
scp -r /home/sunrise/app/ 192.168.1.10:/userdata/app/ 
```

上面命令将本地 `app` 目录复制到远程`/userdata/app/`目录下。

**从远程复制到本地**

从远程复制到本地的scp命令与上面的命令雷同，只要将从本地复制到远程的命令后面2个参数互换顺序就行了。

从远程机器复制文件到本地目录

```shell
scp sunrise@192.168.1.10:/userdata/log.log /home/sunrise/
```

从192.168.1.10机器上的`/userdata/`的目录中下载`log.log` 文件到本地`/home/sunrise/`目录中。
