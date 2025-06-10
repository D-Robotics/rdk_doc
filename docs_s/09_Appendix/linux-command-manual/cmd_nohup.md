---
sidebar_position: 1
---

# nohup

**nohup** 英文全称 no hang up（不挂起），用于在系统后台不挂断地运行命令，退出终端不会影响程序的运行。

在默认情况下（非重定向时），会输出一个名叫 nohup.out 的文件到当前目录下。如果当前目录的 nohup.out 文件不可写，输出重定向到`$HOME/nohup.out`文件中。如果没有文件能创建或打开以用于追加，那么 command 参数指定的命令不可调用。如果标准错误是一个终端，那么把指定的命令写给标准错误的所有输出作为标准输出重定向到相同的文件描述符。

## 语法说明

```
nohup COMMAND [ARG]... [　& ]
nohup OPTION
```

**COMMAND**：要执行的命令。

**ARG**：一些参数，可以指定输出文件。

**&**：让命令在后台执行，终端退出后命令仍旧执行。

## 选项说明

- `--help`：显示帮助信息。
- ` --version`：显示版本信息。

## 常用命令

以下命令在后台执行 root 目录下的 runoob.sh 脚本

```
nohup /root/runoob.sh &
```

如果要停止运行，你需要使用以下命令查找到 nohup 运行脚本到 PID，然后使用 kill 命令来删除

```
ps -aux | grep "runoob.sh" 
```

以下命令在后台执行 root 目录下的 runoob.sh 脚本，并重定向输入到 runoob.log 文件：

```
nohup /root/runoob.sh > runoob.log 2>&1 &
```

**2>&1** 解释：

将标准错误 2 重定向到标准输出 &1 ，标准输出 &1 再被重定向输入到 runoob.log 文件中。

- 0 – stdin (standard input，标准输入)
- 1 – stdout (standard output，标准输出)
- 2 – stderr (standard error，标准错误输出)
