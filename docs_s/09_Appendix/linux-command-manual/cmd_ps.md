---
sidebar_position: 1
---

# ps

**ps命令** 用于显示当前系统的进程状态。ps命令是最基本同时也是非常强大的进程查看命令，使用该命令可以确定有哪些进程正在运行和运行的状态、进程是否结束、进程有没有僵死、哪些进程占用了过多的资源等。

## 语法说明

```
ps [options]
```

## 选项说明

**Basic options（基本选项）:**

- `-A, -e`: 包括所有进程。
- `-a`: 包括所有具有终端（tty）的进程，但不包括会话领导者。
- `a`: 包括所有具有终端（tty）的进程，包括其他用户的进程。
- `-d`: 包括除会话领导者之外的所有进程。
- `-N, --deselect`: 取消选择（反选）进程。
- `r`: 仅显示正在运行的进程。
- `T`: 显示与当前终端相关的所有进程。
- `x`: 显示没有控制终端的进程。

**Selection by list（根据列表选择）:**

- `-C <command>`: 根据命令名选择进程。
- `-G, --Group <GID>`: 根据真实组 ID 或组名选择进程。
- `-g, --group <group>`: 根据会话或有效组名选择进程。
- `-p, p, --pid <PID>`: 根据进程 ID 选择进程。
- `--ppid <PID>`: 根据父进程 ID 选择进程。
- `-q, q, --quick-pid <PID>`: 快速模式下，根据进程 ID 选择进程。
- `-s, --sid <session>`: 根据会话 ID 选择进程。
- `-t, t, --tty <tty>`: 根据终端选择进程。
- `-u, U, --user <UID>`: 根据有效用户 ID 或用户名选择进程。
- `-U, --User <UID>`: 根据真实用户 ID 或用户名选择进程。

**Output formats（输出格式）:**

- `-F`: 显示额外详细信息。
- `-f`: 完整格式，包括命令行。
- `f, --forest`: 以 ASCII 艺术的方式显示进程树。
- `-H`: 显示进程层次结构。
- `-j`: 作业格式。
- `j`: BSD 作业控制格式。
- `-l`: 长格式。
- `l`: BSD 长格式。
- `-M, Z`: 添加安全数据（针对 SELinux）。
- `-O <format>`: 使用默认列预加载。
- `O <format>`: 以 BSD 风格预加载列。
- `-o, o, --format <format>`: 用户自定义格式。
- `s`: 信号格式。
- `u`: 面向用户的格式。
- `v`: 虚拟内存格式。
- `X`: 寄存器格式。
- `-y`: 不显示标志，显示 RSS 与地址（与 -l 一起使用）。
- `--context`: 显示安全上下文（针对 SELinux）。
- `--headers`: 每页重复标题行。
- `--no-headers`: 完全不打印标题。
- `--cols, --columns, --width <num>`: 设置屏幕宽度。
- `--rows, --lines <num>`: 设置屏幕高度。

**Show threads（显示线程）:**

- `H`: 显示线程，就像它们是进程一样。
- `-L`: 可能包括 LWP 和 NLWP 列。
- `-m, m`: 在进程之后显示线程。
- `-T`: 可能包括 SPID 列。

**Miscellaneous options（其他选项）:**

- `-c`: 在 -l 选项下显示调度类别。
- `c`: 显示真实命令名称。
- `e`: 显示命令后的环境。
- `k, --sort`: 指定排序顺序，如：[+|-]key[,[+|-]key[,...]]。
- `L`: 显示格式说明符。
- `n`: 显示数字用户 ID 和 wchan。
- `S, --cumulative`: 包括一些已终止的子进程数据。
- `-y`: 不显示标志，显示 RSS（仅与 -l 一起使用）。
- `-V, V, --version`: 显示版本信息并退出。
- `-w, w`: 无限制的输出宽度。

**Help options（帮助选项）:**

- `--help <simple|list|output|threads|misc|all>`: 显示帮助并退出。可选择不同的帮助模式。

## 常用命令

将目前属于您自己这次登入的 PID 与相关信息列示出来

```
sunrise@ubuntu:~$ ps -l
F S   UID     PID    PPID  C PRI  NI ADDR SZ WCHAN  TTY          TIME CMD
4 S  1000    4295    4294  8  80   0 -  3304 do_wai pts/0    00:00:00 bash
0 R  1000    4304    4295  0  80   0 -  3504 -      pts/0    00:00:00 ps
```

- `F`：代表这个程序的标志 (flag)， 4 代表使用者为 super user
- `S`：代表这个程序的状态 (STAT)，关于各 STAT 的意义将在内文介绍
  - `S`：进程正在运行（Sleeping）
  - `R`：进程正在运行或准备运行（Running）
  - `D`：进程不可中断（Uninterruptible Sleep）
  - `T`：进程已经停止（Stopped）
  - `Z`：僵尸进程（Zombie）
  - `t`：进程被跟踪或已停止（Traced or stopped）
  - `P`：进程被追踪或已停止，但是在等待中（Parked）
- `UID`：进程的用户 ID，表示运行该进程的用户。
- `PID`：进程 ID，是操作系统分配给每个进程的唯一标识符。
- `PPID`：父进程 ID，表示启动当前进程的父进程的 ID。
- `C`：CPU 使用百分比，表示进程占用的 CPU 时间的百分比。
- `PRI`：进程的优先级。
- `NI`：进程的 nice 值，通常用于调整进程的优先级。
- `ADDR`：进程的地址空间。这个是 kernel function，指出该程序在内存的那个部分。如果是个 running的程序，一般就是 "-"
- `SZ`：进程的虚拟内存大小（以页为单位）。
- `WCHAN`：进程当前正在等待的事件或锁。若为 - 表示正在运作
- `TTY`：与进程关联的终端（如果有的话）。
- `TIME`：进程已经运行的 CPU 时间。
- `CMD`：进程的命令行。

列出目前所有的正在内存当中的程序

```
sunrise@ubuntu:~$ ps aux
USER         PID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND
root           1  0.4  0.4 167592 10076 ?        Ss   10:16   0:02 /sbin/init
root           2  0.0  0.0      0     0 ?        S    10:16   0:00 [kthreadd]
root           4  0.0  0.0      0     0 ?        I<   10:16   0:00 [kworker/0:0H]
root           5  0.0  0.0      0     0 ?        I    10:16   0:00 [kworker/u8:0]
```

- `USER`：表示进程的所属用户，即运行该进程的用户。
- `PID`：表示进程 ID，是唯一的标识符，用于标识每个进程。
- `%CPU`：表示进程的 CPU 使用率，即进程使用的 CPU 时间占总 CPU 时间的百分比。
- `%MEM`：表示进程的内存使用率，即进程使用的物理内存占总物理内存的百分比。
- `VSZ`：表示进程的虚拟内存大小（Virtual Set Size），即进程能够访问的虚拟内存的大小，通常以千字节（KB）为单位。
- `RSS`：表示进程的物理内存大小（Resident Set Size），即进程当前占用的物理内存大小，通常以千字节（KB）为单位。
- `TTY`：表示进程所关联的终端，如果进程没有关联终端，则显示"?"。
- `STAT`：表示进程的状态，通常包括以下一些状态：
  - `R`：运行状态（Running）
  - `S`：睡眠状态（Sleeping）
  - `D`：不可中断状态（Uninterruptible Sleep）
  - `Z`：僵尸状态（Zombie）
  - `T`：停止状态（Stopped）
  - 其他状态也可能存在，具体含义会根据操作系统而有所不同。
- `START`：表示进程的启动时间，通常以小时:分钟格式表示。
- `TIME`：表示进程已经使用的 CPU 时间，通常以小时:分钟:秒格式表示。
- `COMMAND`：表示进程的命令行，即进程执行的命令和参数。

列出类似程序树的程序显示

```shell
sunrise@ubuntu:~$ ps -axjf
   PPID     PID    PGID     SID TTY        TPGID STAT   UID   TIME COMMAND
      1    2973    2973    2973 ?             -1 Ss       0   0:00 sshd: /usr/sbin/sshd -D [listener] 0 of 10-100 startups
   2973    4067    4067    4067 ?             -1 Ss       0   0:00  \_ sshd: root@pts/0
   4067    4239    4239    4239 pts/0       4364 Ss       0   0:00  |   \_ -bash
   4239    4294    4294    4239 pts/0       4364 S        0   0:00  |       \_ su sunrise
   4294    4295    4295    4239 pts/0       4364 S     1000   0:00  |           \_ bash
   4295    4364    4364    4239 pts/0       4364 R+    1000   0:00  |               \_ ps -axjf
   2973    4069    4069    4069 ?             -1 Ss       0   0:00  \_ sshd: root@notty
   4069    4242    4242    4242 ?             -1 Ss       0   0:00      \_ /usr/lib/openssh/sftp-server
```

其他命令

```
ps axo pid,comm,pcpu # 查看进程的PID、名称以及CPU 占用率
ps aux | sort -rnk 4 # 按内存资源的使用量对进程进行排序
ps aux | sort -nk 3  # 按 CPU 资源的使用量对进程进行排序
ps -A # 显示所有进程信息
ps -u root # 显示指定用户信息
ps -efL # 查看线程数
ps -e -o "%C : %p :%z : %a"|sort -k5 -nr # 查看进程并按内存使用大小排列
ps -ef # 显示所有进程信息，连同命令行
ps -ef | grep ssh # ps 与grep 常用组合用法，查找特定进程
ps -C nginx # 通过名字或命令搜索进程
ps aux --sort=-pcpu,+pmem # CPU或者内存进行排序,-降序，+升序
ps -f --forest -C nginx # 用树的风格显示进程的层次关系
ps -o pid,uname,comm -C nginx # 显示一个父进程的子进程
ps -e -o pid,uname=USERNAME,pcpu=CPU_USAGE,pmem,comm # 重定义标签
ps -e -o pid,comm,etime # 显示进程运行的时间
ps -aux | grep named # 查看named进程详细信息
ps -o command -p 91730 | sed -n 2p # 通过进程id获取服务名称
```

