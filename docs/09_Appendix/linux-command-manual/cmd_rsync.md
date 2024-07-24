---
sidebar_position: 1
---

# rsync

Rsync 是一个快速且功能强大的文件复制工具。它可以在本地复制文件，也可以通过任何远程 shell 从另一台主机复制文件，还可以与远程的 rsync 守护程序进行文件复制。它提供了大量选项，可以控制其行为的各个方面，并允许非常灵活地指定要复制的文件集合。Rsync 以其增量传输算法而闻名，该算法通过仅发送源文件与目标中现有文件之间的差异来减少通过网络发送的数据量。Rsync 广泛用于备份和镜像操作，同时也作为日常使用的改进型复制命令。

Rsync 通过默认的"快速检查"算法查找需要传输的文件，该算法查找大小或最后修改时间发生变化的文件。当快速检查表明文件的数据不需要更新时，对目标文件上的其他保留属性（根据选项请求）的任何更改将直接应用于目标文件。

## 语法说明

```shell
Usage: rsync [OPTION]... SRC [SRC]... DEST
  or   rsync [OPTION]... SRC [SRC]... [USER@]HOST:DEST
  or   rsync [OPTION]... SRC [SRC]... [USER@]HOST::DEST
  or   rsync [OPTION]... SRC [SRC]... rsync://[USER@]HOST[:PORT]/DEST
  or   rsync [OPTION]... [USER@]HOST:SRC [DEST]
  or   rsync [OPTION]... [USER@]HOST::SRC [DEST]
  or   rsync [OPTION]... rsync://[USER@]HOST[:PORT]/SRC [DEST]
  
The ':' usages connect via remote shell, while '::' & 'rsync://' usages connect
to an rsync daemon, and require SRC or DEST to start with a module name.
```

简易写法:

```
rsync [OPTION...] SRC... [DEST]
```

## 选项说明

- -v, --verbose：增加详细信息输出
  - --info=FLAGS：指定详细信息输出的标志
  - --debug=FLAGS：指定调试详细信息输出的标志
  - --msgs2stderr：为调试输出特殊处理
- -q, --quiet：抑制非错误消息的输出
  - --no-motd：抑制守护程序模式下的MOTD（请参阅man页注意事项）
- -c, --checksum：基于校验和而不是修改时间和文件大小进行跳过
- -a, --archive：存档模式；等同于 -rlptgoD（不包括 -H、-A、-X）
  - --no-OPTION：关闭隐含的OPTION（例如，--no-D）
- -r, --recursive：递归处理目录
- -R, --relative：使用相对路径名
  - --no-implied-dirs：不随 --relative 一起发送隐含目录
- -b, --backup：创建备份文件（参见 --suffix 和 --backup-dir）
  - --backup-dir=DIR：将备份文件放入基于DIR的层次结构中
  - --suffix=SUFFIX：设置备份文件的后缀（默认为~，如果没有 --backup-dir）
- -u, --update：跳过接收端上更新的文件
  - --inplace：在原地更新目标文件（请参阅man页）
  - --append：将数据附加到较短的文件
  - --append-verify：类似于 --append，但附加的文件有旧数据的文件校验和
- -d, --dirs：传输目录而不递归
- -l, --links：复制符号链接作为符号链接
- -L, --copy-links：将符号链接转换为所指定的文件/目录
  - --copy-unsafe-links：仅转换"不安全"的符号链接
  - --safe-links：忽略指向源树外部的符号链接
  - --munge-links：混淆符号链接以使其更安全（但不可用）
- -k, --copy-dirlinks：将目标目录的符号链接转换为所指定的目录
- -K, --keep-dirlinks：将接收端上的符号链接目录视为目录
- -H, --hard-links：保留硬链接
- -p, --perms：保留权限
- -E, --executability：保留文件的可执行性
  - --chmod=CHMOD：影响文件和/或目录的权限
- -A, --acls：保留ACL（隐含 --perms）
- -X, --xattrs：保留扩展属性
- -o, --owner：保留所有者（仅超级用户）
- -g, --group：保留组
  - --devices：保留设备文件（仅超级用户）
  - --copy-devices：将设备内容复制为普通文件
  - --specials：保留特殊文件
- -D：等同于 --devices --specials
- -t, --times：保留修改时间
- -O, --omit-dir-times：从 --times 中省略目录
- -J, --omit-link-times：从 --times 中省略符号链接
  - --super：接收端尝试超级用户活动
  - --fake-super：使用xattrs存储/恢复特权属性
- -S, --sparse：将连续的空块转换为稀疏块
  - --preallocate：在写入文件之前分配目标文件
- -n, --dry-run：执行试运行，不进行实际更改
- -W, --whole-file：整个文件传输（不使用增量传输算法）
  - --checksum-choice=STR：选择校验和算法
- -x, --one-file-system：不跨越文件系统边界
- -B, --block-size=SIZE：强制使用固定的校验块大小
- -e, --rsh=COMMAND：指定要使用的远程 shell
  - --rsync-path=PROGRAM：指定在远程机器上运行的rsync
  - --existing：跳过在接收端创建新文件
  - --ignore-existing：跳过已经存在于接收端的文件的更新
  - --remove-source-files：发送端删除已同步的文件（非目录）
  - --del：--delete-during 的别名
  - --delete：从目标目录中删除多余的文件
  - --delete-before：在传输期间接收端删除
  - --delete-during：在传输期间接收端删除
  - --delete-delay：找到删除操作后才删除
  - --delete-after：在传输后接收端删除
  - --delete-excluded：也从目标目录中删除被排除的文件
  - --ignore-missing-args：忽略缺失的源参数而不报错
  - --delete-missing-args：从目标中删除缺失的源参数
  - --ignore-errors：即使出现I/O错误也进行删除
  - --force：即使不为空也强制删除目录
  - --max-delete=NUM：最多删除 NUM 个文件
  - --max-size=SIZE：不传输大于 SIZE 的文件
  - --min-size=SIZE：不传输小于 SIZE 的文件
  - --partial：保留部分传输的文件
  - --partial-dir=DIR：将部分传输的文件放入 DIR
  - --delay-updates：在传输结束时将所有更新的文件放在指定位置
- -m, --prune-empty-dirs：从文件列表中剪除空目录链
  - --numeric-ids：不通过用户名/组名映射uid/gid值
  - --usermap=STRING：自定义用户名映射
  - --groupmap=STRING：自定义组名映射
  - --chown=USER:GROUP：简单的用户名/组名映射
  - --timeout=SECONDS：设置I/O超时时间（以秒为单位）
  - --contimeout=SECONDS：设置守护程序连接的超时时间（以秒为单位）
- -I, --ignore-times：不跳过大小和修改时间匹配的文件
- -M, --remote-option=OPTION：仅将选项发送到远程端
  - --size-only：跳过大小匹配的文件
- @, --modify-window=NUM：设置修改时间比较的精度
- -T, --temp-dir=DIR：在目录DIR中创建临时文件
- -y, --fuzzy：在没有目标文件的情况下查找相似的文件作为基准
  - --compare-dest=DIR：相对于DIR，也比较目标文件
  - --copy-dest=DIR：并包括未更改的文件的副本
  - --link-dest=DIR：在未更改时将文件硬链接到DIR中
- -z, --compress：在传输期间压缩文件数据
  - --compress-level=NUM：明确设置压缩级别
  - --skip-compress=LIST：跳过具有LIST中后缀的文件的压缩
- -C, --cvs-exclude：自动忽略与CVS相同的文件
- -f, --filter=RULE：添加文件过滤规则
- -F：与 --filter='dir-merge /.rsync-filter' 相同
  - --exclude=PATTERN：排除与PATTERN匹配的文件
  - --exclude-from=FILE：从FILE中读取排除模式
  - --include=PATTERN：不排除与PATTERN匹配的文件
  - --include-from=FILE：从FILE中读取包含模式
  - --files-from=FILE：从FILE中读取源文件名的列表
- -0, --from0：所有 *-from/filter 文件以0分隔
- -s, --protect-args：不对空格进行拆分，只对通配符特殊字符进行拆分
  - --trust-sender：信任远程发送方的文件列表
  - --address=ADDRESS：将传出套接字绑定到守护程序的地址
  - --port=PORT：指定双冒号的备用端口号
  - --sockopts=OPTIONS：指定自定义TCP选项
  - --blocking-io：使用阻塞I/O进行远程shell操作
  - --stats：提供一些文件传输统计信息
- -8, --8-bit-output：在输出中保留高位字符
- -h, --human-readable：以人类可读的格式输出数字
  - --progress：在传输过程中显示进度
- -P：与 --partial --progress 相同
- -i, --itemize-changes：输出所有更新的变更摘要
  - --out-format=FORMAT：使用指定的格式输出更新
  - --log-file=FILE：将操作记录到指定的文件
  - --log-file-format=FMT：使用指定的FMT记录更新
  - --password-file=FILE：从文件中读取守护程序访问密码
  - --list-only：仅列出文件，不复制它们
  - --bwlimit=RATE：限制套接字I/O带宽
  - --stop-at=y-m-dTh:m：在年-月-日Thour:minute时停止rsync
  - --time-limit=MINS：在MINS分钟后停止rsync
  - --outbuf=N|L|B：将输出缓冲设置为无、行或块
  - --write-batch=FILE：将批处理更新写入文件
  - --only-write-batch=FILE：类似于 --write-batch，但不更新目标
  - --read-batch=FILE：从文件中读取批处理更新
  - --protocol=NUM：强制使用旧的协议版本
  - --iconv=CONVERT_SPEC：请求对文件名进行字符集转换
  - --checksum-seed=NUM：设置块/文件校验和种子（高级选项）
  - --noatime：在打开源文件时不改变atime
- -4, --ipv4：优先使用IPv4
- -6, --ipv6：优先使用IPv6
  - --version：显示版本号
- (-h) --help：显示帮助信息（只有在单独使用 -h 时才是 --help）

## 常用命令

- 拷贝本地文件，将/app目录下的文件拷贝到/userdata目录下

```
rsync -avSH /app/ /userdata/
```

- 拷贝本地机器的内容到远程机器

```
rsync -av /app 192.168.1.12:/app
```

- 拷贝远程机器的内容到本地机器

```
rsync -av 192.168.1.12:/app /app
```

- 拷贝远程rsync服务器(daemon形式运行rsync)的文件到本地机。

```
rsync -av root@192.168.1.12::www /userdata
```

- 拷贝本地机器文件到远程rsync服务器(daemon形式运行rsync)中。当DST路径信息包含”::”分隔符时启动该模式。

```
rsync -av /userdata root@192.168.1.12::www
```

- 显示远程机的文件列表。这类似于rsync传输，不过只要在命令中省略掉本地机信息即可。

```
rsync -v rsync://192.168.1.12/app
```

- 指定密码存放文件，无需输入密码，直接执行rsync传输

```
rsync -rvzP --password-file=/etc/rsync.password rsync@$192.168.1.12::app/ /app
```
