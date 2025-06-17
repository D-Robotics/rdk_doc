---
sidebar_position: 1
---

# find

find 命令用于在指定目录下查找文件和目录。

它可以使用不同的选项来过滤和限制查找的结果。任何位于参数之前的字符串都将被视为欲查找的目录名。如果使用该命令时，不设置任何参数，则find命令将在当前目录下查找子目录与文件。并且将查找到的子目录和文件全部进行显示。

## 语法说明

```
  find [-H] [-L] [-P] [-Olevel] [-D debugopts] [path...] [expression]
```

## 选项说明

**path** 是要查找的目录路径，可以是一个目录或文件名，也可以是多个路径，多个路径之间用空格分隔，如果未指定路径，则默认为当前目录。

**expression** 是可选参数，用于指定查找的条件，可以是文件名、文件类型、文件大小等等。

expression 中可使用的选项有二三十个之多，以下列出最常用的部份：

- `-name pattern`：按文件名查找，支持使用通配符 `*` 和 `?`。
- `-iname pattern`：此参数的效果和指定`-name`参数类似，但忽略字符大小写的差别；
- `-type type`：按文件类型查找，可以是 `f`（普通文件）、`d`（目录）、`l`（符号链接）等。
- `-size [+-]size[cwbkMG]`：按文件大小查找，支持使用 `+` 或 `-` 表示大于或小于指定大小，单位可以是 `c`（字节）、`w`（字数）、`b`（块数）、`k`（KB）、`M`（MB）或 `G`（GB）。
- `-mtime days`：按修改时间查找，支持使用 `+` 或 `-` 表示在指定天数前或后，days 是一个整数表示天数。
- `-user username`：按文件所有者查找。
- `-group groupname`：按文件所属组查找。

find 命令中用于时间的参数如下：

- `-amin n`：查找在 n 分钟内被访问过的文件。
- `-atime n`：查找在 n*24 小时内被访问过的文件。
- `-cmin n`：查找在 n 分钟内状态发生变化的文件（例如权限）。
- `-ctime n`：查找在 n*24 小时内状态发生变化的文件（例如权限）。
- `-mmin n`：查找在 n 分钟内被修改过的文件。
- `-mtime n`：查找在 n*24 小时内被修改过的文件。

在这些参数中，n 可以是一个正数、负数或零。正数表示在指定的时间内修改或访问过的文件，负数表示在指定的时间之前修改或访问过的文件，零表示在当前时间点上修改或访问过的文件。

例如：**-mtime 0** 表示查找今天修改过的文件，**-mtime -7** 表示查找一周以前修改过的文件。

关于时间 n 参数的说明：

- **+n**：查找比 n 天前更早的文件或目录。
- **-n**：查找在 n 天内更改过属性的文件或目录。
- **n**：查找在 n 天前（指定那一天）更改过属性的文件或目录。

## 常用命令

列出当前目录及子目录下所有文件和文件夹

```shell
find .
```

查找当前目录下名为 file.txt 的文件

```
find . -name file.txt
```

将当前目录及其子目录下所有文件后缀为 **.c** 的文件列出来

```
find . -name "*.c"
```

同上，但忽略大小写

```shell
find . -iname "*.c"
```

将当前目录及其子目录中的所有文件列出

```
find . -type f
```

查找 /home 目录下大于 1MB 的文件

```
find . -size +1M
```

搜索小于10KB的文件

```shell
find . -type f -size -10k
```

搜索等于10KB的文件

```shell
find . -type f -size 10k
```

- 文件大小单元：

  - **b** —— 块（512字节）

  - **c** —— 字节

  - **w** —— 字（2字节）

  - **k** —— 千字节

  - **M** —— 兆字节

  - **G** —— 吉字节

查找 /var/log 目录下在 7 天前修改过的文件

```
find /var/log -mtime +7
```

将当前目录及其子目录下所有最近 20 天前更新过的文件列出，不多不少正好 20 天前的

```
find . -ctime  20
```

将当前目录及其子目录下所有 20 天前及更早更新过的文件列出

```
find . -ctime  +20
```

将当前目录及其子目录下所有最近 20 天内更新过的文件列出

```
find . -ctime  20
```

查找 /var/log 目录中更改时间在 7 日以前的普通文件，并在删除之前询问它们

```
find /var/log -type f -mtime +7 -ok rm {} \;
```

查找当前目录中文件属主具有读、写权限，并且文件所属组的用户和其他用户具有读权限的文件

```
find . -type f -perm 644 -exec ls -l {} \;
```

查找系统中所有文件长度为 0 的普通文件，并列出它们的完整路径

```
find / -type f -size 0 -exec ls -l {} \;
```

当前目录及子目录下查找所有以.txt和.pdf结尾的文件

```shell
find . \( -name "*.txt" -o -name "*.pdf" \)
或
find . -name "*.txt" -o -name "*.pdf"
```

匹配文件路径或者文件

```shell
find /usr/ -path "*local*"
```

基于正则表达式匹配文件路径

```shell
find . -regex ".*\(\.txt\|\.pdf\)$"
```

同上，但忽略大小写

```shell
find . -iregex ".*\(\.txt\|\.pdf\)$"
```

否定参数, 找出/home下不是以.txt结尾的文件

```shell
find /home ! -name "*.txt"
```

根据文件类型进行搜索

```shell
find . -type 类型参数
```

- 类型参数列表：
  - **f** 普通文件
  - **l** 符号连接
  - **d** 目录
  - **c** 字符设备
  - **b** 块设备
  - **s** 套接字
  - **p** Fifo

基于目录深度搜索，向下最大深度限制为3

```shell
find . -maxdepth 3 -type f
```

搜索出深度距离当前目录至少2个子目录的所有文件

```shell
find . -mindepth 2 -type f
```

删除当前目录下所有`.log`文件

```shell
find . -type f -name "*.log" -delete
```

当前目录下搜索出权限为777的文件

```shell
find . -type f -perm 777
```

找出当前目录下权限不是644的`.conf`文件

```shell
find . -type f -name "*.conf" ! -perm 644
```

找出当前目录用户`sunrise`拥有的所有文件

```shell
find . -type f -user sunrise
```

找出当前目录用户组`sunrise`拥有的所有文件

```shell
find . -type f -group sunrise
```

找出当前目录下所有`root`的文件，并把所有权更改为用户`sunrise`

```shell
find .-type f -user root -exec chown sunrise {} \;
```

上例中， **{}** 用于与 **-exec** 选项结合使用来匹配所有文件，然后会被替换为相应的文件名。

找出`home`目录下所有的`.txt`文件并删除

```shell
find $HOME/. -name "*.txt" -ok rm {} \;
```

上例中， **-ok** 和 **-exec** 行为一样，不过它会给出提示，是否执行相应的操作。

查找当前目录下所有`.txt`文件并把他们拼接起来写入到`all.txt`文件中

```shell
find . -type f -name "*.txt" -exec cat {} \;> /all.txt
```

查找当前目录或者子目录下所有.txt文件，但是跳过子目录sk

```shell
find . -path "./sk" -prune -o -name "*.txt" -print
```

> ⚠️ ./sk 不能写成 ./sk/ ，否则没有作用。

忽略两个目录

```shell
find . \( -path ./sk -o  -path ./st \) -prune -o -name "*.txt" -print
```

> ⚠️ 如果写相对路径必须加上`./`

要列出所有长度为零的文件

```shell
find . -empty
```

统计代码行数

```shell
find . -name "*.c"|xargs cat|grep -v ^$|wc -l # 代码行数统计, 排除空行
```

其它实例

```shell
find ~ -name '*jpg' # 主目录中找到所有的 jpg 文件。 -name 参数允许你将结果限制为与给定模式匹配的文件。
find ~ -iname '*jpg' # -iname 就像 -name，但是不区分大小写
find ~ ( -iname 'jpeg' -o -iname 'jpg' ) # 一些图片可能是 .jpeg 扩展名。幸运的是，我们可以将模式用“或”（表示为 -o）来组合。
find ~ \( -iname '*jpeg' -o -iname '*jpg' \) -type f # 如果你有一些以 jpg 结尾的目录呢？ （为什么你要命名一个 bucketofjpg 而不是 pictures 的目录就超出了本文的范围。）我们使用 -type 参数修改我们的命令来查找文件。
find ~ \( -iname '*jpeg' -o -iname '*jpg' \) -type d # 也许你想找到那些命名奇怪的目录，以便稍后重命名它们
```
