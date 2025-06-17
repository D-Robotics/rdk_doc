---
sidebar_position: 1
---

# dpkg-deb

**dpkg-deb命令** 是Debian Linux下的软件包管理工具，它可以对软件包执行打包和解包操作以及提供软件包信息。

## 语法说明

```
  dpkg-deb [<option> ...] <command>
```

## command 说明

dpkg-deb 命令不仅有选项可以设置，还需要设置命令来执行不同的功能。

- -b：创建debian软件包。
- -c：显示软件包中的文件列表；
- -e：将主控信息解压；
- -f：把字段内容打印到标准输出；
- -x：将软件包中的文件释放到指定目录下；
- -X：将软件包中的文件释放到指定目录下，并显示释放文件的详细过程；
- -w：显示软件包的信息；
- -l：显示软件包的详细信息；
- -R：提取控制信息和存档的清单文件；

## 选项说明

- `-v, --verbose`：启用详细输出。
- `-D, --debug`：启用调试输出。
  - `--showformat=<format>`：使用替代格式来进行 `--show`。
  - `--deb-format=<format>`：选择归档格式。允许的取值为 0.939000、2.0（默认值）。
  - `--nocheck`：禁止控制文件检查（构建不良软件包）。
  - `--root-owner-group`：强制文件的所有者和组为 root。
  - `--[no-]uniform-compression`：在所有成员上使用压缩参数。如果指定，将使用统一的压缩参数。
- `-z#`：设置构建时的压缩级别。
- `-Z<type>`：设置构建时使用的压缩类型。允许的类型有 gzip、xz、zstd、none。
- `-S<strategy>`：设置构建时的压缩策略。允许的值有 none、extreme（xz）、filtered、huffman、rle、fixed（gzip）。

### 常用命令

- 解压程序文件

```shell
dpkg-deb -x hobot-configs_2.2.0-20231030133209_arm64.deb
```

- 解压控制文件

```shell
dpkg-deb -e hobot-configs_2.2.0-20231030133209_arm64.deb hobot-configs/DEBIAN
```

- 查询deb包中的文件内容

```shell
dpkg-deb -c hobot-configs_2.2.0-20231030133209_arm64.deb
```
