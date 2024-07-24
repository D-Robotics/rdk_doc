---
sidebar_position: 1
---

# rdk-miniboot-update

**rdk-miniboot-update命令** 用于更新RDK硬件的最小启动镜像（miniboot）。

## 语法说明

```
sudo rdk-miniboot-update [options]... [FILE]
```

## 选项说明

选项都是可选的，非必须。如果不带任何选项参数运行，`rdk-miniboot-update`会使用最新版本的`miniboot`镜像完成升级更新。

- `-f`：安装指定的文件，而不是安装最新适用的更新。
- `-h`：显示帮助文本并退出。
- `-l`：根据 FIRMWARE_RELEASE_STATUS 和 FIRMWARE_IMAGE_DIR 设置，返回最新可用的`miniboot`镜像的完整路径。可以查看不带选项参数时会使用什么镜像文件进行更新。
- `-s`：不显示进度消息。

## 常用命令

更新`miniboot`镜像为最新版本

```
sudo rdk-miniboot-update
```

更新使用指定的`miniboot`镜像

```
sudo rdk-miniboot-update -f /userdata/miniboot.img
```

查看不带选项参数时会使用什么镜像文件进行更新

```
sunrise@ubuntu:~$ rdk-miniboot-update -l
/lib/firmware/rdk/miniboot/default/disk_nand_minimum_boot_2GB_3V3_20230413.img
```

