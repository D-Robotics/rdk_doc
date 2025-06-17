---
sidebar_position: 1
---

# rdk-backup

**rdk-backup命令** 用于将当前系统备份成镜像。

## 语法说明

```
sudo rdk-backup [dir]
```

## 参数说明
参数为生成和挂载镜像的打包目录，默认打包目录是/mnt；
打包目录在制作镜像时会被忽略；

## 常用命令

使用前需要先联网，rdk-backup执行过程中会下载安装所需的工具

```
sudo rdk-backup
```

执行完成后在打包目录下得到备份的镜像文件 rdk-时间和日期.img
