---
sidebar_position: 1
---

# apt

apt（Advanced Packaging Tool）是一个在 Debian 和 Ubuntu 中的 Shell 前端软件包管理器。

apt 命令提供了查找、安装、升级、删除某一个、一组甚至全部软件包的命令，而且命令简洁而又好记。

apt 命令执行需要超级管理员权限(root)。

## 语法说明

```
  apt [options] [command] [package ...]
```

- **options：**可选，选项包括 -h（帮助），-y（当安装过程提示选择全部为"yes"），-q（不显示安装的过程）等等。
- **command：**要进行的操作。
- **package**：安装的包名。

## 常用命令

- 更新apt软件源数据库：**sudo apt update**

- 更新已安装的软件包：**sudo apt upgrade**

  列出可更新的软件包及版本信息：**apt list --upgradeable**

  升级软件包，升级前先删除需要更新软件包：**sudo apt full-upgrade**

- 安装指定的软件命令：**sudo apt install `package_name`**

  安装多个软件包：**sudo apt install `package_1` `package_2` `package_3`**

- 显示软件包具体信息,例如：版本号，安装大小，依赖关系等等：**sudo apt show `package_name`**

- 删除软件包命令：**sudo apt remove `package_name`**

- 清理不再使用的依赖和库文件: **sudo apt autoremove**

- 移除软件包及配置文件: **sudo apt purge `package_name`**

- 查找软件包命令： **sudo apt search `package_name`**

- 列出所有已安装的包：**apt list --installed**

- 列出所有已安装的包的版本信息：**apt list --all-versions**
