---
sidebar_position: 2
---

# 1.3.1 RDK S100 入门配置

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 默认登录账户

在进行系统配置前，您需要先登录系统。RDK S100 系统提供了两个默认账户：

- **普通用户：** 用户名 `sunrise`，密码 `sunrise`
- **超级用户 (root)：** 用户名 `root`，密码 `root`

## 连接 Wi-Fi

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

参考 Ubuntu 22.04 Wi-Fi 链接教程进行。

</TabItem>

<TabItem value="server" label="Server">

通过串口或者 SSH，参考下述指令完成连接

```bash
# 扫描wifi⽹络
sudo nmcli device wifi rescan
sudo nmcli device wifi list # 列出找到的wifi
sudo wifi_connect "SSID" "PASSWD" # 连接指定wifi
```

上述命令成功后，会出现`successfully xxx`，使用`ifconfig`便可获得板卡 Wi-Fi 的 IP 地址。

</TabItem>
</Tabs>

## 开启 SSH 服务

当前系统版本默认开启 SSH 登录服务，用户可以使用本方法开、关 SSH 服务。

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

</TabItem>

<TabItem value="server" label="Server">

使用 systemctl 命令可以查看 SSH 服务当前的运行状态，命令如下：

```
sudo systemctl status ssh
```

执行该命令后，会输出 SSH 服务的详细状态信息。如果服务正在运行，输出中会显示 Active: active (running)；如果服务未运行，则会显示 Active: inactive (dead) 等相关信息。

以下为 SSH 的控制命令：

```bash
sudo systemctl start ssh #开启 SSH 服务
sudo systemctl stop ssh  #关闭 SSH 服务
sudo systemctl enable ssh #设置 SSH 服务开机自启
sudo systemctl disable ssh #禁止 SSH 服务开机自启
sudo systemctl restart ssh #重启 SSH 服务

```

</TabItem>

</Tabs>

SSH 的使用请查看 [远程登录 - SSH 登录](../remote_login#ssh)。

## 设置登录模式

### 字符终端自动登录

修改`serial-getty@.service`文件可以设置免密登陆，操作如下

1. 打开serial-getty@.service

```bash
# root用户登陆
vim /lib/systemd/system/serial-getty@.service
# sunrise用户登陆
sudo vim /lib/systemd/system/serial-getty@.service
```

2.  将`ExecStart=-/sbin/agetty`所在行修改为:

```
ExecStart=-/sbin/agetty -a root --keep-baud 921600,115200,38400,9600 %I $TERM
```

**参数解释：** `-a `参数用于指定自动登录的用户名,`-o '-p -- \\u' `则对登录过程进行了额外的定制：保留当前环境变量，并在登录提示中显示用户名

3. 重启后用户将自动登录

### 图形化终端自动登录

待更新

## 设置中文环境

1. 安装命令包

```bash
sudo apt install language-pack-zh-hans language-pack-zh-hans-base fonts-wqy-microhei
```

- language-pack-zh-hans：包含中文语言的翻译文件，能让系统界面显示为中文。
- language-pack-zh-hans-base：基础语言包，提供基本的中文支持。
- fonts-wqy-microhei：安装中文字体

2. 打开终端，输入以下命令打开语言设置配置文件：

```bash
sudo vim /etc/default/locale
```

文件中添加或修改以下内容：

```text
LANG=zh_CN.UTF-8
LANGUAGE=zh_CN:zh
LC_ALL=zh_CN.UTF-8
```

3. 执行以下命令更新配置：

```bash
fc-cache -fv
source /etc/default/locale
```

## 设置中文输入法

安装好中文环境之后，默认支持系统自带的输入法，按下 `Super（Windows 键）` + `Space `组合键，即可在不同的输入法之间进行切换。

## 设置 RDK Studio

待更新

## NoMachine 配置

`NoMachine`当前不支持`apt`下载，需要到官网获取`deb`包。

NoMachine下载官方网址： [NoMachine Download](https://downloads.nomachine.com/download/?id=30&platform=linux&distro=arm)

**下载安装包**

进入官网后找到适合`RDK S100`的`ARM64`版本的安装包，点击`Download`

![image_s100_nomachine_dl](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/configuration_wizard/image_s100_nomachine_dl.PNG)

**安装**

```shell
sudo apt update; sudo apt upgrade -y   // 确保当前软件是较新的状态
dpkg -i nomachine_*_arm64.deb
```

**配置启动**

配置服务器以允许远程连接

```shell
sudo systemctl start nxserver
```

设置`NoMachine`为开机启动：

```shell
sudo systemctl enable nxserver
```

重启`NoMachine`服务:

```shell
sudo systemctl restart nxserver
```

**重启**

由于`NXServer`的配置问题，完成上述操作后直接连接会黑屏，需要重启后使用。

`NoMachine`的使用请查看 [远程登录 - NoMachine登录](../remote_login#Nomachine登陆)。

## 用户管理

**修改用户名**

以新用户名为 usertest 为例

```shell
#关闭sunrise用户所有进程
sudo pkill -u sunrise
#sunrise用户改名为usertest
sudo usermod -l usertest sunrise
#用户的家目录改为/home/usertest
sudo usermod -d /home/usertest -m usertest
#修改用户密码
sudo passwd usertest
```

最后将更新桌面服务自动登录的用户名称:
  - gdm：RDK S100默认桌面服务，将`/etc/gdm3/custom.conf`文件中的`AutomaticLogin = sunrise`改为`AutomaticLogin = usertest`
  - lighttdm：将`/etc/lightdm/lightdm.conf.d/22-hobot-autologin.conf`文件中的 `autologin-user=sunrise` 改为`autologin-user=usertest`，

**增加新用户**

以新增用户为 usertest 为例

```shell
sudo useradd -U -m -d /home/usertest -k /etc/skel/ -s /bin/bash -G disk,kmem,dialout,sudo,audio,video,render,i2c,lightdm,vpu,gdm,weston-launch,graphics,jpu,ipu,vps,misc,gpio usertest
sudo passwd usertest
sudo cp -aRf /etc/skel/. /home/usertest
sudo chown -R usertest:usertest /home/usertest
```

也可以参考修改用户名，将新增用户设为自动登录用户
