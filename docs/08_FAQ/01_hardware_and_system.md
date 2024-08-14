---
sidebar_position: 1
---

# 8.1 硬件和系统

认证配件及购买链接请参考[认证配件清单](https://developer.d-robotics.cc/documents_rdk/hardware_development/rdk_x3/accessory)

详细请参考[D-Robotics RDK套件用户手册的常见问题](https://developer.d-robotics.cc/documents_rdk/category/common_questions)

## 什么是D-Robotics RDK套件？

D-Robotics Developer Kits，简称[D-Robotics RDK套件](https://developer.d-robotics.cc/documents_rdk/)，是基于D-Robotics 智能芯片打造的机器人开发者套件，包括**RDK X3（旭日X3派）**、**RDK X3 Module（旭日X3模组）**、**RDK Ultra**。

## 如何查看系统版本号

系统安装完成后，登录系统并使用命令`apt list --installed | grep hobot`查看系统核心功能包版本，使用`cat /etc/version`命令查看系统大版本号。

2.x版本（以2.0.0版本为例说明）系统信息如下：

```shell
root@ubuntu:~# apt list --installed | grep hobot

WARNING: apt does not have a stable CLI interface. Use with caution in scripts.

hobot-boot/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-bpu-drivers/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-camera/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-configs/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-display/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-dnn/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-dtb/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-io-samples/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-io/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-kernel-headers/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-models-basic/unknown,now 1.0.1 arm64 [installed]
hobot-multimedia-dev/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-multimedia-samples/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-multimedia/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-sp-samples/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-spdev/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-utils/unknown,now 2.0.0-20230530181103 arm64 [installed]
hobot-wifi/unknown,now 2.0.0-20230530181103 arm64 [installed]
root@ubuntu:~#
root@ubuntu:~# cat /etc/version
2.0.0
root@ubuntu:~#

```

1.x版本（以1.1.6版本为例说明）系统信息如下：

```shell
root@ubuntu:~# apt list --installed | grep hobot

WARNING: apt does not have a stable CLI interface. Use with caution in scripts.

hobot-arm64-bins/unknown,now 1.1.5 arm64 [installed]
hobot-arm64-boot/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-configs/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-desktop/unknown,now 1.1.5 arm64 [installed]
hobot-arm64-dnn-python/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-gpiopy/unknown,now 1.1.5 arm64 [installed]
hobot-arm64-hdmi-sdb/unknown,now 1.1.5 arm64 [installed]
hobot-arm64-includes/unknown,now 1.1.5 arm64 [installed]
hobot-arm64-libs/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-modules/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-sdb-ap6212/unknown,now 1.1.6 arm64 [installed]
hobot-arm64-srcampy/unknown,now 1.1.5 arm64 [installed]
hobot-linux-headers/unknown,now 1.1.5 arm64 [installed]
hobot-models-basic/unknown,now 1.0.1 arm64 [installed]
hobot-sp-cdev/unknown,now 1.1.6 arm64 [installed]
root@ubuntu:~#
root@ubuntu:~# cat /etc/version
x3_ubuntu_v1.1.6
root@ubuntu:~#

```

## 系统版本和RDK平台硬件对应关系

系统版本说明：

- 2.x版本系统：基于RDK Linux开源代码包制作，支持RDK X3、RDK X3 Module等全系列硬件。

- 1.x版本系统：基于闭源Linux系统制作，历史版本，仅支持RDK X3硬件。

**注意**

- **1.x版本系统无法通过apt命令直接升级到2.x版本系统，需要以烧录镜像的方式重新[安装系统](https://developer.d-robotics.cc/documents_rdk../../../01_Quick_start/install_os.md)。**

- **2.x版本tros.b仅支持2.x版本系统，1.x版本tros.b仅支持1.x版本系统。**

## 摄像头插拔注意事项

**严禁在开发板未断电的情况下插拔摄像头，否则非常容易烧坏摄像头模组**。

## 串口线如何连接?

串口线一端（白色）接到RDK X3，由于接口有凹槽正反面通常不会接反，另外一端接到串口转接板，此处需要重点关注，连接图如下：

![](../../static/img/08_FAQ/image/hardware_and_system/connect.png)

## RDK X3供电有什么要求？

RDK X3通过USB Type C接口供电，并兼容QC、PD快充协议。推荐使用支持QC、PD快充协议的电源适配器，或者至少搭配**5V 直流 2A**的电源适配器为开发板供电。

**注意，请不要使用PC机USB接口为开发板供电，否则会因供电不足造成开发板工作异常（例如RDK X3上电后， HDMI 无输出（完全黑屏），绿灯没有熄灭，连接串口后，发现系统在反复重启，无法进入操作系统）。**

## RDK X3是否有推荐SD卡？

建议使用高速C10 SD卡，老卡可能会存在烧录镜像无法启动问题，SD卡16G以上；

金士顿： `https://item.jd.com/25263496192.html`

闪迪： `https://item.jd.com/1875992.html#crumb-wrap>

## F37和GC4663 MIPI摄像头如何连接?

F37和GC4663摄像头模组通过24pin异面FPC排线跟开发板连接，**注意排线两端蓝面向上插入连接器**。F37摄像头连接示意图如下：

![](../../static/img/08_FAQ/image/hardware_and_system/image-X3-PI-Camera.png)

正常连接后接通电源，执行命令：

```bash
cd /app/ai_inference/03_mipi_camera_sample
sudo python3 mipi_camera.py
```

算法渲染结果的HDMI输出如下图，示例图像中检测到了`teddy bear`、`cup`和`vase`。

![](../../static/img/08_FAQ/image/hardware_and_system/image-20220511181747071.png)

```text
输入命令：i2cdetect -y -r 1   
F37：
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- UU -- -- -- -- 
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --   

GC4663：
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- UU -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
```

## 如何查看RDK X3的CPU、BPU等运行状态?

```bash
sudo hrut_somstatus
```

## 如何设置自启动?

通过在sudo vim /etc/rc.local文件末尾添加命令，可实现开机自启动功能，例如：

```bash
#!/bin/bash -e
# 
# rc.local
#re
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

#!/bin/sh

chmod a=rx,u+ws /usr/bin/sudo
chown sunrise:sunrise /home/sunrise

which "hrut_count" >/dev/null 2>&1
if [ $? -eq 0 ]; then
        hrut_count 0
fi

# Insert what you need
```

## 开发板上电后无显示

<font color='Blue'>【问题】</font> 

- 开发板上电后，显示器持续无画面输出，连接串口发现系统反复重启或者阻塞在uboot命令行

<font color='Green'>【解答】</font> 

- 供电不足导致反复重启，更换符合开发板要求的5V/3A适配器，推荐使用官方推荐的适配器
- 使用的USB线质量不好，会引起供电的不稳定导致异常断电、反复重启
- 串口误触导致阻塞在uboot，重新给设备上电恢复
- Micro SD卡镜像格式错误，当串口提示如下log时，需要重新制作镜像 

![image-20221124194527634](../../static/img/08_FAQ/image/system/image-20221124194527634.png)
- Micro SD卡质量问题，当串口提示如下log，说明Micro SD卡损坏，需要更换新卡  

![image-20221124194636213](../../static/img/08_FAQ/image/system/image-20221124194636213.png)  

![image-20221124194721750](../../static/img/08_FAQ/image/system/image-20221124194721750.png)

## 开发板供电异常的常见现象

如果发现开发板上电后状态灯一直不熄灭或者不闪烁，HDMI显示器上也不显示任何画面，则建议先排查供电是否正常。

- 需要使用支持**5V 3A**的电源适配器为开发板供电，推荐使用[基础配件清单](/hardware_development/rdk_x3/accessory#basic_accessories)中推荐的电源适配器型号。
- 如果使用自己的充电器，请选用正规品牌的USB Type C 口供电线，并且满足**5V 3A**
- 不要直接使用电脑的USB口给开发板供电

要确定是否由供电问题导致无法正常启动，我们需要将开发板连接到串口，并观察启动日志，目前以下现象可以明确判断是供电异常。

### 现象1：从Uboot引导内核时重启

此时处在uboot阶段，uboot的大部分任务已经完成，在把内核、设备树等从SD卡上加载到内存中时，或在跳转进内核执行时，开发板异常重启了。

![image-20230914173433676](../../static/img/08_FAQ/image/system/image-20230914173433676.png)

![image-20230914173911690](../../static/img/08_FAQ/image/system/image-20230914173911690.png)

### 现象2：已运行至内核，数秒后重启

此时内核加载并已经运行，正在进行内核、驱动的加载和初始化，开发板异常重启。

![image-20230914174123619](../../static/img/08_FAQ/image/system/image-20230914174123619.png)

### 其他现象：

当前供电不足的现象只能通过串口日志分析，如果启动日志上观察到启动过程中，没有任何的**错误**和**告警**信息，开发板直接打印 `NOTICE:  fast_boot:0` 重启，那么基本可以判断是供电不足。

目前因为供电不足引起的现象容易和其他比如SD卡不识别、硬件损坏等现象混淆，如果不接入串口查看日志则不好明确判断。推荐使用[基础配件清单](/hardware_development/rdk_x3/accessory#basic_accessories)中推荐的电源适配器型号。

## 开发板默认账户

<font color='Blue'>【问题】</font> 

- 开发板默认支持的账户类型

<font color='Green'>【解答】</font> 

- 开发板默认支持两种账户，具体如下：
  - 默认账户：用户名`sunrise`  密码`sunrise` 
  - root账户：用户名`root`  密码`root`

## NTFS文件系统挂载
<font color='Blue'>【问题】</font> 

- NTFS文件系统挂载后，如何支持读写模式

<font color='Green'>【解答】</font> 

- 安装ntfs-3g功能包后，再挂载NTFS文件系统即可支持写模式。安装命令如下：
    ```bash
    sudo apt -y install ntfs-3g
    ```

## vscode工具支持
<font color='Blue'>【问题】</font> 

- 开发板是否支持`vscode`开发工具

<font color='Green'>【解答】</font> 

- 开发板不支持`vscode`本地安装，用户可在PC端通过`ssh-remote`插件方式远程链接开发板

## adb调试功能
<font color='Blue'>【问题】</font> 

- 开发板如何启动adb调试功能

<font color='Green'>【解答】</font> 

- Ubuntu系统中默认启动了`adbd`服务，用户只需在电脑安装adb工具后即可使用，方法可参考[bootloader镜像更新](https://developer.d-robotics.cc/forumDetail/88859074455714818)。

## apt update更新失败

<font color='Blue'>【问题】</font> 

- Ubuntu系统中运行`apt update`命令，提示以下错误：
    ```bash
    Reading package lists... Done
    E: Could not get lock /var/lib/apt/lists/lock. It is held by process 4299 (apt-get)
    N: Be aware that removing the lock file is not a solution and may break your system.
    E: Unable to lock directory /var/lib/apt/lists/
    ```

<font color='Green'>【解答】</font> 

- Ubuntu系统自动运行的更新程序，跟用户`apt update`操作发生冲突。可以杀死系统自动运行的更新进程后重试，例如`kill 4299`。

## 开发板文件传输方式

<font color='Blue'>【问题】</font> 

- 开发板和电脑之间的文件传输的方式有哪些

<font color='Green'>【解答】</font> 

- 可以通过网络、USB等方式进行传输。其中，网络可使用ftp工具、scp命令等方式，USB可使用u盘、adb等方式。以scp命令为例，文件传输的方式如下：

    - 拷贝`local_file`单个文件到开发板：

    ```bash
    scp local_file sunrise@192.168.1.10:/userdata/
    ```

    - 拷贝`local_folder`整个目录到开发板：

    ```bash
    scp -r local_folder sunrise@192.168.1.10:/userdata/
    ```