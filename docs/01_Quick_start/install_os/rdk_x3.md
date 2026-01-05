---
sidebar_position: 1
---

# 1.2.1 RDK X3


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

在使用RDK X3开发板前，需要做下述准备工作。

## 烧录准备

### **供电**

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

RDK X3开发板通过USB Type C接口供电，需要使用支持**5V/3A**的电源适配器为开发板供电，推荐使用[基础配件清单](../../07_Advanced_development/01_hardware_development/rdk_x3/accessory.md)中推荐的电源适配器型号。

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module通过载板上的电源接口供电，[官方载板](../../07_Advanced_development/01_hardware_development/rdk_x3_module/accessory.md)通过DC接口供电，推荐使用认证配件清单中推荐的**12V/2A**适配器。

</TabItem>


</Tabs>

:::caution

请不要使用电脑USB接口为开发板供电，否则会因供电不足造成开发板**异常断电、反复重启**等异常情况。

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节。

:::



### **存储** 
<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

RDK X3开发板采用Micro SD存储卡作为系统启动介质，推荐至少8GB容量的存储卡，以便满足Ubuntu系统、应用功能软件对存储空间的需求。

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module板载eMMC（可选），支持从eMMC和SD卡两种模式启动系统。


</TabItem>
</Tabs>



### **显示** 
<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

RDK X3开发板支持HDMI显示接口，通过HDMI线缆连接开发板和显示器，支持图形化桌面显示。

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module支持HDMI显示接口，通过HDMI线缆连接官方载板和显示器，支持图形化桌面显示。

</TabItem>

</Tabs>



### **网络连接**
<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

RDK X3开发板支持以太网、Wi-Fi两种网络接口，用户可通过任意接口实现网络连接功能。

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module支持以太网、Wi-Fi（可选）两种网络接口，用户可通过任意接口实现网络连接功能。

</TabItem>
</Tabs>


## 系统烧录


RDK套件目前提供Ubuntu 20.04/22.04系统镜像，可支持Desktop桌面图形化交互。

:::info 注意

**RDK X3 Module**出厂已经烧写测试版本系统镜像，为确保使用最新版本的系统，<font color='Red'>建议参考本文档完成最新版本系统镜像的烧写</font>。
:::

### 镜像下载 {#img_download}

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=1

点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images)，进入版本选择页面，选择对应版本目录，进入文件下载页。以下载2.0.0版本的系统镜像为例：

![image-20230510143353330](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20230510143353330.png)

![image-20230510143353330](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20230510143353330.png)


下载完成后，解压出Ubuntu系统镜像文件，如`ubuntu-preinstalled-desktop-arm64.img`

**版本说明：**

- 2.0版本：基于RDK Linux开源代码包制作，支持RDK X3派、X3模组等全系列硬件
- 1.0版本：旭日X3派历史版本，仅支持旭日X3派硬件，系统镜像名为`system_sdcard.img`

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

点击 [**下载镜像**](https://archive.d-robotics.cc/downloads/os_images)，进入版本选择页面，选择对应版本目录，进入文件下载页。以下载2.0.0版本的系统镜像为例：

![image-20230510143353330](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20230510143353330.png)

下载完成后，解压出Ubuntu系统镜像文件，如`ubuntu-preinstalled-desktop-arm64.img`

**版本说明：**

- 2.0版本：基于RDK Linux开源代码包制作，支持RDK X3派、X3模组等全系列硬件
- 1.0版本：旭日X3派历史版本，仅支持旭日X3派硬件，系统镜像名为`system_sdcard.img`

</TabItem>


</Tabs>

:::tip

- desktop：带有桌面的Ubuntu系统，可以外接屏幕、鼠标操作
- server：无桌面的Ubuntu系统，可以通过串口、网络远程连接操作
:::



### 系统烧录

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

:::tip

在烧录Ubuntu系统镜像前，需要做如下准备：
- 准备至少8GB容量的Micro SD卡
- SD 读卡器
- 下载地瓜提供的烧录工具 RDK Studio（可[点击此处下载](https://developer.d-robotics.cc/rdkstudio)）或镜像烧录工具Rufus（可[点击此处前往官网](https://rufus.ie/)）
  
:::

<Tabs groupId="flashing-type">

<TabItem value="RDK Studio" label="RDK Studio 工具">

使用 RDK Studio 工具烧录系统后可以添加设备进行管理，建议使用地 RDK Studio 工具，详细步骤参见 [使用 RDK Studio 烧录系统](../09_RDK_Studio/03_flashing.md)。


</TabItem>

<TabItem value="Rufus" label="Rufus 工具">

Rufus是一款支持Windows平台的启动盘制作工具，使用Rufus制作SD启动卡流程如下：
1. 打开Rufus工具，在“设备”下拉框中选择对应的Micro SD存储卡作为目标设备。

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. 点击“选择”按钮，选择解压出来的`ubuntu-preinstalled-desktop-arm64.img`文件作为烧录镜像。

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. 其他参数保持默认，点击“开始”按钮，等待烧录完成。烧录完成后，可以关闭Rufus并取出存储卡。

    ![image-rufus-flash](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-flash.png)

</TabItem>



</Tabs>
</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module支持从eMMC和SD卡两种模式启动系统：

- **使用SD卡**：如需烧录系统到SD上（不从eMMC模式启动），系统烧录步骤与 [RDK X3 系统烧录步骤](#系统烧录) 相同。
- **使用eMMC**：使用UMS方式烧录系统镜像（**以下主要介绍该方法**）

#### 硬件连接


   （1）使用跳线帽将RDK X3载板切换到3.3V供电。    
   ![image-X3MD-3v3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-3v3.PNG)  

   （2）将载板的Micro USB接口（调试串口）与电脑通过USB线连接，接口位置参考下图。 
   ![image-X3MD-MicroUSB](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-MicroUSB.PNG)   

   （3）将载板的Micro USB接口（烧录口）与电脑通过USB线连接，接口位置参考下图。  
   ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)  

#### 系统烧录

   （1）在 PC 上通过串口工具连接设备，波特率设置为 921600。启动设备时，长按空格键即可进入 U-Boot 命令行界面。
   ![imagex3md-ums1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums1.PNG)  

   （2）在 U-Boot 中执行 watchdog off 关闭看门狗，防止设备重启，执行 ums 0 mmc 0 ，将板载 eMMC 设备（设备号 0）通过 USB OTG 接口 0 映射为 USB Mass Storage 设备，使得主机系统可以将其识别为标准 U盘，从而进行直接读写或烧录操作。

  ```shell
    watchdog off
    ums 0 mmc 0
  ```

   ![imagex3md-ums2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums2.png) 

   （3）PC识别到标准 U盘就是RDK X3 Module的EMMC分区。
   ![imagex3md-ums3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums3.png) 

   （4）打开Rufus工具，在“设备”下拉框中选择对应的盘符作为目标设备，完成镜像烧录。
   ![imagex3md-ums4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums4.png) 




:::caution

如烧录过程发生中断，请按照上述步骤重新进行。
:::
</TabItem>

</Tabs>


### 启动系统

<Tabs groupId="rdk-type">
<TabItem value="x3" label="RDK X3">

首先保持开发板断电，然后将制作好的存储卡插入开发板的Micro SD卡槽，并通过HDMI线缆连接开发板与显示器，最后给开发板上电。

系统首次启动时会进行默认环境配置，整个过程持续45秒左右，配置结束后会在显示器输出Ubuntu系统桌面。



:::tip 开发板指示灯说明



* **<font color='Red'>红色</font>**指示灯：点亮代表硬件上电正常
* **<font color='Green'>绿色</font>**指示灯：点亮代表系统启动中，熄灭或闪烁代表系统启动完成



如果开发板上电后长时间没有显示输出（2分钟以上），说明开发板启动异常。此时用户可通过指示灯确认系统状态，方法如下：

* **<font color='Green'>绿灯</font>**常亮：说明系统启动失败，可检查使用的电源适配器是否满足要求，可尝试重新制作系统镜像
* **<font color='Green'>绿灯</font>**熄灭或闪烁：说明系统启动成功，但显示服务启动失败，请确认连接的显示器符合支持列表规格

:::

</TabItem>

<TabItem value="x3md" label="RDK X3 Module">

RDK X3 Module支持从eMMC和SD卡两种模式启动系统：

- 当模组上的eMMC没有烧录过系统镜像的情况下，插入制作好系统的SD卡到载板即可通过从SD卡启动系统。

- 如果模组上的eMMC已经烧录过系统镜像，可以按照以下步骤进行eMMC和SD卡启动的切换。

  1、默认情况下会从eMMC启动系统

  2、禁用eMMC的启动切换到使用SD卡启动系统，登录系统后，执行以下命名把eMMC的第二个分区的启动标志删除，并重启系统生效：

  ```
  sudo parted /dev/mmcblk0 set 2 boot off
  sudo reboot
  ```

  3、在uboot下会发现eMMC没有启动分区而去寻找SD卡的启动分区，从SD卡加载系统启动，登录系统后执行`mount`命令可以看到跟文件系统挂载在 SD 卡的 第二个分区，config分区也使用的SD卡的第一个分区。

  ```
  /dev/mmcblk2p2 on / type ext4 (rw,relatime,data=ordered) 
  /dev/mmcblk2p1 on /boot/config type vfat
  ```

- 从SD卡启动切换回从eMMC启动

  当在使用SD卡启动系统时，并且eMMC上已经烧录过系统，执行以下命令恢复回从eMMC启动，重启系统生效。

  ```
  sudo parted /dev/mmcblk0 set 2 boot on
  sudo reboot
  ```

</TabItem>


</Tabs>



Ubuntu Desktop版本系统启动完成后，会通过HDMI接口在显示器上输出系统桌面，如下图：

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)

## **常见问题**  

首次使用开发板时的常见问题如下：

- **<font color='Blue'>上电不开机</font>** ：请确保使用[供电](#供电)中推荐的适配器供电；请确保开发板的Micro SD卡或eMMC已经烧录过Ubuntu系统镜像
- **<font color='Blue'>USB Host接口无反应</font>** ：请确保开发板Micro USB接口没有接入数据线
- **<font color='Blue'>使用中热插拔存储卡</font>** ：开发板不支持热插拔Micro SD存储卡，如发生误操作请重启开发板



### **注意事项**

- 禁止带电时拔插除USB、HDMI、网线之外的任何设备
- RDK X3的 Type C USB接口仅用作供电 
- 选用正规品牌的USB Type C 口供电线，否则会出现供电异常，导致系统异常断电的问题



:::tip

更多问题的处理，可以查阅 [常见问题](../../08_FAQ/01_hardware_and_system.md) 章节，同时可以访问 [D-Robotics 开发者官方论坛](https://developer.d-robotics.cc/forum) 获得帮助。

:::