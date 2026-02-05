---
sidebar_position: 4
---

# 1.2.2.4 使用 hbupdate 工具

## 安装 USB 驱动

使用 Windows 系统的用户，在烧录前，请按照如下步骤确认是否安装过 fastboot 驱动程序：

1. 使用跳线帽将 RDK X3 载板的 `Boot` 管脚接地，管脚位置参考下图。
   
       ![image-carrier-board-bootstrap](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-bootstrap.png)

2. 将载板的 Micro USB 接口与电脑通过 USB 线连接，接口位置参考下图。
   
       ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)

3. 给设备上电，然后观察电脑设备管理器端口状态，如出现 `USB download gadget` 未知设备时，需要更新设备驱动，否则可跳过下述步骤。
   
       ![image-usb-driver1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-usb-driver1.png)

4. 下载并解压驱动包 `android_hobot.zip`，下载链接 [android_hobot](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) 。
   
5. 进入解压后的目录，以管理员身份运行 `5-runasadmin_register-CA-cer.cmd`，完成驱动程序的注册。
   
6. 双击`USB download gadget`未知设备，选择驱动包解压目录，然后点击下一步。
   
       ![image-usb-driver2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-usb-driver2.png)

7. 驱动安装完成后，设备管理器会显示 fastboot 设备 `Android Device`。
   
       ![image-usb-driver3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-usb-driver3.png)

## 硬件连接

1. 通过跳线帽将`BOOT`管脚接地，管脚位置参考[功能控制接口](../../hardware_introduction/rdk_x3.md#功能控制接口)。

2. 将 Micro USB 接口连接到电脑，电脑设备管理器中会识别出 `Android Device` 的设备。


## 烧录系统{#flash_system}

1. 运行 `hbupdate.exe` 打开烧录工具，并按照以下步骤进行烧录：

    ![image-flash-system1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system1.png)

2. 选择开发板型号，必选项。

      - RDK_X3_2GB： RDK X3（旭日X3派），2GB内存版本，仅支持烧写最小系统镜像

      - RDK_X3_4GB： RDK X3（旭日X3派），4GB内存版本，仅支持烧写最小系统镜像

      - RDK_X3_MD_2GB： RDK X3 Module，2GB内存版本

      - RDK_X3_MD_4GB： RDK X3 Module，4GB内存版本

        ![image-flash-system2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system2.png)

3. 点击 `Browse` 按钮选择将要烧录的镜像文件，必选项。

        ![image-flash-system3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system3.png)

4. 点击 `Start` 按钮开始烧录：

        ![image-flash-system4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system4.png)


5. 烧录完毕断开电源，断开和电脑的连接线，将BOOT管脚跳线帽拔下，重新上电即可。

    如果启动正常，在硬件上的`ACT LED`灯会进入`两次快闪一次慢闪`的状态

6. 检查升级结果

   - 镜像烧录成功时，工具提示如下：

        ![image-flash-system6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system6.png)

   - 镜像烧录失败时，工具提示如下，此时需要确认PC设备管理器是否存在`Android Device`设备

        ![image-flash-system7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-flash-system7.png)

## 启动系统

## 启动系统

### eMMC 未烧录系统

插入制作好系统的 SD 卡到载板即可通过 SD 卡启动系统。

### eMMC 已烧录系统

**默认情况**

默认从 eMMC 启动系统。

**切换到从 SD 卡启动**

1. 禁用 eMMC 的启动切换到使用 SD 卡启动系统，登录系统后，执行以下命名把 eMMC 的第二个分区的启动标志删除，并重启系统生效：
    
      ```
      sudo parted /dev/mmcblk0 set 2 boot off
      sudo reboot
      ```

2. 在 uboot 下会发现 eMMC 没有启动分区而去寻找 SD 卡的启动分区，从 SD 卡加载系统启动，登录系统后执行 `mount` 命令可以看到跟文件系统挂载在 SD 卡的 第二个分区，config 分区也使用的 SD 卡的第一个分区。

      ```
      /dev/mmcblk2p2 on / type ext4 (rw,relatime,data=ordered) 
      /dev/mmcblk2p1 on /boot/config type vfat
      ```

**从 SD 卡启动切换回从 eMMC 启动**

  当在使用 SD 卡启动系统时，并且 eMMC 上已经烧录过系统，执行以下命令恢复回从 eMMC 启动，重启系统生效。

      ```
      sudo parted /dev/mmcblk0 set 2 boot on
      sudo reboot
      ```
        ![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)