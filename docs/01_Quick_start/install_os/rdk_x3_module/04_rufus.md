---
sidebar_position: 4
---

# 1.2.2.4 使用 Rufus 工具

## SD 卡烧录
### 硬件连接

将 SD 卡插入读卡器，将读卡器插入 PC 相应接口。

### 系统烧录



如需烧录系统到SD上（不从eMMC模式启动），系统烧录步骤与 [RDK X3 系统烧录步骤](../rdk_x3/04_rufus.md#系统烧录) 相同。

## eMMC 烧录

### 硬件连接

1. 使用跳线帽将 RDK X3 载板切换到 3.3V 供电。
       
       ![image-X3MD-3v3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-3v3.PNG)  

2. 将载板的 Micro USB 接口（调试串口）与电脑通过 USB 线连接，接口位置参考下图。 
   
       ![image-X3MD-MicroUSB](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/X3MD-MicroUSB.PNG)   

3. 将载板的 Micro USB 接口（烧录口）与电脑通过 USB 线连接，接口位置参考下图。  
   
       ![image-carrier-board-microusb](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/image-carrier-board-microusb.png)


### 系统烧录

1. 在 PC 上通过串口工具连接设备，波特率设置为 921600。启动设备时，长按空格键即可进入 U-Boot 命令行界面。
   
       ![imagex3md-ums1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums1.PNG)  

2. 在 U-Boot 中执行 watchdog off 关闭看门狗，防止设备重启，执行 ums 0 mmc 0 ，将板载 eMMC 设备（设备号 0）通过 USB OTG 接口 0 映射为 USB Mass Storage 设备，使得主机系统可以将其识别为标准 U盘，从而进行直接读写或烧录操作。

          ```shell
            watchdog off
            ums 0 mmc 0
          ```

           ![imagex3md-ums2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums2.png) 

3. PC 识别到标准 U 盘就是 RDK X3 Module 的 eMMC 分区。
   
       ![imagex3md-ums3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums3.png) 

4. 打开 Rufus 工具，在 “设备” 下拉框中选择对应的盘符作为目标设备，其余步骤与[RDK X3 烧录](../rdk_x3/04_rufus.md#系统烧录)一致，完成镜像烧录。
   
       ![imagex3md-ums4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x3_module/image/rdk_x3_module/x3md-ums4.png)

    :::warning 注意

    如烧录过程发生中断，请按照上述步骤重新进行。
    :::

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