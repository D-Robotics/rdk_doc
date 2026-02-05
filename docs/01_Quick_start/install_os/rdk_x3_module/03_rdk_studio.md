---
sidebar_position: 3
---

# 1.2.2.3 使用 RDK Studio 工具

## SD 卡烧录

RDK Studio 工具提供烧录系统功能，并且可以连接设备并进行管理，支持在 Windows、Linux、Mac 平台上使用，详细步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

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

4. 烧录步骤参见 [使用 RDK Studio 烧录系统](../../09_RDK_Studio/04_flashing.md)。

