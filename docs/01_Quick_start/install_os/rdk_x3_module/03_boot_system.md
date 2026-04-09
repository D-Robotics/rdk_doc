---
sidebar_position: 3
---

# 1.2.2.3 启动系统

## eMMC 未烧录系统

首先保持开发板断电，然后将制作好系统的 SD 卡插入载板 Micro SD 卡槽，并通过 HDMI 线缆连接开发板与显示器，最后给开发板上电即可通过 SD 卡启动系统。

## eMMC 已烧录系统

### 默认情况

通过 HDMI 线缆连接开发板与显示器，给开发板上电，默认从 eMMC 启动系统。

### 切换到从 SD 卡启动

1. 将制作好系统的 SD 卡插入载板 Micro SD 卡槽，并通过 HDMI 线缆连接开发板与显示器。
2. 禁用 eMMC 的启动以切换到使用 SD 卡启动系统：登录系统后，执行以下命令把 eMMC 的第二个分区的启动标志删除，并重启系统生效：

        ```bash
        sudo parted /dev/mmcblk0 set 2 boot off
        sudo reboot
        ```

3. 在 U-Boot 下会发现 eMMC 没有启动分区而去寻找 SD 卡的启动分区，从 SD 卡加载系统启动。登录系统后执行 `mount` 命令可以看到根文件系统挂载在 SD 卡的第二个分区，`config` 分区也使用 SD 卡的第一个分区：

        ```text
        /dev/mmcblk2p2 on / type ext4 (rw,relatime,data=ordered)
        /dev/mmcblk2p1 on /boot/config type vfat
        ```

### 从 SD 卡启动切换回从 eMMC 启动

当在使用 SD 卡启动系统时，并且 eMMC 上已经烧录过系统，执行以下命令恢复从 eMMC 启动，重启系统生效：

        ```bash
        sudo parted /dev/mmcblk0 set 2 boot on
        sudo reboot
        ```

        ![Ubuntu 桌面示例](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)
