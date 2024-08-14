---
sidebar_position: 1
---

# 8.1 Hardware and System

For certified accessories and purchasing links, please refer to the [Certified Accessories List](https://developer.d-robotics.cc/rdk_doc/en/Advanced_development/hardware_development/rdk_x3/accessory).

For more details, please refer to the [Frequently Asked Questions](../08_FAQ/01_hardware_and_system.md) section in the D-Robotics RDK User Manual.

## What is D-Robotics Developer Kit (RDK)?

D-Robotics Developer Kit [RDK](https://developer.d-robotics.cc/rdk_doc/), is a developer kit for robotics based on D-Robotics's intelligent chips, including **RDK X3**, **RDK X3 Module**.

## How to check the system version number?

After the system installation, log in to the system and use the command `apt list --installed | grep hobot` to check the version of the system's core function packages. Use the command `cat /etc/version` to check the major version number of the system.

The system information for version 2.0.0 is as follows:

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

## Considerations for plugging and unplugging the camera:

**It is strictly prohibited to plug or unplug the camera while the development board is powered on, as it can easily damage the camera module.**

## How to connect the serial cable?

One end of the serial cable (white) is connected to the RDK X3. Since the interface has a groove, the correct orientation is usually maintained. The other end is connected to the serial port adapter board. Pay close attention to the connection diagram below:

![](../../../../../static/img/08_FAQ/image/hardware_and_system/connect.png)

## What are the power requirements for RDK X3?

RDK X3 is powered through a USB Type C interface and is compatible with QC and PD fast charging protocols. It is recommended to use a power adapter that supports QC and PD fast charging protocols, or at least a power adapter with **5V DC 2A** output to supply power to the development board.

**Note: Please do not use the USB interface of a PC to power the development board, as it may cause abnormal operation of the board due to insufficient power supply (such as no HDMI output (completely black screen) after RDK X3 is powered on, the green light is not off, after connecting the serial port, the system keeps rebooting repeatedly and cannot enter the operating system)**.

## Does RDK X3 have a recommended SD card?

It is recommended to use a high-speed C10 SD card with a capacity of at least 16GB. Older cards may have issues with booting the image.

Kingston:  `https://item.jd.com/25263496192.html`

SanDisk:  `https://item.jd.com/1875992.html#crumb-wrap>

## How to connect F37 and GC4663 MIPI cameras?

The F37 and GC4663 camera modules are connected to the development board via a 24-pin FPC cable with opposite side connectors. **Note that the blue side of the cable should be facing up when inserting it into the connector**. The connection diagram for the F37 camera is shown below:

![](../../../../../static/img/08_FAQ/image/hardware_and_system/image-X3-PI-Camera.png)

After a successful connection, power on the board and execute the following command:

```bash
cd /app/ai_inference/03_mipi_camera_sample
sudo python3 mipi_camera.py
```

The HDMI output of the algorithm rendering result is shown in the following image, which detects a "teddy bear", a "cup", and a "vase" in the sample image.

![](../../../../../static/img/08_FAQ/image/hardware_and_system/image-20220511181747071.png)

```text
Enter the command: i2cdetect -y -r 1   
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

## How to view the CPU, BPU, and other running statuses of RDK X3?

```bash
sudo hrut_somstatus
```

## How to set up auto-startup?

You can achieve auto-startup functionality by adding commands to the end of the file "/etc/rc.local" using the following command:

```bash
sudo vim /etc/rc.local
```

Then add the following lines at the end of the file:

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
## No display after power on the development board

<font color='Blue'>[Issue]</font> 

- After power on the development board, the monitor continuously has no output, and connecting to the serial port indicates that the system is repeatedly rebooting or stuck at the uboot command line.

<font color='Green'>[Answer]</font> 

- Insufficient power supply causing repeated reboots, replace the 5V/3A adapter that meets the requirements of the development board, it is recommended to use the officially recommended adapter.
- Poor quality USB cable may cause power instability and abnormal power loss, resulting in repeated reboots.
- UART misoperation causing it to be stuck at the uboot, power cycle the device to recover.
- Micro SD card image format error, when the serial port prompts the following log, it needs to remake the image.

![image-20221124194527634](../../../../../static/img/08_FAQ/image/system/image-20221124194527634.png)

- Micro SD card quality issue, when the serial port prompts the following log, it means the Micro SD card is damaged and needs to be replaced.

![image-20221124194636213](../../../../../static/img/08_FAQ/image/system/image-20221124194636213.png)

![image-20221124194721750](../../../../../static/img/08_FAQ/image/system/image-20221124194721750.png)

## Common phenomena of abnormal power supply for development board

If the status LED of the development board does not turn off or blink continuously after power on, and no display is shown on the HDMI monitor, it is recommended to first check if the power supply is normal.

- Use a power adapter that supports **5V 3A** to power the development board, it is recommended to use the power adapter model recommended in the [Basic Accessories list](https://developer.d-robotics.cc/rdk_doc/en/Advanced_development/hardware_development/rdk_x3/accessory).
- If using your own charger, please choose a USB Type C cable from a reputable brand and ensure it meets the requirement of **5V 3A**.
- Do not directly power the development board from the USB port of a computer.

To determine whether the failure to start properly is caused by a power supply problem, we need to connect the development board to a serial port and observe the startup log. Currently, the following phenomena can clearly determine a power supply abnormality.

### Phenomenon 1: Restarting during Uboot kernel boot

At this time, it is in the Uboot stage, most of the tasks of Uboot have been completed, but when loading the kernel, device tree, etc. from the SD card to memory, or when jumping into the kernel for execution, the development board restarts abnormally.

![image-20230914173433676](../../../../../static/img/08_FAQ/image/hardware_and_system/image-20230914173433676.png)

![image-20230914173911690](../../../../../static/img/08_FAQ/image/hardware_and_system/image-20230914173911690.png)

### Phenomenon 2: Already running in the kernel, restarts after a few seconds

At this time, the kernel has been loaded and running, and the loading and initialization of the kernel and drivers are in progress, but the development board restarts abnormally.

![image-20230914174123619](../../../../../static/img/08_FAQ/image/hardware_and_system/image-20230914174123619.png)

### Other Phenomena:

The phenomenon of insufficient power supply can only be analyzed through serial port logs. If no **errors** or **warnings** are observed during the startup process in the log, and the development board directly prints `NOTICE: fast_boot: 0` and restarts, it can be basically determined that the issue is due to insufficient power supply.

Currently, the phenomena caused by insufficient power supply are easily confused with other phenomena such as SD card recognition failure or hardware damage. It is not easy to make a clear judgment without connecting to the serial port to view the logs. It is recommended to use the power adapter models recommended in the [basic accessory list](/hardware_development/rdk_x3/accessory#basic_accessories).

## Default Accounts of the Development Board

<font color='Blue'>【Question】</font>

- What types of accounts are supported by default on the development board?

<font color='Green'>【Answer】</font>

- The development board supports two types of accounts by default, as follows:
  - Default account: username `sunrise`, password `sunrise`
  - Root account: username `root`, password `root`

## Mounting NTFS File System
<font color='Blue'>【Question】</font>

- How to support read-write mode after mounting the NTFS file system?

<font color='Green'>【Answer】</font>

- After installing the ntfs-3g package, you can mount the NTFS file system to support write mode. The installation command is as follows:
    ```bash
    sudo apt -y install ntfs-3g
    ```

## Supported by vscode Tool
<font color='Blue'>【Question】</font>

- Does the development board support the `vscode` development tool?

<font color='Green'>【Answer】</font>

- The development board does not support local installation of `vscode`. Users can remotely connect to the development board through the `ssh-remote` plugin on the PC.

## adb Debugging Function
<font color='Blue'>【Question】</font>

- How to enable the adb debugging function on the development board?

<font color='Green'>【Answer】</font>

- The `adbd` service is enabled by default in Ubuntu system. Users only need to install the adb tool on the computer to use it. The method can refer to [bootloader image update](https://developer.d-robotics.cc/forumDetail/88859074455714818).

## apt update Update Fail

<font color='Blue'>[Question]</font> 

- When running the `apt update` command in Ubuntu system, the following error is prompted:
    ```bash
    Reading package lists... Done
    E: Could not get lock /var/lib/apt/lists/lock. It is held by process 4299 (apt-get)
    N: Be aware that removing the lock file is not a solution and may break your system.
    E: Unable to lock directory /var/lib/apt/lists/
    ```

<font color='Green'>[Answer]</font> 

- The automatic update program in Ubuntu system conflicts with the operation `apt update` by the user. You can kill the automatic update process and try again, for example, `kill 4299`.

## File Transfer Methods for Development Boards

<font color='Blue'>[Question]</font> 

- What are the methods for file transfer between development boards and computers?

<font color='Green'>[Answer]</font> 

- File transfer can be done through network, USB, and other methods. For network transfer, you can use ftp tools, scp command, etc. For USB transfer, you can use USB flash drive, adb, etc. Taking scp command as an example, the file transfer methods are as follows:

    - Copy a single file `local_file` to the development board:

    ```bash
    scp local_file sunrise@192.168.1.10:/userdata/
    ```

    - Copy the entire directory `local_folder` to the development board:

    ```bash
    scp -r local_folder sunrise@192.168.1.10:/userdata/
    ```