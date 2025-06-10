---
sidebar_position: 1
---

# 微雪 Audio Driver HAT REV2

## 产品简介

Audio Driver HAT REV2是由微雪电子生产的一款音频转接板，采用ES7210+ES8156双Codec方案，可实现环形4麦克风录音、双通道音频播放、音频信号回采等功能。转接板外观如下图：

![image-audio-driver-hat](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat.jpg)

关于音频子板的详细介绍，请参考[Audio Driver HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm)。

## 安装方法

- ### 硬件部署

1. 按照下图方式，将转接板接入RDK X3的40pin header。  
![image-audio-driver-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-setup.jpg)

2. 使用命令`cat /sys/class/socinfo/som_name`，查询开发板类型，并根据返回值设置音频子板的拨码开关状态。
   - 返回值为5或者6时，3个拨码开关全部拨到`ON`位置。
   - 返回值为8时，3个拨码开关全部拨到`OFF`位置。


- ### 软件配置

1. 使用`srpi-config`配置音频板  
进入`3 Interface Options`->`I5 Audio`  
选择`Audio Driver HAT V2`：
![image-audio-driver-hat-config00](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config00.png)  

2. 运行命令`sync && reboot`重启开发板，如`ls /dev/snd`下出现如下设备节点，说明转接板安装成功。
    ```shell
    root@ubuntu:/userdata# ls /dev/snd
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

- ### 卸载方法
1. 使用`srpi-config`配置音频板   
进入`3 Interface Options`->`I5 Audio`  
选择`UNSET`,即可卸载音频驱动和相关配置

2. 将载板拔掉。

## 运行

### 1. 检查声卡设备

首先，确认声卡是否被系统正确识别和注册。

- 查看已注册的声卡列表：

    ```shell
    cat /proc/asound/cards
    ```
    输出示例：
    ```
     root@ubuntu:~# cat /proc/asound/cards
     0 [hobotsnd5      ]: hobotsnd5 - hobotsnd5
                          hobotsnd5

    ```
    若能看到类似“ hobotsnd5 ”的条目，说明声卡已被识别。

- 查看声卡下的功能设备：
    ```shell
    cat /proc/asound/devices
    ```
    输出示例：
    ```
    root@ubuntu:~# cat /proc/asound/devices
        2: [ 0]   : control
        3: [ 0- 0]: digital audio playback
        4: [ 0- 1]: digital audio capture
        33:        : timer
    ```

### 2. 录音操作

- **2通道麦克风录音**  
  使用tinycap录制2通道音频，参数说明如下：
  - `-D 0`：声卡编号（以上操作查看）
  - `-d 1`：设备编号（以上操作查看）
  - `-c 2`：通道数（2通道）
  - `-b 16`：位宽16bit
  - `-r 48000`：采样率48kHz
  - `-p 512`：每帧采样点数
  - `-n 4`：缓冲区数量
  - `-t 5`：录音时长5秒

  ```shell
  tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **4通道麦克风录音**

  ```shell
  tinycap ./4chn_test.wav -D 0 -d 1 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

### 3. 播放操作

- **双通道音频播放（不支持播放4通道）**  
  使用tinyplay播放录制好的音频文件，常用参数如下：
  - `-D 0`：声卡编号
  - `-d 0`：播放设备编号（以上操作查看）

  ```shell
  tinyplay ./2chn_test.wav -D 0 -d 0
  ```

### 4. 音频回采测试

音频回采功能可用于采集播放通道的信号，便于后续分析。

- **8通道麦克风录音（含回采）**  
  该音频板的回采信号映射在录音通道7和8。需使用8通道录音命令：

  ```shell
  tinycap ./8chn_test.wav -D 0 -d 1 -c 8 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **同时启动双通道音频播放**

  ```shell
  tinyplay ./2chn_test.wav -D 0 -d 0
  ```

- **分析回采信号**  
  录制完成后，可使用如Audacity等音频分析软件，打开`8chn_test.wav`，查看第7、8通道的波形或频谱，验证回采功能是否正常。

## 常见问题排查

- 若未检测到声卡，请检查硬件连接和拨码开关设置是否正确。
- 若录音或播放无声，请确认音频文件格式、通道数与命令参数一致。
- 若回采通道无信号，请确认已正确使用8通道录音命令。

如遇其他问题，可参考[音频常见问题](../../../08_FAQ/04_multimedia.md#audio-常见问题)获取更多帮助。