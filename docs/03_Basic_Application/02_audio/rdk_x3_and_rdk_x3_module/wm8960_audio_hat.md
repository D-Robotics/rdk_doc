---
sidebar_position: 2
---

# 微雪 WM8960 Audio HAT

## 产品简介
WM8960 Audio HAT是由微雪电子生产的一款音频转接板，采用WM8960 Codec方案，可实现双通道麦克风录音、音频播放功能，转接板外观如下图：

![image-audio-wm8960](../../../../static/img/03_Basic_Application/02_audio/image/image-audio-wm8960.jpg)

关于音频转接板的详细介绍，请参考[WM8960音频转接板](https://www.waveshare.net/wiki/WM8960_Audio_HAT)。

## 安装方法

- ### 硬件部署

1. 按照下图方式，将转接板接入RDK X3的40pin header  
![image-wm8960-audio-hat-setup](../../../../static/img/03_Basic_Application/02_audio/image/image-wm8960-audio-hat-setup.jpg)

- ### 软件配置

2. 使用`srpi-config`配置音频板  
进入`3 Interface Options`->`I5 Audio`  
选择`WM8960 Audio HAT`：
![image-audio-driver-hat-config00](../../../../static/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-config01.png)  


3. 运行命令`sync && reboot`重启开发板，如`/dev/snd`下出现如下设备节点，说明转接板安装成功
    ```shell
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  timer
    ```
该音频板在`RDK X3`上的的播放节点为`pcmC0D0p`，录制节点为`pcmC0D1c`

- ### 卸载方法
1. 使用`srpi-config`配置音频板   
进入`3 Interface Options`->`I5 Audio`  
选择`UNSET`,即可卸载音频驱动和相关配置

2. 将载板拔掉。

## 运行

检查声卡是否存在，检查设备编号。

通过如下命令确认声卡是否注册
```
cat /proc/asound/cards 
```

通过如下命令确认功能设备的位置
```
cat /proc/asound/devices
```


- ### 录音
2通道麦克风录音：

```
tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

- ### 播放
双通道音频播放：

```
tinyplay ./2chn_test.wav -D 0 -d 0
```


## 常见问题
[可以查看如下链接](../../../08_FAQ/04_multimedia.md#audio-常见问题)