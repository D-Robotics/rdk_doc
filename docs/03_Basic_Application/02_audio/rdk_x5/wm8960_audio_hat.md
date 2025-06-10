---
sidebar_position: 4
---

# 微雪 WM8960 Audio HAT

## 产品简介
WM8960 Audio HAT是由微雪电子生产的一款音频转接板，采用WM8960 Codec方案，可实现双通道麦克风录音、音频播放功能，转接板外观如下图：

![image-audio-wm8960](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-wm8960.jpg)

关于音频转接板的详细介绍，请参考[WM8960音频转接板](https://www.waveshare.net/wiki/WM8960_Audio_HAT)。
## 安装方法

- ### 硬件部署

按照下图方式，将转接板接入RDK X5的40pin header  
![image-wm8960-audio-hat-rkd-x5-setup-2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-wm8960-audio-hat-rkd-x5-setup-2.png)

- ### 软件配置

1. 使用`srpi-config`配置音频板  
进入`3 Interface Options`->`I5 Audio`  
选择`WM8960 Audio Driver HAT`：
![image-audio-codec-select-wm8960-hat-rdkx5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-codec-select-wm8960-hat-rdkx5.png)  


2. 根据提示执行`reboot`命令，或者运行命令`sync && reboot`重启开发板，`cat /proc/asound/cards`出现了`duplexaudioi2s1`的声卡，说明转接板安装成功。

```shell
root@ubuntu:~# cat /proc/asound/cards 
 0 [duplexaudioi2s1]: simple-card - duplex-audio-i2s1
                      duplex-audio-i2s1
 1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```

- ### 卸载方法
1. 使用`srpi-config`配置音频板   
进入`3 Interface Options`->`I5 Audio`  
选择`UNSET`,即可卸载音频驱动和相关配置

2. 将载板拔掉。

## 运行
检查声卡是否存在，检查设备编号。

通过 `cat /proc/asound/cards` 命令确认声卡是否注册(上述有提到)
```shell
root@ubuntu:~# cat /proc/asound/cards 
 0 [duplexaudioi2s1]: simple-card - duplex-audio-i2s1
                      duplex-audio-i2s1
 1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```

通过`cat /proc/asound/devices` 命令确认逻辑设备
```shell
root@ubuntu:~# cat /proc/asound/devices
  2: [ 0- 0]: digital audio playback
  3: [ 0- 1]: digital audio capture
  4: [ 0]   : control
  5: [ 1- 0]: digital audio playback
  6: [ 1- 0]: digital audio capture
  7: [ 1]   : control
 33:        : timer
```

通过 `ls /dev/snd/` 命令检查用户空间的实际设备文件
```shell
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  controlC1  pcmC0D0p  pcmC0D1c  pcmC1D0c  pcmC1D0p  timer
```
通过上述查询，结合[板载 Earphone 音频口](in_board_es8326.md#运行)的介绍，可以确认，声卡0对应的是 `WM8960 Audio Driver HAT` 节点；设备也是存在的, 且设备号为 `0-0` 和 `0-1` , 实际我们操作的设备应该是`pcmC0D0p 和 pcmC0D1c`。

板载声卡对应的是1，设备号为`1-0`，这里我们不会用到它。

**该音频板需要配置 Audio 路由才可以执行对应的功能，所以在以下每个场景功能执行之前都有加载特定路由的命令。**

### 1. 录音操作

#### 1.1 录音前的音频路由与增益配置

在录音前，需要正确配置音频输入通路和增益，以保证录音效果。以下命令依次设置左右声道的输入增益、录音音量、输入通路开关等：

```shell
# 设置左/右通道输入增益
tinymix -D 0 set 'Left Input Boost Mixer LINPUT1 Volume' 3
tinymix -D 0 set 'Right Input Boost Mixer RINPUT1 Volume' 3

# 可根据实际情况调整增益（如有啸叫可适当降低）
tinymix -D 0 set 'Left Input Boost Mixer LINPUT1 Volume' 1
tinymix -D 0 set 'Right Input Boost Mixer RINPUT1 Volume' 1

# 设置录音音量
tinymix -D 0 set 'Capture Volume' 40,40
tinymix -D 0 set 'ADC PCM Capture Volume' 200,200

# 打开输入通路开关
tinymix -D 0 set 'Left Boost Mixer LINPUT1 Switch' 1
tinymix -D 0 set 'Right Boost Mixer RINPUT1 Switch' 1
tinymix -D 0 set 'Left Input Mixer Boost Switch' 1
tinymix -D 0 set 'Right Input Mixer Boost Switch' 1

# 打开录音开关
tinymix -D 0 set 'Capture Switch' 1,1
```

#### 1.2 开始录音

使用 `tinycap` 命令进行录音，常用参数说明如下：

- `-D 0`：指定声卡编号（此处为 0，即 WM8960）
- `-d 0`：指定设备编号
- `-c 2`：录制双通道（立体声）
- `-b 16`：采样位宽 16bit
- `-r 48000`：采样率 48kHz
- `-p 512`：每周期帧数
- `-n 4`：周期数
- `-t 5`：录音时长 5 秒

示例命令：

```shell
tinycap ./2chn_test.wav -D 0 -d 0 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
```

录音完成后，会生成 `2chn_test.wav` 文件。


### 2. 播放操作

#### 2.1 喇叭播放（双通道）

播放前需配置音频输出通路和音量：

```shell
# 打开左右声道 PCM 播放开关
tinymix -D 0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D 0 set 'Right Output Mixer PCM Playback Switch' 1

# 设置喇叭音量
tinymix -D 0 set 'Speaker DC Volume' 3
tinymix -D 0 set 'Speaker AC Volume' 3
tinymix -D 0 set 'Speaker Playback Volume' 127,127
tinymix -D 0 set 'Playback Volume' 255,255

# 再次确保输出开关已打开
tinymix -D 0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D 0 set 'Right Output Mixer PCM Playback Switch' 1

# 播放录音文件
tinyplay ./2chn_test.wav -D 0 -d 0
```

#### 2.2 耳机与喇叭同时播放

```shell
tinymix -D 0 set 'Headphone Playback Volume' 80,80
tinymix -D 0 set 'Playback Volume' 220,220
tinymix -D 0 set 'Speaker DC Volume' 4
tinymix -D 0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D 0 set 'Right Output Mixer PCM Playback Switch' 1
tinyplay ./2chn_test.wav -D 0 -d 0
```

#### 2.3 仅耳机播放（喇叭静音）

```shell
tinymix -D 0 set 'Headphone Playback Volume' 115,115
tinymix -D 0 set 'Speaker Playback Volume' 0,0
tinymix -D 0 set 'Playback Volume' 244,244
tinymix -D 0 set 'Speaker DC Volume' 4
tinymix -D 0 set 'Left Output Mixer PCM Playback Switch' 1
tinymix -D 0 set 'Right Output Mixer PCM Playback Switch' 1
tinyplay ./2chn_test.wav -D 0 -d 0
```

---


## 常见问题
Q1 : 硬件、软件都运行正常，Audio 通路也正常，喇叭有声音，但是耳机没有声音。

A1 : 可能耳机音量比较小，将耳机音量调大。\
比如使用 `tinymix -D  0 set 'Headphone Playback Volume' 115,115` 命令。

[更多问题可以查看如下链接](../../../08_FAQ/04_multimedia.md#audio-常见问题)