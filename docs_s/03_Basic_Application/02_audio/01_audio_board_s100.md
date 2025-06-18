---
sidebar_position: 1
---

# 3.2.1 RDK S100 系列音频使用指南

S100音频基于标准alsa框架进行开发，用户空间使用开源代码alsa-lib提供的库和二进制bin进行功能测试。该章节基于此说明S100上音频功能测试的基本方法。

## alsa-lib介绍

### arecord/aplay参数介绍

- 常用参数说明

| 参数              | 说明                                                         | 引申含义                                |
|-------------------|--------------------------------------------------------------|-----------------------------------------|
| -D                | 指定设备节点信息。通过 hw:x,x 设置                           | x,x 分别对应声卡号和设备号             |
| -c                | Channel：声道数，分为单声道 mono 和立体声 stereo，还有多声道如 8/16 通道通道数 |                                         |
| -r                | Rate：又称 sample rate，采样率，即每秒的采样次数             |                                         |
| -f                | 样本长度                                                     | 常见位宽格式设置：U8/S16_LE/S32_LE     |
| -t                | 文件类型。                                                   | 包含 wav、voc、raw 等                   |
| -d                | 录音时间（以秒为单位）。录制指定时长后结束                   |                                         |
| --period-size     | 每次硬件中断处理音频数据的帧数                               |                                         |
| --buffer-size     | 数据缓冲区大小                                               |                                         |
| -I                | 非交错方式                                                   |                                         |

- 常用命令

  - 列出声卡和数字音频设备信息

  ```
  arecord -l
  aplay -l
  ```

  - 硬件支持能力

  ```
  arecord --dump-hw-params -Dhw:0,0
  aplay --dump-hw-params -Dhw:0,1 /dev/zero
  ```

  - 回环测试命令

  ```
  arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 10 | aplay -Dhw:0,1
  ```

  - 非交错方式数据存储。注意：非交错存储的数据是PCM裸流

  ```
  arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 5 -I 0
  aplay -Dhw:0,1 -c 2 -r 48000 -f S16_LE -t wav -I 0
  ```

  - 交错方式数据存储

  ```
  arecord -Dhw:0,0 -c 2 -r 16000 -f S16_LE -t wav -d 5 test.wav
  aplay -Dhw:0,1 test.wav
  ```

### 控制命令

- 查询当前codec的所有control信息以及对应值

```
amixer scontrols
amixer scontents
```

- 参数调整

比如调整播放音量值

```
amixer sset 'DAC' 120 //设置
amixer sget 'DAC' //获取
```

:::info 注意

默认声卡/设备号是0,0，如果实际场景接入多个声卡设备，需要通过-D，-c指定设备、声卡号。
:::

确认当前要调整的声卡/设备号方法：

- arecord -l

```
**** List of CAPTURE Hardware Devices ****
card 0: s100snd2 [s100snd2], device 0: s100dailink0 ES7210 4CH ADC 0-0 []
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

- /proc/asound/cards节点

```
root@ubuntu:/userdata# cat /proc/asound/cards
 0 [s100snd2       ]: s100snd2 - s100snd2
                      s100snd2
```

### 应用空间接口

API描述及使用参考官方文档介绍：https://www.alsa-project.org/alsa-doc/alsa-lib/pcm_2pcm_8c.html

## Audio Driver HAT REV2

### 使用准备

S100适配微雪电子生产的音频转接板，通过40PIN实现和S100开发板连接。子板介绍和具体连接方法参考：[音频子板使用说明](../../07_Advanced_development/02_linux_development/04_driver_development_s100/11_driver_audio.md#音频子板使用说明)

### 设备节点

该音频板接入S100加载驱动后，生成的设备节点：

- pcmC0D0c：录音

- pcmC0D1p：播放

- controlC0：控制

### 功能测试

- 录音

```
arecord -Dhw:0,0 -c 2 -r 48000 -f S16_LE -t wav -d 5 test.wav
```

表示录制5s，48k/2ch/16bit位宽的wav音频文件。数据格式用户可以根据自己的需求调整并设置。

- 播放

```
aplay -Dhw:0,1 test.wav
```

表示播放一个wav文件，数据格式会解析wav文件头获取并写入驱动
