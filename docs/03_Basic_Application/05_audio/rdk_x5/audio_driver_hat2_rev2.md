---
sidebar_position: 3
---

# 微雪 Audio Driver HAT REV2

## 产品简介
Audio Driver HAT REV2是由微雪电子生产的一款音频转接板，采用ES7210+ES8156双Codec方案，可实现环形4麦克风录音、双通道音频播放、音频信号回采等功能。转接板外观如下图：

![image-audio-driver-hat](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat.jpg)

关于音频子板的详细介绍，请参考[Audio Driver HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm)。

## 安装方法

- ### 硬件部署
1. 按照下图方式，将转接板接入RDK X5的40pin header。  
![image-audio-driver-hat-setup](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-driver-hat-setup.jpg)

2. 3个拨码开关全部拨到`OFF`位置。

- ### 软件配置
1. 使用`srpi-config`配置音频板  
进入`3 Interface Options`->`I5 Audio`  
选择`Audio Driver HAT V2`：
![image-audio-codec-select-hat-v2-rdkx5.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_audio/image/image-audio-codec-select-hat-v2-rdkx5.png)

2. 根据提示执行`reboot`命令，或者运行命令`sync && reboot`重启开发板，`cat /proc/asound/cards`出现除了`duplexaudio`的声卡，说明转接板安装成功。
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

### 1. 检查声卡设备

通过 `cat /proc/asound/cards` 命令确认声卡是否注册(上述有提到)
```shell
root@ubuntu:~# cat /proc/asound/cards 
 0 [duplexaudioi2s1]: simple-card - duplex-audio-i2s1
                      duplex-audio-i2s1
 1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```

通过 `cat /proc/asound/devices` 命令确认逻辑设备
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
通过上述查询，结合[板载 Earphone 音频口](in_board_es8326.md#运行)的介绍，可以确认，声卡0对应的是 `Audio Driver HAT REV2` 节点；设备也是存在的, 且设备号为 `0-0` 和 `0-1` , 实际我们操作的设备应该是`pcmC0D0p 和 pcmC0D1c`。

板载声卡对应的是1，设备号为`1-0`，这里我们不会用到它。


- ### 2. 录音操作

- **2通道麦克风录音**  
  使用tinycap录制2通道音频：

  ```shell
  tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **4通道麦克风录音**

  ```shell
  tinycap ./4chn_test.wav -D 0 -d 1 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

### 3. 播放操作

- **双通道音频播放（不支持播放4通道）**  
  使用tinyplay播放录制好的音频文件：

  ```shell
  tinyplay ./2chn_test.wav -D 0 -d 0
  ```

### 4. 音频回采测试

音频回采功能可用于采集播放通道的信号，便于后续分析。

- **8通道麦克风录音（含回采）**  
  该音频板的回采信号映射在录音通道 7 和 8 。需使用 8 通道录音命令（如果使用同一个 I2S ，需要保持通道数、位深、采样率对齐，这款转接板根据拨码开关固定了一路 I2S）：

  ```shell
  tinycap ./8chn_test.wav -D 0 -d 1 -c 8 -b 16 -r 48000 -p 512 -n 4 -t 5
  ```

- **同时启动格式对齐的8通道音频播放**

  ```shell
  tinyplay ./8chn_test.wav -D 0 -d 0
  ```
  这个 8ch_test.wav 可以是自己制作的格式对齐的正弦波音频文件，这样方便分析。

- **分析回采信号**  
  录制完成后，可使用如 Audacity 等音频分析软件，打开 `8chn_test.wav`，查看第 7 、 8 通道的波形或频谱，验证回采功能是否正常。

## 常见问题排查

- 若未检测到声卡，请检查硬件连接和拨码开关设置是否正确。
- 若录音或播放无声，请确认音频文件格式、通道数与命令参数一致。
- 若回采通道无信号，请确认已正确使用 8 通道录音命令，如果是使用了同一组 I2S ，请确认数据是否对齐。

如遇其他问题，可参考[音频常见问题](../../../08_FAQ/04_multimedia.md#audio-常见问题)获取更多帮助。