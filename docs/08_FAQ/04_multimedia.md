---
sidebar_position: 4
---

# 8.4 多媒体类

## 视频编\解码

<font color='Blue'>【问题】</font> 

- 开发板解码rtsp视频流报错，错误如下：  
![image-20220728110439753](../../static/img/08_FAQ/image/multimedia/image-20220728110439753.png)

<font color='Green'>【解答】</font> 

- 推流服务器推送的rtsp码流里面需要包含`PPS`和`SPS`参数信息

- 使用`ffmpeg`打开`.mp4 .avi`等格式的视频文件推流时，需要添加`-vbsf h264_mp4toannexb`选项，以添加码流的`PPS` 和`SPS`信息，例如：

    ```
    ffmpeg -re -stream_loop -1 -i xxx.mp4 -vcodec copy -vbsf h264_mp4toannexb -f rtsp rtsp://192.168.1.195:8554/h264_stream
    ```

- rtsp视频流目前仅支持1080p分辨率

- 不支持使用vlc软件进行rtsp推流，原因是vlc软件不支持添加`PPS`和`SPS`信息


## Audio 常见问题

### USB 声卡和板载声卡如何区分使用

我们发现有些小伙伴在使用的时候可能会插上一些USB音频设备，导致复制粘贴下文的参考命令无法正常使用。

嵌入式音频设备中一般通过 `cat /proc/asound/cards`可以看到有多少张声卡，每张的序号。
插上USB设备之后重启，UAC设备声卡首先注册上，执行上述命令可能出现如下打印：

```
  0 [RC08           ]: USB-Audio - ROCWARE RC08
                      ROCWARE RC08 at usb-xhci-hcd.2.auto-1.2, high speed
  1 [duplexaudio    ]: simple-card - duplex-audio
                      duplex-audio
```
此时我们发现板载audio声卡的序号变为了1，amixer或者tinymix在没有指定device和card编号的时候，默认
都是使用序号为0的设备，所以此时我们在查看板载声卡的controls等属性的时候可以使用如下命令：

```
  amixer -D 0 -c 1 controls
```
此时如果我们想要通过amixer调整麦克风增益，可以通过如下命令来尝试设置：
```
  amixer -D 0 -c 1 sget 'ADC PGA Gain',0
```


### RDK X3 系列音频子板和USB声卡共存
如果您有USB声卡，并且想它和上述的音频子板共存，请参考下列的教程进行操作：

1. 根据对应设备的教程，确保音频子板可用

2. 接入USB声卡，驱动加载完成后观察`/dev/snd`下面的新增节点，此处以`WM8960` + USB**全双工**声卡为例：
```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D0p  pcmC0D1c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer
```
其中`pcmC1D0c  pcmC1D0p`为USB声卡节点，全双工共用一个节点

3. 修改`/etc/pulse/default.pa`，在`load-module module-alsa-source`下面添加对应的节点信息：
```apacheconf
...

.ifexists module-udev-detect.so
load-module module-alsa-sink device=hw:0,1 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2 rewind_safeguard=960
load-module module-alsa-source device=hw:0,0 mmap=false tsched=0 fragments=2 fragment_size=960 rate=48000 channels=2
load-module alsa device=hw:1,0 #对应上面的节点
load-module module-alsa-source device=hw:1,0 #对应上面的节点
.else

...
```

4. 保存配置，并重启开发板
