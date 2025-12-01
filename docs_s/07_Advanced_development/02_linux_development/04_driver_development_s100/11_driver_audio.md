---
sidebar_position: 11
---

# 音频调试指南

本章主要是S100 I2S的基本特性，语音基础知识以及添加codec、调试声卡的说明

## 概述

CPU DAI: CPU侧的数字音频接口，一般是i2s接口，控制总线传输

CODEC DAI：即codec。控制codec工作流，提供给core层

DAI LINK：绑定CPU DAI和CODEC DAI，指硬件控制器驱动

PLATFORM：指定CPU侧的平台驱动，通常是DMA驱动，用于传输

DAPM：动态音频电源管理

## 音频开发说明

一个完整的声卡由cpu dai、codec dai、platform、sound card构成。

cpu dai driver：一般是I2S接口驱动。

codec dai driver：指的是外接codec驱动。

platform driver：通常是dma驱动，用于完成对音频数据的管理。

sound card driver：用来负责连接cpu dai和codec dai，比如sound/soc/hobot/hobot-snd-s100-ac-fdx-host.c。

### I2S参数说明

S100 I2S芯片支持能力说明：

- 通道支持：支持1/2/4/8/16通道；
- 采样率支持：8k/16k/32k/48k/44.1k/96k/192k；
- 采样精度支持：8bit/16bit/24bit/32bit；
- 传输协议支持：I2S/DSP(TDM);
- I2S支持配置master或者slave模式；
- I2S模块的sysclk需要为bclk的6倍及以上；
- 支持全双工模式。在全双工模式时sdio0为输入，sdio1为输出，不能更改；
- bclk输出不能超过25MHz

### 新增Codec说明

#### 添加codec driver

将所添加的codec驱动文件增加到sound/soc/codecs/目录下。

#### 添加编译选项

- 通过修改sound/soc/codec/Kconfig以及Makefile添加codec驱动的编译配置。

其中Kconfig添加内容参考如下：

```
config SND_SOC_ES7210
        tristate "ES7210 Audio Codec"
        depends on I2C
```

Makefile添加内容参考：

```
obj-$(CONFIG_SND_SOC_ES7210)   += snd-soc-es7210.o
```

- 通过menuconfig修改config配置，使能编译

```
sudo ./mk_kernel.sh menuconfig
```
执行以上命令后，打开Kernel Configuration配置界面，输入```CONFIG_SND_SOC_ES7210```并使能。

#### 修改dts

dts修改一般涉及以下：

- 增加codec驱动节点。一般通过i2c控制codec寄存器，因此需要在i2c节点添加codec信息，参考如下：

```
i2c5: i2c@39470000 {
    es7210_0: es7210_0@40 {
            compatible = "MicArray_0";
            reg = <0x40>;
            #sound-dai-cells = <1>;
            channels = <8>;
            adc_dev_num = <2>;
            status = "okay";
    };
};
```

- 增加sound card节点。建立codec与i2s的dai link绑定关系，参考如下：

```
snd2: snd2 {
        status = "okay";
        model = "s100snd2";
        compatible = "hobot, s100-snd-ac-fdx-master";
        i2s_mode = <1>;/*1:i2s mode; 7:dsp_a mode*/
        work_mode = <1>;/*0:hal-duplex; 1:full-duplex*/
        channel_max = <2>;
        mclk_set = <24576000>;
        dai-link@0 {
                dai-format = "dsp_a"; //"i2s"/"dsp_a" //对应SND_SOC_DAIFMT_DSP_A
                // bitclock-master; /配置*-master，对应dai_fmt等价于SND_SOC_DAIFMT_CBM_CFM。表示指定codec作为master提供时钟源
                // frame-master;
                // frame-inversion; //对应SND_SOC_DAIFMT_NB_IF，对应帧同步信号的极性反转。正常情况左声道对应低电平，右声道对应高电平。启动frame-inversion后极性反转
                link-name = "s100dailink0";
                cpu {
                        sound-dai = <&i2s0 0>; //数字音频接口，控制总线传输
                };
                codec {
                        sound-dai = <&es7210_0 0>; //codec芯片接口
                };
        };

        dai-link@1 {
                dai-format = "dsp_a";
                link-name = "s100dailink1";
                cpu {
                        sound-dai = <&i2s0 1>;
                };
                codec {
                        sound-dai = <&es8156>;
                };
        };
};
```

- 另外，在dts中增加了dummy_codec节点，在没有外部codec或codec需要单独配置不依赖alsa框架的情况下，通过注册绑定虚拟codec的声卡设备方式实现用户空间创建设备节点控制I2S/DMA。

## 调试说明

### 声卡调试

:::info 注意

本节声卡调试基于S100外接微雪音频子板的方案进行介绍。
:::

S100上声卡模块默认以ko形式存在，以动态加载方式挂载。挂载命令如下：

```
modprobe hobot_cpudai_super
modprobe snd-soc-es8156
modprobe snd-soc-es7210
modprobe hobot_snd_s100_ac_fdx_host
```

驱动加载后确认声卡是否加载成功的方法：

- 检查/proc/asound/cards节点

```
root@ubuntu:/# cat /proc/asound/cards
 0 [s100snd2       ]: s100snd2 - s100snd2
                      s100snd2
```

- 检查/dev/snd/节点

```
root@ubuntu:/# ls -l /dev/snd
total 0
crw-rw-r--+ 1 root misc 116,  4 Apr 29 19:09 controlC0
crw-rw-r--+ 1 root misc 116,  2 Apr 29 19:09 pcmC0D0c
crw-rw-r--+ 1 root misc 116,  3 Apr 29 19:09 pcmC0D1p
```

声卡加载成功后功能测试参考命令：

```
arecord -Dhw:0,0 -c 2 -r 16000 -f S16_LE -t wav -d 5 test.wav
aplay -Dhw:0,1 test.wav
```

:::info 注意

PDMA驱动限制数据大小满足64字节对齐。

这里特别强调在配置period-size值时，设置值需满足字节对齐要求。

arecord/aplay对应参数配置：--period-size=X
:::


### 常用调试手段

#### 调整debug log等级

```
echo "8 4 1 7" > /proc/sys/kernel/printk
echo -n "file hhobot-cpudai-super.c +p" > /sys/kernel/debug/dynamic_debug/control
echo -n "file hobot-i2s-super.c +p" > /sys/kernel/debug/dynamic_debug/control
echo -n "file hobot-platform-super.c +p" > /sys/kernel/debug/dynamic_debug/control
```

#### ALSA profs节点说明

profs是linux的一个文件系统，提供关于内核数据结构的接口。挂载目录为/proc。

ALSA procfs挂载目录为：/proc/asound，ALSA使用/proc/asound目录下的文件保存设备信息和控制目的。通过proc节点，我们可以快速查看某些信息用于定位调试遇到的问题。

##### /proc/asound/cards

已注册声卡的列表。通过查看该节点，检查当前系统注册的声卡列表或者检查声卡是否注册成功

##### /proc/asound/pcm

分配的pcm流设备的信息。通过查看该节点，找到当前声卡支持的设备列表。这有助于测试时选择如何设置设备节点的card/device值。

##### /proc/asound/cardX/pcmY[c, p]/*
系统中声卡对应的每个pcm流设备都有一个类似上面的procfs目录。其中X代表声卡号，/proc/asound/cards或者/dev/snd下的设备节点信息可确认；
Y代表设备号，/proc/asound/pcm或者/dev/snd下的设备节点信息可确认。c/p分别代表capture/playback。该目录可查看PCM设备的信息以及status。

###### info

pcm流的一般信息，比如声卡号、设备号、流类型、所绑定的codec类型等

###### hw_params

pcm流打开状态下，可以查看pcm的基础参数配置。比如采样率、位宽、通道数、period_size、buffer_size等。配置的period_size、buffer_size可能与这里打印的值不同。实际以这里查看为准，对应的是硬件的实际参数

```
root@ubuntu:/proc/asound/card0/pcm0c/sub0# cat hw_params
access: RW_INTERLEAVED
format: S16_LE
subformat: STD
channels: 2
rate: 48000 (48000/1)
period_size: 1024
buffer_size: 4096
```

###### sw_params
pcm流打开状态下，查看start_threshold、stop_threshold、silence_threshold等。重点关注start_threshold、stop_threshold阈值

start_threshold: 该值设置太大，从开始播放到声音出来延时太长，会导致太短促的声音播不出来

stop_threshold: 判断是否触发xrun的条件。当可用空间大小超过该值时，会触发xrun。

```
root@ubuntu:/proc/asound/card0/pcm0c/sub0# cat sw_params
tstamp_mode: ENABLE
period_step: 1
avail_min: 1
start_threshold: 1
stop_threshold: 40960
silence_threshold: 0
silence_size: 0
boundary: 4611686018427387904
```

###### status

可查看当前substream的状态(running、xrun等)，appl_ptr/hw_ptr指针的值。其中，hw_ptr和app_ptr可以识别底层硬件无法传输或接收任何数据的情况。
比如，启动测试但是中断未正常触发，数据传输将会停滞。此时，hw_ptr/appl_ptr指针将持续得不到更新

#### xrun

音频在播放时偶现或者连续某个时间段出现断断续续、声音类似"呲呲"或者"爆破"的杂音， 一般是发生了xrun。xrun丢帧受系统性能限制不可避免，在满足使用场景的需求下，允许有一定的丢帧率，只能通过某些方法优化达到尽量规避。如果出现xrun频发，无法恢复的情况，就需要排查代码实现上是否有缺陷

##### 可能出现xrun的场景

播放时，应用会不断把音频数据填入驱动buffer，驱动buffer经过i2s fifo送给codec播放。当应用填入慢了，导致驱动buffer为空，就会触发underrun导致丢帧，可能会有声音异常的现象

录音时，codec转化的数字信号经过i2s fifo填入驱动buffer，应用会从驱动buffer中读取音频数据。当应用读取的速度赶不上写入的速度，超过stop_threshold阈值就会触发overrun

##### 定位xrun

检查xrun是否由于IO操作耗时导致

- 音频文件写入ram disk或者特定的设备文件，参考命令

```
写入ram disk
mkdir /data/audio_test
tinycap /data/audio_test/test.wav

特定的设备文件
tinycap /dev/null
```

注意，以上只能用于定位xrun是否由于IO操作耗时导致，不能作为解决xrun的方案

- 优化应用程序测试

使用独立的线程访问媒体存储和读写PCM设备。
以录音为例，写文件创建单独的线程和创建ring buffer(ring buffer大小可自由调整)，pcm_read的buffer写入ring buffer，fwrite从ring buffer读取数据。只要fwrite陷入时间不超过ring buffer的限制，就不会发生xrun导致数据。

- 依靠/proc下配置功能查看信息

开启编译xrun_debug的config配置项。编译并重新烧写镜像。（如果xrun_debug存在，表示该功能处于使能状态，不需要执行这一步）

```
CONFIG_SND_PCM_XRUN_DEBUG=y
CONFIG_SND_VERBOSE_PROCFS=y
CONFIG_SND_DEBUG=y
```

proc节点对应位置```/proc/asound/cardX/pcmY[c,p]/xrun_debug```

比如，往xrun_debug写入3，也就是启用基本调试和堆栈功能，可以查看PCM流是否由于某种原因而停止

```
# Enable basic debugging and dump stack
# Usefull to just see, if PCM stream is stopped for a reason (usually wrong audio process timing from scheduler)
echo 3 > /proc/asound/card0/pcm0p/xrun_debug
```

##### 修复xrun

- 提高线程优先级(设置实时线程+优先权值)
- 调大period_size，改变DMA传输数据量
- 异步实现I/O读写和ALSA 设备读写

### 通用问题

#### 打开设备节点报错

- Unable to open PCM device (cannot open device '/dev/snd/pcmC0D3c': No such file or director)

  - 驱动加载失败，导致/dev/snd下没有生成设备节点；
  - 设置的card/device值错误导致找不到对应的设备节点；

其中，/dev/snd下没有生成设备节点的可能原因：

  - i2s/codec驱动加载失败，可以通过dmesg看是否有以下信息打印

  ```
  soc:sndcard@0: BUG: Can't get codec
  soc:sndcard@0: BUG: Can't get cpu
  probe of soc:sndcard@0 failed with error -22
  ```

  - dts中各驱动节点的status属性未设置"okay"状态，导致驱动加载时不会执行任何操作。可以在驱动加载后，通过dmesg查看是否有类似success register cpu dai0 driver信息打印

- Unable to open PCM device (cannot set hw params: Unknown error -22)

当前测试设置的参数值，超过i2s和codec驱动中对于snd_soc_dai_driver定义的rate、format、channel最值范围交集

- 打开设备节点卡住，应用无任何log日志打印

ALSA框架允许单个设备同一时刻只能打开一次。检查应用实现上，是否存在上次执行还未退出的情况下，又启动应用

##### 录制/播放问题

- pcm_read/pcm_write返回异常值为-5

调整ALSA框架log打印等级，检查是否由于中断或者硬件链接导致中断没有产生引起异常

```
echo "8 4 1 7" > /proc/sys/kernel/printk
echo -n "file pcm_lib.c +p" > /sys/kernel/debug/dynamic_debug/control
```

出现```write error (DMA or IRQ trouble?)```打印，说明没有中断。需要检查寄存器配置以及硬件链接情况

- pcm_read/pcm_write返回异常值为-32

一般是发生了xrun

- pcm_read返回异常值为-16

正常录音(S100做slave)时，时钟突然断掉或者硬件连接中断会发生该异常

- 录制数据均为0或者播放没有声音

  - 示波器测量i2s的时钟线频率是否正常，data线是否有数据输出。
  - 如果时钟频率不符合预期或者量到某个时钟信号持续保持低电平的情况，检查i2s 寄存器的配置是否符合预期。比如主从模式、时钟比值状态
  - 如果以上没有问题，就需要检查codec芯片是否正常工作。播放一个正弦波，测量codec侧模拟信号的 输出，如果这里没有输出，确认codec的时钟比值配置和J5端是否匹配，以及当前S100端的时钟比值是否在codec可支持范围。
  - 如果codec的输出也能量到准确的信号，检查功放芯片是否正常工作。确认功放芯片各管脚幅值是否正常

- 录制/播放噪声

一些codec设计的时候，codec做slave时，需要芯片的驱动时钟和bclk/lrclk时钟同源，否则在录音的时候会出现数据跳变或者重复的问题，导致产生噪声

## 音频子板使用说明

### 音频子板介绍

音频子板与S100开发板连接，接收单路I2S全双工信号；

包含2颗es7210芯片，I2C地址为0x40/0x42，可以录制8通道音频，其中4路接收模拟mic，2路aec回采喇叭数据；

包含1颗es8156芯片，I2C地址为0x8，可以播放2通道音频，连接耳机或者喇叭最终播放声音；

实物如下图所示，其中红圈为es7210，黄圈为es8156，绿圈为4mic。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/audio1.png)

### 音频子板与开发板连接

连接方式如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/audio2.png)

**注意事项**：40pin上的PCM pin脚与PCIE的WIFI模组复用，硬件上提供了拨码开关实现pin脚功能切换。

拨码开关切换pin脚功能说明如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/audio3.png)



