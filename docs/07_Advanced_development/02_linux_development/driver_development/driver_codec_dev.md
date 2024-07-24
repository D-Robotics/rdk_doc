# 音频Codec适配说明

## 概述

本章主要描述音频的概念，以及添加声卡调试声卡的的说明。

相关概念：
- `DAI`: **Digital Audio Interface** 数字音频接口
- `CPU DAI`: `CPU`侧的数字音频接口，可以理解为`X3`的`I2S`接口
- `CODEC DAI`:即`Codec`。控制`Codec`工作流，简单理解为`Codec`的驱动
- `DAI LINK`:绑定`CPU DAI`和`CODEC DAI`
- `PLATFORM`:指定`CPU`侧的平台驱动，通常是`DMA`驱动

## 音频开发说明

一个完整的声卡信息由`CPU DAI`，`CODEC DAI`，`PLATFORM`，`DAI LINK`组成。分别对应`i2s`驱动，`codec`的驱动，`dma`驱动，以及声卡驱动，如`source/kernel/sound/soc/hobot/hobot-snd-wm8960.c`。本章节以新增`WM8960`这款**双声道全双工**`Codec`为例说明如何添加声卡。

### I2S参数

X3芯片的I2S有以下特性：
-   通道支持：音频输入支持`1/2/4/8/16`通道输入；音频输出支持`1/2`通道输出

-   采样率支持：`8k/16k/32k/48k/44.1k/64k`

-   采样精度支持：`8bit/16bit`

-   传输协议支持：`i2s/dsp(TDM)A`

-   I2S做master模式下的默认时钟：mclk为12.288M bclk为2.048M。mclk不发生变化的情况下，bclk支持6.144M、4.096M、3.072M、2.048M、1.536M,根据应用层传输的参数动态调整，调频策略在sound/soc/hobot/hobot-cpudai.c的hobot_i2s_sample_rate_set函数中.对于44.1k采样率的支持，在不调整PLL的情况下，可以调整出的最接近频率44.11764khz
    
-   **I2S做slave，在需要读写i2s寄存器操作前，需要有bclk时钟灌入，否则会访问i2s模块寄存器异常，导致系统不能正常工作**

针对板级`RDM X3 Module`，还有以下限制：

- 仅引出`I2S0`的所有时钟信号(`BCLK`,`LRCK`,)，`I2S1`的时钟信号在核心板上短接到`I2S0`时钟信号上，即两者共用时钟。

:::danger 警告

再次提醒，如果需要`I2S`做**Slave**，**必须**要在`bclk`信号灌入之后，才能去访问**X3**端的`I2S`相关寄存器！

:::

### 新增Codec说明

#### 添加codec driver
您获取`Codec`驱动代码后，将其复制到`source/kernel/sound/soc/codecs/`目录下。
#### 添加编译选项


修改`source/kernel/sound/soc/codecs/`目录下的`Kconfig`以及`Makefile`，将`WM8960`的驱动加入驱动编译。

其中，Kconfig添加

```
config SND_SOC_WM8960
	tristate "Wolfson Microelectronics WM8960 CODEC"
	depends on I2C
```

`Makefile`添加

```
snd-soc-wm8960-objs := wm8960.o
```
#### Kernel启用驱动

```bash
Device Drivers --->
    <*> Sound card support --->
        <*> Advanced Linux Sound Architecture --->
            <*> ALSA for SoC audio support --->
                CODEC drivers -->
                    [M] Wolfson Microelectronics WM8960 CODEC
```

保存`kernel`配置
### 修改dts文件

在对应的`i2c`添加`codec`信息，例如`WM8960`挂在`i2c0`总线上，则在`i2c0`中增加信息如下配置：

```c
&i2c0 {   
        status = "okay";
        #address-cells = <1>;
        #size-cells = <0>;
           
        wm8960:wm8960@0x1a{
            compatible = "wlf,wm8960";
            reg = <0x1a>;
            #sound-dai-cells = <0>;
        }; 
}
```

在`dts`文件中，配置对应`Codec`所需要的`sound card`信息。  

`WM8960`在`RDK X3 Module`上以一主（播放）一从（录制）使用，两个`I2S`共享时钟

```c
&i2s0 {
	status = "okay";
	#sound-dai-cells = <0>;
	clocks = <&i2s0_mclk>, <&i2s0_bclk>;
	clock-names = "i2s-mclk", "i2s-bclk";
	ms = <1>; // 这个属性决定这个i2s是做主还是从，1 为主
    share_clk = <1>;  //必须有这个属性，共享时钟才能生效
	bclk_set = <1536000>; //duplex
};

&i2s1 {
	status = "okay";
	#sound-dai-cells = <0>;
	clocks = <&i2s1_mclk>, <&i2s1_bclk>;
	clock-names = "i2s-mclk", "i2s-bclk";
	ms = <4>; // 这个属性决定这个i2s是做主还是从，4 为从
    share_clk = <1>; //必须有这个属性，共享时钟才能生效
	bclk_set = <1536000>;
};



&snd6 {
    status = "okay";
    model = "hobotsnd6"; //声卡名称
    work_mode = <0>; /*0: simple mode; 1: duplex mode*/
    dai-link@0 {
        dai-format = "i2s";//工作模式
        // bitclock-master; //从模式，不发送 bclk
        // frame-master; // 从模式，不发送lrck
        //frame-inversion; // clock极性
        link-name = "hobotdailink0";
        cpu {
            sound-dai = <&i2s1>; // 该通路绑定的i2s，这里根据您的硬件通路判断
        };
        codec {
            sound-dai = <&wm8960>; //对应刚才设置i2c0的名称
        };
        platform {
            sound-dai = <&i2sidma1>; //与sound-dai对应，使用i2s1这里就是i2sidma1
        };
    };
    dai-link@1 {
        dai-format = "i2s";//工作模式
        bitclock-master;// 主模式 发送 bclk
        frame-master; // 主模式 发送 lrck
        //frame-inversion;
        link-name = "hobotdailink1";
        cpu {
            sound-dai = <&i2s0>; // 通路绑定的i2s，这里根据您的硬件通路判断
        };
        codec {
            sound-dai = <&wm8960>; //对应刚才设置i2c0的名称
        };
        platform {
            sound-dai = <&i2sidma0>; //与sound-dai对应，使用i2s0这里就是i2sidma0
        };
    };
};
```
### 编写DAI LINK驱动

这部分驱动比较通用，可以参考`source/kernel/sound/soc/hobot/hobot-snd-wm8960.c`编写，注意要和设备树对应上

驱动编写完之后，自行添加进`Makefile`和`Kconfig`，然后将这个驱动添加进内核进行整体编译出驱动


## 调试说明

### 声卡调试

#### 驱动模块挂载

```bash
modprobe snd-soc-wm8960
modprobe hobot-i2s-dma
modprobe hobot-cpudai
modprobe hobot-snd-wm8960
```

#### 确认声卡驱动是否注册成功

```bash
root@ubuntu:~# cat /proc/asound/cards
0 [hobotsnd6      ]: hobotsnd6 - hobotsnd6
                     hobotsnd6
root@ubuntu:~# ls -l /dev/snd/
total 0
drwxr-xr-x  2 root root       60 Mar 28 01:54 by-path
crw-rw----+ 1 root audio 116,  2 Mar 28 01:54 controlC0
crw-rw----+ 1 root audio 116,  4 Mar 28 01:54 pcmC0D0c
crw-rw----+ 1 root audio 116,  3 Mar 28 01:54 pcmC0D0p
crw-rw----+ 1 root audio 116,  6 Mar 28 01:54 pcmC0D1c
crw-rw----+ 1 root audio 116,  5 Mar 28 01:54 pcmC0D1p
crw-rw----+ 1 root audio 116, 33 Mar 28 01:54 timer
```
#### 声卡使用

请参考 [音频转接板使用](../../hardware_development/rdk_x3_module/audio_board.md)


### 常用调试方法

- 查看`i2s`寄存器。录制播放不能正常工作时，一般需要先查看寄存器的配置是否正确，以便定位问题

查看`i2s0`寄存器配置信息：
```bash
cat /sys/devices/platform/soc/a5007000.i2s/reg_dump
```

查看`i2s1`寄存器配置信息：
```bash
cat /sys/devices/platform/soc/a5008000.i2s/reg_dump
```

- 查看`codec`寄存器配置信息，请向您的供应商咨询相关方法

- 使用示波器量音频的信号，包括`mclk/bclk/lrck/data`，确认时钟信号频率`bclk/lrck`是否符合设置的采样率，如果不符合，会导致音频加快或变缓
