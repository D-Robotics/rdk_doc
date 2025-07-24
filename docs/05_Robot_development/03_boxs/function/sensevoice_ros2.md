---
sidebar_position: 20
---
# 智能语音2

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

智能语音算法采用SenseVoiceGGUF的算法，订阅音频数据后送给sensevoicegguf模型处理，然后发布**命令词识别**、**语音ASR识别结果**等消息。智能语音功能的实现对应于TogetheROS.Bot的**sensevoice_ros2** package，适用于3.5mm的耳麦。

代码仓库： (https://github.com/D-Robotics/sensevoice_ros2.git)

应用场景：智能语音算法能够识别音频中自定义的命令词，并将语音内容解读为对应指令或转化为文字，可实现语音控制以及语音翻译等功能，主要应用于智能家居、智能座舱、智能穿戴设备等领域。

语音控制小车运动案例：[4.6 语音控制小车运动](../../apps/car_audio_control)

## 支持平台

| 平台   | 运行方式     | 示例功能                           |
| ------ | ------------ | ---------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动音频模块算法，并在终端显示结果 |

## 准备工作

1. RDK已烧录好Ubuntu 22.04系统镜像。
2. RDK已成功安装TogetheROS.Bot。
3. RDK已成功安装智能语音2算法包，安装命令：

   <Tabs groupId="tros-distro">
   <TabItem value="humble" label="Humble">

   ```bash
   sudo apt update
   sudo apt install tros-humble-sensevoice-ros2
   ```

   </TabItem>
   </Tabs>

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::
   
4. 音频板正确连接到RDK X5的3.5mm的耳麦接口。
5. USB音响正确连接到RDK X5的usb接口。


## 使用介绍

智能语音sensevoice_ros2 package开始运行之后，会从麦克风采集音频，并且将采集到的音频数据送入语音智能算法做智能处理，输出命令词、ASR结果等智能信息，其中命令词通过`audio_msg::msg::SmartAudioData`类型消息发布，ASR结果通过`std_msgs::msg::String`类型消息发布。


智能语音功能支持对原始音频进行ASR识别，默认的命令词定义在智能语音功能代码模块根目录下*config/cmd_word.json*文件，默认为：

```json
{
    "cmd_word": [
        "向前走",
        "向后退",
        "向左转",
        "向右转",
        "停止运动"
    ]
}
```

RDK板端运行sensevoice_ros2 package：


1. 配置tros.b环境和启动应用

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

   ```shell
   # 配置tros.b环境
   
   source /opt/tros/humble/setup.bash

   #启动launch文件
   ros2 launch sensevoice_ros2 sensevoice_ros2.launch.py micphone_name:="plughw:0,0"
   ```

</TabItem>

</Tabs>

## 结果分析

在旭日X3板端运行终端输出如下信息：

```text
alsa_device_init, snd_pcm_open. handle((nil)), name(plughw:0,0), direct(1), mode(0)
snd_pcm_open succeed. name(plughw:0,0), handle(0xaaaad1248290)
Rate set to 16000Hz (requested 16000Hz)
Buffer size range from 32 to 131072
Period size range from 16 to 1024
Requested period size 512 frames
Periods = 4
was set period_size = 512
was set buffer_size = 2048
alsa_device_init. hwparams(0xaaaad12484a0), swparams(0xaaaad124a7a0)

```

以上log显示，音频设备初始化成功，并且打开了音频设备，可正常采集音频。

当人依次在麦克风旁边说出“向前走”、“向左转”、“向右转”、“向后退”命令词，语音算法经过智能处理后输出识别结果，log显示如下：

```text
cost time :769 ms
[WARN] [1745810610.317172494] [sensevoice_ros2]: recv cmd word:向前走
result_str:向前走,
[WARN] [1745810610.479493615] [sensevoice_ros2]: asr msg:向前走,
result_str:向前走,
cost time :785 ms
[WARN] [1745810614.078700989] [sensevoice_ros2]: recv cmd word:向左转
result_str:向左转,
[WARN] [1745810614.187793932] [sensevoice_ros2]: asr msg:向左转,
result_str:向左转,
cost time :761 ms
[WARN] [1745810616.453310236] [sensevoice_ros2]: recv cmd word:向右转
result_str:向右转,
[WARN] [1745810616.587498515] [sensevoice_ros2]: asr msg:向右转,
result_str:向右转,
cost time :737 ms
[WARN] [1745810618.700084757] [sensevoice_ros2]: recv cmd word:向后退
result_str:向后退,
[WARN] [1745810618.857481535] [sensevoice_ros2]: asr msg:向后退,
result_str:向后退,

```


sensevoice_ros2默认发布的智能语音消息话题名为：**/audio_smart** 和 **/audio_asr**，`ros2 topic list`结果为：

```shell
$ ros2 topic list
/audio_smart
/audio_asr
```
