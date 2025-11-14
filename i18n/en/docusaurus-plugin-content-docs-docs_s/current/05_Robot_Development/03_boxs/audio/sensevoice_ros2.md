---
sidebar_position: 2
---
# Sensevoice


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The D-Robotics intelligent voice algorithm adopts SenseVoiceGGUF, subscribes to audio data and sends it to SenseVoiceGGUF for processing, and then publishes messages such as **command word recognition**, and **ASR（Automatic Speech Recognition）**. The implementation of intelligent voice function corresponds to the **sensevoice_ros2** package of TogetheROS.Bot, which is suitable for 3.5mm micphone and USB speaker supported by RDK.

Code repository:  (https://github.com/D-Robotics/sensevoice_ros2.git)

Application scenarios: The intelligent voice algorithm can recognize custom command words in audio, interpret speech content as corresponding instructions or convert it into text, and can achieve functions such as voice control and speech translation. It is mainly used in areas such as smart home, intelligent cockpit, and smart wearables.

Example of voice-controlled car movement: [Voice-controlled car movement](../../apps/car_audio_control)

## Supported Platforms

| Platform | System | Function                    |
| -------- | ------------ | ---------------------------------- |
| RDK X5，RDK X5 module |  Ubuntu 22.04 (Humble) | Start the audio module algorithm and display the results in the terminal |


## Preparation

1. The RDK has been burned with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.
2. The TogetheROS.Bot has been successfully installed on the RDK.
3. The intelligent voice 2 algorithm package has been successfully installed on the RDK, installation command:

   <Tabs groupId="tros-distro">
   <TabItem value="humble" label="Humble">

   ```bash
   sudo apt update
   sudo apt install tros-humble-sensevoice-ros2
   ```

   </TabItem>
   </Tabs>

4. The audio board is correctly connected to the 3.5mm headphone jack of RDK X5.
5. Connect the USB speaker correctly to the USB port of RDK X5.



## Usage

After the smart voice sensevoice_ros2 package starts running, it will capture audio from the microphone array and send the captured audio data to the smart voice algorithm SDK module for intelligent processing. It will output command words, ASR results, and other smart information. command words are published as `audio_msg::msg::SmartAudioData` type messages, while ASR results are published as `std_msgs::msg::String` type messages.


The smart voice function supports ASR recognition after denoising the raw audio. The default command words are defined in the *config/cmd_word.json* file under the root directory of the smart voice function module, which are set in chinese as:
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

The command words can be configured according to your needs. It is recommended to use Chinese command words, preferably phrases that are easy to pronounce, with a length of 3 to 5 characters.


To run the sensevoice_ros2 package on the RDK:


1. Configure the tros.b environment and launch the application

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

   ```shell
   # Configure the tros.b environment
   source /opt/tros/humble/setup.bash

   # Start the launch file
   ros2 launch sensevoice_ros2 sensevoice_ros2.launch.py micphone_name:="plughw:0,0"
   ```

</TabItem>

</Tabs>

## Result Analysis

The following information is outputted on the terminal:

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

The above log shows that the audio device initialization is successful, and the audio device is opened for audio collection.

When a person speaks the Chinese command words ""向前走", "向左转", "向右转", "向后退" one by one next to the microphone, the speech algorithm SDK outputs the recognition results after intelligent processing, and the log shows the following:

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

The default topic name for the smart audio messages published by hobot_audio is: **/audio_smart** and **/audio_asr**, and the `ros2 topic list` result is:

```shell
$ ros2 topic list
/audio_smart
/audio_asr
```