---
sidebar_position: 2
---
# Sensevoice

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The intelligent voice algorithm utilizes the SenseVoiceGGUF model. After subscribing to audio data, it sends the data to the sensevoicegguf model for processing and then publishes messages such as **keyword/command recognition** and **speech ASR (Automatic Speech Recognition) results**. The implementation of this intelligent voice functionality corresponds to the **sensevoice_ros2** package in TogetheROS.Bot and is compatible with 3.5mm headsets.

Code repository: (https://github.com/D-Robotics/sensevoice_ros2.git)

Application scenarios: The intelligent voice algorithm can recognize custom-defined command words from audio input and interpret spoken content as corresponding commands or transcribe it into text. This enables functionalities such as voice control and speech-to-text translation, primarily applied in smart home systems, intelligent vehicle cockpits, wearable smart devices, and similar domains.

Example: Voice-controlled robot car movement — [5.4.6 Voice-Controlled Robot Car Movement](../../apps/car_audio_control)

## Supported Platforms

| Platform               | Runtime Environment     | Example Functionality                                  |
| ---------------------- | ----------------------- | ------------------------------------------------------ |
| RDK X5, RDK X5 Module  | Ubuntu 22.04 (Humble)   | Launch audio module algorithms and display results in terminal |
| RDK S100, RDK S100P    | Ubuntu 22.04 (Humble)   | Launch audio module algorithms and display results in terminal |

## Prerequisites

1. The RDK has been flashed with the Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. The intelligent voice algorithm package (version 2) has been successfully installed on the RDK. Installation command:

   <Tabs groupId="tros-distro">
   <TabItem value="humble" label="Humble">

   ```bash
   sudo apt update
   sudo apt install tros-humble-sensevoice-ros2
   ```

   </TabItem>
   </Tabs>

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, please refer to the FAQ section [Common Issues](../../../08_FAQ/01_hardware_and_system.md), specifically Q10: "How to resolve failures or errors when running `apt update`?"**
:::
   
4. The audio board is properly connected to the RDK X5’s 3.5mm headset jack.
5. A USB speaker is correctly connected to the RDK X5’s USB port.

## Usage Guide

After launching the intelligent voice **sensevoice_ros2** package, audio will be captured from the microphone and fed into the intelligent voice algorithm for processing. The algorithm outputs intelligent information such as recognized command words and ASR results. Specifically:
- Command words are published via messages of type `audio_msg::msg::SmartAudioData`.
- ASR results are published via messages of type `std_msgs::msg::String`.

By default, the intelligent voice feature supports ASR recognition on raw audio input. The default command words are defined in the file *config/cmd_word.json* located in the root directory of the intelligent voice module, with the following default configuration:

```json
{
    "cmd_word": [
        "Move forward",
        "Move backward",
        "Turn left",
        "Turn right",
        "Stop moving"
    ]
}
```

To run the **sensevoice_ros2** package on the RDK:

1. Configure the TogetheROS.Bot environment and launch the application:

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

   ```shell
   # Configure the TogetheROS.Bot environment
   
   source /opt/tros/humble/setup.bash

   # Launch the launch file
   ros2 launch sensevoice_ros2 sensevoice_ros2.launch.py micphone_name:="plughw:0,0"
   ```

</TabItem>

</Tabs>

## Result Analysis

When running on the Sunrise X3 board, the terminal outputs the following logs:

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

The above log indicates successful initialization and opening of the audio device, confirming that audio capture is functioning normally.

When a user speaks the command words “Move forward,” “Turn left,” “Turn right,” and “Move backward” sequentially near the microphone, the voice algorithm processes the audio intelligently and outputs the recognition results as shown below:

```text
cost time :769 ms
[WARN] [1745810610.317172494] [sensevoice_ros2]: recv cmd word:Move forward
result_str:Move forward,
[WARN] [1745810610.479493615] [sensevoice_ros2]: asr msg:Move forward,
result_str:Move forward,
cost time :785 ms
[WARN] [1745810614.078700989] [sensevoice_ros2]: recv cmd word:Turn left
result_str:Turn left,
[WARN] [1745810614.187793932] [sensevoice_ros2]: asr msg:Turn left,
result_str:Turn left,
cost time :761 ms
[WARN] [1745810616.453310236] [sensevoice_ros2]: recv cmd word:Turn right
result_str:Turn right,
[WARN] [1745810616.587498515] [sensevoice_ros2]: asr msg:Turn right,
result_str:Turn right,
cost time :737 ms
[WARN] [1745810618.700084757] [sensevoice_ros2]: recv cmd word:Move backward
result_str:Move backward,
[WARN] [1745810618.857481535] [sensevoice_ros2]: asr msg:Move backward,
result_str:Move backward,

```

By default, the **sensevoice_ros2** package publishes intelligent voice messages to the topics **/audio_smart** and **/audio_asr**. Running `ros2 topic list` yields:

```shell
$ ros2 topic list
/audio_smart
/audio_asr
```

The **/audio_asr** topic only produces output after hearing the specific wake-up phrase “Hello, Digua Robot.” The result of `ros2 topic echo /asr_text` is shown below:

![Execution Result](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/audio_asr.jpg)