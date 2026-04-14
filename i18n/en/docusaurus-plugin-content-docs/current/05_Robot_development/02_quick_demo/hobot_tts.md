---
sidebar_position: 8
---

# 5.2.8 Text-to-Speech

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

This section describes how to convert a piece of text into speech signals and play them through an audio output interface.

Code repository: (https://github.com/D-Robotics/hobot_tts.git)

## Supported Platforms

| Platform | Runtime Environment | Example Functionality |
| ------- | ------------ | ------------------------------ |
| RDK X3 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Subscribe to text messages, convert them into speech data, and then play the audio |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Subscribe to text messages, convert them into speech data, and then play the audio |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | Subscribe to text messages, convert them into speech data, and then play the audio |

**Note: Only RDK X3 is supported; RDK X3 Module is currently unsupported. RDK S100 supports only USB audio devices.**

## Prerequisites

### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. An RDK-compatible audio driver board is available, and the environment has been set up according to the [Smart Audio chapter](../03_boxs/audio/hobot_audio.md).
4. Headphones or speakers are connected to the headphone jack of the audio board.

## Usage Instructions

### RDK Platform

1. On first run, you need to download and extract the model files using the following commands:

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>
    <TabItem value="humble" label="Humble">

    ```bash
    # Configure the tros.b environment
    sudo apt update
    sudo apt install tros-humble-hobot-tts
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>
    </Tabs>

    ```bash
    wget http://archive.d-robotics.cc/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_tts/
    ```

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, refer to the FAQ section [Common Issues](../../08_FAQ/01_hardware_and_system.md), specifically Q10: "How to resolve issues when `apt update` fails or returns errors?"**
:::

2. Run the following command to check whether the audio device is functioning properly:

    ```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

    If an audio playback device such as `pcmC0D1p` appears, the device is working correctly.

    <Tabs groupId="board_type">
    <TabItem value="rdk_x3" label="RDK_X3">

    When using the audio board for the first time, you must configure it using `srpi-config`. For configuration instructions, refer to the RDK User Manual section: [RDK X3 Waveshare Audio Driver](../../03_Basic_Application/05_audio/rdk_x3_and_rdk_x3_module/audio_driver_hat2_rev2.md).

    </TabItem>
    <TabItem value="rdk_x5" label="RDK_X5">

    When using the audio board for the first time, you must configure it using `srpi-config`. For configuration instructions, refer to the RDK User Manual section: [RDK X5 Waveshare Audio Driver](../../03_Basic_Application/05_audio/rdk_x5/audio_driver_hat2_rev2.md).
    
    </TabItem>
    </Tabs>

3. Launch the hobot_tts program:

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure the tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    # Suppress debug log messages
    export GLOG_minloglevel=1

    ros2 run hobot_tts hobot_tts
    ```

    Note: If the audio playback device is not `pcmC0D1p`, you need to specify the playback device using the `playback_device` parameter. For example, if the audio playback device is `pcmC1D1p`, the startup command is:  
`ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,1"`  
For a USB device, the startup command is:  
`ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="plughw:1,1"`

4. Open a new terminal and publish a message to the topic using the echo command:

  <Tabs groupId="tros-distro">
  <TabItem value="foxy" label="Foxy">

  ```bash
  # Configure the tros.b environment
  source /opt/tros/setup.bash
  ```

  </TabItem>

  <TabItem value="humble" label="Humble">

  ```bash
  # Configure the tros.b environment
  source /opt/tros/humble/setup.bash
  ```

  </TabItem>

  </Tabs>

   ```bash
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""Do you know D-Robotics? Yes, I know D-Robotics. It is a line stretching from the ground to the sky, defining the boundary between earth and sky.""}"
   ```

5. You should hear the audio played through your headphones or speakers.

## Notes

Currently, only Chinese and English text content are supported. Do **not** publish text messages in other languages.