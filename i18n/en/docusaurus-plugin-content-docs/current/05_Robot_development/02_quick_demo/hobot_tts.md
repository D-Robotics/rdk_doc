---
sidebar_position: 8
---

# 5.2.8 Text to Speech

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

This section describes how to convert a text into a speech and play it through an audio output interface.

Code repository:  (https://github.com/D-Robotics/hobot_tts.git)

## Supported Platforms

| Platform | System | Function               |
| -------- | ---------------- | ------------------------------ |
| RDK X3   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Subscribe to text messages, convert them into speech, and play them out |

**Note: Only supports RDK X3, RDK X3 Module is not supported yet.**

## Preparation

### Horizon RDK

1. Horizon RDK has been flashed with  Ubuntu 20.04/22.04 system image provided by Horizon.
2. TogetheROS.Bot has been successfully installed on Horizon RDK.
3. An audio driver board compatible with Horizon has been obtained, and the environment has been set up according to [Smart Voice section](../boxs/function#Intelligent Voice).

4. Connect the audio board's headphone interface with headphones or speakers.

## Usage

### Horizon RDK

1. For the first run, you need to download the models file and extract it. The commands are as follows:

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
    wget http://sunrise.horizon.cc/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_tts/
    ```

2. Run the following command to check if the audio device is normal:

    ```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

    If a similar audio playback device like `pcmC0D1p` appears, it means the device is working fine.

    :::caution
    When using the audio board for the first time, you need to use `srpi-config` for configuration, otherwise the audio device will not be recognized.
   
    For the configuration method, refer to the RDK User Manual [Audio Adapter Board] (https://developer.horizon.cc/documents_rdk/hardware_development/rdk_x3/audio_board) chapter.
    :::

3. Start the hobot_tts program.

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
   # Disable debug information
   export GLOG_minloglevel=1

   ros2 run hobot_tts hobot_tts
   ```

Note: If the audio playback device is not `pcmC0D1p`, you need to use the `playback_device` parameter to specify the playback audio device. For example, if the audio playback device is `pcmC1D1p`, the launch command should be: `ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,1"`

4. Open a new terminal and use the echo command to publish a topic

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
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""Do you know the horizon? Yes, I know the horizon. It is a line that extends from the ground to the sky, defining the boundary between the ground and the sky.""}"
   ```

5. You can hear the playback sound from your headphones or speakers.

## Notes

Currently, only Chinese and English text content are supported. Do not publish text messages in other languages.