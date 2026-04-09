---
sidebar_position: 10
---

# 5.4.10 Vision Voice Box

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

This section describes how to experience the full ASR + VLM/LLM + TTS pipeline on the RDK platform.

Code repository: (https://github.com/D-Robotics/hobot_llamacpp.git)

## Supported Platforms

| Platform                        | Runtime Environment | Example Feature       |
| ------------------------------- | ------------------- | --------------------- |
| RDK X5, RDK X5 Module           | Ubuntu 22.04 (Humble) | Vision Voice Box Experience |
| RDK S100, RDK S100P             | Ubuntu 22.04 (Humble) | Vision Voice Box Experience |

## Prerequisites

### RDK Platform

1. RDK must be the 4GB RAM version.
2. RDK must have Ubuntu 22.04 system image flashed.
3. TogetherROS.Bot must already be successfully installed on the RDK.
4. Install the ASR module for voice input by running: `apt install tros-humble-sensevoice-ros2`.

## Usage Instructions

### RDK Platform

- You can use the Vision-Language Model [Vision-Language Model](../03_boxs/generate/hobot_llamacpp.md)

- You can use the TTS tool [Text-to-Speech](../02_quick_demo/hobot_tts.md)

- ASR tool is already installed.

- Connect a USB speaker with microphone (or a wired headset if your RDK product has a 3.5mm audio jack). After connecting, verify that the audio device is recognized properly:

        
```bash
root@ubuntu:~# ls /dev/snd/

by-id  by-path  controlC0  controlC2  pcmC0D0c  pcmC0D0p  pcmC2D0c  pcmC2D0p  timer
```

The audio device name shown in the figure should be "plughw:0,0".

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5 Audio Connection">


![headset](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/vlm_boxs/headset.jpg)

</TabItem>

<TabItem value="s100" label="RDK S100 Audio Connection">


![headset](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/vlm_boxs/usb_audio.jpg)

</TabItem>

</Tabs>

### Usage Guide

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

**Publish images using MIPI camera**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure MIPI camera
export CAM_TYPE=mipi
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=plughw:0,0
```

**Publish images using USB camera**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure USB camera
export CAM_TYPE=usb
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=plughw:0,0
```

**Use locally looped-back images**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure local image loopback
export CAM_TYPE=fb
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=plughw:0,0
```

</TabItem>

<TabItem value="s100" label="RDK S100">

**Publish images using MIPI camera**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure MIPI camera
export CAM_TYPE=mipi
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=plughw:0,0
```

**Publish images using USB camera**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure USB camera
export CAM_TYPE=usb
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=plughw:0,0
```

**Use locally looped-back images**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# Configure local image loopback
export CAM_TYPE=fb
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=plughw:0,0
```

</TabItem>

</Tabs>

After the program starts, you can interact with the device via voice prompts. To use it: say "Hello" to wake up the device, then describe your task—for example, "Please describe this picture." Upon receiving the request, the device will reply "OK," and you should wait while it performs inference and outputs the result.

Example interaction flow:

1. User: "Hello, describe this picture."
2. Device: "OK, let me take a look first."
3. Device: "This picture shows xxx."

## Advanced Features

In addition to supporting Vision-Language Models (VLM), this package also supports using pure Language Models (LLM) for conversation:

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .

ros2 launch hobot_llamacpp llama_llm.launch.py llamacpp_gguf_model_file_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf audio_device:=plughw:0,0 
```

After launching the program, you can interact with the device via voice prompts. Usage instructions: after the device finishes initialization, it will say "I'm here!" Say "Hello" to wake it up, then give your query—for example, "How should I rest on weekends?" The device will then start inference and output its response via speech.

Example interaction flow:

1. Device: "I'm here!"
2. User: "Hello, how should I rest on weekends?"
3. Device: "Rest is important—you could read books, listen to music, draw, or exercise."

## Notes

1. **Regarding the ASR module**: Even when no wake word is detected, the ASR module will still output logs to the serial console after startup. You can speak to verify whether audio is being detected. If nothing is detected, first check the device status and device ID using `ls /dev/snd/`.

2. **Regarding wake-word functionality**: The wake word "Hello" may occasionally fail to be recognized, causing subsequent commands to be ignored. If issues occur, check the logs for entries like `[llama_cpp_node]: Recved string data: xxx`. If present, it indicates that text was successfully recognized.

3. **Regarding audio devices**: It is generally recommended to use the same device for both recording and playback to avoid echo. If separate devices are used, you can modify the device name by searching for the `audio_device` field in the file `/opt/tros/${TROS_DISTRO}/share/hobot_llamacpp/launch/llama_vlm.launch.py`.

4. **Regarding model selection**: Currently, the VLM only supports the large model provided in this example. For LLM, you can use any GGUF-converted model from the Hugging Face community: https://huggingface.co/models?search=GGUF.