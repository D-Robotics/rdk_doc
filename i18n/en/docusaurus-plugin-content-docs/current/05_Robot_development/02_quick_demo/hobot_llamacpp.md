---
sidebar_position: 10
---

# 5.2.10 Vision Language Model

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

This section describes how to experience on-device Vision Language Model (VLM) on the RDK platform. Thanks to the excellent results of [InternVL2_5-1B](https://huggingface.co/OpenGVLab/InternVL2_5-1B), we have achieved quantization and deployment on the RDK platform. This demo leverages the powerful KV Cache management in [llama.cpp](https://github.com/ggml-org/llama.cpp), combined with the computational advantages of the RDK platform's BPU module, to enable local VLM model deployment.

Code repository: (https://github.com/D-Robotics/hobot_llamacpp.git)

## Supported Platforms

| Platform             | OS / Method             | Demo Functionality             |
| -------------------- | ---------------------- | ------------------------------ |
| RDK X5 (4GB RAM)     | Ubuntu 22.04 (Humble)  | On-device Vision Language Model|

**Note: Only supported on RDK X5 with 4GB RAM.**

## Preparation

### RDK Platform

1. RDK must be the 4GB RAM version.
2. RDK should be flashed with the Ubuntu 22.04 system image.
3. TogetheROS.Bot must be successfully installed on the RDK.

## Usage

### RDK Platform

Before running the program, download the model files to the working directory with the following commands:

```bash
# Download model files
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/blob/main/Qwen2.5-0.5B-Instruct-Q4_0.gguf
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/blob/main/rdkx5/vit_model_int16_v2.bin
```

Use the `srpi-config` command to set the ION memory size to 2.5GB. For details, refer to the [Performance Options](https://developer.d-robotics.cc/rdk_doc/en/System_configuration/srpi-config#performance-options) section in the RDK User Manual.

After rebooting, set the CPU maximum frequency to 1.5GHz and the scheduling mode to `performance` with the following commands:

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu2/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu4/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu5/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu6/cpufreq/scaling_governor'
sudo bash -c 'echo performance >/sys/devices/system/cpu/cpu7/cpufreq/scaling_governor'
```

Currently, two demo modes are provided: direct terminal input (image and text), or subscribing to image and text messages and publishing the results as text.

#### Single Image Inference Demo

```bash
# Set up tros.b environment
source /opt/tros/humble/setup.bash
```

```bash
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .

ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=0 -p image:=config/image2.jpg -p image_type:=0 -p user_prompt:="Describe this image."
```

After starting the program, you can use a local image and custom prompt for output.

![vlm_result](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/hobot_llamacpp/vlm_result.png)

## Notes

Ensure the development board has 4GB RAM and the ION memory size is set to 2.5GB, otherwise the model may fail to load.
