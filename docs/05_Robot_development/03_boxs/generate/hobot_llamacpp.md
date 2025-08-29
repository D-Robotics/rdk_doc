---
sidebar_position: 2
---

# 视觉语言模型

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

本章节介如何在RDK平台体验端侧 Vision Language Model (VLM)。得益于[书生大模型](https://hf-mirror.com/OpenGVLab/InternVL2_5-1B), [SmolVLM](https://hf-mirror.com/HuggingFaceTB/SmolVLM2-256M-Video-Instruct) 的优秀成果, 我们在RDK平台上实现了量化与部署。同时, 本示例基于 [llama.cpp](https://github.com/ggml-org/llama.cpp) 中 KV Cache 的强大管理能力, 结合 RDK 平台 BPU 模块的计算优势, 实现了本地 VLM 模型部署。

代码仓库： (https://github.com/D-Robotics/hobot_llamacpp.git)

## 支持平台

| 平台                            | 运行方式     | 示例功能           |
| ------------------------------- | ------------ | ------------------ |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 端侧视觉语言大模型体验 |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 端侧视觉语言大模型体验 |

## 支持模型

| 模型类型 | 参数量 | 平台 |图像编码模型 | 文本编解码模型 |
| ------- | ------ | ------- | ---------- | ------------- |
| InternVL2_5 | 1B | X5 | [vit_model_int16_v2.bin](https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/rdkx5/vit_model_int16_v2.bin) | [Qwen2.5-0.5B-Instruct-Q4_0.gguf](https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/Qwen2.5-0.5B-Instruct-Q4_0.gguf) |
| InternVL2_5 | 1B | S100 | [vit_model_int16.hbm](https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/rdks100/vit_model_int16.hbm) | [Qwen2.5-0.5B-Instruct-Q4_0.gguf](https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/Qwen2.5-0.5B-Instruct-Q4_0.gguf) |
| InternVL3 | 1B | X5 | [vit_model_int16_VL3_1B_Instruct_X5.bin](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/rdkx5/vit_model_int16_VL3_1B_Instruct_X5.bin) | [qwen2_5_q8_0_InternVL3_1B_Instruct.gguf](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/qwen2_5_q8_0_InternVL3_1B_Instruct.gguf) |
| InternVL3 | 1B | S100 | [vit_model_int16_VL3_1B_Instruct.hbm](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/rdks100/vit_model_int16_VL3_1B_Instruct.hbm) | [qwen2_5_q8_0_InternVL3_1B_Instruct.gguf](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/qwen2_5_q8_0_InternVL3_1B_Instruct.gguf) |
| InternVL3 | 2B | X5 | [vit_model_int16_VL3_2B_Instruct.bin](https://hf-mirror.com/D-Robotics/InternVL3-2B-Instruct-GGUF-BPU/resolve/main/rdkx5/vit_model_int16_VL3_2B_Instruct.bin) | [qwen2_5_1.5b_q8_0_InternVL3_2B_Instruct.gguf](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/qwen2_5_q8_0_InternVL3_1B_Instruct.gguf) |
| InternVL3 | 2B | S100 | [vit_model_int16_VL3_2B_Instruct.hbm](https://hf-mirror.com/D-Robotics/InternVL3-2B-Instruct-GGUF-BPU/resolve/main/rdks100/vit_model_int16_VL3_2B_Instruct.hbm) | [qwen2_5_1.5b_q8_0_InternVL3_2B_Instruct.gguf](https://hf-mirror.com/D-Robotics/InternVL3-1B-Instruct-GGUF-BPU/resolve/main/qwen2_5_q8_0_InternVL3_1B_Instruct.gguf) |
| SmolVLM2 | 256M | X5 | [SigLip_int16_SmolVLM2_256M_Instruct_MLP_C1_UP_X5.bin](https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/rdkx5/SigLip_int16_SmolVLM2_256M_Instruct_MLP_C1_UP_X5.bin) | [SmolVLM2-256M-Video-Instruct-Q8_0.gguf](https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/SmolVLM2-256M-Video-Instruct-Q8_0.gguf) |
| SmolVLM2 | 256M | S100 | [SigLip_int16_SmolVLM2_256M_Instruct_S100.hbm](https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/rdks100/SigLip_int16_SmolVLM2_256M_Instruct_S100.hbm) | [SmolVLM2-256M-Video-Instruct-Q8_0.gguf](https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/SmolVLM2-256M-Video-Instruct-Q8_0.gguf) |
| SmolVLM2 | 500M | X5 | [SigLip_int16_SmolVLM2_500M_Instruct_MLP_C1_UP_X5.bin](https://hf-mirror.com/D-Robotics/SmolVLM2-500M-Video-Instruct-GGUF-BPU/resolve/main/rdkx5/SigLip_int16_SmolVLM2_500M_Instruct_MLP_C1_UP_X5.bin) | [SmolVLM2-500M-Video-Instruct-Q8_0.gguf](https://hf-mirror.com/D-Robotics/SmolVLM2-500M-Video-Instruct-GGUF-BPU/resolve/main/SmolVLM2-500M-Video-Instruct-Q8_0.gguf)

## 算法信息


| 模型 | 参数量 | 量化方式 | 平台 | 输入尺寸 | image encoder time(ms) | prefill eval time(ms/token) | eval time(ms/token) |
| ---- | ---- | ---- | ---- | ------------ | ---- | ---- | ---- |
| InternVL2_5 | 0.5B | Q4_0 | X5 | 1x3x448x448 | 2456.00 | 7.7 | 51.6 |
| InternVL3 | 0.5B | Q8_0 | S100 | 1x3x448x448 | 100.00 | 9.19 | 41.65 |
| Smolvlm2 | 256M | Q8_0 | X5 | 1x3x512x512 | 1053 | 9.3 | 27.8 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。
2. RDK已成功安装TogetheROS.Bot。
3. 下载安装功能包

```shell
sudo apt update
sudo apt install tros-humble-hobot-llamacpp
```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

## 使用方式

使用命令`srpi-config`修改ION memory大小为1.6GB，设置方法参考RDK用户手册配置工具`srpi-config`使用指南[Performance Options](https://developer.d-robotics.cc/rdk_doc/System_configuration/srpi-config#performance-options)章节。

重启后设置CPU最高频率为1.5GHz，以及调度模式为`performance`，命令如下：

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

目前提供两种体验方式，一种直接终端输入图片,文本体验，一种订阅图片和文本消息，然后将结果以文本方式发布出去。

### Internvl

运行程序前，需要下载模型文件到运行路径，命令如下：

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

```bash
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/Qwen2.5-0.5B-Instruct-Q4_0.gguf
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/rdkx5/vit_model_int16_v2.bin
```

</TabItem>

<TabItem value="s100" label="RDK S100">

```bash
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/Qwen2.5-0.5B-Instruct-Q4_0.gguf
wget https://hf-mirror.com/D-Robotics/InternVL2_5-1B-GGUF-BPU/resolve/main/rdks100/vit_model_int16.hbm
```

</TabItem>

</Tabs>

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=0 -p image:=config/image2.jpg -p image_type:=0 -p user_prompt:="描述一下这张图片." -p model_file_name:=vit_model_int16_v2.bin -p llm_model_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf
```

</TabItem>

<TabItem value="s100" label="RDK S100">

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=0 -p image:=config/image2.jpg -p image_type:=0 -p user_prompt:="描述一下这张图片." -p model_file_name:=vit_model_int16.hbm -p llm_model_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf
```

</TabItem>

</Tabs>

程序启动后，可使用本地图片与自定义提示词进行输出。

![internvlm_result](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/hobot_llamacpp/vlm_result.png)

### SmolVLM

运行程序前，需要下载模型文件到运行路径，命令如下：

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

```bash
wget https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/rdkx5/SigLip_int16_SmolVLM2_256M_Instruct_MLP_C1_UP_X5.bin
wget https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/SmolVLM2-256M-Video-Instruct-Q8_0.gguf
```

</TabItem>

<TabItem value="s100" label="RDK S100">

```bash
wget https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/rdks100/SigLip_int16_SmolVLM2_256M_Instruct_S100.hbm
wget https://hf-mirror.com/D-Robotics/SmolVLM2-256M-Video-Instruct-GGUF-BPU/resolve/main/SmolVLM2-256M-Video-Instruct-Q8_0.gguf
```

</TabItem>

</Tabs>

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=0 -p model_type:=1 -p image:=config/image2.jpg -p image_type:=0 -p user_prompt:="Describe the image." -p model_file_name:=SigLip_int16_SmolVLM2_256M_Instruct_MLP_C1_UP_X5.bin -p llm_model_name:=SmolVLM2-256M-Video-Instruct-Q8_0.gguf
```

</TabItem>

<TabItem value="s100" label="RDK S100">

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
ros2 run hobot_llamacpp hobot_llamacpp --ros-args -p feed_type:=0 -p model_type:=1 -p image:=config/image2.jpg -p image_type:=0 -p user_prompt:="Describe the image." -p model_file_name:=SigLip_int16_SmolVLM2_256M_Instruct_S100.hbm -p llm_model_name:=SmolVLM2-256M-Video-Instruct-Q8_0.gguf
```

</TabItem>

</Tabs>

程序启动后，可使用本地图片与自定义提示词进行输出。

![smolvlm_result](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/hobot_llamacpp/smolvlm_result.png)


## 注意事项

X5平台 修改ION memory大小为1.6GB, S100平台 修改ION memory大小大于1.6GB, 否则会导致模型加载失败。
