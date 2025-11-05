---
sidebar_position: 10
---

# 5.4.10 视觉语音盒子

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

本章节介如何在RDK平台体验 ASR + VLM/LLM + TTS 全链路的使用方法。

代码仓库： (https://github.com/D-Robotics/hobot_llamacpp.git)

## 支持平台

| 平台                            | 运行方式     | 示例功能           |
| ------------------------------- | ------------ | ------------------ |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 视觉语音盒子体验 |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 视觉语音盒子体验 |

## 准备工作

### RDK平台

1. RDK为4GB内存版本
2. RDK已烧录好Ubuntu 22.04系统镜像。
3. RDK已成功安装TogetheROS.Bot。
4. 安装ASR模块用于语言输入，命令为 `apt install tros-humble-sensevoice-ros2`。

## 使用方式

### RDK平台

- 可以使用视觉语言模型 [视觉语言模型](/docs/05_Robot_development/03_boxs/generate/hobot_llamacpp.md)

- 可以使用TTS工具 [文本转语音](/docs/05_Robot_development/02_quick_demo/hobot_tts.md)

- 已安装ASR工具

- RDK 设备包含3.5mm 耳机孔, 且设备插入有线耳机。插入后检测音频设备是否正常:

```bash
root@ubuntu:~# ls /dev/snd/

by-path  controlC1  pcmC1D0c  pcmC1D0p  timer
```

如图显示的音频设备名应为 "hw:1,0"。

![headset](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/vlm_boxs/headset.jpg)

### 使用说明

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

<Tabs groupId="tros-distro">
<TabItem value="x5" label="RDK X5">

**使用MIPI摄像头发布图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置MIPI摄像头
export CAM_TYPE=mipi
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=hw:1,0
```

**使用USB摄像头发布图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置USB摄像头
export CAM_TYPE=usb
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=hw:1,0
```

**使用本地回灌图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置本地回灌图片
export CAM_TYPE=fb
ros2 launch hobot_llamacpp llama_vlm.launch.py audio_device:=hw:1,0
```

</TabItem>

<TabItem value="s100" label="RDK S100">

**使用MIPI摄像头发布图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置MIPI摄像头
export CAM_TYPE=mipi
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=hw:1,0
```

**使用USB摄像头发布图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置USB摄像头
export CAM_TYPE=usb
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=hw:1,0
```

**使用本地回灌图片**

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .
# 配置本地回灌图片
export CAM_TYPE=fb
ros2 launch hobot_llamacpp llama_vlm.launch.py llamacpp_vit_model_file_name:=vit_model_int16.hbm audio_device:=hw:1,0
```

</TabItem>

</Tabs>

程序启动后，可通过语音提示与设备交互。具体使用方法：通过"你好"唤醒设备, 然后给设备说明任务。如"请描述这种图片"。设备收到任务后, 会回复"好的", 此时请等待设备推理完成并开始输出文字。

示例流程：

1. 用户："你好, 描述这张图片."

2. 设备："好的, 让我看看先哈."

3. 设备："这张图片显示了xxx."

## 进阶功能

功能包本身除了支持视觉语言大模型能力, 同时支持单独使用纯语言模型进行对话：

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_llamacpp/config/ .

ros2 launch hobot_llamacpp llama_llm.launch.py llamacpp_gguf_model_file_name:=Qwen2.5-0.5B-Instruct-Q4_0.gguf audio_device:=hw:1,0 
```

程序启动后，可通过语音提示与设备交互。具体使用方法：等待设备初始化后, 会说"我来啦"; 通过"你好"唤醒设备, 然后给设备说明任务。如"周末应该怎么休息?"。设备收到任务后, 开始推理并输出文字播报。

1. 设备："我来啦"

2. 用户："你好, 周末应该怎么休息?"

3. 设备："休息很重要，可以看看书、听音乐、画画、运动"

## 注意事项

1. 关于ASR模块: ASR 启动后, 即使没检测到唤醒词, 程序串口也会输出日志, 此时可以说话验证是否有检测到。如无检测到, 请先通过 `ls /dev/snd/` 检查设备状态与设备号。

2. 关于唤醒功能：使用"你好"唤醒词有概率未识别到, 导致后面的内容无法输出。功能异常时, 可关注日志中是否有 `[llama_cpp_node]: Recved string data: xxx` 字段, 如有为识别到文本。

3. 关于音频设备: 一般建议录音播放设备为同一个设备, 避免回音。如录音、播放设备不是同个设备, 可在`/opt/tros/${TROS_DISTRO}/share/hobot_llamacpp/launch/llama_vlm.launch.py` 文件中搜索 `audio_device` 字段修改设备名。

4. 关于模型选择：目前 VLM 模型只支持本示例提供的大模型, LLM 模型则支持使用 https://huggingface.co/models?search=GGUF 社区上 GGUF 转换的模型进行推理。