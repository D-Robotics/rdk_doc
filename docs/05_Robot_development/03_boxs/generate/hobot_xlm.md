---
sidebar_position: 3
---

# DeepSeek大语言模型

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

本章节介绍如何在RDK S100系列平台体验端侧 Large Language Model (LLM)。

代码仓库： (https://github.com/D-Robotics/hobot_xlm.git)

## 支持平台

| 平台                            | 运行方式     | 示例功能           |
| ------------------------------- | ------------ | ------------------ |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 端侧大语言模型体验 |

## 算法信息

| 模型 | 参数量 | 平台 | prefill eval (tokens/s) | eval (tokens/s) |
| ---- | ---- | ---- | ------------ | ---- |
| Deepseek-R1 | 1.5B | S100 | 648.10 | 13.61 |
| Deepseek-R1 | 1.5B | S100P | 914.29 | 20.80 |
| Deepseek-R1 | 7B | S100P | 404.42 | 5.89 |

## 准备工作

### 系统准备

1. RDK已烧录好Ubuntu 22.04系统镜像。
2. RDK已成功安装TogetheROS.Bot。

### 模型下载

运行程序前，需要下载模型文件，命令如下：

- DeepSeek_R1_Distill_Qwen_1.5B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm_v0.9.0/DeepSeek_R1_Distill_Qwen_1.5B_4096.hbm --ftp-password=Oeftp~123$%
```

- DeepSeek_R1_Distill_Qwen_7B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm_v0.9.0/DeepSeek_R1_Distill_Qwen_7B_1024.hbm --ftp-password=Oeftp~123$%
```

### 系统配置

- 设置 ION 内存空间最大, 满足大模型推理需求

```shell
/usr/hobot/bin/hb_switch_ion.sh bpu_first
reboot
```

- 设置性能模式
```shell
devmem 0x2b047000 32 0x99
devmem 0x2b047004 32 0x99
```

## 使用方式

目前提供两种体验方式，一种直接终端输入文本聊天体验，一种订阅文本消息，然后将结果以文本方式发布出去。

#### 终端交互体验

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```bash
lib=/opt/tros/humble/lib/hobot_xlm/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=0 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
```

程序启动后，可直接在当前终端和机器人聊天。

当前支持的模型类型为`DeepSeek_R1_Distill_Qwen_1.5B"`，`"DeepSeek_R1_Distill_Qwen_7B"`。其中 7B 模型仅适用于RDK S100P。

#### 订阅发布体验

1. 启动 hobot_llm

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=1 -p ros_string_sub_topic_name:="/prompt_text" -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
    ```

    当前支持的模型类型为`DeepSeek_R1_Distill_Qwen_1.5B"`，`"DeepSeek_R1_Distill_Qwen_7B"`。其中 7B 模型仅适用于RDK S100P。

2. 新开一个终端订阅输出结果topic

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic echo /tts_text
    ```

3. 新开一个终端发布消息

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: ""1258+1485x3等于多少？""}"
    ```

消息发送后，可以在订阅输出结果终端查看输出结果。

## 结果示例

```bash
[UCP]: log level = 3
[UCP]: UCP version = 3.6.1
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
[WARN] [1757949703.788157149] [xlm_node]: This is hobot xlm node!
[WARN] [1757949703.800199173] [xlm_node]: Parameter:
 feed_type(0:local, 1:sub): 0
 model_name: DeepSeek_R1_Distill_Qwen_1.5B
 ai_msg_pub_topic_name: /generation/lanaguage/deepseek
 text_msg_pub_topic_name: /tts_text
 ros_string_sub_topic_name: /prompt_text
[WARN] [1757949703.800428372] [xlm_node]: Model Parameter:
 model_path:   ./DeepSeek_R1_Distill_Qwen_1.5B_4096.hbm
 token_path:   ./config/DeepSeek_R1_Distill_Qwen_1.5B_config/
 k_cache_int8: 0
 model_type:   3
 context_size: 1024
 prompt_file:
 path_prompt_cache:
 sampling: {
     top_k:    3
     top_p:    0.95
     min_p:    0.1
     temp:     0.1
     typ_p:    1
     min_keep: 5
 }
[BPU][[BPU_MONITOR]][281473285378048][INFO]BPULib verison(2, 1, 2)[0d3f195]!
[DNN] HBTL_EXT_DNN log level:6
[DNN]: 3.6.1_(4.2.7post0.dev202307211111+6aaae37 HBRT)
[WARN] [1757949705.795194210] [xlm_node]: model init successed!
板端大模型多轮对话交互demo，请输入你的问题并按下回车
- 退出请输入exit
- 清除缓存请输入reset
[User] <<< 1258+1485x3等于多少？
[Assistant] >>> ...

**Step 1: Identify the Components**

First, identify the numbers and operations in the expression:
1258 + 1485 × 3

**Step 2: Perform the Multiplication**

Next, perform the multiplication part of the expression:
1485 × 3 = 4455

**Step 3: Add the Result to 1258**

Finally, add the result of the multiplication to 1258:
1258 + 4455 = 5713

**Final Answer:**
The result of 1258 + 1485 × 3 is 5713.
</think>

**Solution:**

We need to evaluate the expression:
\[ 1258 + 1485 \times 3 \]

**Step 1: Perform the Multiplication**

First, calculate the multiplication part of the expression:
\[ 1485 \times 3 = 4455 \]

**Step 2: Add the Result to 1258**

Next, add the result of the multiplication to 1258:
\[ 1258 + 4455 = 5713 \]

**Final Answer:**
\[
\boxed{5713}
\]
Performance prefill: 941.18tokens/s    decode: 22.63tokens/s
```
