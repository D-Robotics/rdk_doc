---
sidebar_position: 3
---

# DeepSeek Large Language Model

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

This section introduces how to experience Large Language Model (LLM) on RDK S100.

Code repository:  (https://github.com/D-Robotics/hobot_xlm.git)

## Supported Platforms

| Platform                       | System | Function |
| ------------------------------ | -------------- | ---------------- |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble)   | Edge-side LLM Experience |

**Note: Only supports RDK X3 and RDK X3 Module with 4GB RAM version.**

## Al

| Model Type | Param | Token Length | Quant Method | Platform | Prefill eval (tokens/s) | Eval (tokens/s) |
| ---- | ---- | ---- | ------------ | ---- | ---- | ---- |
| Deepseek-R1 | 1.5B | 1024 | Q8 | S100 | 635.24 | 17.05 |
| Deepseek-R1 | 7B | 1024 | Q8 | S100 | 279.17 | 3.72 |
| Deepseek-R1 | 1.5B | 1024 | Q8 | S100P | 1326.40 | 26.52 |
| Deepseek-R1 | 7B | 1024 | Q8 | S100P | 468.86 | 6.68 |

## Preparation

### RDK

1. RDK has been flashed with the provided  Ubuntu 22.04 system image.
2. RDK has successfully installed TogetheROS.Bot.

### Download Model

Before running the program, you need to download the model file and extract it, the commands are as follows:

#### DeepSeek_R1_Distill_Qwen_1.5B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_1.5B_1024.hbm --ftp-password=Oeftp~123$%
```

#### DeepSeek_R1_Distill_Qwen_7B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_7B_1024.hbm --ftp-password=Oeftp~123$%
```

### 系统配置

- Setup the the maximum of ION.

```shell
/usr/hobot/bin/hb_switch_ion.sh bpu_first
reboot
```

- Setup the performance mode. `Attention：only RDK S100P support performance mode.`
```shell
devmem 0x2b047000 32 0x99
devmem 0x2b047004 32 0x99
```

## Usage

#### Terminal Interaction

```bash
source /opt/tros/humble/setup.bash
```

```bash
lib=/opt/tros/humble/lib/hobot_xlm/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
cp -r /opt/tros/humble/lib/hobot_xlm/config/ .
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=0 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
```

After the program starts, you can chat directly with the robot in the current terminal.

The support type of model: `DeepSeek_R1_Distill_Qwen_1.5B"`，`"DeepSeek_R1_Distill_Qwen_7B"`。RDK S100 is not support Qwen 7B。

#### Subscribe and Publish Messages

1. Start hobot_llm

    ```bash
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    lib=/opt/tros/humble/lib/hobot_xlm/lib
    export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
    cp -r /opt/tros/humble/lib/hobot_xlm/config/ .
    ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=1 -p ros_string_sub_topic_name:="/prompt_text" -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
    ```

2. Open a new terminal to subscribe

    ```bash
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic echo /tts_text
    ```

3. Open a new terminal to publish

    ```bash
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: ""简单描述人工智能的发展""}"
    ```

After sending the message, you can check the output result in the subscribed terminal.

## Result

```bash
[UCP]: log level = 3
[UCP]: UCP version = 3.7.3
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
- 退出请输入Ctrl C
- 清除缓存请输入reset
[User] <<< 简单描述人工智能的发展
[Assistant] >>> ...

人工智能（AI）的发展可以分为几个主要阶段：

1. **早期AI**：
   - **人工智能**：最初用于特定任务，如游戏和客服。
   - **机器学习**：1950年代，计算机开始学习，如自动识别和语音识别。
   - **专家系统**：1970年代，如“维基”系统，模拟人类专家。

2. **计算机视觉**：
   - **图像识别**：1980年代，计算机识别简单的图像，如手写数字。
   - **自然语言处理**：1990年代，如维基百科的自动搜索和编辑。

3. **深度学习**：
   - **神经网络**：1980年代，神经网络用于处理复杂数据。
   - **卷积神经网络（CNN）**：1990年代，用于图像识别，如自动驾驶汽车。
   - **深度学习**：2010年代，如GPT和BERT，用于自然语言处理。

4. **强化学习**：
   - **机器人控制**：1980年代，机器人学习动作。
   - **自动驾驶**：2010年代，如自动驾驶汽车。

5. **深度学习和神经网络**：
   - **图像识别**：如分类、分割和生成。
   - **自然语言处理**：如文本生成、翻译和对话。
   - **语音识别**：如转录和语音合成。

6. **AI应用**：
   - **医疗**：如诊断和药物研发。
   - **交通**：如自动驾驶和交通管理系统。
   - **教育**：如智能学习系统。
   - **金融**：如自动交易和风险管理。

7. **伦理和挑战**：
   - **隐私问题**：数据泄露和隐私侵犯。
   - **伦理争议**：如算法偏见和隐私问题。

8. **未来展望**：
   - **AI芯片**：用于训练和推理。
   - **边缘AI**：在设备上运行，减少数据传输。
   - **多模态AI**：结合视觉、听觉等多模态数据。
   - **人类助手**：如聊天机器人和生命支持系统。

AI将继续在多个领域发展，推动技术进步和社会变革。
Performance prefill: 1113.04tokens/s    decode: 20.22tokens/s
```

