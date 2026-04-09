---
sidebar_position: 3
---

# DeepSeek Large Language Model

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

This section describes how to experience on-device Large Language Models (LLMs) on the RDK S100 series platform.

Code repository: (https://github.com/D-Robotics/hobot_xlm.git)

## Supported Platforms

| Platform                        | Runtime Environment | Example Functionality     |
| ------------------------------- | ------------------- | -------------------------- |
| RDK S100, RDK S100P             | Ubuntu 22.04 (Humble) | On-device LLM Experience   |

## Algorithm Details

| Model          | Parameters | Token Length | Quantization | Platform | Prefill eval (tokens/s) | Eval (tokens/s) |
| -------------- | ---------- | ------------ | ------------ | -------- | ----------------------- | --------------- |
| Deepseek-R1    | 1.5B       | 1024         | Q8           | S100     | 635.24                  | 17.05           |
| Deepseek-R1    | 7B         | 1024         | Q8           | S100     | 279.17                  | 3.72            |
| Deepseek-R1    | 1.5B       | 1024         | Q8           | S100P    | 1326.40                 | 26.52           |
| Deepseek-R1    | 7B         | 1024         | Q8           | S100P    | 468.86                  | 6.68            |

## Preparation

### System Setup

1. The RDK has been flashed with the Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.

### Model Download

Before running the program, you need to download the model files using the following commands:

#### DeepSeek_R1_Distill_Qwen_1.5B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_1.5B_1024.hbm --ftp-password=Oeftp~123$%
```

#### DeepSeek_R1_Distill_Qwen_7B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_7B_1024.hbm --ftp-password=Oeftp~123$%
```

### System Configuration

- Maximize ION memory allocation to meet large-model inference requirements:

```shell
/usr/hobot/bin/hb_switch_ion.sh bpu_first
reboot
```

- Set performance mode (`Note: Only RDK S100P supports performance mode`):

```shell
devmem 0x2b047000 32 0x99
devmem 0x2b047004 32 0x99
```

## Usage

Currently, two interaction methods are provided:  
1. Direct terminal-based text chat.  
2. Subscribing to text messages and publishing results as text topics.

#### Terminal Interaction

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

```bash
lib=/opt/tros/humble/lib/hobot_xlm/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
# config contains example model configuration files
cp -r /opt/tros/humble/lib/hobot_xlm/config/ .
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=0 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
```

After launching the program, you can directly chat with the robot in the current terminal.

Supported model types are `"DeepSeek_R1_Distill_Qwen_1.5B"` and `"DeepSeek_R1_Distill_Qwen_7B"`. Note that the 7B model is only compatible with RDK S100P.

#### Subscription/Publishing Interaction

1. Launch `hobot_llm`:

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    lib=/opt/tros/humble/lib/hobot_xlm/lib
    export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
    # config contains example model configuration files
    cp -r /opt/tros/humble/lib/hobot_xlm/config/ .
    ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=1 -p ros_string_sub_topic_name:="/prompt_text" -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
    ```

    Supported model types are `"DeepSeek_R1_Distill_Qwen_1.5B"` and `"DeepSeek_R1_Distill_Qwen_7B"`. Note that the 7B model is only compatible with RDK S100P.

2. Open a new terminal and subscribe to the output topic:

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic echo /tts_text
    ```

3. Open another new terminal and publish a message:

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    ```bash
    ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: \"Briefly describe the development of artificial intelligence\"}"
    ```

After sending the message, you can view the output in the terminal subscribed to the result topic.

## Example Output

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
On-device large language model multi-turn dialogue demo. Please enter your question and press Enter.
- Press Ctrl+C to exit.
- Type "reset" to clear conversation history.
[User] <<< Briefly describe the development of artificial intelligence
[Assistant] >>> ...


The development of Artificial Intelligence (AI) can be divided into several key stages:

1. **Early AI**:
   - **Artificial Intelligence**: Initially applied to specific tasks such as gaming and customer service.
   - **Machine Learning**: In the 1950s, computers began learning tasks like automatic recognition and speech recognition.
   - **Expert Systems**: In the 1970s, systems like "MYCIN" simulated human experts.

2. **Computer Vision**:
   - **Image Recognition**: In the 1980s, computers recognized simple images, such as handwritten digits.
   - **Natural Language Processing**: In the 1990s, systems enabled automated search and editing (e.g., early Wikipedia bots).

3. **Deep Learning**:
   - **Neural Networks**: In the 1980s, neural networks started processing complex data.
   - **Convolutional Neural Networks (CNNs)**: In the 1990s, CNNs were used for image recognition, e.g., in autonomous vehicles.
   - **Modern Deep Learning**: In the 2010s, models like GPT and BERT revolutionized natural language processing.

4. **Reinforcement Learning**:
   - **Robotics Control**: Since the 1980s, robots learned actions through trial and error.
   - **Autonomous Driving**: In the 2010s, reinforcement learning powered self-driving cars.

5. **Advanced Deep Learning & Neural Networks**:
   - **Image Recognition**: Tasks include classification, segmentation, and generation.
   - **Natural Language Processing**: such as text generation, translation, and dialogue.  
   - **Speech Recognition**: such as transcription and speech synthesis.

6. **AI Applications**:  
   - **Healthcare**: such as diagnosis and drug discovery.  
   - **Transportation**: such as autonomous driving and traffic management systems.  
   - **Education**: such as intelligent learning systems.  
   - **Finance**: such as automated trading and risk management.

7. **Ethics and Challenges**:  
   - **Privacy Issues**: data breaches and privacy violations.  
   - **Ethical Concerns**: such as algorithmic bias and privacy issues.

8. **Future Outlook**:  
   - **AI Chips**: used for training and inference.  
   - **Edge AI**: running on devices to reduce data transmission.  
   - **Multimodal AI**: integrating multimodal data such as vision and audio.  
   - **Human Assistants**: such as chatbots and life-support systems.

AI will continue to advance across multiple domains, driving technological progress and societal transformation.  
Performance prefill: 1113.04 tokens/s    decode: 20.22 tokens/s
```