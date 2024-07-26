---
sidebar_position: 9
---

# 5.2.9 Large Language Model

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

This section introduces how to experience Large Language Model (LLM) on Horizon RDK.

Code repository:  (https://github.com/D-Robotics/hobot_llm.git)

## Supported Platforms

| Platform                       | System | Function |
| ------------------------------ | -------------- | ---------------- |
| RDK X3, RDK X3 Module (4GB RAM) | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)   | Edge-side LLM Experience |

**Note: Only supports RDK X3 and RDK X3 Module with 4GB RAM version.**

## Preparation

### Horizon RDK

1. Horizon RDK with 4GB RAM version.
2. Horizon RDK has been flashed with the provided  Ubuntu 20.04/22.04 system image.
3. Horizon RDK has successfully installed TogetheROS.Bot.
4. Install transformers, the command is `pip3 install transformers -i https://pypi.tuna.tsinghua.edu.cn/simple`.
5. Update hobot-dnn, the command is `sudo apt update; sudo apt install hobot-dnn`.

## Usage

### Horizon RDK

Before running the program, you need to download the model file and extract it, the commands are as follows:

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
# Download the model file
wget http://archive.d-robotics.cc/tros/llm-model/llm_model.tar.gz

# Extract
sudo tar -xf llm_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_llm/
```

Use the command `srpi-config` to modify the ION memory size to 1.7GB. The configuration method can be referred to the "Performance Options" section of the RDK User Manual Configuration Tool `srpi-config` Guide [Performance Options](https://developer.horizon.cc/documents_rdk/configuration/srpi-config#performance-options).

After restarting, set the CPU maximum frequency to 1.5GHz and the scheduling mode to `performance`, the commands are as follows:

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

Currently, there are two ways to experience it. One is to directly input text in the terminal for chat interaction, and the other is to subscribe to text messages and then publish the results in text format.

#### Terminal Interaction

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
ros2 run hobot_llm hobot_llm_chat
```

After the program starts, you can chat directly with the robot in the current terminal.

#### Subscribe and Publish Messages

1. Start hobot_llm

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
    ros2 run hobot_llm hobot_llm
    ```

2. Open a new terminal to subscribe

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
    ros2 topic echo /text_result
    ```

3. Open a new terminal to publish

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
    ros2 topic pub --once /text_query std_msgs/msg/String "{data: ""What is the highest mountain?""}"
    ```

After sending the message, you can check the output result in the subscribed terminal.

## Note

Make sure the development board has 4GB of memory and modify the ION memory size to 1.7GB, otherwise the model loading will fail.