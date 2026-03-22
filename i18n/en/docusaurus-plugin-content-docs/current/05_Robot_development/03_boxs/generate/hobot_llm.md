---
sidebar_position: 1
---

# Bloom Large Language Model

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

This section describes how to experience on-device Large Language Models (LLMs) on the RDK platform.

Code repository: (https://github.com/D-Robotics/hobot_llm.git)

## Supported Platforms

| Platform                            | Runtime Environment     | Example Functionality           |
| ----------------------------------- | ----------------------- | ------------------------------- |
| RDK X3, RDK X3 Module (4GB RAM)     | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | On-device large language model experience |

**Note: Only supports RDK X3 and RDK X3 Module with 4GB RAM.**

## Model Information

| Model | Parameters | Platform | Prefill Eval Time (ms/token) | Eval Time (ms/token) |
| ----- | ---------- | -------- | ---------------------------- | -------------------- |
| Bloom | 1.4B       | X3       | 305.34                       | 364.78               |

## Prerequisites

### RDK Platform

1. RDK must be the 4GB RAM version.
2. RDK must have Ubuntu 20.04 or Ubuntu 22.04 system image flashed.
3. TogetherROS.Bot must already be installed successfully on the RDK.
4. Install `transformers` by running:  
   ```bash
   pip3 install transformers -i https://pypi.tuna.tsinghua.edu.cn/simple
   ```

## Usage Instructions

### RDK Platform

Before running the program, download and extract the model files using the following commands:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>
<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>
</Tabs>

```bash
# Download model files
wget http://archive.d-robotics.cc/llm-model/llm_model.tar.gz

# Extract
sudo tar -xf llm_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_llm/
```

Use the command `srpi-config` to adjust ION memory size to 1.9GB. For configuration details, refer to the **Performance Options** section in the RDK User Manual under the `srpi-config` usage guide: [Performance Options](https://developer.d-robotics.cc/rdk_doc/en/System_configuration/srpi-config#performance-options).

After rebooting, set the CPU maximum frequency to 1.5GHz and the governor mode to `performance` using the following commands:

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

Currently, two interaction methods are provided:
- Direct terminal-based text chat
- Subscribe to input text messages and publish results as text messages

#### Terminal Interaction Experience

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```bash
ros2 run hobot_llm hobot_llm_chat
```

After the program starts, you can directly chat with the robot in the current terminal.

#### Publish-Subscribe Interaction Experience

1. Launch `hobot_llm`

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 run hobot_llm hobot_llm
    ```

2. Open a new terminal and subscribe to the output topic

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 topic echo /text_result
    ```

3. Open another new terminal and publish a message

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # Configure tros.b environment
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # Configure tros.b environment
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 topic pub --once /text_query std_msgs/msg/String "{data: \"What is the capital of China?\"}"
    ```

After sending the message, you can view the output result in the terminal that subscribed to `/text_result`.

## Notes
Confirm that the development board memory is 4GB, and simultaneously adjust the ION memory size to 1.9GB; otherwise, model loading will fail.