---
sidebar_position: 9
---

# 2.9 大语言模型

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

本章节介如何在地平线RDK平台体验端侧Large Language Model (LLM)。

代码仓库： (https://github.com/D-Robotics/hobot_llm.git)

## 支持平台

| 平台                            | 运行方式     | 示例功能           |
| ------------------------------- | ------------ | ------------------ |
| RDK X3, RDK X3 Module (4GB内存) | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 端侧大语言模型体验 |

**注意：仅支持RDK X3，RDK X3 Module 4GB内存版本。**

## 准备工作

### 地平线RDK平台

1. 地平线RDK为4GB内存版本
2. 地平线RDK已烧录好地平线提供的Ubuntu 20.04/Ubuntu 22.04系统镜像。
3. 地平线RDK已成功安装TogetheROS.Bot。
4. 安装transformers，命令为 `pip3 install transformers -i https://pypi.tuna.tsinghua.edu.cn/simple`。
5. 更新hobot-dnn，命令为 `sudo apt update; sudo apt install hobot-dnn`。

## 使用方式

### 地平线RDK平台

运行程序前，需要下载模型文件并解压，命令如下：

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

 ```bash
 # 配置tros.b环境
 source /opt/tros/setup.bash
 ```

 </TabItem>
 <TabItem value="humble" label="Humble">

 ```bash
 # 配置tros.b环境
 source /opt/tros/humble/setup.bash
 ```

 </TabItem>
 </Tabs>

```bash
# 下载模型文件
wget http://sunrise.horizon.cc/llm-model/llm_model.tar.gz

# 解压
sudo tar -xf llm_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_llm/
```

使用命令`srpi-config`修改ION memory大小为1.9GB，设置方法参考RDK用户手册配置工具`srpi-config`使用指南[Performance Options](https://developer.horizon.cc/documents_rdk/configuration/srpi-config#performance-options)章节。

重启后设置CPU最高频率为1.5GHz，以及调度模式为`performance`，命令如下：

```bash
sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

目前提供两种体验方式，一种直接终端输入文本聊天体验，一种订阅文本消息，然后将结果以文本方式发布出去。

#### 终端交互体验

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# 配置tros.b环境
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```bash
ros2 run hobot_llm hobot_llm_chat
```

程序启动后，可直接在当前终端和机器人聊天。

#### 订阅发布体验

1. 启动 hobot_llm

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 run hobot_llm hobot_llm
    ```

2. 新开一个终端订阅输出结果topic

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 topic echo /text_result
    ```

3. 新开一个终端发布消息

    <Tabs groupId="tros-distro">
    <TabItem value="foxy" label="Foxy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/setup.bash
    ```

    </TabItem>

    <TabItem value="humble" label="Humble">

    ```bash
    # 配置tros.b环境
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    ros2 topic pub --once /text_query std_msgs/msg/String "{data: ""中国的首都是哪里""}"
    ```

消息发送后，可以在订阅输出结果终端查看输出结果。

## 注意事项

确认开发板内存为4GB，同时修改ION memory大小为1.9GB，否则会导致模型加载失败。
