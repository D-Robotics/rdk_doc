---
sidebar_position: 8
---

# 5.2.8 文本转语音

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

本章节介绍如何将一段文本转化为语音信号，并通过音频输出接口播放。

代码仓库： (https://github.com/D-Robotics/hobot_tts.git)

## 支持平台

| 平台    | 运行方式     | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 订阅文本消息，然后转化为语音数据，最后播放出去 |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 订阅文本消息，然后转化为语音数据，最后播放出去 |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 订阅文本消息，然后转化为语音数据，最后播放出去 |
| RDK S600 | Ubuntu 24.04 (Jazzy) | 订阅文本消息，然后转化为语音数据，最后播放出去 |

**注意：仅支持RDK X3，RDK X3 Module暂不支持， RDK S100/S600只支持USB语音设备。**

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu系统镜像。
2. RDK已成功安装TogetheROS.Bot。
3. 已有RDK适配的音频驱动板，并参考[智能语音章节](../03_boxs/audio/hobot_audio.md)搭建好环境。
4. 音频板耳机接口连接耳机或音响。

## 使用方式

### RDK平台

1. 首次运行需要下载模型文件并解压，详细命令如下：

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
    sudo apt update
    sudo apt install tros-humble-hobot-tts
    source /opt/tros/humble/setup.bash
    ```

    </TabItem>
    <TabItem value="jazzy" label="Jazzy">

    ```bash
    # 配置tros.b环境
    sudo apt update
    sudo apt install tros-jazzy-hobot-tts
    source /opt/tros/jazzy/setup.bash
    ```

    </TabItem>
    </Tabs>

    ```bash
    wget http://archive.d-robotics.cc/tts-model/tts_model.tar.gz
    sudo tar -xf tts_model.tar.gz -C /opt/tros/${TROS_DISTRO}/lib/hobot_tts/
    ```

:::caution **注意**
**如果`sudo apt update`命令执行失败或报错，请查看[常见问题](/docs/08_FAQ/01_hardware_and_system.md)章节的`Q10: apt update 命令执行失败或报错如何处理？`解决。**
:::

2. 运行如下命令检查音频设备是否正常：

    ```bash
    root@ubuntu:~# ls /dev/snd/
    by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
    ```

    如果出现类似`pcmC0D1p`音频播放设备则表示设备正常。

    <Tabs groupId="board_type">
    <TabItem value="rdk_x3" label="RDK_X3">

    首次使用音频板需要使用`srpi-config`进行配置，配置方法参考RDK用户手册[RDK X3微雪Audio Drive](/docs/03_Basic_Application/02_audio/rdk_x3_and_rdk_x3_module/audio_driver_hat2_rev2.md)章节。

    </TabItem>
    <TabItem value="rdk_x5" label="RDK_X5">

    首次使用音频板需要使用`srpi-config`进行配置，配置方法参考RDK用户手册[RDK X5微雪Audio Drive](/docs/03_Basic_Application/02_audio/rdk_x5/audio_driver_hat2_rev2.md)章节。
    
    </TabItem>
    </Tabs>

3. 启动hobot_tts程序

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
    <TabItem value="jazzy" label="Jazzy">

    ```bash
    # 配置tros.b环境
    source /opt/tros/jazzy/setup.bash
    ```

    </TabItem>

    </Tabs>

    ```bash
    # 屏蔽调式打印信息
    export GLOG_minloglevel=1

    ros2 run hobot_tts hobot_tts
    ```

    注意：若音频播放设备不是`pcmC0D1p`，则需要使用参数`playback_device`指定播放音频设备。例如音频播放设备为`pcmC1D1p`，启动命令为：`ros2 run hobot_tts hobot_tts --ros-args -p playback_device:="hw:1,1"`

4. 新开一个终端，使用echo命令发布一条topic

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
<TabItem value="jazzy" label="Jazzy">

```bash
# 配置tros.b环境
source /opt/tros/jazzy/setup.bash
```

  </TabItem>

  </Tabs>

   ```bash
   ros2 topic pub --once /tts_text std_msgs/msg/String "{data: ""你知道D-Robotics 吗？是的，我知道D-Robotics 。它是一条从地面延伸到天空的线，它定义了地面和天空之间的分界线。""}"
   ```

5. 耳机或音响可以听到播放的声音

## 注意事项

目前仅支持中文和英文文本内容，切记勿发布其他语言文本消息。
