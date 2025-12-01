---
sidebar_position: 8
---
# 人手关键点及手势识别(mediapipe)

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

人手关键点检测算法示例订阅图片和包含人手框信息的智能msg，利用BPU进行算法推理，发布包含人手关键点和手势信息的算法msg。

人手关键点索引如下图：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_lmk_index.jpeg)

代码仓库：

 (https://github.com/D-Robotics/palm_detection_mediapipe)

 (https://github.com/D-Robotics/hand_landmarks_mediapipe)

算法支持的手势识别类别，以及手势类别在算法msg（Attribute成员，type为"gesture"）中对应的数值如下：

| 手势       | 说明       | 数值 |
| ---------- | ---------- | ---- |
| ThumbUp    | 竖起大拇指 | 2    |
| Victory    | “V”手势    | 3    |
| Mute       | “嘘”手势   | 4    |
| Palm       | 手掌       | 5    |
| Okay       | OK手势     | 11   |
| ThumbLeft  | 大拇指向左 | 12   |
| ThumbRight | 大拇指向右 | 13   |
| Awesome    | 666手势    | 14   |

应用场景：手势识别算法集成了人手关键点检测，手势分析等技术，使得计算机能够将人的手势解读为对应指令，可实现手势控制以及手语翻译等功能，主要应用于智能家居，智能座舱、智能穿戴设备等领域。

小车手势控制案例：[小车手势控制](/docs/05_Robot_development/04_apps/car_gesture_control.md)

## 支持平台

| 平台                             | 运行方式     | 示例功能                                        |
| -------------------------------- | ------------ | ----------------------------------------------- |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头，并通过web展示推理渲染结果 |
| RDK S600 | Ubuntu 24.04 (Jazzy) | 启动MIPI/USB摄像头，并通过web展示推理渲染结果 |

## 算法信息

| 模型 | 平台 | 输入尺寸 | 推理帧率(fps) |
| ---- | ---- | ------------ | ---- |
| mediapipe | S100 | 224x224 | 1114 |

## 准备工作

### RDK平台

1. RDK已烧录好RDK OS系统。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

人手关键点检测(hand_landmarks_mediapipe)package订阅sensor package发布的图片以及人体检测和跟踪package发布的人手框检测结果，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。


**使用MIPI摄像头发布图片**

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

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/palm_detection_mediapipe/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_landmarks_mediapipe/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch hand_landmarks_mediapipe hand_landmarks.launch.py
```

**使用USB摄像头发布图片**

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

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/palm_detection_mediapipe/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_landmarks_mediapipe/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch hand_landmarks_mediapipe hand_landmarks.launch.py
```

**使用本地图片回灌**

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
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/palm_detection_mediapipe/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_landmarks_mediapipe/config/ .

# 配置本地回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch hand_landmarks_mediapipe hand_landmarks.launch.py publish_image_source:=config/example.jpg publish_image_format:=jpg publish_output_image_w:=640 publish_output_image_h:=480
```

## 结果分析

在运行终端输出如下信息：

```shell
[palm_detection_mediapipe-4] [DNN]: 3.7.3_(4.2.11 HBRT)
[hand_landmarks_mediapipe-3] [WARN] [1757389272.651945922] [mono2d_hand_lmk]: Get model name: hand_224_224 from load model.
[palm_detection_mediapipe-4] [WARN] [1757389272.653466536] [mono2d_palm_det]: Get model name: palm_det_192_192 from load model.
[palm_detection_mediapipe-4] [WARN] [1757389272.657688231] [mono2d_palm_det]: Enabling zero-copy
[palm_detection_mediapipe-4] [WARN] [1757389272.657755005] [mono2d_palm_det]: Create hbmem_subscription with topic_name: /hbmem_img
[hand_landmarks_mediapipe-3] [WARN] [1757389272.658734823] [mono2d_hand_lmk]: Enabling zero-copy
[hand_landmarks_mediapipe-3] [WARN] [1757389272.658829973] [mono2d_hand_lmk]: Create hbmem_subscription with topic_name: /hbmem_img
[hand_landmarks_mediapipe-3] [WARN] [1757389272.679073504] [mono2d_hand_lmk]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[palm_detection_mediapipe-4] [WARN] [1757389272.679083479] [mono2d_palm_det]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hand_landmarks_mediapipe-3] [WARN] [1757389272.679384552] [mono2d_hand_lmk]: SharedMemImgProcess Recved img encoding: nv12, h: 480, w: 640, step: 640, index: 0, stamp: 1757389272_411007134, data size: 460800, comm delay [268.3575]ms
[palm_detection_mediapipe-4] [WARN] [1757389272.679384452] [mono2d_palm_det]: SharedMemImgProcess Recved img encoding: nv12, h: 480, w: 640, step: 640, index: 0, stamp: 1757389272_411007134, data size: 460800, comm delay [268.3576]ms
[hand_landmarks_mediapipe-3] [WARN] [1757389273.715343396] [mono2d_hand_lmk]: input fps: 13.58, out fps: 13.94, infer time ms: 71, post process time ms: 1
[palm_detection_mediapipe-4] [WARN] [1757389273.723452363] [mono2d_palm_det]: input fps: 13.59, out fps: 13.94, infer time ms: 71, post process time ms: 0
[palm_detection_mediapipe-4] [WARN] [1757389275.711869066] [mono2d_palm_det]: SharedMemImgProcess Recved img encoding: nv12, h: 480, w: 640, step: 640, index: 33, stamp: 1757389275_710984298, data size: 460800, comm delay [0.8785]ms
[hand_landmarks_mediapipe-3] [WARN] [1757389275.711873416] [mono2d_hand_lmk]: SharedMemImgProcess Recved img encoding: nv12, h: 480, w: 640, step: 640, index: 33, stamp: 1757389275_710984298, data size: 460800, comm delay [0.8835]ms
[hobot_codec_republish-2] [WARN] [1757389277.211724002] [hobot_codec_decoder]: Pub img fps [9.66]
[hand_landmarks_mediapipe-3] [WARN] [1757389278.811834846] [mono2d_hand_lmk]: SharedMemImgProcess Recved img encoding: nv12, h: 480, w: 640, step: 640, index: 64, stamp: 1757389278_810957877, data size: 460800, comm delay [0.8710]ms
```

输出log显示，程序运行成功，初始化完成之后单次的推理耗时0.87ms。

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_lmk_web.jpg)
