---
sidebar_position: 2
---
# Hand Keypoint Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The hand keypoint detection algorithm subscribes to image data and smart messages containing hand bounding box information, performs inference using the BPU, and publishes algorithm messages containing detected hand keypoints.

Hand keypoint indices are shown in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_lmk_index.jpeg)

Code repositories:

 (https://github.com/D-Robotics/hand_lmk_detection)

 (https://github.com/D-Robotics/mono2d_body_detection)

Application scenarios: The hand keypoint detection algorithm is primarily used to capture skeletal keypoints of the human hand, enabling functionalities such as custom gesture recognition. It is mainly applied in fields like smart homes, virtual reality, and gaming/entertainment.

## Supported Platforms

| Platform                             | Runtime Environment     | Example Functionality                                        |
| -------------------------------- | ------------ | ----------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch MIPI/USB camera and display inference results via web browser |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Launch MIPI/USB camera and display inference results via web browser |
| RDK Ultra | Ubuntu 20.04 (Foxy) | Launch MIPI/USB camera and display inference results via web browser |

## Algorithm Specifications

| Model | Platform | Input Size | Inference FPS |
| ---- | ---- | ------------ | ---- |
| handLMKs | X3 | 8x21 | 806 |
| handLMKs | X5 | 8x21 | 948 |

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on RDK.
3. An MIPI or USB camera has been installed on RDK.
4. Ensure your PC can access the RDK over the network.

## Usage Instructions

The hand keypoint detection (`hand_lmk_detection`) package subscribes to images published by the sensor package and hand bounding boxes published by the human detection and tracking package. After inference, it publishes algorithm messages, which are rendered and displayed in a web browser on the PC via the websocket package.

**Publishing Images Using an MIPI Camera**


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

```shell
# Copy required configuration files for the example from tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**Publishing Images Using a USB Camera**


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

```shell
# Copy required configuration files for the example from tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

## Result Analysis

The following output appears in the terminal during execution:

```shell
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3] 
[hand_lmk_detection-4] [WARN] [1660269063.553205182] [hand_lmk_det]: input fps: 31.43, out fps: 31.47
[hand_lmk_detection-4] [WARN] [1660269064.579457516] [hand_lmk_det]: input fps: 30.21, out fps: 30.21
[hand_lmk_detection-4] [WARN] [1660269065.612579058] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269066.612778892] [hand_lmk_det]: input fps: 30.00, out fps: 30.00
[hand_lmk_detection-4] [WARN] [1660269067.646101309] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269068.679036184] [hand_lmk_det]: input fps: 30.04, out fps: 30.04
```

The log output indicates successful program execution. The algorithm processes input and outputs at approximately 30 FPS, with FPS statistics refreshed once per second.

Enter `http://IP:8000` in your PC's web browser to view the rendered images and algorithm results (replace "IP" with the RDK's actual IP address):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_render.jpeg)