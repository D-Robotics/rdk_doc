---
sidebar_position: 2
---
# Hand Keypoint Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The hand keypoint detection example subscribes to images and algorithm messages containing hand bounding box information. It uses BPU for inference and publishes messages containing hand keypoint.

The index of hand keypoints is shown in the following figure:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_lmk_index.jpeg)

Code repository:

 (https://github.com/D-Robotics/hand_lmk_detection)

 (https://github.com/D-Robotics/mono2d_body_detection)

Application scenarios: The hand keypoint detection algorithm is mainly used to capture keypoints of the hand, enabling functions such as custom gesture recognition. It is mainly applied in areas such as smart homes, virtual reality, and gaming entertainment.

## Supported Platforms

| Platform                        | System | Function                           |
| ------------------------------- | ------------ | ----------------------------------------------- |
| RDK X3, RDK X3 Module, RDK X5  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · Start MIPI/USB camera and display inference results on the web |

## Preparation

### RDK

1. The RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. MIPI or USB cameras have been installed on the RDK.

4. Confirm that the PC can access the RDK through the network.

## Usage

The hand keypoint detection (hand_lmk_detection) package subscribes to images published by the sensor package and hand bounding box detection results published by the human body detection and tracking package. After inference, it publishes algorithm messages. The websocket package is used to render and display the published images and corresponding algorithm results on a PC browser.

**Use MIPI Camera to Publish Images**

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

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

**Use USB Camera to Publish Images**

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

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch hand_lmk_detection hand_lmk_detection.launch.py
```

## Result Analysis

The log during execution is as follows:

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
[hand_lmk_detection-4] [WARN] [1660269064.579457516] [hand_lmk_det]: input fps: 30.21, out fps: 30.21[hand_lmk_detection-4] [WARN] [1660269065.612579058] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269066.612778892] [hand_lmk_det]: input fps: 30.00, out fps: 30.00
[hand_lmk_detection-4] [WARN] [1660269067.646101309] [hand_lmk_det]: input fps: 30.01, out fps: 30.01
[hand_lmk_detection-4] [WARN] [1660269068.679036184] [hand_lmk_det]: input fps: 30.04, out fps: 30.04
```

The log shows that the program runs successfully and the input and output frame rates of the algorithm are 30fps, refreshing the statistical frame rate once per second.

On the PC browser, enter http://IP:8000 to view the image and algorithm rendering effect (where IP is the IP address of the RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hand_render.jpeg)