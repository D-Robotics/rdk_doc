---
sidebar_position: 3
---
# Gesture Recognition

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The gesture recognition algorithm subscribes to algorithm messages containing hand bounding boxes and hand keypoint information, performs inference using the BPU, and publishes intelligent result messages containing gesture information.

The gesture categories supported by the algorithm, along with their corresponding numerical values in the algorithm message (Attribute member with type "gesture"), are listed below:

1. Static Gestures

| Gesture      | Description       | Value |
| ------------ | ----------------- | ----- |
| ThumbUp      | Thumbs up         | 2     |
| Victory      | "V" sign          | 3     |
| Mute         | "Shh" gesture     | 4     |
| Palm         | Open palm         | 5     |
| Okay         | OK gesture        | 11    |
| ThumbLeft    | Thumb left        | 12    |
| ThumbRight   | Thumb right       | 13    |
| Awesome      | "666" gesture     | 14    |

2. Dynamic Gestures

| Gesture                        | Description                | Value |
| ------------------------------ | -------------------------- | ----- |
| PinchMove                      | Three-finger pinch drag    | 15    |
| PinchRotateAntiClockwise       | Three-finger pinch rotating counterclockwise | 16 |
| PinchRotateClockwise           | Three-finger pinch rotating clockwise        | 17 |

Code repositories:

(https://github.com/D-Robotics/hand_lmk_detection)

(https://github.com/D-Robotics/hand_gesture_detection)

(https://github.com/D-Robotics/mono2d_body_detection)

Application scenarios: The gesture recognition algorithm integrates hand keypoint detection and gesture analysis technologies, enabling computers to interpret human gestures as corresponding commands. This facilitates functionalities such as gesture control and sign language translation, primarily applied in smart homes, intelligent vehicle cabins, smart wearable devices, and similar fields.

Gesture-controlled robot car example: [Robot Car Gesture Control](../../04_apps/car_gesture_control.md)

Game character control example based on gesture recognition and human pose analysis: [Play with X3 Pi—Fun Fitness Gaming](https://developer.d-robotics.cc/forumDetail/112555512834430487)

## Supported Platforms

| Platform                       | Runtime Environment                              | Example Functionality                                               |
| ------------------------------ | ------------------------------------------------ | ------------------------------------------------------------------- |
| RDK X3, RDK X3 Module          | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)       | Launch MIPI/USB camera and display inference results via web browser |
| RDK X5, RDK X5 Module          | Ubuntu 22.04 (Humble)                            | Launch MIPI/USB camera and display inference results via web browser |
| RDK Ultra                      | Ubuntu 20.04 (Foxy)                              | Launch MIPI/USB camera and display inference results via web browser |

## Algorithm Information

| Model        | Platform | Input Size | Inference FPS |
| ------------ | -------- | ---------- | ------------- |
| gestureDet   | X3       | 8x21       | 2020          |
| gestureDet   | X5       | 8x21       | 1252.44       |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on the RDK.
3. An MIPI or USB camera has been installed on the RDK.
4. Ensure your PC can access the RDK over the network.

## Usage Guide

The `hand_gesture_detection` package subscribes to hand keypoint detection results published by the `hand_lmk_detection` package. After inference, it publishes algorithm messages, which are rendered and displayed in a web browser on the PC via the websocket package.

### Launching Dynamic Gesture Recognition

By default, the launch script only starts static gesture recognition. You can switch to dynamic gesture recognition exclusively at runtime using the `is_dynamic_gesture` parameter, for example:  
`ros2 launch hand_gesture_detection hand_gesture_detection.launch.py is_dynamic_gesture:=True`.

The following examples assume static gesture recognition is enabled by default.

:::warning
1. The command `ros2 launch hand_gesture_detection hand_gesture_detection.launch.py` supports either static or dynamic gesture recognition exclusively. To run both simultaneously, use:  
   `ros2 launch hand_gesture_detection hand_gesture_fusion.launch.py`.

2. Dynamic gesture recognition is only available starting from `TROS Humble 2.3.1`.  
   TROS release notes: [Click here](../../01_quick_start/changelog.md)  
   How to check your TROS version: [Click here](../../01_quick_start/install_tros.md)
:::

### Using an MIPI Camera

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py
```

### Using a USB Camera

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py
```

### Using Local Image Re-injection

:::warning
This feature is only supported in `TROS Humble 2.3.1` and later versions.

TROS release notes: [Click here](../../01_quick_start/changelog.md)  
How to check your TROS version: [Click here](../../01_quick_start/install_tros.md)  
:::

```bash
# Set up the tros.b environment
source /opt/tros/humble/setup.bash

# Copy required configuration files from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Enable local image re-injection
export CAM_TYPE=fb

# Launch the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py publish_image_source:=config/person_face_hand.jpg publish_image_format:=jpg publish_output_image_w:=960 publish_output_image_h:=544 publish_fps:=30
```

## Result Analysis

The following output appears in the terminal during execution:

```shell
[hand_gesture_detection-5] [C][32711][08-12][09:39:39:575][configuration.cpp:49][EasyDNN]EasyDNN version: 0.4.11
[hand_gesture_detection-5] [DNN] Runtime version = 1.9.7_(3.14.5 HBRT)
[hand_gesture_detection-5] [WARN] [1660268379.611419981] [hand gesture det node]: input_idx: 0, tensorType = 8, tensorLayout = 0
[hand_gesture_detection-5] [WARN] [1660268379.619313022] [hand gesture det node]: Create subscription with topic_name: /hobot_hand_lmk_detection
[hand_gesture_detection-5] [WARN] [1660268379.629207314] [hand gesture det node]: ai_msg_pub_topic_name: /hobot_hand_gesture_detection
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
[hand_gesture_detection-5] [WARN] [1660268381.026173815] [hand_gesture_det]: Sub smart fps 31.16  
[hand_gesture_detection-5] [WARN] [1660268381.206196565] [hand_gesture_det]: Pub smart fps 30.17  
[hand_gesture_detection-5] [WARN] [1660268382.054034899] [hand_gesture_det]: Sub smart fps 30.19  
[hand_gesture_detection-5] [WARN] [1660268382.234087357] [hand_gesture_det]: Pub smart fps 30.19  
[hand_gesture_detection-5] [WARN] [1660268383.055988982] [hand_gesture_det]: Sub smart fps 29.97  
[hand_gesture_detection-5] [WARN] [1660268383.235230316] [hand_gesture_det]: Pub smart fps 30.00  
[hand_gesture_detection-5] [WARN] [1660268384.087152150] [hand_gesture_det]: Sub smart fps 30.10  
[hand_gesture_detection-5] [WARN] [1660268384.256141566] [hand_gesture_det]: Pub smart fps 30.39  
```

The output log shows that the program ran successfully, with the algorithm achieving an input and output frame rate of 30 fps during inference, and the FPS statistics are updated once per second.

The output log also indicates that the subscribed algorithm message contains one "hand" (including hand bounding box and hand keypoint detection results). The gesture recognition algorithm classified the gesture as "Palm" (classification result: 5).

To view the image and algorithm rendering results, enter http://IP:8000 in a web browser on your PC (replace "IP" with the RDK's IP address):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/gesture_render.jpeg)