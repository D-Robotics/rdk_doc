---
sidebar_position: 3
---
# Gesture Recognition

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The gesture recognition example subscription package includes algorithm messages containing hand bounding boxes and hand keypoint information. It uses BPU for inference and publishes algorithm result messages containing gesture information.

The supported gesture recognition categories and their corresponding values in the algorithm message (Attribute member, with type "gesture") are as follows:

1. Static Gesture

| Gesture    | Description   | Value |
| ---------- | ------------- | ----- |
| ThumbUp    | Thumbs up     | 2     |
| Victory    | "V" gesture   | 3     |
| Mute       | mute gesture | 4     |
| Palm       | Palm          | 5     |
| Okay       | OK gesture    | 11    |
| ThumbLeft  | Thumb left    | 12    |
| ThumbRight | Thumb right   | 13    |
| Awesome    | "666" gesture | 14    |

2. Dynamic Gesture

| Gesture     | Description     | Value     |
| ---------- | ---------- | ---------- |
| PinchMove    | Three fingers kneading and dragging | 15 |
| PinchRotateAntiClockwise    | Three fingers kneading counterclockwise circle    | 16 |
| PinchRotateClockwise       | Three fingers kneading clockwise circle   | 17 |

Code repositories:

 (https://github.com/D-Robotics/hand_lmk_detection)

 (https://github.com/D-Robotics/hand_gesture_detection)

 (https://github.com/D-Robotics/mono2d_body_detection)

Application scenarios: Gesture recognition algorithms integrate hand keypoint detection, gesture analysis, and other technologies, enabling computers to interpret human gestures as corresponding commands. It can be used for gesture control, sign language translation, and other functions, primarily in the fields of smart homes, smart cabins, smart wearables, etc.

Example of gesture-controlled car: [Car Gesture Control](../../apps/car_gesture_control)

## Supported Platforms

| Platform                             | System | Function                                 |
| ------------------------------------ | ---------------- | ------------------------------------------------ |
| RDK X3, RDK X3 Module, RDK X5      | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | Start MIPI/USB camera and display inference results via web |

## Preparations

### RDK

1. RDK is flashed with  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on RDK.

3. The RDK is installed with a MIPI or USB camera.

4. Confirm that the PC can access the RDK through the network.

## Usage

The gesture recognition package (hand_gesture_detection) subscribes to the hand key point detection package and publishes the hand key point detection results. After inference, it publishes the algorithm message. The WebSocket package is used to display the images and corresponding algorithm results on the PC browser.

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
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py
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
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py
```

**Using Local Replay Images**

:::warning
Only `TROS Humble 2.3.1` and later versions support this feature.

`TROS` version release records: [Click to jump](/i18n/en/docusaurus-plugin-content-docs/current/05_Robot_development/01_quick_start/changelog.md), version check method: [Click to jump](/i18n/en/docusaurus-plugin-content-docs/current/05_Robot_development/01_quick_start/install_tros.md).
:::

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration files needed for the running example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

# Configure local replay images
export CAM_TYPE=fb

# Launch the launch file
ros2 launch hand_gesture_detection hand_gesture_detection.launch.py publish_image_source:=config/person_face_hand.jpg publish_image_format:=jpg publish_output_image_w:=960 publish_output_image_h:=544 publish_fps:=30
```

:::info
The launch script defaults to starting the static gesture recognition. For the `TROS Humble` version, you can specify to start the dynamic gesture recognition using the `is_dynamic_gesture` parameter: `ros2 launch hand_gesture_detection hand_gesture_detection.launch.py is_dynamic_gesture:=True`.
:::

## Analysis of Results

The following information will be displayed in the terminal output:

```shell
[hand_gesture_detection-5] [C][32711][08-12][09:39:39:575][configuration.cpp:49][EasyDNN]EasyDNN version: 0.4.11[hand_gesture_detection-5] [DNN] Runtime version = 1.9.7_(3.14.5 HBRT)
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

The log shows that the program runs successfully, with an input and output frame rate of 30fps during inference, and the statistics of frame rate are refreshed once per second.

The output log shows that the subscribed algorithm message includes hand information (including the bounding box and key points detection results of the hand), and the gesture recognition algorithm outputs a gesture classification result of "Palm" (the classification result is 5).

On the PC side, enter http://IP:8000 in the browser to view the image and the rendering effect of the algorithm (where IP is the IP address of the RDK).

![Gesture Rendering](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/gesture_render.jpeg)