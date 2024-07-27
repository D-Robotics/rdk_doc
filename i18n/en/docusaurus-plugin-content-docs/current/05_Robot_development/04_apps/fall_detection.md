---
sidebar_position: 3
---

# 5.4.3 Pose Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The Pose Detection subscribes to the image messages published by the camera, detects key points of the human body, analyzes the body posture, and publishes pose events.

Pose events are published using a custom algorithm message. Users can subscribe to the msg of this topic for application development.

Currently, only fall detection function is supported, which detects whether a person falls.

Code Repository:  (https://github.com/D-Robotics/hobot_falldown_detection)

## Supported Platforms

| Platform     | System     | Function                       |
| -------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start MIPI/USB camera to capture images, perform human body key point detection and pose detection, and finally display images and algorithm effects through web, and publish pose events |

## Preparation

### RDK Platform

1. RDK has flashed the  Ubuntu 20.04/22.04 image provided by D-Robotics.

2. The TogetheROS.Bot has been successfully installed on the RDK.

3. Make sure the PC is in the same network segment as the RDK, and the IP address of the first three segments should be consistent.

4. The RDK has installed MIPI or USB cameras.

## Usage

### RDK

The pose detection package subscribes to the data published by the human body key point detection package, publishes algorithm messages after algorithm inference, and uses the websocket package to render and display the published images and corresponding algorithm results on the PC browser.

Friendly reminder: When experiencing the app, rotate the camera by 90 degrees to simulate the effect of a person falling.

**Publish images using MIPI camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
export CAM_TYPE=mipi

ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
export CAM_TYPE=mipi

ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

</Tabs>

**Publish images using USB camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
export CAM_TYPE=usb

ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
export CAM_TYPE=usb

ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

</Tabs>

For the explanation of the parameters in the command, please refer to the README.md in the hobot_falldown_detection package source code.

## Result analysis

After starting the pose detection package, the following information will be displayed in the terminal:

```shell
[hobot_falldown_detection-4] [INFO] [1660271558.250055538] [body_kps_Subscriber]: receive targetType: personpointType: body_kps
[hobot_falldown_detection-4] [INFO] [1660271558.250598996] [fall_down_publisher]: track_id: 1 is fall down
```

The output log shows that the body_kps data is subscribed and the pose event is published.

In the PC's browser, enter `http://IP:8000`, and the body detection frame, keypoints, and pose detection results will be displayed in the web interface (IP refers to the IP address of the RDK):

![](/../static/img/05_Robot_development/04_apps/image/fall_detection/falldown.jpg)