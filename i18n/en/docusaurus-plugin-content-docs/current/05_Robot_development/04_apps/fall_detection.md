---
sidebar_position: 3
---

# 5.4.3 Pose Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The Pose Detection App subscribes to image messages published by the camera, detects human body keypoints, analyzes human poses, and publishes pose events.

Pose events are published using a custom algorithm message format. Users can subscribe to this topic's messages for application development.

Currently, only fall detection is supported—detecting whether a person has fallen.

Code repository: (https://github.com/D-Robotics/hobot_falldown_detection)

## Supported Platforms

| Platform                  | Runtime Environment             | Example Functionality                                                                                                                                     |
| ------------------------- | ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| RDK X3, RDK X3 Module     | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch MIPI/USB camera to capture images, perform human body keypoint detection and pose detection, display images and algorithm results via Web, and publish pose events |
| RDK X5, RDK X5 Module     | Ubuntu 22.04 (Humble)           | Launch MIPI/USB camera to capture images, perform human body keypoint detection and pose detection, display images and algorithm results via Web, and publish pose events |
| RDK Ultra                 | Ubuntu 20.04 (Foxy)             | Launch MIPI/USB camera to capture images, perform human body keypoint detection and pose detection, display images and algorithm results via Web, and publish pose events |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. Ensure that your PC and the RDK are on the same network subnet—the first three segments of their IP addresses must match.
4. An MIPI or USB camera has been installed on the RDK.

## Usage Guide

### RDK Platform

The pose detection package subscribes to data published by the human body keypoint detection package. After algorithmic inference, it publishes algorithm messages, which are rendered in a web browser on the PC via the websocket package, displaying both the published images and corresponding algorithm results.

Friendly tip: When trying out the app, rotate the camera by 90 degrees to simulate a person falling.

**Publish images using an MIPI camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy the required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

</Tabs>

**Publish images using a USB camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Copy the required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py
```

</TabItem>

</Tabs>

For details about parameters used in the above commands, please refer to the README.md file in the source code of the `hobot_falldown_detection` package.

## Result Analysis

After launching the pose detection package, the following information appears in the terminal:

```shell
[hobot_falldown_detection-4] [INFO] [1660271558.250055538] [body_kps_Subscriber]: receive targetType: person pointType: body_kps
[hobot_falldown_detection-4] [INFO] [1660271558.250598996] [fall_down_publisher]: track_id: 1 is fall down
```

The log output indicates that `body_kps` data has been successfully subscribed to, and a pose event has been published.

On your PC’s web browser, navigate to `http://IP:8000` (replace "IP" with the RDK’s actual IP address) to view the rendered results, including human detection bounding boxes, keypoints, and pose detection outcomes:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/fall_detection/falldown.jpg)