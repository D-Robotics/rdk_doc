---
sidebar_position: 18
---

# Face 106 Landmarks Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The **Face 106 Landmarks Detection Example** subscribes to images and smart messages containing face bounding box information, utilizes the BPU for algorithmic inference, and publishes algorithmic messages containing face 106 keypoints information.

Code Repository: (https://github.com/D-Robotics/face_landmarks_detection)

## Supported Platforms

| Platform                | Operating Mode                | Example Functionality                        |
|------------------------|-------------------------------|----------------------------------------------|
| RDK X3, RDK X3 Module  | Ubuntu 22.04 (Humble)         | Starts MIPI/USB camera and displays inference rendering results via the web |
| RDK X5                 | Ubuntu 22.04 (Humble)         | Starts MIPI/USB camera and displays inference rendering results via the web |

## Preparation

### RDK Platform

1. The RDK has been flashed with the Ubuntu 22.04 system image.

2. The RDK has successfully installed TogetheROS.Bot.

3. The RDK has a MIPI or USB camera installed.

4. Confirm that the PC can access the RDK via the network.

## Usage Instructions

The **face_landmarks_detection package** subscribes to images published by the sensor package and face bounding box detection results published by the human detection and tracking package. After inference, it publishes algorithmic messages and displays the published images and corresponding algorithmic results on a PC browser through the websocket package.

**Using MIPI Camera to Publish Images**

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration files required for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch face_landmarks_detection body_det_face_landmarks_det.launch.py
```

**Using USB Camera to Publish Images**

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration files required for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch face_landmarks_detection body_det_face_landmarks_det.launch.py
```

## Result Analysis

The following information is displayed in the running terminal:

```shell
[mono2d_body_detection-3] [WARN] [1731988336.541394391] [example]: This is mono2d body det example!
[face_landmarks_detection-5] [WARN] [1731988336.637554206] [face_landmarks_det_node]: => face_landmarks_det_node params:
[face_landmarks_detection-5] => feed_type: 0
[face_landmarks_detection-5] => is_sync_mode: 0
[face_landmarks_detection-5] => model_file_name: /root/zhikang.zeng/work_humble_ws_x5/tros_ws/install/share/face_landmarks_detection/config/faceLandmark106pts.hbm
[face_landmarks_detection-5] => is_shared_mem_sub: 1
[face_landmarks_detection-5] => dump_render_img: 0
[face_landmarks_detection-5] => ai_msg_pub_topic_name: /hobot_face_landmarks_detection
[face_landmarks_detection-5] [INFO] [1731988336.638429674] [dnn]: Node init.
[face_landmarks_detection-5] [INFO] [1731988336.638482188] [face_landmarks_det_node]: => Set node para.
[face_landmarks_detection-5] [INFO] [1731988336.638589050] [dnn]: Model init.
[mono2d_body_detection-3] [WARN] [1731988336.641041791] [mono2d_body_det]: Parameter:
[mono2d_body_detection-3]  is_sync_mode_: 0
[mono2d_body_detection-3]  model_file_name_: config/multitask_body_head_face_hand_kps_960x544.hbm
[mono2d_body_detection-3]  is_shared_mem_sub: 1
[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection
[mono2d_body_detection-3]  ros_img_topic_name: /image_raw
[mono2d_body_detection-3]  image_gap: 1
[face_landmarks_detection-5] [BPU_PLAT]BPU Platform Version(1.3.6)!
[face_landmarks_detection-5] [HBRT] set log level as 0. version = 3.15.54.0
[face_landmarks_detection-5] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[mono2d_body_detection-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono2d_body_detection-3] [HBRT] set log level as 0. version = 3.15.54.0
[mono2d_body_detection-3] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
```

The output log shows that the program is running successfully, with an algorithm input and output frame rate of 30fps, and the frame rate is refreshed once per second.

Enter http://IP:8000 in the browser on the PC to view the images and algorithm rendering effects (IP is the IP address of the RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/face_landmarks_det_render.png)

