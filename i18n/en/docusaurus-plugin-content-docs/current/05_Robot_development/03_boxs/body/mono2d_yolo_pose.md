---
sidebar_position: 7
---
# Human Detection and Tracking (Ultralytics YOLO Pose)

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

This example uses [yolo-pose](https://docs.ultralytics.com/en/tasks/pose/) for human detection and tracking. It subscribes to image topics, performs algorithm inference using the BPU (Brain Processing Unit), publishes messages containing detected human bounding boxes and keypoints, and implements multi-object tracking (MOT) to track the detected bounding boxes.

The supported detection categories and their corresponding data types in the algorithm message are as follows:

| Category | Description        | Data Type |
| -------- | ------------------ | --------- |
| body     | Human bounding box | Roi       |
| body_kps | Human keypoints    | Point     |

The keypoint indices used by the human pose estimation algorithm are shown in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/kps_yolo_index.jpeg)


Code repository: (https://github.com/D-Robotics/mono2d_body_detection)

Application scenarios: Human detection and tracking algorithms are essential components of human motion visual analysis. They enable functionalities such as human pose analysis and people counting, and are primarily applied in human-computer interaction, gaming, and entertainment.

Pose detection example: [5.4.3. Pose Detection](../../apps/fall_detection)    
Human-following robot car example: [5.4.4. Robot Car Human Following](../../apps/car_tracking)  
Game character control based on human pose and gesture recognition: [Play with X3 Pi—Fitness and Gaming Combined](https://developer.d-robotics.cc/forumDetail/112555512834430487)

## Supported Platforms

| Platform                     | Runtime Environment     | Example Functionality                                           |
| ---------------------------- | ----------------------- | --------------------------------------------------------------- |
| RDK S100, RDK S100P          | Ubuntu 22.04 (Humble)   | Launch MIPI/USB camera and display inference results via Web    |

## Algorithm Details

| Model      | Platform | Input Size     | Inference FPS |
| ---------- | -------- | -------------- | ------------- |
| yolo-pose  | S100     | 1x3x640x640    | 68.70         |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with the Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on the RDK.
3. An MIPI or USB camera has been installed on the RDK.
4. Ensure your PC can access the RDK over the network.

## Usage Instructions

The human detection and tracking package (`mono2d_body_detection`) subscribes to images published by the sensor package, performs inference, publishes algorithm messages, and renders both the original images and algorithm results in a web browser on the PC via the websocket package.

### RDK Platform

**Publish images using an MIPI camera**

```shell
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py kps_model_type:=1 kps_image_width:=1920 kps_image_height:=1080 kps_model_file_name:=config/yolo11x_pose_nashe_640x640_nv12.hbm
```

**Publish images using a USB camera**

```shell
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py kps_model_type:=1 kps_image_width:=1920 kps_image_height:=1080 kps_model_file_name:=config/yolo11x_pose_nashe_640x640_nv12.hbm
```

**Publish images from local file (image replay)**

```shell
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# Configure local image replay
export CAM_TYPE=fb

# Launch the launch file
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py publish_image_source:=config/person_body.jpg publish_image_format:=jpg kps_model_type:=1 kps_image_width:=640 kps_image_height:=640 kps_model_file_name:=config/yolo11x_pose_nashe_640x640_nv12.hbm
```

## Result Analysis

The terminal outputs the following logs during execution:

```shell
[mono2d_body_detection-3] [WARN] [1660219823.214730286] [example]: This is mono2d body det example!
[mono2d_body_detection-3] [WARN] [1747724998.166714029] [mono2d_body_det]: Parameter:
[mono2d_body_detection-3]  is_sync_mode_: 0
[mono2d_body_detection-3]  model_file_name_: config/yolo11x_pose_nashe_640x640_nv12.hbm
[mono2d_body_detection-3]  is_shared_mem_sub: 1
[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection
[mono2d_body_detection-3]  ros_img_topic_name: /image_raw
[mono2d_body_detection-3]  image_gap: 1
[mono2d_body_detection-3]  dump_render_img: 0
[mono2d_body_detection-3]  model_type: 1
[mono2d_body_detection-3] [BPU][[BPU_MONITOR]][281473010090784][INFO]BPULib verison(2, 1, 2)[0d3f195]!
[mono2d_body_detection-3] [DNN] HBTL_EXT_DNN log level:6
[mono2d_body_detection-3] [DNN]: 3.3.3_(4.1.17 HBRT)
[mono2d_body_detection-3] [WARN] [1747724998.912552895] [mono2d_body_det]: Get model name: yolo11x_pose_nashe_640x640_nv12 from load model.
[mono2d_body_detection-3] [WARN] [1747724998.916663825] [mono2d_body_det]: Enabling zero-copy
[mono2d_body_detection-3] [WARN] [1747724998.916748774] [mono2d_body_det]: Create hbmem_subscription with topic_name: /hbmem_img
[mono2d_body_detection-3] [WARN] [1660219824.895102286] [mono2d_body_det]: input fps: 31.34, out fps: 31.22
[mono2d_body_detection-3] [WARN] [1660219825.921873870] [mono2d_body_det]: input fps: 30.16, out fps: 30.21
[mono2d_body_detection-3] [WARN] [1660219826.922075496] [mono2d_body_det]: input fps: 30.16, out fps: 30.00
[mono2d_body_detection-3] [WARN] [1660219827.955463330] [mono2d_body_det]: input fps: 30.01, out fps: 30.01
[mono2d_body_detection-3] [WARN] [1660219828.955764872] [mono2d_body_det]: input fps: 30.01, out fps: 30.00
```

The logs indicate successful program execution, with both input and output frame rates around 30 FPS, and statistics refreshed once per second.

Open a browser on your PC and navigate to `http://IP:8000` to view the rendered results, including detected human bodies, heads, faces, hands, bounding box types, tracking IDs, and human keypoints (replace `IP` with the actual IP address of your RDK/X86 device):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/yolo_pose_render.png)