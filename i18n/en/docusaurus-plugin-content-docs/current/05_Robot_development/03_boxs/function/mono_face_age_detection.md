---
sidebar_position: 17
---

# Face Age Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The **Face Age Detection Example** subscribes to images and smart messages containing face bounding box information,
performs algorithm inference using the BPU, and publishes algorithm messages containing age information.

Code Repository: [https://github.com/D-Robotics/face_age_detection](https://github.com/D-Robotics/face_age_detection)

## Supported Platforms

| Platform              | Operating Mode        | Example Functionality                                                       |
|-----------------------|-----------------------|-----------------------------------------------------------------------------|
| RDK X3, RDK X3 Module | Ubuntu 22.04 (Humble) | Starts MIPI/USB camera and displays inference rendering results via the web |
| RDK X5                | Ubuntu 22.04 (Humble) | Starts MIPI/USB camera and displays inference rendering results via the web |

## Preparation

### RDK Platform

1. The RDK has been flashed with the Ubuntu 22.04 system image.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. A MIPI or USB camera has been installed on the RDK.

4. Ensure that the PC can access the RDK via the network.

## Usage Instructions

The **Face Age Detection (face_age_detection) package** subscribes to images published by the sensor package and face
bounding box detection results published by the human detection and tracking package. After performing inference, it
publishes algorithm messages and displays the published images and corresponding algorithm results on the PC's browser
through the websocket package.

**Using MIPI Camera to Publish Images**

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration files needed for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch face_age_detection body_det_face_age_det.launch.py
```

**Using USB Camera to Publish Images**

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration files needed for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch face_age_detection body_det_face_age_det.launch.py
```

## Result Analysis

The following information is output on the running terminal:

```shell
[mono2d_body_detection-3] [WARN] [1731986598.310822365] [example]: This is mono2d body det example!
[face_age_detection-5] [WARN] [1731986598.405068602] [face_age_det_node]: => face_age_det_node params:
[face_age_detection-5] => feed_type: 0
[face_age_detection-5] => is_sync_mode: 0
[face_age_detection-5] => model_file_name: /root/zhikang.zeng/work_humble_ws_x5/tros_ws/install/share/face_age_detection/config/faceAge.hbm
[face_age_detection-5] => is_shared_mem_sub: 1
[face_age_detection-5] => dump_render_img: 0
[face_age_detection-5] => ai_msg_pub_topic_name: /hobot_face_age_detection
[face_age_detection-5] => max_slide_window_size: 30
[face_age_detection-5] [INFO] [1731986598.406117839] [dnn]: Node init.
[face_age_detection-5] [INFO] [1731986598.406189046] [face_age_det_node]: => Set node para.
[face_age_detection-5] [INFO] [1731986598.406316544] [dnn]: Model init.
[mono2d_body_detection-3] [WARN] [1731986598.408599348] [mono2d_body_det]: Parameter:
[mono2d_body_detection-3]  is_sync_mode_: 0
[mono2d_body_detection-3]  model_file_name_: config/multitask_body_head_face_hand_kps_960x544.hbm
[mono2d_body_detection-3]  is_shared_mem_sub: 1
[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection
[mono2d_body_detection-3]  ros_img_topic_name: /image_raw
[mono2d_body_detection-3]  image_gap: 1
[mono2d_body_detection-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono2d_body_detection-3] [HBRT] set log level as 0. version = 3.15.54.0
[mono2d_body_detection-3] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[face_age_detection-5] [BPU_PLAT]BPU Platform Version(1.3.6)!
[face_age_detection-5] [HBRT] set log level as 0. version = 3.15.54.0
[face_age_detection-5] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
```

The log output indicates that the program is running successfully, with the algorithm input and output frame rate set to
30fps, and the frame rate statistics refreshed every second.

Enter http://IP:8000 in the browser on the PC to view the images and algorithm rendering effects (where IP is the IP
address of the RDK):

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/face_age_det_render.png)

