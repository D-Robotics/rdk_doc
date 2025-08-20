---
sidebar_position: 6
---

# Stereo OCC Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The **Stereo OCC Algorithm** subscribes to stereo images, uses the BPU for algorithm inference, and publishes occupancy grid information.

Code Repository: (https://github.com/D-Robotics/dstereo_occnet)

## Supported Platforms

| Platform              | Runtime Environment   | Example Function                                                |
|-----------------------|-----------------------|-----------------------------------------------------------------|
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Launch stereo camera, display stereo images via Web, show occupancy grid results in rviz2 |

## Preparation

### RDK Platform

1. RDK has been flashed with Ubuntu 22.04 system image.

2. RDK has successfully installed TogetheROS.Bot.

3. For real-time online inference, currently only supports ZED-2i camera; for offline inference, please prepare stereo images.

4. Confirm that the PC can access the RDK through the network.

### Version Requirements

|                                                                     | Version | Query Method                                                          |
|---------------------------------------------------------------------|---------|-----------------------------------------------------------------------|
| System Version                                                      | 3.2.3   | Execute on RDK X5 board: `cat /etc/version`                          |
| [dstereo_occnet](https://github.com/D-Robotics/dstereo_occnet) package version | 1.0.0   | Execute on RDK X5 board: `apt list \| grep tros-humble-dstereo-occnet/` |
| [hobot_zed_cam](https://github.com/D-Robotics/hobot_zed_cam) package version   | 2.3.3   | Execute on RDK X5 board: `apt list \| grep tros-humble-hobot-zed-cam/` |

## Usage Instructions

### 1. Using with ZED-2i Camera

- Need to use together with the [`hobot_zed_cam`](https://github.com/D-Robotics/hobot_zed_cam.git) package
- Execute the following commands on RDK X5:

```bash
# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch ZED-2i camera and occupancy network inference program
ros2 launch dstereo_occnet zed2i_occ_node.launch.py
```

After the program starts, you can view the stereo images published by ZED-2i through the web page and view the occupancy grid through rviz2.

To view stereo images through web, enter [http://ip:8000](http://ip:8000) in the PC browser to view the stereo images. The ip is the ip of the RDK X5 board, which is `192.168.128.10` in the example. Make sure the PC and RDK X5 can communicate through the network.
![ZED-2i-stereo-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/ZED-2i-stereo-img.png)

You can view the occupancy grid through rviz2. You need to set up an Ubuntu22.04 + ROS2 Humble host computer, and the host computer and RDK X5 board can communicate through ROS2, then start rviz2. The specific settings are as follows:
![rviz2-occ](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/rviz2-occ.png)

- To save results, please add the following parameters:

```bash
# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch ZED-2i camera and occupancy network inference program
ros2 launch dstereo_occnet zed2i_occ_node.launch.py \
save_occ_flag:=True save_occ_dir:=./occ_result save_freq:=4 save_total:=10
```

### 2. Using Custom Data for Offline Inference

- Need to prepare offline data and upload it to the RDK X5 board. The format of offline data is as follows:
    - The offline directory needs to contain left and right images. The program will make judgments. The left image needs to contain the `left` field, in png or jpg format. The right image needs to contain the `right` field, and other aspects should be consistent with the left image
    - Image resolution is `640*352`, other resolutions are not supported
    - Left and right images need to be corrected to achieve epipolar alignment
    - Since the model is currently trained with ZED-2i data, try to make the intrinsic parameters of offline images close to ZED-2i. ZED-2i camera parameters are: `fx=354.9999, fy=354.9999, cx=322.9469, cy=176.2076, baseline=0.12`

- Execute the following commands on RDK X5:

```bash
ros2 launch dstereo_occnet offline_infer_web_visual.launch.py \
local_image_dir:=./offline_images save_occ_flag:=True save_occ_dir:=./offline_result
```

After the program starts, you can also view the stereo images published by ZED-2i through the web page and view the occupancy grid through rviz2

## Package Description

### Parameters

| Name                | Value                                           | Description                                                      |
|---------------------|------------------------------------------------|------------------------------------------------------------------|
| stereo_msg_topic    | Default /image_combine_raw                     | Name of the subscribed stereo image topic                       |
| camera_info_topic   | Default /image_combine_raw/camera_info         | Name of the subscribed camera intrinsic parameters topic        |
| occ_model_file_path | Default X5-OCC-32x64x96x2_constinput_modified.bin | Path of the stereo occupancy network model                      |
| use_local_image     | Default False                                  | Whether to use offline inference                                 |
| local_image_dir     | Default config                                 | Directory for storing images in offline inference               |
| save_occ_flag       | Default False                                  | Whether to save inference results                                |
| save_occ_dir        | Default ./occ_results                          | Directory for saving inference results                           |
| save_freq           | Default 1                                      | Save frequency, e.g., setting to 4 means saving once every 4 frames, default saves every frame inference result |
| save_total          | Default -1                                     | Total number of saves, e.g., setting to 10 means saving 10 frame results in total, -1 means save continuously |
| voxel_size          | Default 0.02                                   | Size of each occupancy grid, unit m, 0.02 means each occupancy grid is 2\*2\*2cm |
| log_level           | Default INFO                                   | Log level, default INFO                                          |

### Subscribed Topics

| Topic Name                     | Message Type                 | Description                                                                                    |
|--------------------------------|------------------------------|------------------------------------------------------------------------------------------------|
| /image_combine_raw             | sensor_msgs::msg::Image      | Subscribe to stereo images, image format is NV12, images are required to be arranged vertically with left image on top and right image at bottom, can be modified through stereo_msg_topic parameter |
| /image_combine_raw/camera_info | sensor_msgs::msg::CameraInfo | Subscribe to camera intrinsic parameters, optional, this topic may not exist, if it exists, camera parameters can be saved together when saving results |

### Published Topics

| Name                           | Message Type                  | Description                                        |
|--------------------------------|-------------------------------|----------------------------------------------------|
| /dstereo_occnet_node/voxel     | sensor_msgs::msg::PointCloud2 | Published occupancy grid data, can be displayed using rviz2 |
