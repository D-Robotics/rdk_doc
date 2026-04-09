---
sidebar_position: 6
---

# Stereo OCC Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

**Digua Stereo OCC Algorithm** subscribes to binocular images, performs inference using the BPU, and publishes occupancy grid information.

Stereo OCC algorithm code repository: https://github.com/D-Robotics/dstereo_occnet

ZED camera code repository: https://github.com/D-Robotics/hobot_zed_cam

## Supported Platforms

| Platform              | OS Support            | Example Features                                                                 |
| --------------------- | --------------------- | -------------------------------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Launch binocular camera, display binocular images via web, and visualize occupancy grids in rviz2 |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | Launch binocular camera, display binocular images via web, and visualize occupancy grids in rviz2 |

## Algorithm Details

| Model         | Platform | Input Size      | Inference FPS |
| ------------- | -------- | --------------- | ------------- |
| DStereoOccNet | X5       | 2x3x352x640     | 6             |
| DStereoOccNet | S100     | 2x3x352x640     | 45            |

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. For real-time online inference, only ZED-2i cameras are currently supported. For offline inference, please prepare **rectified** binocular image data.
4. Ensure your PC can access the RDK over the network.

### System and Package Versions

|                          | Version Requirement    | Verification Command                                   |
| ------------------------ | ---------------------- | ------------------------------------------------------ |
| RDK X5 System Image      | v3.3.1 or later        | `cat /etc/version`                                     |
| RDK S100 System Image    | v4.0.2-Beta or later   | `cat /etc/version`                                     |
| dstereo_occnet Package   | v1.0.1 or later        | `apt list \| grep tros-humble-dstereo-occnet/`         |
| hobot_zed_cam            | v2.3.3 or later        | `apt list \| grep tros-humble-hobot-zed-cam/`          |

## Usage Guide

### 1. Using ZED-2i Camera

- Requires the `hobot_zed_cam` package.
- Execute the following commands on the RDK (supported on both X5 and S100):

```bash
# Source the TogetheROS.Bot Humble environment
source /opt/tros/humble/setup.bash

# Launch ZED-2i camera and occupancy network inference node
ros2 launch dstereo_occnet zed2i_occ_node.launch.py
```

- After launching, you can view the binocular images published by ZED-2i through a web browser. Open `http://<ip>:8000` in your PC's browser, where `<ip>` is the RDK board's IP address (e.g., `192.168.128.10`). Ensure network connectivity between your PC and the RDK.

![ZED-2i-stereo-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/ZED-2i-stereo-img.png)

- You can also visualize the occupancy grid using rviz2. Install and launch rviz2 directly on the RDK as follows:

```bash
# Install rviz2
sudo apt install ros-humble-rviz2
# Launch rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![rviz2-occ](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/rviz2-occ.png)

- To save results, add the following parameters:
  - `save_occ_flag`: Enable saving results.
  - `save_occ_dir`: Specify the directory to save results (automatically created if it doesn't exist).
  - `save_freq`: Set saving frequency (e.g., 4 means saving every 4 frames).
  - `save_total`: Set total number of frames to save (e.g., 10); `-1` means save indefinitely.

```bash
# Source the TogetheROS.Bot Humble environment
source /opt/tros/humble/setup.bash

# Launch ZED-2i camera and occupancy network inference node with saving enabled
ros2 launch dstereo_occnet zed2i_occ_node.launch.py \
save_occ_flag:=True save_occ_dir:=./occ_result save_freq:=4 save_total:=10
```

### 2. Offline Inference with Custom Data

- Prepare offline data and upload it to the RDK. The data must meet the following requirements:
  - The directory must contain left and right images.
    - Left image filenames must include the substring `left` (format: PNG or JPG).
    - Right image filenames must include the substring `right`, with the same format as the left image.
  - Image resolution must be `640×352`. Other resolutions are not supported.
  - Images must be rectified to achieve epipolar alignment.
  - Since the model was trained using ZED-2i data, try to match the intrinsic parameters of ZED-2i as closely as possible:
    - ZED-2i camera parameters: `fx=354.9999, fy=354.9999, cx=322.9469, cy=176.2076, baseline=0.12`

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- Run the following command on the RDK (supported on both X5 and S100):
  - `local_image_dir`: Specifies the directory containing offline images.
  - `save_occ_flag`: Enables result saving.
  - `save_occ_dir`: Specifies the output directory (automatically created if it doesn't exist).

```bash
ros2 launch dstereo_occnet offline_infer_web_visual.launch.py \
local_image_dir:=./offline_images save_occ_flag:=True save_occ_dir:=./offline_result
```

- After launching, you can view the binocular images via a web browser and visualize the occupancy grid using rviz2, just like in the online case.

## Package Description

### Parameters

| Name                  | Default Value                                      | Description                                                                 |
| --------------------- | -------------------------------------------------- | --------------------------------------------------------------------------- |
| stereo_msg_topic      | `/image_combine_raw`                               | Topic name for subscribed binocular images                                  |
| camera_info_topic     | `/image_combine_raw/camera_info`                   | Topic name for subscribed camera intrinsics                                 |
| occ_model_file_path   | `X5-OCC-32x64x96x2_constinput_modified.bin`        | Path to the binocular occupancy network model                               |
| use_local_image       | `False`                                            | Whether to use offline inference                                            |
| local_image_dir       | `config`                                           | Directory containing offline images for inference                           |
| save_occ_flag         | `False`                                            | Whether to save inference results                                           |
| save_occ_dir          | `./occ_results`                                    | Directory to save inference results                                         |
| save_freq             | `1`                                                | Saving frequency (e.g., 4 = save every 4 frames; default = every frame)      |
| save_total            | `-1`                                               | Total number of frames to save (e.g., 10); `-1` = save indefinitely          |
| voxel_size            | `0.02`                                             | Size of each occupancy voxel in meters (0.02 = 2×2×2 cm per voxel)           |
| log_level             | `INFO`                                             | Logging level (default: INFO)                                               |

### Subscribed Topics

| Topic Name                    | Message Type                 | Description                                                                                                                              |
| ----------------------------- | ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `/image_combine_raw`          | `sensor_msgs::msg::Image`    | Subscribes to binocular images in NV12 format, arranged vertically (left image on top, right image on bottom). Configurable via `stereo_msg_topic`. |
| `/image_combine_raw/camera_info` | `sensor_msgs::msg::CameraInfo` | Subscribes to camera intrinsics (optional). If available, camera parameters will be saved along with results.                            |

### Published Topics

| Name                              | Message Type                   | Description                                      |
| --------------------------------- | ------------------------------ | ------------------------------------------------ |
| `/dstereo_occnet_node/voxel`      | `sensor_msgs::msg::PointCloud2` | Publishes occupancy grid data for visualization in rviz2 |