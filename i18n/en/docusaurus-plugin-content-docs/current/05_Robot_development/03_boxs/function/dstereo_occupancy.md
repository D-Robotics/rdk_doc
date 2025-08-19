---
sidebar_position: 23
---

# Stereo OCC Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The **Digua Dual-lens OCC Algorithm** subscribes to dual-lens images, uses the BPU for algorithm inference, and publishes occupancy grid information.

Dual-lens OCC algorithm repository: https://github.com/D-Robotics/dstereo_occnet

ZED camera repository: https://github.com/D-Robotics/hobot_zed_cam

## Supported Platforms

| Platform              | System Support        | Example Functions                                                                                              |
| --------------------- | --------------------- | -------------------------------------------------------------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Launch the dual-lens camera, display dual-lens images via the web, and display occupancy grid results in rviz2 |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | Launch the dual-lens camera, display dual-lens images via the web, and display occupancy grid results in rviz2 |

## Algorithm Information

| Model         | Platform | Input Size  | Inference Frame Rate (fps) |
| ------------- | -------- | ----------- | -------------------------- |
| DStereoOccNet | X5       | 2x3x352x640 | 6                          |
| DStereoOccNet | S100     | 2x3x352x640 | 45                         |

## Preparations

### RDK Platform

1. The RDK has been flashed with the Ubuntu 22.04 system image.

2. The RDK has successfully installed TogetheROS.Bot.

3. If real-time online inference is required, it currently only supports the ZED-2i camera; if offline inference is required, please prepare <strong style={{ color: 'red' }}>rectified</strong> dual-lens image data.

4. Confirm that the PC can access the RDK over the network.

### System and Package Versions

|                                | Version Requirements | Query Method                                   |
| ------------------------------ | -------------------- | ---------------------------------------------- |
| RDK X5 system image version    | 3.3.1 and above      | `cat /etc/version`                             |
| RDK S100 system image version  | 4.0.2-Beta and above | `cat /etc/version`                             |
| dstereo_occnet package version | 1.0.1 and above      | `apt list \| grep tros-humble-dstereo-occnet/` |
| hobot_zed_cam                  | 2.3.3 and above      | `apt list \| grep tros-humble-hobot-zed-cam/`  |

## Usage Instructions

### 1. With ZED-2i Camera

- Requires the use of the hobot_zed_cam package.

- Execute the following command on the RDK (supported on both X5 and S100):

```bash
# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the ZED-2i camera and the occupancy network inference program
ros2 launch dstereo_occnet zed2i_occ_node.launch.py
```

- After the program starts, you can view the dual-lens images published by the ZED-2i via the web. Enter http://ip:8000 in the browser on the PC, where "ip" is the IP address of the RDK board (e.g., `192.168.128.10`), and ensure that the PC and RDK can communicate over the network.

![ZED-2i-stereo-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/ZED-2i-stereo-img.png)

- After the program starts, you can view the occupancy grid in rviz2. You can install rviz2 directly on the RDK for viewing. Note the following configurations in rviz2:

```bash
# Install rviz2
sudo apt install ros-humble-rviz2
# Launch rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![rviz2-occ](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/rviz2-occ.png)

- To save the results, add the following parameters. `save_occ_flag` turns on the save switch, `save_occ_dir` controls the save directory (it will be created automatically if it does not exist), `save_freq` controls the save frequency, and `save_total` controls the total number of saves:

```bash
# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the ZED-2i camera and the occupancy network inference program
ros2 launch dstereo_occnet zed2i_occ_node.launch.py \
save_occ_flag:=True save_occ_dir:=./occ_result save_freq:=4 save_total:=10
```

### 2. Offline Inference with Custom Data

- Prepare offline data and upload it to the RDK board. The format of the offline data is as follows:
    - The offline directory should contain left and right images. The program will determine them. The left image should contain the "left" field, and the format should be png or jpg. The right image should contain the "right" field and be consistent with the left image in other aspects.
    - The image resolution should be `640*352`, and other resolutions are not supported.
    - The left and right images should be rectified to achieve epipolar alignment.
    - Since the model is currently trained with ZED-2i data, try to make the intrinsic parameters of the offline images close to those of the ZED-2i. The camera parameters of the ZED-2i are: `fx=354.9999, fy=354.9999, cx=322.9469, cy=176.2076, baseline=0.12`.

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- Execute the following command on the RDK (supported on both X5 and S100). `local_image_dir` controls the directory of the offline data, `save_occ_flag` turns on the save switch, and `save_occ_dir` controls the save directory (it will be created automatically if it does not exist):

```bash
ros2 launch dstereo_occnet offline_infer_web_visual.launch.py \
local_image_dir:=./offline_images save_occ_flag:=True save_occ_dir:=./offline_result
```

- After the program starts, you can also view the dual-lens images published by the ZED-2i via the web and view the occupancy grid in rviz2.

## Package Description

### Parameters

| Name                | Parameter Values                                   | Description                                                                                                                                   |
| ------------------- | -------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| stereo_msg_topic    | Default: /image_combine_raw                        | The topic name of the subscribed dual-lens images                                                                                             |
| camera_info_topic   | Default: /image_combine_raw/camera_info            | The topic name of the subscribed camera intrinsic parameters                                                                                  |
| occ_model_file_path | Default: X5-OCC-32x64x96x2_constinput_modified.bin | The path of the dual-lens occupancy network model                                                                                             |
| use_local_image     | Default: False                                     | Whether to use offline inference                                                                                                              |
| local_image_dir     | Default: config                                    | The directory where the images for offline inference are stored                                                                               |
| save_occ_flag       | Default: False                                     | Whether to save the inference results                                                                                                         |
| save_occ_dir        | Default: ./occ_results                             | The directory where the inference results are saved                                                                                           |
| save_freq           | Default: 1                                         | The save frequency. For example, setting it to 4 means saving once every 4 frames. By default, the inference results of each frame are saved. |
| save_total          | Default: -1                                        | The total number of saves. For example, setting it to 10 means saving a total of 10 frames of results. -1 means saving continuously.          |
| voxel_size          | Default: 0.02                                      | The size of each occupancy grid, in meters. 0.02 means each occupancy grid is 2*2*2 cm.                                                       |
| log_level           | Default: INFO                                      | The log level, default is INFO                                                                                                                |

### Subscribed Topics

| Topic Name                     | Message Type                 | Description                                                                                                                                                                                                                          |
| ------------------------------ | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| /image_combine_raw             | sensor_msgs::msg::Image      | Subscribe to dual-lens images. The image format is NV12, and the images are arranged vertically with the left image on top and the right image on the bottom. The topic name can be modified through the stereo_msg_topic parameter. |
| /image_combine_raw/camera_info | sensor_msgs::msg::CameraInfo | Subscribe to the camera intrinsic parameters. This is optional and not required. If available, the camera parameters can be saved together with the inference results.                                                               |

### Published Topics

| Name                       | Message Type                  | Description                                                        |
| -------------------------- | ----------------------------- | ------------------------------------------------------------------ |
| /dstereo_occnet_node/voxel | sensor_msgs::msg::PointCloud2 | The published occupancy grid data, which can be displayed in rviz2 |
