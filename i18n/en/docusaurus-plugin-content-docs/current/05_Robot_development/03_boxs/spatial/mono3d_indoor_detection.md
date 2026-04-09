---
sidebar_position: 2
---
# Monocular 3D Indoor Detection

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The `mono3d_indoor_detection` package is an example implementation of an indoor object 3D detection algorithm developed based on the `hobot_dnn` package. It leverages the BPU to perform model inference on RDK using a 3D detection model and indoor data, thereby producing inference results.

Compared with 2D object detection—which can only identify object categories and bounding boxes—3D object detection provides precise object positions and orientations. For instance, in navigation and obstacle avoidance scenarios, the rich information provided by 3D object detection algorithms helps the planning and control module achieve better obstacle avoidance performance.

The supported indoor object categories for detection include: charging dock, trash can, and slippers.

The detection result for each category includes:

- **Length, Width, Height**: Dimensions (in meters) of the 3D object (i.e., a cuboid).

- **Rotation**: The orientation of the object relative to the camera, measured in radians within the range [-π, π]. This represents the angle between the object’s forward direction and the x-axis of the camera coordinate system.

- **Depth**: Distance from the camera to the object, in meters.

Code Repository: [https://github.com/D-Robotics/mono3d_indoor_detection](https://github.com/D-Robotics/mono3d_indoor_detection)

Application Scenarios: The monocular 3D indoor detection algorithm directly identifies the exact position and orientation of objects in images, enabling object pose recognition. It is primarily applied in autonomous driving, smart homes, and similar fields.

Monocular 3D Vehicle Detection Example: [https://github.com/RayXie29/Kaggle-Peking-University-Baidu-Autonomous-Driving-32-place-solution](https://github.com/RayXie29/Kaggle-Peking-University-Baidu-Autonomous-Driving-32-place-solution)

## Supported Platforms

| Platform              | Runtime Environment                    | Example Functionality                                               |
| --------------------- | -------------------------------------- | ------------------------------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch MIPI/USB camera and display rendered inference results via Web |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble)                  | Launch MIPI/USB camera and display rendered inference results via Web |
| X86                   | Ubuntu 20.04 (Foxy)                    | • Play back local images; save rendered inference results locally    |

## Algorithm Details

| Model      | Platform | Input Size     | Inference FPS |
| ---------- | -------- | -------------- | ------------- |
| centernet  | X3       | 1x3x512x960    | 85.93         |
| centernet  | X5       | 1x3x512x960    | 196.33        |

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on RDK.

### X86 Platform

1. The X86 environment has been configured with Ubuntu 20.04 system image.
2. tros.b has been successfully installed in the X86 environment.

## Usage Guide

Since the 3D detection model is related to camera parameters, adjustments are required for different cameras.

The `mono3d_indoor_detection` example package performs detection inference by reading local images. After inference, it outputs object categories and 3D localization information, and publishes algorithm messages containing 3D detection results. Users can subscribe to these messages for application development.

### RDK Platform

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy required configuration files from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono3d_indoor_detection/config/ .

# Launch the launch file
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

### X86 Platform

```bash
# Configure tros.b environment
source /opt/tros/setup.bash

# Copy required configuration files from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono3d_indoor_detection/config/ .

# Launch the launch file
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

## Result Analysis

After processing one frame of image data, the `mono3d_indoor_detection` package outputs the following information in the terminal:

```shell
[mono3d_indoor_detection-1] [INFO] [1662612553.868256257] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868303755] [mono3d_detection]: target type: width, value: 0.236816
[mono3d_indoor_detection-1] [INFO] [1662612553.868358420] [mono3d_detection]: target type: height, value: 0.305664
[mono3d_indoor_detection-1] [INFO] [1662612553.868404002] [mono3d_detection]: target type: length, value: 0.224182
[mono3d_indoor_detection-1] [INFO] [1662612553.868448000] [mono3d_detection]: target type: rotation, value: -1.571989
[mono3d_indoor_detection-1] [INFO] [1662612553.868487790] [mono3d_detection]: target type: x, value: -0.191978
[mono3d_indoor_detection-1] [INFO] [1662612553.868530705] [mono3d_detection]: target type: y, value: -0.143963
[mono3d_indoor_detection-1] [INFO] [1662612553.868570870] [mono3d_detection]: target type: z, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868611119] [mono3d_detection]: target type: depth, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868651409] [mono3d_detection]: target type: score, value: 0.973215
[mono3d_indoor_detection-1] [INFO] [1662612553.868760238] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868799486] [mono3d_detection]: target type: width, value: 0.253052
[mono3d_indoor_detection-1] [INFO] [1662612553.868842610] [mono3d_detection]: target type: height, value: 0.282349
[mono3d_indoor_detection-1] [INFO] [1662612553.868885191] [mono3d_detection]: target type: length, value: 0.257935
[mono3d_indoor_detection-1] [INFO] [1662612553.868929273] [mono3d_detection]: target type: rotation, value: -1.542728
[mono3d_indoor_detection-1] [INFO] [1662612553.868968855] [mono3d_detection]: target type: x, value: 0.552460
[mono3d_indoor_detection-1] [INFO] [1662612553.869010645] [mono3d_detection]: target type: y, value: -0.164073
[mono3d_indoor_detection-1] [INFO] [1662612553.869050018] [mono3d_detection]: target type: z, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869088767] [mono3d_detection]: target type: depth, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869126765] [mono3d_detection]: target type: score, value: 0.875521
```

The log excerpt above shows the processing results for one frame. It indicates that the `target type` field in the subscribed algorithm message is `trash_can`, along with its 3D dimensions, distance, and rotation angle.

Rendered results from processing local images (the input image can be changed by modifying the `feed_image` field in `mono3d_indoor_detection.launch.py`) are saved as images in the `result` directory under the program's working directory. The corresponding inference and rendering results are shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/indoor_render.jpeg)