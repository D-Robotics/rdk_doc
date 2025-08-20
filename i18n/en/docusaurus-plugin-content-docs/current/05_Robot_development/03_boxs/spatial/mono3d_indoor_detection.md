---
sidebar_position: 2
---
# Monocular 3D Indoor Detection


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The mono3d_indoor_detection package is an example of indoor object 3D detection algorithm based on the hobot_dnn package. It uses the 3D detection model and indoor data on the D-Robotics's RDK to perform model inference using BPU and obtain the inference results.

Compared to 2D object detection, which can only recognize the object category and bounding box, 3D object detection can identify the precise position and orientation of the object. For example, in navigation the rich information provided by 3D object detection algorithms can help the planning and control robot achieve better effects.

The supported indoor object detection categories of the algorithm include: charging docks, trash cans, and slippers.

The detection results for each category include:

- Length, width, height: The length, width, and height of the three-dimensional object (i.e. hexahedron), measured in meters.

- Orientation: The orientation of the object relative to the camera, measured in radians. The range is from -π to π, representing the angle between the camera coordinate system x-axis and the object's forward direction in the camera coordinate system.

- Depth information: The distance from the camera to the object, measured in meters.

Code repository:  (https://github.com/D-Robotics/mono3d_indoor_detection)

Applications: The monocular 3D indoor detection algorithm can directly identify the exact position and orientation of objects in images, enabling object posture recognition. It is mainly used in autonomous driving, smart home, and other fields.

## Supported Platforms

| Platform              | System | Function                                       |
| --------------------- | ---------------- | ----------------------------------------------------- |
| RDK X3, RDK X3 Module, RDK X5 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | · Start MIPI/USB camera/local data and save the inference rendering result locally |

## Preparation

### RDK

1. RDK has been flashed with the  Ubuntu 20.04/22.04 system system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on the RDK.

## Usage

Because the 3D detection model is related to camera parameters, different cameras need to adjust the parameters accordingly.

The mono3d_indoor_detection algorithm package uses local image input for inference. After the inference, it can detect object categories and 3D positioning information, and publish the algorithm message for 3D information. Users can subscribe to the 3D detection result message for application development.

### RDK

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono3d_indoor_detection/config/ .

# Start the launch file
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

## Result Analysis

After processing one frame of image data, the mono3d_indoor_detection package outputs the following information:

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

The log displays the processing result of a frame. The result shows that the target type in the subscribed algorithm message is trash_can, and it also provides the 3D coordinates, distance, and rotation angle information of the trash_can.

The rendered result of a local image (which can be replaced by modifying the feed_image field in mono3d_indoor_detection.launch.py) is saved as an image in the result directory of the program. The corresponding inference result and rendering information of the image are as follows:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/indoor_render.jpeg)