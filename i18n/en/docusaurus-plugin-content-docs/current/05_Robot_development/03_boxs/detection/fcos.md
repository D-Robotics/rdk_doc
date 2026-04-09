---
sidebar_position: 1
---
# FCOS

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The FCOS object detection algorithm example takes images as input, performs inference using the BPU, and publishes intelligent messages containing object categories and bounding boxes.

FCOS is an open-source ONNX model provided by D-Robotics, trained on the [COCO dataset](http://cocodataset.org/). It supports 80 object categories, including people, animals, fruits, vehicles, and more.

Code repository: (https://github.com/D-Robotics/hobot_dnn)

Application scenarios: Introduced in 2019, FCOS is a single-stage object detection algorithm capable of pedestrian detection, vehicle detection, and similar tasks. It is primarily used in autonomous driving, smart homes, and related fields.

Multispectral object detection example: (https://github.com/hdjsjyl/Multispectral-FCOS)

## Supported Platforms

| Platform              | Runtime Environment     | Example Features                                                     |
| --------------------- | ------------------------ | -------------------------------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble)    | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| X86                   | Ubuntu 20.04 (Foxy)      | · Use local image/video replay; rendered results saved locally        |

## Algorithm Details

| Model | Platform | Input Size     | Inference FPS |
| ----- | -------- | -------------- | ------------- |
| fcos  | X3       | 1x3x512x512    | 74.91         |
| fcos  | X5       | 1x3x512x512    | 258.92        |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with an Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on the RDK.
3. An MIPI or USB camera is installed on the RDK. If no camera is available, you can evaluate the algorithm using local JPEG/PNG images or MP4/H.264/H.265 video files via replay.
4. Ensure your PC can access the RDK over the network.

### X86 Platform

1. The X86 environment has been configured with an Ubuntu 20.04 system image.
2. TogetherROS.Bot (`tros.b`) has been successfully installed on the X86 system.

## Usage Guide

### RDK Platform

#### Publishing Images Using an MIPI Camera

The FCOS object detection example subscribes to images published by the sensor package, performs inference, and publishes algorithm messages. Results are rendered and displayed in a web browser on the PC via the websocket package.

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
# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Publishing Images Using a USB Camera

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
# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Using Local Image Replay

The FCOS object detection example replays local JPEG/PNG images, performs inference, and saves the rendered result images in the current working directory.

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
# Launch the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image:=config/target.jpg
```

### X86 Platform

#### Using Local Image Replay

The FCOS object detection example replays local JPEG/PNG images, performs inference, and saves the rendered result images in the local working directory.

```bash
# Configure tros.b environment
source /opt/tros/setup.bash

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image:=config/target.jpg
```

## Result Analysis

### Publishing Images Using a Camera

The terminal output shows the following logs:

```text
[example-3] [WARN] [1655092908.847609539] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655092908.849393011] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655092543.834432739] [img_sub]: Sub img fps 31.16
[example-3] [WARN] [1655092543.864126080] [example]: Smart fps 31.56
[example-3] [WARN] [1655092544.867603759] [img_sub]: Sub img fps 30.01
[example-3] [WARN] [1655092544.899715339] [example]: Smart fps 29.95
[example-3] [WARN] [1655092545.900991853] [img_sub]: Sub img fps 30.01
[example-3] [WARN] [1655092545.931518037] [example]: Smart fps 30.07
[example-3] [WARN] [1655092546.901658559] [img_sub]: Sub img fps 30.00
[example-3] [WARN] [1655092546.938970895] [example]: Smart fps 29.79
[example-3] [WARN] [1655092547.934894494] [img_sub]: Sub img fps 30.01
[example-3] [WARN] [1655092547.973566486] [example]: Smart fps 29.98
[example-3] [WARN] [1655092548.967549745] [img_sub]: Sub img fps 30.10
[example-3] [WARN] [1655092548.997125216] [example]: Smart fps 30.30

```

The log indicates that:
- The topic for publishing inference results is `hobot_dnn_detection`.
- The topic for subscribing to input images is `/hbmem_img`.
- Both the input image subscription rate and the algorithm inference output rate are approximately 30 FPS.

Enter `http://IP:8000` in your PC’s web browser to view the rendered images and algorithm results (replace `IP` with the RDK’s IP address):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/detection/image/box_basic/fcos_render_web.jpeg)
### Using Local Images for Feedback Injection

The terminal outputs the following information when running:

```text
[example-1] [INFO] [1654766336.839353395] [PostProcessBase]: out box size: 6
[example-1] [INFO] [1654766336.839427767] [PostProcessBase]: det rect: 87.2364 259.123 409.917 371.59, det type: couch, score:0.782941
[example-1] [INFO] [1654766336.839523764] [PostProcessBase]: det rect: 374.212 175.732 510.993 375.211, det type: potted plant, score:0.719925
[example-1] [INFO] [1654766336.839597637] [PostProcessBase]: det rect: 167.183 335.857 234.13 355.308, det type: book, score:0.548071
[example-1] [INFO] [1654766336.839671426] [PostProcessBase]: det rect: 139.87 313.279 183.4 352.292, det type: potted plant, score:0.542984
[example-1] [INFO] [1654766336.839738966] [PostProcessBase]: det rect: 57.9695 148.59 83.5923 186.552, det type: potted plant, score:0.502935
[example-1] [INFO] [1654766336.839823755] [PostProcessBase]: det rect: 165.691 339.25 237.475 366.896, det type: book, score:0.500648
```

The output log indicates that the algorithm inferred six objects from the input image and provided the bounding box coordinates (in the order of top-left x, top-left y, bottom-right x, and bottom-right y) along with their corresponding categories. The rendered image is saved as `render_feedback_0_0.jpeg`. Rendered image result:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/detection/image/box_basic/fcos_render_feedback.jpeg)