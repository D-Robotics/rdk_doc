---
sidebar_position: 6
---
# 5.2.6 Model Inference


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

This section introduces the usage of the model inference function. You can input a local image for inference, and get the rendered image saved locally.

Code repository: [https://github.com/HorizonRDK/hobot_dnn](https://github.com/HorizonRDK/hobot_dnn)

## Supported Platforms

| Platform    | System     |
| ------------ | ---------------- |
| RDK X3, RDK X3 Module| Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |

## Prerequisites

### Horizon RDK

1. The Horizon RDK is already burned with the provided  Ubuntu 20.04/22.04 system image.

2. The TogetheROS.Bot has been successfully installed on the Horizon RDK.

## Usage

Use the local JPEG image and model in the hobot_dnn configuration file (FCOS object detection model, supporting 80 types of object detection including human, animal, fruit, and transportation, etc.), perform inference through offline, and save the rendered image.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Copy the configuration file needed for the example to run from the installation path of tros.b. config contains the model used by the example and the local image used for feedback
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# Perform feedback prediction using the local jpg format image and save the rendered image
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image:=config/target.jpg
```

After a successful run, the rendered image will be automatically saved in the execution path, named as render_feedback_0_0.jpeg. Use Ctrl+C to exit the program.

## Analysis of Results

The terminal output during the execution of the command provides the following information:

```text
[example-1] [INFO] [1679901151.612290039] [ImageUtils]: target size: 6
[example-1] [INFO] [1679901151.612314489] [ImageUtils]: target type: couch, rois.size: 1
[example-1] [INFO] [1679901151.612326734] [ImageUtils]: roi.type: couch, x_offset: 83 y_offset: 265 width: 357 height: 139
[example-1] [INFO] [1679901151.612412454] [ImageUtils]: target type: potted plant, rois.size: 1
[example-1] [INFO] [1679901151.612426522] [ImageUtils]: roi.type: potted plant, x_offset: 379 y_offset: 173 width: 131 height: 202
[example-1] [INFO] [1679901151.612472961] [ImageUtils]: target type: book, rois.size: 1
[example-1] [INFO] [1679901151.612497709] [ImageUtils]: roi.type: book, x_offset: 167 y_offset: 333 width: 67 height: 22
[example-1] [INFO] [1679901151.612522859] [ImageUtils]: target type: vase, rois.size: 1
[example-1] [INFO] [1679901151.612533487] [ImageUtils]: roi.type: vase, x_offset: 44 y_offset: 273 width: 26 height: 45
[example-1] [INFO] [1679901151.612557172] [ImageUtils]: target type: couch, rois.size: 1
[example-1] [INFO] [1679901151.612567740] [ImageUtils]: roi.type: couch, x_offset: 81 y_offset: 265 width: 221 height: 106
[example-1] [INFO] [1679901151.612606444] [ImageUtils]: target type: potted plant, rois.size: 1
[example-1] [INFO] [1679901151.612617518] [ImageUtils]: roi.type: potted plant, x_offset: 138 y_offset: 314 width: 45 height: 38
[example-1] [WARN] [1679901151.612652352] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg
```

The log output shows that the algorithm has inferred 6 targets based on the input image and provided the class (target type) and the coordinates of the detection boxes (x_offset and y_offset for the top left corner of the box, and width and height of the box). The rendered image file is saved as render_feedback_0_0.jpeg.

The rendered image, render_feedback_0_0.jpeg, is shown below:

![](./image/ai_predict/render1.jpg)