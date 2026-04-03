---
sidebar_position: 1
---
# mobilenetv2

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The mobilenetv2 image classification algorithm example takes images as input, performs inference using the BPU, and publishes algorithm messages containing object categories.

mobilenetv2 is a Caffe model trained on the [ImageNet data](http://www.image-net.org/) dataset. Model source: https://github.com/shicai/MobileNet-Caffe.  
It supports 1,000 object categories, including people, animals, fruits, vehicles, etc. For the complete list of supported categories, refer to the RDK onboard file `/opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/imagenet.list` (requires TogetheROS.Bot installation).

Code repository: https://github.com/D-Robotics/hobot_dnn

Application scenarios: mobilenetv2 can predict the category of a given image, enabling functionalities such as digit recognition and object recognition. It is primarily applied in fields like text recognition and image retrieval.

Food classification example: https://github.com/frotms/Chinese-and-Western-Food-Classification

## Supported Platforms

| Platform                     | Operating System               | Example Features                                                                                      |
| ---------------------------- | ------------------------------ | ----------------------------------------------------------------------------------------------------- |
| RDK X3, RDK X3 Module        | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| RDK X5, RDK X5 Module        | Ubuntu 22.04 (Humble)          | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| RDK Ultra                    | Ubuntu 20.04 (Foxy)            | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| RDK S100, RDK S100P          | Ubuntu 22.04 (Humble)          | · Launch MIPI/USB camera and display inference results via web browser<br/>· Use local image/video replay; rendered results saved locally |
| X86                          | Ubuntu 20.04 (Foxy)            | · Use local image/video replay; rendered results saved locally                                        |

## Algorithm Details

| Model        | Platform | Input Size     | Inference FPS |
| ------------ | -------- | -------------- | ------------- |
| mobilenetv2  | X3       | 1x3x224x224    | 414.17        |
| mobilenetv2  | X5       | 1x3x224x224    | 683.46        |
| mobilenetv2  | S100     | 1x3x224x224    | 1722.25       |

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. tros.b has been successfully installed on the RDK.
3. An MIPI or USB camera is installed on the RDK. If no camera is available, you can evaluate the algorithm by replaying local JPEG/PNG images or MP4/H.264/H.265 videos.
4. Ensure your PC can access the RDK over the network.

### X86 Platform

1. The X86 environment has been set up with Ubuntu 20.04 system image.
2. tros.b has been successfully installed in the X86 environment.

## Usage Guide

### RDK Platform

The mobilenetv2 image classification node subscribes to images published by the sensor package, performs inference, publishes algorithm messages, and renders both the image and inference results in a web browser on the PC via the websocket package.

#### Using MIPI Camera to Publish Images


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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Using USB Camera to Publish Images


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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Replaying Local Images

The mobilenetv2 image classification example replays local JPEG/PNG images. After inference, the rendered result image is saved in the current working directory.


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
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image:=config/target_class.jpg
```

### X86 Platform

#### Replaying Local Images

The mobilenetv2 image classification example replays local JPEG/PNG images. After inference, the rendered result image is saved in the current working directory.

```bash
# Configure tros.b environment
source /opt/tros/setup.bash

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image:=config/target_class.jpg
```

## Result Analysis

### Using Camera to Publish Images

The terminal output shows the following logs:

```shell
[example-3] [WARN] [1655095481.707875587] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655095481.707983957] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655095482.985732162] [img_sub]: Sub img fps 31.07
[example-3] [WARN] [1655095482.992031931] [example]: Smart fps 31.31
[example-3] [WARN] [1655095484.018818843] [img_sub]: Sub img fps 30.04
[example-3] [WARN] [1655095484.025123362] [example]: Smart fps 30.04
[example-3] [WARN] [1655095485.051988567] [img_sub]: Sub img fps 30.01
[example-3] [WARN] [1655095486.057854228] [example]: Smart fps 30.07
```

The logs indicate that:
- The algorithm inference results are published on the topic `hobot_dnn_detection`.
- Images are subscribed from the topic `/hbmem_img`.
- Both the subscribed image stream and inference output run at approximately 30 FPS.

Enter `http://IP:8000` in your PC's web browser to view the rendered image and inference results (replace `IP` with the RDK’s IP address):

![render_web](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/classification/image/mobilenetv2/mobilenetv2_render_web.jpeg)

### Replaying Local Images

The terminal output shows the following logs:
```shell
[example-1] [INFO] [1654767648.897132079] [example]: The model input width is 224 and height is 224
[example-1] [INFO] [1654767648.897180241] [example]: Dnn node feed with local image: config/target_class.jpg
[example-1] [INFO] [1654767648.935638968] [example]: task_num: 2
[example-1] [INFO] [1654767648.946566665] [example]: Output from image_name: config/target_class.jpg, frame_id: feedback, stamp: 0.0
[example-1] [INFO] [1654767648.946671029] [ClassificationPostProcess]: outputs size: 1
[example-1] [INFO] [1654767648.946718774] [ClassificationPostProcess]: out cls size: 1
[example-1] [INFO] [1654767648.946773602] [ClassificationPostProcess]: class type:window-shade, score:0.776356
[example-1] [INFO] [1654767648.947251721] [ImageUtils]: target size: 1
[example-1] [INFO] [1654767648.947342212] [ImageUtils]: target type: window-shade, rois.size: 1
[example-1] [INFO] [1654767648.947381666] [ImageUtils]: roi.type: , x_offset: 112 y_offset: 112 width: 0 height: 0
[example-1] [WARN] [1654767648.947563731] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg
```

The output log shows that the algorithm inferred the image classification result for the input image `config/target_class.jpg` as "window-shade", with a confidence score of 0.776356 (the algorithm only outputs the classification result with the highest confidence). The rendered image is saved as `render_feedback_0_0.jpeg`, and the rendering effect is shown below:

![render_feedback](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/classification/image/mobilenetv2/mobilenetv2_render_feedback.jpeg)