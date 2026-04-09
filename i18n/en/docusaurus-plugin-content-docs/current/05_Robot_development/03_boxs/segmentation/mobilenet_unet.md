---
sidebar_position: 1
---
# mobilenet_unet

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The mobilenet_unet segmentation algorithm example takes images as input, performs inference using the BPU, and publishes messages containing segmentation results.

mobilenet_unet is an ONNX model trained on the [Cityscapes](https://www.cityscapes-dataset.com/) dataset, supporting segmentation of categories such as pedestrians, vehicles, roads, and road signs.

Code repository: https://github.com/D-Robotics/hobot_dnn

Application scenarios: mobilenet_unet combines MobileNet and UNet architectures to enable pixel-level image segmentation. It can be used for road recognition, remote sensing map analysis, medical image diagnosis, and similar tasks, primarily applied in autonomous driving, geological inspection, and medical imaging analysis.

Background blur example: https://github.com/rusito-23/mobile_unet_segmentation

## Supported Platforms

| Platform                | Runtime Environment                     | Example Features                                                   |
| ----------------------- | --------------------------------------- | ------------------------------------------------------------------ |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · Start MIPI/USB camera or local image playback; rendered results saved locally |
| RDK X5, RDK X5 Module   | Ubuntu 22.04 (Humble)                   | · Start MIPI/USB camera or local image playback; rendered results saved locally |
| RDK S100, RDK S100P      | Ubuntu 22.04 (Humble)                   | · Start MIPI/USB camera or local image playback; rendered results saved locally |
| X86                     | Ubuntu 20.04 (Foxy)                     | · Use local image playback; rendered results saved locally         |

## Algorithm Information

| Model           | Platform | Input Size       | Inference FPS |
| --------------- | -------- | ---------------- | ------------- |
| mobilenet_unet  | X3       | 1x3x1024x2048    | 24.34         |
| mobilenet_unet  | X5       | 1x3x224x224      | 50.33         |
| deeplabv3       | S100     | 1x3x224x224      | 14.70         |

## Prerequisites

### RDK Platforms

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. An MIPI or USB camera is installed on the RDK. If no camera is available, you can evaluate the algorithm by feeding local JPEG/PNG images.

### X86 Platform

1. The X86 environment has been set up with Ubuntu 20.04 system image.
2. tros.b has been successfully installed in the X86 environment.

## Usage Guide

### RDK Platforms

#### Publishing Images via Camera

##### Using MIPI Camera

The mobilenet_unet segmentation example subscribes to images published by the sensor package, performs inference, publishes algorithm messages, and automatically saves rendered images in the current working directory. Rendered images are named as `render_frameid_timestamp_sec_timestamp_nsec.jpg`.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=1 dnn_example_config_file:=config/mobilenet_unet_workconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

##### Using USB Camera

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=1 dnn_example_config_file:=config/mobilenet_unet_workconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

#### Feeding Local Images

The mobilenet_unet segmentation example feeds local JPEG/PNG images, performs inference, and saves the rendered result images in the current working directory.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>


```shell
# Launch the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenet_unet_workconfig.json dnn_example_image:=config/raw_unet.jpg
```

### X86 Platform

#### Feeding Local Images

The mobilenet_unet segmentation example feeds local JPEG/PNG images, performs inference, and saves the rendered result images in the current working directory.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Set up tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Set up tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# Launch the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenet_unet_workconfig.json dnn_example_image:=config/raw_unet.jpg
```

## Result Analysis

### Publishing Images via Camera

The following output appears in the terminal:

```shell
[example-3] [WARN] [1655095719.035374293] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655095719.035493746] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655095720.693716453] [img_sub]: Sub img fps 6.85
[example-3] [WARN] [1655095721.072909861] [example]: Smart fps 5.85
[example-3] [WARN] [1655095721.702680885] [img_sub]: Sub img fps 3.97
```
[example-3] [WARN] [1655095722.486407545] [example]: Smart fps 3.54  
[example-3] [WARN] [1655095722.733431396] [img_sub]: Sub img fps 4.85  
[example-3] [WARN] [1655095723.888407681] [example]: Smart fps 4.28  
[example-3] [WARN] [1655095724.069835983] [img_sub]: Sub img fps 3.74  
[example-3] [WARN] [1655095724.900725522] [example]: Smart fps 3.95  
[example-3] [WARN] [1655095725.093525634] [img_sub]: Sub img fps 3.91  

The output log shows that the topic for publishing algorithm inference results is `hobot_dnn_detection`, and the topic for subscribing to images is `/hbmem_img`. The image publishing frame rate adaptively adjusts according to the algorithm's inference output frame rate. Additionally, semantic segmentation results are rendered on the RDK, and the resulting images are saved to the execution path, which reduces the frame rate.

Original image:  
![raw](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/mobilenet_unet/mobilenet_unet_raw.jpeg)

Rendered image:  
![render_web](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/mobilenet_unet/mobilenet_unet_render_web.jpeg)

### Using Local Images for Replay

The following messages appear in the terminal output:

```shell
[example-1] [INFO] [1654769881.171005839] [dnn]: The model input 0 width is 2048 and height is 1024
[example-1] [INFO] [1654769881.171129709] [example]: Set output parser.
[example-1] [INFO] [1654769881.171206707] [UnetPostProcess]: Set out parser
[example-1] [INFO] [1654769881.171272663] [dnn]: Task init.
[example-1] [INFO] [1654769881.173427170] [dnn]: Set task_num [2]
[example-1] [INFO] [1654769881.173587414] [example]: The model input width is 2048 and height is 1024
[example-1] [INFO] [1654769881.173646870] [example]: Dnn node feed with local image: config/raw_unet.jpeg
[example-1] [INFO] [1654769881.750748126] [example]: task_num: 2
[example-1] [INFO] [1654769881.933418736] [example]: Output from image_name: config/raw_unet.jpeg, frame_id: feedback, stamp: 0.0
[example-1] [INFO] [1654769881.933542440] [UnetPostProcess]: outputs size: 1
[example-1] [INFO] [1654769881.995920396] [UnetPostProcess]: Draw result to file: render_unet_feedback_0_0.jpeg
```

The output log indicates that the algorithm performs inference using the input image `config/raw_unet.jpeg`, and saves the rendered result as `render_unet_feedback_0_0.jpeg`. The rendered image appears as follows:

![render_feedback](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/mobilenet_unet/mobilenet_unet_render_feedback.jpeg)