---
sidebar_position: 4
---
# MobileSAM Segmentation Everything

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The `mono_mobilesam` package provides a usage example based on quantized deployment of Mobile SAM. Image data comes from either local image replay or subscribed image messages. SAM requires bounding box input to perform segmentation—it segments objects within the provided bounding boxes without needing any class information, only the box coordinates. The algorithm results are published via ROS topics and visualized on a web page.

In this example, we provide two deployment modes:
- **Fixed-box segmentation**: A fixed detection box (centered in the image) is used for segmentation.
- **Subscribed-box segmentation**: Subscribes to detection boxes output by an upstream detection network and segments objects within those boxes.

Code repository: [https://github.com/D-Robotics/mono_mobilesam.git](https://github.com/D-Robotics/mono_mobilesam.git)

Application scenarios: Obstacle segmentation, water stain area segmentation, etc., when combined with detection boxes.

## Supported Platforms

| Platform              | Runtime Environment | Example Features                                                     |
| --------------------- | ------------------- | -------------------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | · Launch MIPI/USB camera or local image replay; inference results rendered on Web or saved locally |

## Algorithm Details

| Model       | Platform | Input Size     | Inference FPS |
| ----------- | -------- | -------------- | ------------- |
| mobilesam   | X5       | 1×3×384×384    | 6.6           |

## Prerequisites

### RDK Platform

1. RDK has been flashed with the Ubuntu 22.04 system image.
2. TogetherROS.Bot has been successfully installed on the RDK.

## Usage Guide

The package publishes algorithm messages containing both semantic segmentation and object detection information. Users can subscribe to these messages for application development.

### RDK Platform

**Publish images from MIPI camera**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure ROS2 environment
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

**Publish images from USB camera**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure ROS2 environment
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

**Use single replay image**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure ROS2 environment
source /opt/tros/humble/setup.bash

# Copy required configuration files for the example from the tros installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configure replay image
export CAM_TYPE=fb

# Launch the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

## Result Analysis

**Publish images from MIPI camera**

After initializing the package, the following logs appear in the terminal:

```
[INFO] [launch]: All log files can be found below .ros/log/1970-01-02-22-39-09-001251-buildroot-22955
[INFO] [hobot_codec_republish-2]: process started with pid [22973]
[INFO] [mono_mobilesam-3]: process started with pid [22975]
[INFO] [websocket-4]: process started with pid [22977]
[hobot_codec_republish-2] [WARN] [0000167949.975123376] [HobotCodec]: This is HobotCodecNode: hobot_codec_22973.
[hobot_codec_republish-2] [WARN] [0000167950.040208542] [HobotCodecNode]: Parameters:
[hobot_codec_republish-2] sub_topic: /image
[hobot_codec_republish-2] pub_topic: /hbmem_img
[hobot_codec_republish-2] channel: 1
[hobot_codec_republish-2] in_mode: ros
[hobot_codec_republish-2] out_mode: shared_mem
[hobot_codec_republish-2] in_format: jpeg
[hobot_codec_republish-2] out_format: nv12
[hobot_codec_republish-2] enc_qp: 10
[hobot_codec_republish-2] jpg_quality: 60
[hobot_codec_republish-2] input_framerate: 30
[hobot_codec_republish-2] output_framerate: -1
[hobot_codec_republish-2] dump_output: 0
[hobot_codec_republish-2] [WARN] [0000167950.050887417] [HobotCodecImpl]: platform x5
[websocket-4] [WARN] [0000167950.068235417] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: hobot_sam
[websocket-4]  output_fps: 0
[mono_mobilesam-3] [WARN] [0000167950.510756918] [mono_mobilesam]: Parameter:
[mono_mobilesam-3]  cache_len_limit: 8
[mono_mobilesam-3]  dump_render_img: 0
[mono_mobilesam-3]  feed_type(0:local, 1:sub): 1
[mono_mobilesam-3]  image: config/00131.jpg
[mono_mobilesam-3]  is_regular_box: 1
[mono_mobilesam-3]  is_shared_mem_sub: 1
[mono_mobilesam-3]  is_sync_mode: 0
[mono_mobilesam-3]  ai_msg_pub_topic_name: /hobot_sam
[mono_mobilesam-3]  ai_msg_sub_topic_name: /hobot_dnn_detection
[mono_mobilesam-3]  ros_img_sub_topic_name: /image
[mono_mobilesam-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono_mobilesam-3] [HBRT] set log level as 0. version = 3.15.52.0
[mono_mobilesam-3] [DNN] Runtime version = 1.23.9_(3.15.52 HBRT)
[mono_mobilesam-3] [A][DNN][packed_model.cpp:247][Model](1970-01-02,22:39:10.889.592) [HorizonRT] The model builder version = 1.23.5
[mono_mobilesam-3] [W][DNN]bpu_model_info.cpp:491][Version](1970-01-02,22:39:11.25.90) Model: mobilesam_encoder_384_all_BPU. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[mono_mobilesam-3] [A][DNN][packed_model.cpp:247][Model](1970-01-02,22:39:11.239.603) [HorizonRT] The model builder version = 1.23.5
[mono_mobilesam-3] [WARN] [0000167951.353811293] [mono_mobilesam]: Create hbmem_subscription with topic_name: /hbmem_img
[mono_mobilesam-3] [W][DNN]bpu_model_info.cpp:491][Version](1970-01-02,22:39:11.318.569) Model: mobilesam_decoder_384. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[mono_mobilesam-3] [WARN] [0000167951.606431085] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 43, infer time ms: 152, post process time ms: 24
[mono_mobilesam-3] [WARN] [0000167951.779821293] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 36, infer time ms: 149, post process time ms: 21
[mono_mobilesam-3] [WARN] [0000167951.952713293] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 36, infer time ms: 150, post process time ms: 22
[mono_mobilesam-3] [WARN] [0000167952.123928377] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 37, infer time ms: 149, post process time ms: 21
[mono_mobilesam-3] [WARN] [0000167952.295540585] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 35, infer time ms: 150, post process time ms: 21
```

In this example, inference results are rendered on a web page. Open a browser on your PC and navigate to `http://IP:8000` (replace IP with your RDK's IP address) to view the image and algorithm visualization. Click the settings icon in the top-right corner of the interface and enable the "Full-image Segmentation" option to display the rendering effect.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_sam.png)

## Advanced Usage

To adjust the detection box size, refer to the method below. More importantly, you can use detection results from an upstream detection node as input for SAM.

Run SAM with fixed-box mode disabled (`sam_is_regular_box:=0`):
```shell
ros2 launch mono_mobilesam sam.launch.py sam_is_regular_box:=0
```

In another terminal, publish an AI message topic:
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
```

Explanation: The published topic name is `/hobot_dnn_detection`. The detection box starts at coordinate (96, 96) with width 192 and height 96. Note that the box coordinates must not exceed the input image dimensions—please pay attention to this in actual usage.