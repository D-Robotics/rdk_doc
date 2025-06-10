---
sidebar_position: 14
---
# Segment Anything


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The mono mobile sam package is a usage example based on Mobile SAM quantification deployment. The image data comes from local image feedback and subscribed image msg. SAM relies on the input of the detection box for segmentation, and segments the targets in the detection box without specifying the category information of the targets, only providing the box. The algorithm information will be published through topics and the results will be rendered and visualized on the web page. It also supports saving the rendered images in the result directory during program execution.

In this example, we provide two deployment methods:
-Regular box for segmentation: A detection box in the center of the image is fixed for segmentation.
-Subscription box for segmentation: Subscribe to the detection box information output by the upstream detection network and segment the information in the box.

Code repository:  (https://github.com/D-Robotics/mono_mobilesam.git)

Application scenario: Combining detection boxes for obstacle segmentation, water stain area segmentation, etc.

## Supported Platforms

| Platform             | System | Function                                            |
| -------------------- | ---------------- | ------------------------------------------------------------|
| RDK X5| Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local image offline, inference rendering results displayed/saved locally on the Web| 

## Preparation

### RDK

1. The RDK has burned the  Ubuntu 22.04 system image provided by D-Robotics.

2. The RDK has successfully installed TogetheROS.Bot.

## Usage

The package publishes algorithm messages that include semantic segmentation and object detection information, and users can subscribe to these messages for application development.

### RDK

**Publishing images from MIPI camera**

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

**Publishing images from USB camera**

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

**Using a single image offline**

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# Configure the local playback image.
export CAM_TYPE=fb

# Start the launch file
ros2 launch mono_mobilesam sam.launch.py 
```

</TabItem>

</Tabs>

## Result Analysis

**Using a MIPI camera to publish images**

After the package is initialized, the following information will be displayed in the terminal:

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

**Using single image offline**

The result will be rendered on web. On the PC-side browser, you can view the image and algorithm rendering effect by entering http://IP:8000 (IP is the IP address of the RDK), and open the settings in the upper right corner of the interface.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_sam.png)

## Advance

If you need to adjust the size of the detection box, you can refer to the following method for verification. What's more, the detection results of the other dnn detection node can be used as the input for Sam.

Start the launch file with cancel regular box mode, sam_is_regular_box:=0.
```shell
ros2 launch mono_mobilesam sam.launch.py sam_is_regular_box:=0
```

Publish ai msg in another terminal.
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
```

Explanation: The topic name published here is "/hobot_dnn_detection". The starting point of the detection box is (96, 96), with a width of 192 and a height of 96. The starting and ending points of the detection box should not exceed the size of the input image. Please check while using it.