---
sidebar_position: 21
---
# EdgeSAM Segment Anything


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The mono edge sam package is a usage example based on [EdgeSAM](https://github.com/chongzhou96/EdgeSAM) quantification deployment. The image data comes from local image feedback and subscribed image msg. SAM relies on the input of the detection box for segmentation, and segments the targets in the detection box without specifying the category information of the targets, only providing the box. The algorithm information will be published through topics and the results will be rendered and visualized on the web page. It also supports saving the rendered images in the result directory during program execution.

In this example, we provide two deployment methods:
-Regular box for segmentation: A detection box in the center of the image is fixed for segmentation.
-Subscription box for segmentation: Subscribe to the detection box information output by the upstream detection network and segment the information in the box.

Code repository:  (https://github.com/D-Robotics/mono_edgesam.git)

Application scenario: Combining detection boxes for obstacle segmentation, water stain area segmentation, etc.

## Supported Platforms

| Platform             | System | Function                                            |
| -------------------- | ---------------- | ------------------------------------------------------------|
| RDK X5 | Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local image offline, inference rendering results displayed/saved locally on the Web| 

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_edgesam/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch mono_edgesam sam.launch.py 
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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_edgesam/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch mono_edgesam sam.launch.py 
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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_edgesam/config/ .

# Configure the local playback image.
export CAM_TYPE=fb

# Start the launch file
ros2 launch mono_edgesam sam.launch.py 
```

</TabItem>

</Tabs>

## Result Analysis

**Using a image publish tool to publish images**

After the package is initialized, the following information will be displayed in the terminal:

```
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-07-28-19-51-28-488985-ubuntu-107175
[INFO] [launch]: Default logging verbosity is set to INFO
mono_edgesam basic_path is  /root/install/lib/mono_edgesam/config
camera_type is  fb
using feedback
Hobot shm pkg enables zero-copy with fastrtps profiles file: /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
Hobot shm pkg sets RMW_FASTRTPS_USE_QOS_FROM_XML: 1
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [107191]
[INFO] [hobot_codec_republish-2]: process started with pid [107193]
[INFO] [mono_edgesam-3]: process started with pid [107195]
[INFO] [websocket-4]: process started with pid [107197]
[hobot_codec_republish-2] [WARN] [1753703489.112526140] [hobot_codec_encoder]: Parameters:
[hobot_codec_republish-2] sub_topic: /hbmem_img
[hobot_codec_republish-2] pub_topic: /image
[hobot_codec_republish-2] channel: 1
[hobot_codec_republish-2] in_mode: shared_mem
[hobot_codec_republish-2] out_mode: ros
[hobot_codec_republish-2] in_format: nv12
[hobot_codec_republish-2] out_format: jpeg
[hobot_codec_republish-2] enc_qp: 10
[hobot_codec_republish-2] jpg_quality: 60
[hobot_codec_republish-2] input_framerate: 30
[hobot_codec_republish-2] output_framerate: -1
[hobot_codec_republish-2] dump_output: 0
[hobot_codec_republish-2] [WARN] [1753703489.118608825] [HobotCodecImpl]: platform x5
[hobot_codec_republish-2] [WARN] [1753703489.118873534] [hobot_codec_encoder]: Enabling zero-copy
[websocket-4] [WARN] [1753703489.499688698] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: perception/segmentation/edgesam
[websocket-4]  output_fps: 0
[hobot_image_pub-1] [WARN] [1753703489.864104228] [image_pub_node]: parameter:
[hobot_image_pub-1] image_source: /root/install/lib/mono_edgesam/config/4.jpg
[hobot_image_pub-1] source_image_w: 960
[hobot_image_pub-1] source_image_h: 544
[hobot_image_pub-1] output_image_w: 1920
[hobot_image_pub-1] output_image_h: 1080
[hobot_image_pub-1] fps: 10
[hobot_image_pub-1] is_shared_mem: 1
[hobot_image_pub-1] is_loop: 1
[hobot_image_pub-1] is_compressed_img_pub: 0
[hobot_image_pub-1] image_format: jpg
[hobot_image_pub-1] pub_encoding: nv12pub_name_mode: 0
[hobot_image_pub-1] msg_pub_topic_name: /hbmem_img
[hobot_image_pub-1] [WARN] [1753703489.864484396] [hobot_image_pub]: Enabling zero-copy
[mono_edgesam-3] [WARN] [1753703489.948943404] [mono_edgesam]: Parameter:
[mono_edgesam-3]  cache_len_limit: 8
[mono_edgesam-3]  dump_render_img: 0
[mono_edgesam-3]  feed_type(0:local, 1:sub): 1
[mono_edgesam-3]  image: ./config/4.jpg
[mono_edgesam-3]  encoder_model_file_name: /root/install/lib/mono_edgesam/config/edgesam_encoder_1024.bin
[mono_edgesam-3]  decoder_model_file_name: /root/install/lib/mono_edgesam/config/edgesam_decoder_1024.bin
[mono_edgesam-3]  is_regular_box: 1
[mono_edgesam-3]  is_padding_seg: 0
[mono_edgesam-3]  is_shared_mem_sub: 1
[mono_edgesam-3]  is_sync_mode: 0
[mono_edgesam-3]  ai_msg_pub_topic_name: /perception/segmentation/edgesam
[mono_edgesam-3]  ai_msg_sub_topic_name: /hobot_dnn_detection
[mono_edgesam-3]  ros_img_sub_topic_name: /image
[mono_edgesam-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono_edgesam-3] [HBRT] set log level as 0. version = 3.15.55.0
[mono_edgesam-3] [DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[hobot_codec_republish-2] [WARN] [1753703490.160037466] [hobot_codec_encoder]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1753703490.160368467] [HobotVenc]: init_pic_w_: 1920, init_pic_h_: 1080, alined_pic_w_: 1920, alined_pic_h_: 1088, aline_w_: 16, aline_h_: 16
[mono_edgesam-3] [A][DNN][packed_model.cpp:247][Model](2025-07-28,19:51:31.34.979) [HorizonRT] The model builder version = 1.24.3
[hobot_codec_republish-2] [WARN] [1753703491.369838914] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 11.9485, pub jpeg, fps: 11.9485, comm delay [1.7692]ms, codec delay [15.5385]ms
[mono_edgesam-3] [A][DNN][packed_model.cpp:247][Model](2025-07-28,19:51:33.829.850) [HorizonRT] The model builder version = 1.24.3
[mono_edgesam-3] [WARN] [1753703494.035526230] [mono_edgesam]: Create hbmem_subscription with topic_name: /hbmem_img
[mono_edgesam-3] [WARN] [1753703494.057170296] [edgesam_node]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[mono_edgesam-3] [WARN] [1753703494.213043065] [mono_edgesam]: Smart fps: 9.00, pre process time ms: 14, infer time ms: 92, post process time ms: 48
[mono_edgesam-3] [WARN] [1753703494.342125459] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 12, infer time ms: 81, post process time ms: 41
[hobot_codec_republish-2] [WARN] [1753703494.362260146] [hobot_codec_encoder]: Pub img fps [8.45]
[mono_edgesam-3] [WARN] [1753703494.471613064] [mono_edgesam]: Smart fps: 10.00, pre process time ms: 11, infer time ms: 88, post process time ms: 39
[mono_edgesam-3] [WARN] [1753703494.592498850] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 12, infer time ms: 81, post process time ms: 38
[mono_edgesam-3] [WARN] [1753703494.724020335] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 11, infer time ms: 84, post process time ms: 46
[mono_edgesam-3] [WARN] [1753703494.849518302] [mono_edgesam]: Smart fps: 10.00, pre process time ms: 12, infer time ms: 85, post process time ms: 38
[mono_edgesam-3] [WARN] [1753703494.973585807] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 13, infer time ms: 82, post process time ms: 40
```

**Using single image offline**

The result will be rendered on web. On the PC-side browser, you can view the image and algorithm rendering effect by entering http://IP:8000 (IP is the IP address of the RDK), and open the settings in the upper right corner of the interface.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_sam.png)

## Advance

If you need to adjust the size of the detection box, you can refer to the following method for verification. What's more, the detection results of the other dnn detection node can be used as the input for Sam.

Start the launch file with cancel regular box mode, sam_is_regular_box:=0.
```shell
ros2 launch mono_edgesam sam.launch.py sam_is_regular_box:=0
```

Publish ai msg in another terminal.
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 160, "y_offset": 120, "width": 320, "height": 240}, "type": "anything"}]}] }'
```

Explanation: The topic name published here is "/hobot_dnn_detection". The starting point of the detection box is (96, 96), with a width of 192 and a height of 96. The starting and ending points of the detection box should not exceed the size of the input image. Please check while using it.