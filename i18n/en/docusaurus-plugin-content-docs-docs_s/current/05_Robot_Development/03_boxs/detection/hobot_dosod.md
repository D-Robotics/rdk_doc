---
sidebar_position: 6
---
# DOSOD

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

DOSOD (Decoupled Open-Set Object Detector)[https://github.com/D-Robotics-AI-Lab/DOSOD] is an advanced open-vocabulary object detection algorithm capable of efficiently detecting various novel category targets in a zero-shot manner.

Code Repository: (https://github.com/D-Robotics/hobot_dosod)

Application Scenarios: The robust zero-shot detection capability of DOSOD endows it with enhanced generalization, making it applicable in domains such as autonomous driving, smart home, and other fields.

## Supported Platforms

| Platform                            | System | Function                                     |
| ----------------------------------- | -------------- | -------------------------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local video and display inference rendering results via web      |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local video and display inference rendering results via web      |

## Preparation

### RDK

1. RDK has flashed the Ubuntu 22.04 system image provided by D-Robotics.

2. RDK has successfully installed TogetheROS.Bot.

3. RDK has installed the MIPI or USB camera.

4. Confirm that the PC is able to access the RDK via the network.

## Usage

DOSOD (hobot_dosod) package subscribes to images published by the sensor package. It has the capability to alter the detection categories by parameterizing the model. The websocket package is used to render and display the images and corresponding algorithm results on a PC browser.

### RDK

**Use MIPI Camera to Publish Images**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch hobot_dosod dosod.launch.py
```

</TabItem>

</Tabs>

**Use USB Camera to Publish Images**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch hobot_dosod dosod.launch.py
```

</TabItem>

</Tabs>

**Use Local Image Offline**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# Configure the local playback image.
export CAM_TYPE=fb

# Start the launch file
ros2 launch hobot_dosod dosod.launch.py
```

</TabItem>

</Tabs>

## Result analysis

The following information is outputted in the terminal:

```shell
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-01-08-11-03-34-125542-ubuntu-9125
[INFO] [launch]: Default logging verbosity is set to INFO
camera_type is  fb
using feedback
Hobot shm pkg enables zero-copy with fastrtps profiles file: /userdata/install/lib/hobot_shm/config/shm_fastdds.xml
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
[INFO] [hobot_image_pub-1]: process started with pid [9128]
[INFO] [hobot_codec_republish-2]: process started with pid [9130]
[INFO] [hobot_dosod-3]: process started with pid [9132]
[INFO] [websocket-4]: process started with pid [9134]
[hobot_codec_republish-2] [WARN] [1736305415.448727260] [hobot_codec_encoder]: Parameters:
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
[hobot_codec_republish-2] [WARN] [1736305415.455977260] [HobotCodecImpl]: platform x5
[hobot_codec_republish-2] [WARN] [1736305415.456186677] [hobot_codec_encoder]: Enabling zero-copy
[hobot_dosod-3] [WARN] [1736305415.687929557] [hobot_dosod]: This is hobot dosod!
[websocket-4] [WARN] [1736305415.794630560] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: hobot_dosod
[websocket-4]  output_fps: 0
[hobot_dosod-3] [WARN] [1736305415.835729185] [hobot_dosod]: Parameter:
[hobot_dosod-3]  model_file_name: config/dosod_mlp3x_l_rep-int8.bin
[hobot_dosod-3]  vocabulary_file_name: config/offline_vocabulary.json
[hobot_dosod-3]  feed_type(0:local, 1:sub): 1
[hobot_dosod-3]  image: config/000000160864.jpg
[hobot_dosod-3]  dump_ai_result: 0
[hobot_dosod-3]  dump_raw_img: 0
[hobot_dosod-3]  dump_render_img: 0
[hobot_dosod-3]  dump_ai_path: .
[hobot_dosod-3]  dump_raw_path: .
[hobot_dosod-3]  dump_render_path: .
[hobot_dosod-3]  is_shared_mem_sub: 1
[hobot_dosod-3]  score_threshold: 0.2
[hobot_dosod-3]  iou_threshold: 0.5
[hobot_dosod-3]  nms_top_k: 50
[hobot_dosod-3]  is_homography: 0
[hobot_dosod-3]  trigger_mode: 0
[hobot_dosod-3]  class_mode: 0
[hobot_dosod-3]  task_num: 2
[hobot_dosod-3]  roi: 0
[hobot_dosod-3]  y_offset: 950
[hobot_dosod-3]  ai_msg_pub_topic_name: /hobot_dosod
[hobot_dosod-3]  ros_img_sub_topic_name: /image
[hobot_dosod-3] [WARN] [1736305415.836617477] [hobot_dosod]: model_file_name_: config/dosod_mlp3x_l_rep-int8.bin, task_num: 2
[hobot_dosod-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[hobot_dosod-3] [HBRT] set log level as 0. version = 3.15.54.0
[hobot_dosod-3] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[hobot_image_pub-1] [WARN] [1736305416.129590859] [image_pub_node]: parameter:
[hobot_image_pub-1] image_source: ./config/000000160864.jpg
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
[hobot_image_pub-1] [WARN] [1736305416.130613275] [hobot_image_pub]: Enabling zero-copy
[hobot_codec_republish-2] [WARN] [1736305416.348757530] [hobot_codec_encoder]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1736305416.349073988] [HobotVenc]: init_pic_w_: 1920, init_pic_h_: 1080, alined_pic_w_: 1920, alined_pic_h_: 1088, aline_w_: 16, aline_h_: 16
[hobot_dosod-3] [A][DNN][packed_model.cpp:247][Model](2025-01-08,11:03:36.664.601) [HorizonRT] The model builder version = 1.24.3
[hobot_dosod-3] [WARN] [1736305417.323044552] [hobot_dosod]: Get model name: 3x-l_epoch_100_rep-coco80-without-nms from load model.
[hobot_dosod-3] [WARN] [1736305417.323560635] [hobot_dosod]: Create ai msg publisher with topic_name: /hobot_dosod
[hobot_dosod-3] [WARN] [1736305417.350238969] [hobot_dosod]: Create img hbmem_subscription with topic_name: /hbmem_img
[hobot_dosod-3] [WARN] [1736305417.445453471] [dosod]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1736305417.453916388] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 11.4504, pub jpeg, fps: 11.4504, comm delay [0.0833]ms, codec delay [13.5833]ms
[hobot_dosod-3] [W][DNN]bpu_model_info.cpp:491][Version](2025-01-08,11:03:37.311.128) Model: 3x-l_epoch_100_rep-coco80-without-nms. Inconsistency between the hbrt library version 3.15.54.0 and the model build version 3.15.55.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[hobot_dosod-3] [WARN] [1736305418.846408168] [hobot_dosod]: Sub img fps: 12.95, Smart fps: 12.67, pre process time ms: 12, infer time ms: 78, post process time ms: 8
[hobot_dosod-3] [WARN] [1736305419.857350566] [hobot_dosod]: Sub img fps: 9.97, Smart fps: 10.88, pre process time ms: 11, infer time ms: 91, post process time ms: 8
[hobot_dosod-3] [WARN] [1736305420.858769504] [hobot_dosod]: Sub img fps: 10.04, Smart fps: 9.99, pre process time ms: 13, infer time ms: 100, post process time ms: 7
[hobot_dosod-3] [WARN] [1736305421.860964318] [hobot_dosod]: Sub img fps: 9.99, Smart fps: 9.99, pre process time ms: 14, infer time ms: 100, post process time ms: 7
```

You can view the image and algorithm rendering effects by entering http://IP:8000 in the browser on the PC (where IP is the IP address of the RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_dosod.jpeg)



## Advance
If you want to modify the target text, you can re-parameter the model. [DOSOD Deployment to RDK X5](https://horizonrobotics.feishu.cn/docx/MZgtdSDzNoHyOjxFSQbcRoVDnEj).
