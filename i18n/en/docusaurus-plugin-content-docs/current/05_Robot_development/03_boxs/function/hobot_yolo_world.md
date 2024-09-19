---
sidebar_position: 16
---
# YOLO-World

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

YOLO-World is an advanced open-vocabulary object detection algorithm capable of efficiently detecting various novel category targets in a zero-shot manner based on the variation of input text.

Code Repository: (https://github.com/D-Robotics/hobot_yolo_world)

Application Scenarios: The robust zero-shot detection capability of YOLO-World endows it with enhanced generalization, making it applicable in domains such as autonomous driving, smart home, and other fields.

## Supported Platforms

| Platform                            | System | Function                                     |
| ----------------------------------- | -------------- | -------------------------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local video and display inference rendering results via web      |

## Preparation

### RDK

1. RDK has flashed the  Ubuntu 22.04 system image provided by D-Robotics.

2. RDK has successfully installed TogetheROS.Bot.

3. RDK has installed the MIPI or USB camera.

4. Confirm that the PC is able to access the RDK via the network.

## Usage

YOLO-World (hobot_yolo_world) package subscribes to images published by the sensor package. It has the capability to alter the detection categories based on variations in the input text. The text features are sourced from a local feature library, which is queried through the input text to match corresponding features. These features are then inputted into the model for inference. The websocket package is used to render and display the images and corresponding algorithm results on a PC browser.

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
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"
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
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"
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
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# Configure the local playback image.
export CAM_TYPE=fb

# Start the launch file
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"
```

</TabItem>

</Tabs>

## Result analysis

The following information is outputted in the terminal:

```shell
[hobot_yolo_world-3] [WARN] [0000003710.693524477] [hobot_yolo_world]: This is hobot yolo world!
[hobot_yolo_world-3] [WARN] [0000003710.792557185] [hobot_yolo_world]: Parameter:
[hobot_yolo_world-3]  feed_type(0:local, 1:sub): 1
[hobot_yolo_world-3]  image: config/yolo_world_test.jpg
[hobot_yolo_world-3]  dump_render_img: 0
[hobot_yolo_world-3]  is_shared_mem_sub: 1
[hobot_yolo_world-3]  score_threshold: 0.05
[hobot_yolo_world-3]  iou_threshold: 0.45
[hobot_yolo_world-3]  nms_top_k: 50
[hobot_yolo_world-3]  texts: red bottle,trash bin
[hobot_yolo_world-3]  ai_msg_pub_topic_name: /hobot_yolo_world
[hobot_yolo_world-3]  ros_img_sub_topic_name: /image
[hobot_yolo_world-3]  ros_string_sub_topic_name: /target_words
[hobot_yolo_world-3] [WARN] [0000003710.848418019] [hobot_yolo_world]: Parameter:
[hobot_yolo_world-3]  model_file_name: config/yolo_world.bin
[hobot_yolo_world-3]  model_name:
[hobot_yolo_world-3] [WARN] [0000003710.848540935] [hobot_yolo_world]: model_file_name_: config/yolo_world.bin, task_num: 4
[hobot_yolo_world-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[hobot_yolo_world-3] [HBRT] set log level as 0. version = 3.15.49.0
[hobot_yolo_world-3] [DNN] Runtime version = 1.23.8_(3.15.49 HBRT)
[hobot_yolo_world-3] [A][DNN][packed_model.cpp:247][Model](1970-01-01,01:01:51.482.877) [HorizonRT] The model builder version = 1.23.5
[hobot_yolo_world-3] [WARN] [0000003711.739402019] [hobot_yolo_world]: Get model name: yolo_world_pad_pretrain_norm_new from load model.
[hobot_yolo_world-3] [WARN] [0000003711.739551686] [hobot_yolo_world]: Create ai msg publisher with topic_name: /hobot_yolo_world
[hobot_yolo_world-3] [WARN] [0000003711.794810269] [hobot_yolo_world]: Create string subscription with topic_name: /target_words
[hobot_yolo_world-3] [WARN] [0000003711.808682144] [hobot_yolo_world]: Create img hbmem_subscription with topic_name: /hbmem_img
[hobot_yolo_world-3] [WARN] [0000003712.541236020] [yolo_world]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_yolo_world-3] [W][DNN]bpu_model_info.cpp:491][Version](1970-01-01,01:01:51.727.259) Model: yolo_world_pad_pretrain_norm_new. Inconsistency between the hbrt library version 3.15.49.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[hobot_yolo_world-3] [WARN] [0000003714.698775687] [hobot_yolo_world]: Sub img fps: 1.00, Smart fps: 1.51, pre process time ms: 30, infer time ms: 121, post process time ms: 5
[hobot_yolo_world-3] [WARN] [0000003716.714586355] [hobot_yolo_world]: Sub img fps: 1.00, Smart fps: 0.99, pre process time ms: 40, infer time ms: 127, post process time ms: 6
[hobot_yolo_world-3] [WARN] [0000003718.707619939] [hobot_yolo_world]: Sub img fps: 1.00, Smart fps: 1.00, pre process time ms: 39, infer time ms: 121, post process time ms: 6
```

You can view the image and algorithm rendering effects by entering http://IP:8000 in the browser on the PC (where IP is the IP address of the RDK):

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/render_yolo_world.jpeg)



## Advance
If you want to modify the text features, you can utilize the tools to generate them locally. [Usage](https://github.com/D-Robotics/hobot_yolo_world/tree/develop/tool).

```bash
# Copy the configuration file required for generating the text features from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/tool/ .

# Download the model
wget http://archive.d-robotics.cc/models/yoloworld_encode_text/huggingclip_text_encode.tar.gz
sudo tar -xf huggingclip_text_encode.tar.gz -C tool

cd tool/

# Install dependencies
pip3 install -r requirements.txt
```

```bash
# Modify the 'class.list'

# Generate text features embeddings
python3 main.py

# Copy new text features embeddings
mv offline_vocabulary_embeddings.json ../config/
```
