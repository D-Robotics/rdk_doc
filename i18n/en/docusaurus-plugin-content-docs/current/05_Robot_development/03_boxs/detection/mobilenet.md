---
sidebar_position: 3
---
# MobileNet_SSD

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The MobileNet_SSD detection algorithm uses images as input, performs inference using BPU, and publishes messages that include target categories and detection boxes.

MobileNet_SSD is a caffe model obtained from  (https://github.com/chuanqi305/MobileNet-SSD), trained using the VOC dataset (http://host.robots.ox.ac.uk/pascal/VOC/voc2012/). It supports 20 types of target detection, including humans, animals, fruits, and vehicles.

Code repository:  (https://github.com/D-Robotics/hobot_dnn)

Application scenarios: MobileNet_SSD is an object detection algorithm based on MobileNet, which has the advantages of fast speed and easy deployment. It can achieve functions such as object detection and garbage recognition, and is mainly used in the fields of autonomous driving and smart home.

## Supported Platforms

| Platform                 | System | Function                                             |
| ------------------------ | ---------------- | ------------------------------------------------------------ |
| RDK X3, RDK X3 Module    | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | · Start the MIPI/USB camera and display the inference through the web<br/>· Use local data to save the results offline |

## Preparation

### Horizon RDK

1. Horizon RDK has burned the  Ubuntu 20.04/22.04 system image provided by Horizon.

2. Horizon RDK has successfully installed TogetheROS.Bot.

3. Horizon RDK has installed a MIPI or USB camera. If there is no camera available, the algorithm can be experienced by local JPEG/PNG images or MP4, H.264, and H.265 videos offline.

4. Confirm that the PC can access the Horizon RDK through the network.

## Usage

### Horizon RDK

#### Use MIPI Camera to Publish Images

The MobileNet_SSD detection algorithm subscribes to the images published by the sensor package, performs inference, and publishes algorithm messages. The algorithm messages and corresponding images are displayed on the PC browser using the websocket package.

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
# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenet_ssd_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Use USB Camera to Publish Images

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
# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenet_ssd_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### Use Local Images offline

The MobileNet_SSD detection algorithm example uses local JPEG/PNG images offline. After inference, the results are stored in the local path.

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
# Start the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenet_ssd_workconfig.json dnn_example_image:=config/target.jpg
```

## Result Analysis

### Use Camera to Publish Images 

The following information is displayed in the terminal output:

```shell
[example-3] [WARN] [1655095279.473675326] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655095279.473789113] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655095280.697388819] [img_sub]: Sub img fps 31.16
[example-3] [WARN] [1655095280.710505278] [example]: Smart fps 31.50
[example-3] [WARN] [1655095281.697831409] [img_sub]: Sub img fps 30.00
[example-3] [WARN] [1655095281.743811574] [example]: Smart fps 30.01
[example-3] [WARN] [1655095282.730768103] [img_sub]: Sub img fps 30.04
[example-3] [WARN] [1655095282.744084511] [example]: Smart fps 30.00
```

The log shows that the topic for publishing inference results is `hobot_dnn_detection`, and the topic for subscribing to images is `/hbmem_img`. The frame rate of the subscribed images and the algorithm inference output is approximately 30fps.

To view the image and algorithm, input http://IP:8000 in the browser on the PC (where IP is the IP address of the Horizon RDK):

![render_web](/../static/img/05_Robot_development/03_boxs/detection/image/box_basic/mobilenet_ssd_render_web.jpeg)

### Use Local Images offline

The following information is displayed in the terminal output:

```shell
[example-1] [INFO] [1654930510.201326806] [example]: Output from image_name: config/target.jpg, frame_id: feedback, stamp: 0.0
[example-1] [INFO] [1654930510.201485092] [PostProcessBase]: outputs size: 12
[example-1] [INFO] [1654930510.201581047] [PostProcessBase]: out box size: 2
[example-1] [INFO] [1654930510.201672794] [PostProcessBase]: det rect: 227.27 101.873 299.219 223.667, det type: pottedplant, score:0.995207
[example-1] [INFO] [1654930510.201778415] [PostProcessBase]: det rect: 62.3792 155.731 221.676 223.179, det type: sofa, score:0.982129
```

The log shows that the algorithm infers 2 targets from the input image and outputs the coordinates of the bounding boxes (the order of the output coordinates is the top-left x and y coordinates, and the bottom-right x and y coordinates) and the class. The image is named render_feedback_0_0.jpeg, and the rendering effect is:

![render_feedback](/../static/img/05_Robot_development/03_boxs/detection/image/box_basic/mobilenet_ssd_render_feedback.jpeg)