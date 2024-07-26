---
sidebar_position: 4
---
# EfficientNet_Det

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

EfficientNet_Det is an detection algorithm that takes images as input and utilizes BPU for algorithm inference. It publishes algorithm messages containing object categories and detection boxes.

EfficientNet_Det is an Onnx model obtained from  (https://github.com/HorizonRobotics-Platform/ModelZoo/tree/master/EfficientDet) and trained using the [COCO dataset](http://cocodataset.org/). It supports 80 types of object detection, including humans, animals, fruits, and vehicles.

Code repository:  (https://github.com/D-Robotics/hobot_dnn)

Applications: EfficientNet_Det can be used for tasks such as vehicle detection and is mainly applied in the fields of autonomous driving and smart home.

## Supported Platforms

| Platform               | System | Function                                        |
| ---------------------- | ---------------- | ------------------------------------------------------------ |
| RDK X3, RDK X3 Module  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | · Start MIPI/USB camera and display inference results through web<br/>· Use local data to save rendering results offline |

## Preparations

### Horizon RDK

1. The Horizon RDK has been pre-installed with the  Ubuntu 20.04/22.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

3. A MIPI or USB camera has been installed on the Horizon RDK. If there is no camera available, you can experience the algorithm using locally JPEG/PNG images or MP4, H.264, and H.265 videos offline.

4. Make sure the PC can access the Horizon RDK through the network.

## Usage

### Horizon RDK

#### Use MIPI Camera to Publish Images

EfficientNet_Det subscribes to images published by the sensor package, performs inference, and publishes algorithm messages. WebSocket package is used to render and display the published images and corresponding algorithm results on a PC browser.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

</TabItem>

</Tabs>

#### Use USB Camera to Publish Images

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

</TabItem>

</Tabs>

#### Use Local Images Offline

The EfficientNet_Det detection algorithm example uses local JPEG/PNG images for rendering the algorithm results after inference, which are stored in the local path.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Start the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image:=config/target.jpg
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash

# Start the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/efficient_det_workconfig.json dnn_example_image:=config/target.jpg
```

</TabItem>

</Tabs>

## Result Analysis

### Use Camera to Publish Images 

The terminal output during the execution is as follows:

```shell
[example-3] [WARN] [1655093196.041759782] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655093196.041878985] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655093197.405840936] [img_sub]: Sub img fps 8.57
[example-3] [WARN] [1655093197.687361687] [example]: Smart fps 8.04
[example-3] [WARN] [1655093198.559784569] [img_sub]: Sub img fps 6.94
[example-3] [WARN] [1655093198.891958094] [example]: Smart fps 6.64
[example-3] [WARN] [1655093199.735312707] [img_sub]: Sub img fps 6.81
[example-3] [WARN] [1655093200.013067298] [example]: Smart fps 7.14
[example-3] [WARN] [1655093200.890569474] [img_sub]: Sub img fps 6.93
[example-3] [WARN] [1655093201.175239677] [example]: Smart fps 6.88
[example-3] [WARN] [1655093202.011887441] [img_sub]: Sub img fps 7.14
[example-3] [WARN] [1655093202.302124315] [example]: Smart fps 7.10
```

The log output shows that the topic for publishing the inference results of the algorithm is `hobot_dnn_detection`, and the topic for subscribing to the image is `/hbmem_img`.

To view the image and the rendering effect of the algorithm, enter http://IP:8000 in the browser on the PC (where IP is the IP address of the Horizon RDK):

![render_web](/../static/img/05_Robot_development/03_boxs/detection/image/box_basic/efficient_det_render_web.jpeg)

### Use local image offline

The terminal output is as follows:

```shell
[example-1] [INFO] [1654931461.278066695] [example]: Output from image_name: config/target.jpg, frame_id: feedback, stamp: 0.0
[example-1] [INFO] [1654931461.278186816] [PostProcessBase]: outputs size: 10
[example-1] [INFO] [1654931461.278231981] [PostProcessBase]: out box size: 2
[example-1] [INFO] [1654931461.278303520] [PostProcessBase]: det rect: 380.107 170.888 511.048 372.511, det type: potted plant, score:1.16971
[example-1] [INFO] [1654931461.278396934] [PostProcessBase]: det rect: 79.3884 263.497 373.645 372.554, det type: couch, score:1.0287
```

The log shows that the algorithm has inferred 2 targets from the input image, and outputs the coordinates of the detection boxes (the order of the output coordinates is the top-left x and y coordinates, and the bottom-right x and y coordinates) and the categories. The rendered image is saved as render_feedback_0_0.jpeg, and here is the rendering effect:

![render_feedback](/../static/img/05_Robot_development/03_boxs/detection/image/box_basic/efficient_det_render_feedback.jpeg)