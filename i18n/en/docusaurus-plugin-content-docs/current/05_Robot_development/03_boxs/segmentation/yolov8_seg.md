---
sidebar_position: 2
---
# YOLOv8-Seg

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The YOLOv8-Seg algorithm example uses images as input and performs algorithm inference using BPU. It publishes segmentation result messages.

The YOLOv8-Seg is trained on the [COCO128-seg](http://cocodataset.org/) dataset and the Onnx model. It supports instance segmentation for 80 categories including humans, animals, fruits, and vehicles.

Code repository: (https://github.com/D-Robotics/hobot_dnn)

Applications: YOLOv8-Seg is capable of recognizing objects and performing precise segmentation. It can be applied in the fields of autonomous driving, geological detection, and medical image analysis.


## Supported Platforms

| Platform | System | Function                     |
| -------- | ------------ | ---------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | - Start MIPI/USB cameras or local image and save the rendered results offline. |

## Preparation

### RDK

1. The RDK platform has been flashed with the provided 22.04 system image.

2. TogetheROS.Bot has been successfully installed on the RDK platform.

3. A MIPI or USB camera has been installed on the RDK platform. If there is no camera available, the algorithm's effects can be experienced by using local JPEG/PNG images offline.

#### Use the Camera to Publish Images 

##### Use a MIPI Camera to Publish Images 

The YOLOv8-Seg example subscribes to images published by the sensor package. IF set "dnn_example_dump_render_img:=1", it will save the rendered images automatically in the running directory. The saved images are named in the format of `render_frameid_timestampInSeconds_timestampInNanoseconds.jpg`.

<Tabs groupId="tros-distro">

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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=0 dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

##### Use a USB Camera to Publish Images 

<Tabs groupId="tros-distro">

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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=0 dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

#### Use local images offline

The YOLOv8-Seg example uses local JPEG/PNG format images for feedback. After inference, the rendered images of the algorithm results are stored in the local running path.

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>


```shell
# Start the launch file
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image:=config/test.jpeg
```

## Analysis of Results

### Use a Camera to Publishing Images 

The output shows the following information:

```shell
[example-3] [WARN] [0000001244.489045384] [example]: Sub img fps: -1.00, Smart fps: 6.00, infer time ms: 12, post process time ms: 31
[example-3] [WARN] [0000001245.524813052] [example]: Sub img fps: 5.84, Smart fps: 4.99, infer time ms: 8, post process time ms: 64
[example-3] [WARN] [0000001246.526635344] [example]: Sub img fps: 4.96, Smart fps: 5.00, infer time ms: 8, post process time ms: 66
[example-3] [WARN] [0000001247.528846136] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001248.528474095] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001249.528576345] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001250.493265846] [example]: Sub img fps: 5.02, Smart fps: 5.00, infer time ms: 8, post process time ms: 32
[example-3] [WARN] [0000001251.528909346] [example]: Sub img fps: 4.98, Smart fps: 5.00, infer time ms: 8, post process time ms: 67
```

The log output shows that the topic used for publishing the algorithm inference results is `hobot_dnn_detection`, and the topic used for subscribing to the images is `/hbmem_img`. The frame rate at which the images are published will adapt according to the algorithm inference output frame rate. Additionally, rendering the semantic segmentation results on the RDK and saving the images in the running path will cause a decrease in frame rate.


Original image:
![raw](/../static/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/test.jpg)

Rendered image:
![render_web](/../static/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/web.jpeg)

### Use Local Images Offline

The output shows the following information:

```shell
[INFO] [0000001744.811779665] [example]: Dnn node feed with local image: /userdata/config/test.jpg
[INFO] [0000001746.237111249] [example]: Output from frame_id: feedback, stamp: 0.0
[INFO] [0000001746.266157040] [PostProcessBase]: out box size: 6
[INFO] [0000001746.266340040] [PostProcessBase]: det rect: 90.4946 58.2675 192.103 351.403, det type: person, score:0.927177
[INFO] [0000001746.267129832] [PostProcessBase]: det rect: 455.518 77.1254 536.289 354.541, det type: person, score:0.909735
[INFO] [0000001746.267248457] [PostProcessBase]: det rect: 381.604 103.953 464.446 327.9, det type: person, score:0.898899
[INFO] [0000001746.267331624] [PostProcessBase]: det rect: 204.864 71.6262 303.593 351.835, det type: person, score:0.887814
[INFO] [0000001746.267404540] [PostProcessBase]: det rect: 317.885 108.287 389.773 338.197, det type: person, score:0.866887
[INFO] [0000001746.267486457] [PostProcessBase]: det rect: 181.487 111.093 202.097 132.665, det type: car, score:0.443035
[INFO] [0000001746.267548999] [ClassificationPostProcess]: out cls size: 0
[INFO] [0000001746.267662832] [SegmentationPostProcess]: features size: 14240, width: 160, height: 89, num_classes: 80, step: 1
[INFO] [0000001746.270546040] [ImageUtils]: target size: 7
[INFO] [0000001746.270674082] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.270745915] [ImageUtils]: roi.type: person, x_offset: 90 y_offset: 58 width: 101 height: 293
[INFO] [0000001746.271122207] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271162499] [ImageUtils]: roi.type: person, x_offset: 455 y_offset: 77 width: 80 height: 277
[INFO] [0000001746.271325499] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271362082] [ImageUtils]: roi.type: person, x_offset: 381 y_offset: 103 width: 82 height: 223
[INFO] [0000001746.271491040] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271525249] [ImageUtils]: roi.type: person, x_offset: 204 y_offset: 71 width: 98 height: 280
[INFO] [0000001746.271782749] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271819457] [ImageUtils]: roi.type: person, x_offset: 317 y_offset: 108 width: 71 height: 229
[INFO] [0000001746.271947790] [ImageUtils]: target type: car, rois.size: 1
[INFO] [0000001746.271982374] [ImageUtils]: roi.type: car, x_offset: 181 y_offset: 111 width: 20 height: 21
[INFO] [0000001746.272044124] [ImageUtils]: target type: parking_space, rois.size: 0
[WARN] [0000001746.276824624] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg
```

The log shows that the algorithm performs inference using the input image `config/raw_unet.jpeg`, and the rendered image is stored with the file `render_unet_feedback_0_0.jpeg`. The rendered image looks like this:

![render_feedback](/../static/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/local.jpeg)
