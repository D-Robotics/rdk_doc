---
sidebar_position: 9
---
# LiDAR Object Detection Algorithm

## Feature Description

The LiDAR object detection algorithm is based on the `CenterPoint` model, which is trained on the [nuscenes](https://www.nuscenes.org/nuscenes) dataset using [OpenExplorer](https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/centerpoint.html).

The algorithm takes in 32-line LiDAR point cloud data as input and outputs information including the 3D bounding boxes of detected objects, confidence scores, and object categories. The supported object detection categories include car, truck, bus, barrier, motorcycle, and pedestrian, making up a total of six categories.

In this example, local LiDAR point cloud files are used as input. The algorithm runs on a BPU (Brain Processing Unit) for inference and publishes the rendered images containing point cloud data, detection boxes, and orientation to a PC browser via WebSocket. The algorithm results are rendered and displayed in the browser.

Code repository: [GitHub - hobot_centerpoint](https://github.com/D-Robotics/hobot_centerpoint)

## Supported Platforms

| Platform      | Operating Mode               | Example Feature                                      |
| ------------- | ---------------------------- | ---------------------------------------------------- |
| RDK Ultra     | Ubuntu 20.04 (Foxy)           | Use local point cloud data and render inference results via web |

## Prerequisites

### RDK Platform

1. The RDK platform has Ubuntu 20.04 system image pre-installed.

2. TogetheROS.Bot is successfully installed on the RDK platform.

3. Ensure that the PC can access the RDK platform via the network.

## Usage Introduction

### RDK Platform

### Using Local Point Cloud File for Inference

The LiDAR object detection algorithm example uses a local LiDAR point cloud file for inference. After the inference, the results are rendered and published as an image message containing the algorithm's output (detection boxes, confidence scores, and orientations). These results are displayed on a PC browser using a WebSocket package.

**Preparing the LiDAR Point Cloud File:**


```shell
# Download LiDAR Point Cloud File for Local Inference
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_centerpoint_data.tar.gz

# Extract the Data
mkdir config
tar -zxvf hobot_centerpoint_data.tar.gz -C config
# After extraction, the data will be located at config/hobot_centerpoint_data
```

Start Algorithm Example:


```shell
# Configure tros.b environment
source /opt/tros/setup.bash

# Start the websocket service
ros2 launch websocket websocket_service.launch.py

# Launch the CenterPoint algorithm
ros2 launch hobot_centerpoint hobot_centerpoint_websocket.launch.py lidar_pre_path:=config/hobot_centerpoint_data
```

## Result Analysis

After starting the algorithm example, the following information will be output in the terminal:


```text
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_centerpoint-1]: process started with pid [22470]
[INFO] [websocket-2]: process started with pid [22472]
[hobot_centerpoint-1] [WARN] [0948485758.916907430] [centerpoint_node]:
[hobot_centerpoint-1]  preprocess_config: config/centerpoint_preprocess_5dim.json
[hobot_centerpoint-1]  model_file: config/model/model.hbm
[hobot_centerpoint-1]  lidar_list_file: ./config/nuscenes_lidar_val.lst
[hobot_centerpoint-1]  is_show: 1
[hobot_centerpoint-1]  is_loop: 1
[hobot_centerpoint-1]  pub_topic_name: /hobot_centerpoint
[hobot_centerpoint-1]  lidar_pre_path: ./config/hobot_centerpoint_data
[hobot_centerpoint-1] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_centerpoint-1] [HBRT] set log level as 0. version = 3.14.25.0
[hobot_centerpoint-1] [DNN] Runtime version = 1.12.3_(3.14.25 HBRT)
[hobot_centerpoint-1] [WARN] [0948485759.205674972] [dnn]: Run default SetOutputParser.
[hobot_centerpoint-1] [WARN] [0948485759.205820889] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
[hobot_centerpoint-1] [WARN] [0948485759.208895472] [hobot_centerpoint]: A total of 81 files were fetched!
[hobot_centerpoint-1] [WARN] [0948485759.400904472] [CenterPoint_Node]: input fps: -1.00, out fps: -1.00, infer time ms: 61, post process time ms: 57
[hobot_centerpoint-1] [WARN] [0948485759.839328014] [CenterPoint_Node]: input fps: -1.00, out fps: -1.00, infer time ms: 27, post process time ms: 53
[hobot_centerpoint-1] [WARN] [0948485760.281992264] [CenterPoint_Node]: input fps: -1.00, out fps: -1.00, infer time ms: 28, post process time ms: 53
[hobot_centerpoint-1] [WARN] [0948485760.731948223] [CenterPoint_Node]: input fps: 2.93, out fps: 3.01, infer time ms: 27, post process time ms: 56
[hobot_centerpoint-1] [WARN] [0948485761.155906223] [CenterPoint_Node]: input fps: 2.93, out fps: 3.01, infer time ms: 28, post process time ms: 56
[hobot_centerpoint-1] [WARN] [0948485761.572980640] [CenterPoint_Node]: input fps: 2.93, out fps: 3.01, infer time ms: 27, post process time ms: 53
[hobot_centerpoint-1] [WARN] [0948485761.983718973] [CenterPoint_Node]: input fps: 2.40, out fps: 2.40, infer time ms: 28, post process time ms: 55
[hobot_centerpoint-1] [WARN] [0948485762.396930973] [CenterPoint_Node]: input fps: 2.40, out fps: 2.40, infer time ms: 28, post process time ms: 55
[hobot_centerpoint-1] [WARN] [0948485762.816782057] [CenterPoint_Node]: input fps: 2.40, out fps: 2.40, infer time ms: 27, post process time ms: 56
[hobot_centerpoint-1] [WARN] [0948485763.239294099] [CenterPoint_Node]: input fps: 2.39, out fps: 2.39, infer time ms: 27, post process time ms: 57
[hobot_centerpoint-1] [WARN] [0948485763.661555807] [CenterPoint_Node]: input fps: 2.39, out fps: 2.39, infer time ms: 27, post process time ms: 57
[hobot_centerpoint-1] [WARN] [0948485764.084410183] [CenterPoint_Node]: input fps: 2.39, out fps: 2.39, infer time ms: 27, post process time ms: 57
[hobot_centerpoint-1] [WARN] [0948485764.502788849] [CenterPoint_Node]: input fps: 2.37, out fps: 2.37, infer time ms: 27, post process time ms: 55
```

The output log shows that the topic for publishing the algorithm inference results is `/hobot_centerpoint`, and the point cloud file used for inference contains 81 frames. After inference and post-processing (including rendering and publishing the results), the frame rate is approximately 2.4 fps.

To view the images and algorithm rendering results, open a web browser on the PC and enter the following URL (replace `IP` with the IP address of the RDK):http://IP:8000


![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_centerpoint_det.jpg)
