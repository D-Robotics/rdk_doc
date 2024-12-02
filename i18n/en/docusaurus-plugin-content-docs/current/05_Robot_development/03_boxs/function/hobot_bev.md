---
sidebar_position: 10
---
# BEV Perception Algorithm

## Function Introduction

The BEV perception algorithm is a multi-task model trained on the [nuscenes](https://www.nuscenes.org/nuscenes) dataset using [OpenExplorer](https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/bev.html).

The algorithm takes 6 sets of image data as input: front view, left-front, right-front, rear view, left-rear, and right-rear images. The model outputs the 3D detection boxes for 10 categories of objects, including obstacles, various types of vehicles, traffic signs, and semantic segmentation for lane lines, sidewalks, and road edges.

This example uses local image data as input, performs algorithm inference using the BPU, and publishes the rendered images of the perception results. The results are displayed on the PC browser.

Code Repository: [https://github.com/D-Robotics/hobot_bev.git](https://github.com/D-Robotics/hobot_bev.git)

## Supported Platforms

| Platform     | Run Method            | Example Functionality                              |
| ------------ | --------------------- | -------------------------------------------------- |
| RDK Ultra    | Ubuntu 20.04 (Foxy)    | Use local data injection and display inference results via web |

## Preparation Work

1. The RDK has the Ubuntu 20.04 system image flashed.

2. The RDK has TogetheROS.Bot installed successfully.

3. Ensure that the PC can access the RDK through the network.

## Usage Instructions

### Using Local Dataset for Injection

Use the local dataset for injection, perform inference, and publish the rendered images of the algorithm's results. These images will be displayed on the PC browser via a WebSocket package.

***Prepare the Local Dataset for Injection***


```shell
# Download the dataset
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_bev_data.tar.gz

# Extract the dataset
mkdir -p hobot_bev_data
tar -zxvf hobot_bev_data.tar.gz -C hobot_bev_data

# After extraction, the dataset will be located at hobot_bev_data/data

```

***Using Dataset for Injection***



```shell
# 配置tros.b环境
# Configure the tros.b environment
source /opt/tros/setup.bash

# Start the websocket service
ros2 launch websocket websocket_service.launch.py

# Start the run script and specify the dataset path
ros2 launch hobot_bev hobot_bev.launch.py image_pre_path:=hobot_bev_data/data

```

## Result Analysis

The following information will be output in the terminal:



```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-05-17-47-07-232907-hobot-2627970
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_bev-1]: process started with pid [2627972]
[INFO] [websocket-2]: process started with pid [2627974]
[hobot_bev-1] [WARN] [1688579227.907268364] [bev_node]:
[hobot_bev-1]  image_pre_path: hobot_bev_data/data
[hobot_bev-1] [BPU_PLAT]BPU Platform Version(1.3.3)!
[hobot_bev-1] [HBRT] set log level as 0. version = 3.14.25.0
[hobot_bev-1] [DNN] Runtime version = 1.12.3_(3.14.25 HBRT)
[hobot_bev-1] [WARN] [1688579228.714778531] [dnn]: Run default SetOutputParser.
[hobot_bev-1] [WARN] [1688579228.714925489] [dnn]: Set output parser with default dnn node parser, you will get all output tensors and should parse output_tensors in PostProcess.
[hobot_bev-1] [WARN] [1688579228.886846489] [bev_node]: loop 0/1002
[hobot_bev-1] [WARN] [1688579229.474568573] [bev_node]: loop 1/1002
[hobot_bev-1] [WARN] [1688579230.058551781] [bev_node]: loop 2/1002
[hobot_bev-1] [WARN] [1688579230.691667198] [bev_node]: loop 3/1002
[hobot_bev-1] [WARN] [1688579231.324658782] [bev_node]: loop 4/1002
[hobot_bev-1] [WARN] [1688579231.365145532] [bev_node]: input fps: 2.47, out fps: 2.52, infer time ms: 12, post process time ms: 659
[hobot_bev-1] [WARN] [1688579231.915645741] [bev_node]: loop 5/1002
[hobot_bev-1] [WARN] [1688579231.996993824] [bev_node]: input fps: 2.47, out fps: 2.52, infer time ms: 12, post process time ms: 658
```

To view the images and algorithm rendering results, open a web browser on the PC and enter the following URL (replace `IP` with the RDK's IP address):http://IP:8000

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev.jpeg)
