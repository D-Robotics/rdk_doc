---
sidebar_position: 2
---
# LiDAR Object Detection Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction
The LiDAR object detection algorithm is a `CenterPoint` algorithm model trained on the [nuscenes](https://www.nuscenes.org/nuscenes) dataset using [OpenExplorer](https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/centerpoint.html).

The algorithm takes 32-channel LiDAR point cloud data as input and outputs information including 3D detection bounding boxes, confidence scores and categories of targets. It supports six target detection categories: car, truck, bus, barrier, motorcycle and pedestrian.

This example uses local LiDAR point cloud files as input, leverages the BPU for algorithm inference, and publishes rendered image messages containing point cloud data, target detection bounding boxes and orientations. The algorithm results are rendered and displayed in a browser on the PC side.

Code Repository: (https://github.com/D-Robotics/hobot_centerpoint)

## Supported Platforms
| Platform      | Running Method     | Example Function                                |
| ------------- | ------------------ | ----------------------------------------------- |
| RDK Ultra     | Ubuntu 20.04 (Foxy)| Local playback, with inference rendering results displayed via web |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | Local playback, with inference rendering results displayed via web |

## Preparation
### RDK Platform
1. The RDK has been flashed with the Ubuntu 20.04/Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. Ensure the PC can access the RDK via the network.

## Usage Guide
### RDK Platform
### Local Point Cloud File Playback
The LiDAR object detection algorithm example uses LiDAR point cloud file playback for inference, then publishes image messages with rendered algorithm results via the websocket package, enabling the display of published images and corresponding algorithm results in a browser on the PC side.

Prepare LiDAR point cloud files:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Download playback point cloud files on the board
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_centerpoint_data.tar.gz

# Decompress the file
mkdir config
tar -zxvf hobot_centerpoint_data.tar.gz -C config
# Decompressed data is in the config/hobot_centerpoint_data directory
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Download playback point cloud files on the board
cd ~
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_centerpoint_data.tar.gz.gz

# Decompress the file
mkdir -p ~/centerpoint_data
tar -zxvf ~/hobot_centerpoint_data.tar.gz -C ~/centerpoint_data
```

</TabItem>
</Tabs>

Launch the algorithm example:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the tros.b environment
source /opt/tros/setup.bash

# Launch the websocket service
ros2 launch websocket websocket_service.launch.py

# Launch the launch file
ros2 launch hobot_centerpoint hobot_centerpoint_websocket.launch.py lidar_pre_path:=config/hobot_centerpoint_data
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b Humble environment
source /opt/tros/humble/setup.bash

if [ -L qat ]; then rm qat; fi
ln -s `ros2 pkg prefix hobot_centerpoint`/lib/hobot_centerpoint/qat/ qat
ln -s ~/centerpoint_data centerpoint_data

# Launch the launch file
ros2 launch hobot_centerpoint hobot_centerpoint.launch.py
```

</TabItem>
</Tabs>

## Result Analysis
After launching the algorithm example, the following information is output in the running terminal:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

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

</TabItem>

<TabItem value="humble" label="Humble">

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-05-08-10-05-16-060526-ubuntu-20968
[INFO] [launch]: Default logging verbosity is set to INFO
webserver has launch
[INFO] [hobot_centerpoint-1]: process started with pid [20971]
[INFO] [websocket-2]: process started with pid [20973]
[hobot_centerpoint-1] [UCP]: log level = 3
[hobot_centerpoint-1] [UCP]: UCP version = 3.3.3
[hobot_centerpoint-1] [VP]: log level = 3
[hobot_centerpoint-1] [DNN]: log level = 3
[hobot_centerpoint-1] [HPL]: log level = 3
[websocket-2] [WARN] [1746669916.389039854] [websocket]:
[websocket-2] Parameter:
[websocket-2]  image_topic: /image_jpeg
[websocket-2]  image_type: mjpeg
[websocket-2]  only_show_image: 1
[websocket-2]  output_fps: 0
[websocket-2] [INFO] [1746669916.389302684] [websocket]: Websocket using image mjpeg
[hobot_centerpoint-1] [UCPT]: log level = 6
[hobot_centerpoint-1] [DSP]: log level = 3
[hobot_centerpoint-1] [INFO] [1746669916.477961938] [centerpoint_node]: CenterPointNode init
[hobot_centerpoint-1] [WARN] [1746669916.478312520] [centerpoint_node]:
[hobot_centerpoint-1]  topic_name: image_jpeg
[hobot_centerpoint-1]  save_image: false
[hobot_centerpoint-1]  glog_level: 1
[hobot_centerpoint-1] [WARN] [1746669916.482928131] [ai_wrapper]:
[hobot_centerpoint-1]  Set glog level in cmd line with '--glog_level=$num'
[hobot_centerpoint-1]    EXAMPLE_SYSTEM = 0,  EXAMPLE_REPORT = 1,  EXAMPLE_DETAIL = 2,  EXAMPLE_DEBUG = 3
[hobot_centerpoint-1] [BPU][[BPU_MONITOR]][281473110813600][INFO]BPULib verison(2, 1, 2)[0d3f195]!
[hobot_centerpoint-1] [DNN] HBTL_EXT_DNN log level:6
[hobot_centerpoint-1] [DNN]: 3.3.3_(4.1.17 HBRT)
[hobot_centerpoint-1] [INFO] [1746669917.244757440] [centerpoint_node]: Get render imgs size: 1, frame_id: 0, duration ms infer: 46.38, postp: 9.55, prep: 16.01
[hobot_centerpoint-1] [INFO] [1746669917.264258828] [centerpoint_node]: Publish ros compressed image msg, format: jpeg, topic: image_jpeg
```

</TabItem>
</Tabs>

The output log shows that the topic for publishing algorithm inference results is `/hobot_centerpoint`, and a total of 81 playback point cloud files are loaded. After inference and post-processing (including rendering and publishing of inference results), the algorithm runs at a frame rate of approximately 2.4fps.

Enter http://IP:8000 in the browser on the PC side to view the images and algorithm rendering effects (IP is the IP address of the RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_centerpoint_det.jpg)
