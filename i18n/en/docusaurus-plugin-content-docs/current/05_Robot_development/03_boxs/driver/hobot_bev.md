---
sidebar_position: 3
---
# BEV Perception Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction
The BEV Perception Algorithm is a `BEV` multi-task model trained on the [nuscenes](https://www.nuscenes.org/nuscenes) dataset using [OpenExplorer](https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/bev.html).

The algorithm takes six sets of image data as input, namely the front, front-left, front-right, rear, rear-left and rear-right views. The model outputs 10 categories of targets with their corresponding 3D detection bounding boxes (including obstacles, various types of vehicles, traffic signs, etc.), as well as semantic segmentation of lane lines, sidewalks and road edges.

This example uses local image data as input, leverages the BPU for algorithm inference, and publishes image messages with rendered algorithm perception results, which are displayed in a browser on the PC side.

Code Repository: (https://github.com/D-Robotics/hobot_bev.git)

## Supported Platforms
| Platform      | Running Method     | Example Function                                |
| ------------- | ------------------ | ----------------------------------------------- |
| RDK Ultra     | Ubuntu 20.04 (Foxy)| Local playback with inference rendering results displayed via web |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | Local playback with inference rendering results displayed via web |

## Preparation
1. The RDK has been flashed with the Ubuntu 20.04/Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. Ensure the PC can access the RDK via the network.

## Usage Guide
### Local Dataset Playback
This example uses local dataset playback for inference, then publishes image messages with rendered algorithm results. The published images and corresponding algorithm results are displayed in a browser on the PC side via the websocket package.

***Prepare Playback Dataset***

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Download the dataset on the board
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_bev_data.tar.gz

# Decompress the file
mkdir -p hobot_bev_data
tar -zxvf hobot_bev_data.tar.gz -C hobot_bev_data

# The decompressed dataset is in the hobot_bev_data/data directory
```

</TabItem>
<TabItem value="humble" label="Humble">

```shell
# Download the dataset on the board
cd ~
wget http://archive.d-robotics.cc/TogetheROS/data/nuscenes_bev_val/nuscenes_bev_val.tar.gz

# Decompress the file
mkdir -p ~/hobot_bev_data
tar -zxvf ~/nuscenes_bev_val.tar.gz -C ~/hobot_bev_data
```

</TabItem>
</Tabs>

***Run Dataset Playback***

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# Configure the TogetheROS.Bot environment
source /opt/tros/setup.bash

# Launch the websocket service
ros2 launch websocket websocket_service.launch.py

# Launch the run script and specify the dataset path
ros2 launch hobot_bev hobot_bev.launch.py image_pre_path:=hobot_bev_data/data
```

</TabItem>
<TabItem value="humble" label="Humble">

```shell
# Configure the TogetheROS.Bot Humble environment
source /opt/tros/humble/setup.bash

if [ -L qat ]; then rm qat; fi
ln -s `ros2 pkg prefix hobot_bev`/lib/hobot_bev/qat/ qat
ln -s ~/hobot_bev_data/nuscenes_bev_val nuscenes_bev_val

# Launch the run script
ros2 launch hobot_bev hobot_bev.launch.py
```

</TabItem>
</Tabs>

## Result Analysis
The following information is output in the running terminal:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

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

</TabItem>
<TabItem value="humble" label="Humble">

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-05-08-09-44-40-838952-ubuntu-20037
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_bev-1]: process started with pid [20040]
[INFO] [websocket-2]: process started with pid [20042]
[hobot_bev-1] [UCP]: log level = 3
[hobot_bev-1] [UCP]: UCP version = 3.3.3
[hobot_bev-1] [VP]: log level = 3
[hobot_bev-1] [DNN]: log level = 3
[hobot_bev-1] [HPL]: log level = 3
[websocket-2] [WARN] [1746668681.078783258] [websocket]:
[websocket-2] Parameter:
[websocket-2]  image_topic: /image_jpeg
[websocket-2]  image_type: mjpeg
[websocket-2]  only_show_image: 1
[websocket-2]  output_fps: 0
[websocket-2] [INFO] [1746668681.079077507] [websocket]: Websocket using image mjpeg
[hobot_bev-1] [UCPT]: log level = 6
[hobot_bev-1] [DSP]: log level = 3
[hobot_bev-1] [INFO] [1746668681.182092730] [bev_node]: BevNode init
[hobot_bev-1] [WARN] [1746668681.182327429] [bev_node]:
[hobot_bev-1]  topic_name: image_jpeg
[hobot_bev-1]  save_image: false
[hobot_bev-1]  glog_level: 1
[hobot_bev-1] [WARN] [1746668681.186660916] [ai_wrapper]:
[hobot_bev-1]  Set glog level in cmd line with '--glog_level=$num'
[hobot_bev-1]    EXAMPLE_SYSTEM = 0,  EXAMPLE_REPORT = 1,  EXAMPLE_DETAIL = 2,  EXAMPLE_DEBUG = 3
[hobot_bev-1] [BPU][[BPU_MONITOR]][281473498852256][INFO]BPULib verison(2, 1, 2)[0d3f195]!
[hobot_bev-1] [DNN] HBTL_EXT_DNN log level:6
[hobot_bev-1] [DNN]: 3.3.3_(4.1.17 HBRT)
[hobot_bev-1] [INFO] [1746668681.944706857] [bev_node]: Get render imgs size: 8, frame_id: 0, duration ms infer: 12.52, postp: 3.37, prep: 0.00
[hobot_bev-1] [INFO] [1746668681.997575564] [bev_node]: Publish ros compressed image msg, format: jpeg, topic: image_jpeg
```

</TabItem>
</Tabs>

Enter `http://IP:8000` in the browser on the PC side to view the images and algorithm rendering effects (IP refers to the IP address of the RDK):

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev.jpeg)

</TabItem>
<TabItem value="humble" label="Humble">

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev_s100.jpeg)

</TabItem>
</Tabs>
