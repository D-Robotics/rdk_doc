---
sidebar_position: 1
---
# BEV感知算法

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

BEV感知算法是使用[OpenExplorer](https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/bev.html)在[nuscenes](https://www.nuscenes.org/nuscenes)数据集上训练出来的`BEV`多任务模型。

算法输入为6组图像数据，分别是前视，左前，右前，后视，左后，右后图。模型输出为10个类别的目标以及对应的3D检测框，包括障碍物、多种类型车辆、交通标志等，以及车道线、人行道、马路边缘的语义分割。

此示例使用本地图像数据作为输入，利用BPU进行算法推理，发布算法感知结果渲染的图片消息，在PC端浏览器上渲染显示算法结果。

代码仓库： (https://github.com/D-Robotics/hobot_bev.git)

## 支持平台

| 平台      | 运行方式     | 示例功能                                |
| --------- | ------------ | --------------------------------------- |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 使用本地回灌，并通过web展示推理渲染结果 |
| RDK S100, RDK S100P | Ubuntu 22.04 (Humble) | 使用本地回灌，并通过web展示推理渲染结果 |
| RDK S600 | Ubuntu 24.04 (Jazzy) | 使用本地回灌，并通过web展示推理渲染结果 |

## 准备工作

1. RDK已烧录好Ubuntu系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. 确认PC机能够通过网络访问RDK。

## 使用介绍

### 使用本地数据集回灌

使用本地数据集回灌，经过推理后发布算法结果渲染后的图片消息，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。

***准备回灌数据集***

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# 板端下载数据集
wget http://archive.d-robotics.cc/TogetheROS/data/hobot_bev_data.tar.gz

# 解压缩
mkdir -p hobot_bev_data
tar -zxvf hobot_bev_data.tar.gz -C hobot_bev_data

# 解压完成后数据集在hobot_bev_data/data路径下
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# 板端下载数据集
cd ~
wget http://archive.d-robotics.cc/TogetheROS/data/nuscenes_bev_val/nuscenes_bev_val.tar.gz

# 解压缩
mkdir -p ~/hobot_bev_data
tar -zxvf ~/nuscenes_bev_val.tar.gz -C ~/hobot_bev_data
```

</TabItem>

<TabItem value="jazzy" label="Jazzy">

```shell
# 板端下载数据集
cd ~
wget http://archive.d-robotics.cc/TogetheROS/data/nuscenes_bev_val/nuscenes_bev_val.tar.gz

# 解压缩
mkdir -p ~/hobot_bev_data
tar -zxvf ~/nuscenes_bev_val.tar.gz -C ~/hobot_bev_data
```

</TabItem>

</Tabs>

***使用数据集回灌***

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动websocket服务
ros2 launch websocket websocket_service.launch.py

# 启动运行脚本，并指定数据集路径
ros2 launch hobot_bev hobot_bev.launch.py image_pre_path:=hobot_bev_data/data
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
# 配置tros.b humble环境
source /opt/tros/humble/setup.bash

if [ -L qat ]; then rm qat; fi
ln -s `ros2 pkg prefix hobot_bev`/lib/hobot_bev/qat/ qat
ln -s ~/hobot_bev_data/nuscenes_bev_val nuscenes_bev_val

# 启动运行脚本
ros2 launch hobot_bev hobot_bev.launch.py
```

</TabItem>
<TabItem value="jazzy" label="Jazzy">

```shell
# 配置tros.b 环境
source /opt/tros/jazzy/setup.bash

if [ -L qat ]; then rm qat; fi
ln -s `ros2 pkg prefix hobot_bev`/lib/hobot_bev/qat/ qat
ln -s ~/hobot_bev_data/nuscenes_bev_val nuscenes_bev_val

# 启动运行脚本
ros2 launch hobot_bev hobot_bev.launch.py
```

</TabItem>

</Tabs>

## 结果分析

在运行终端输出如下信息：

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

<TabItem value="jazzy" label="Jazzy">

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

在PC端的浏览器输入`http://IP:8000`即可查看图像和算法渲染效果（IP为RDK的IP地址）：

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev.jpeg)

</TabItem>

<TabItem value="humble" label="Humble">

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev_s100.jpeg)

</TabItem>

<TabItem value="jazzy" label="Jazzy">

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_bev_s100.jpeg)

</TabItem>

</Tabs>
