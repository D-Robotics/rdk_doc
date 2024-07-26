---
sidebar_position: 9
---
# 激光雷达目标检测算法

## 功能介绍

激光雷达目标检测算法是使用地平线[OpenExplorer](https://developer.horizon.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/centerpoint.html)在[nuscenes](https://www.nuscenes.org/nuscenes)数据集上训练出来的`CenterPoint`算法模型。

算法输入为32线激光雷达点云数据，输出信息包括目标的3D检测框、置信度、类别。支持的目标检测类型包括car、truck、bus、barrier、motorcycle、pedestrian共六大类别。

此示例使用本地激光雷达点云文件作为输入，利用BPU进行算法推理，发布包含点云数据、目标检测框和朝向的渲染图片消息，在PC端浏览器上渲染显示算法结果。

代码仓库： (https://github.com/D-Robotics/hobot_centerpoint)

## 支持平台

| 平台      | 运行方式     | 示例功能                                |
| --------- | ------------ | --------------------------------------- |
| RDK Ultra | Ubuntu 20.04 (Foxy) | 使用本地回灌，并通过web展示推理渲染结果 |

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04系统镜像。

2. 地平线RDK已成功安装TogetheROS.Bot。

3. 确认PC机能够通过网络访问地平线RDK。

## 使用介绍

### 地平线RDK平台

### 使用本地点云文件回灌

激光雷达物体检测算法示例使用激光雷达点云文件回灌，经过推理后将算法结果渲染后的图片msg，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。

准备激光雷达点云文件：

```shell
# 板端下载回灌的点云文件
wget http://sunrise.horizon.cc/TogetheROS/data/hobot_centerpoint_data.tar.gz

# 解压缩
mkdir config
tar -zxvf hobot_centerpoint_data.tar.gz -C config
# 解压完成后数据在config/hobot_centerpoint_data路径下
```

启动算法示例：

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动websocket服务
ros2 launch websocket websocket_service.launch.py

# 启动launch文件
ros2 launch hobot_centerpoint hobot_centerpoint_websocket.launch.py lidar_pre_path:=config/hobot_centerpoint_data
```

## 结果分析

启动算法示例后在运行终端输出如下信息：

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

输出log显示，发布算法推理结果的topic为`/hobot_centerpoint`, 获取的回灌点云文件为81个。算法经过推理，后处理(包含推理结果的渲染和发布)，帧率约为2.4fps。

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/render_centerpoint_det.jpg)
