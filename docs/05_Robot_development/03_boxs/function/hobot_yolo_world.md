---
sidebar_position: 16
---
# YOLO-World

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

YOLO-World是一种先进的开放词汇目标检测方法，根据输入文本的变化可以实现以零样本的方式高效检测出不同的全新类别目标。

代码仓库： (https://github.com/D-Robotics/hobot_yolo_world)

应用场景：YOLO-World强大的零样本检测能力使得其具有更强的泛化能力，可以应用在智能驾驶、智能家居、地质检测等领域。


## 支持平台

| 平台                             | 运行方式     | 示例功能                                                 |
| -------------------------------- | ------------ | -------------------------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头/本地回灌，并通过Web展示推理渲染结果 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 20.04/Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头。

4. 确认PC机能够通过网络访问RDK。

### X86平台

1. X86环境已配置Ubuntu 20.04系统镜像。

2. X86环境已成功安装tros.b。

## 使用介绍

YOLO-World(hobot_yolo_world) package订阅sensor package发布的图片，同时YOLO-World支持根据输入文本变化改变检测类别，其中文本特征来源于本地特征库，通过输入文本查询对应特征，并输入模型推理，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示sensor发布的图片和对应的算法结果。


### RDK平台

**使用MIPI摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">



</TabItem>

</Tabs>

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"
```

**使用USB摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"
```

**使用本地回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/config/ .

# 配置本地回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch hobot_yolo_world yolo_world.launch.py yolo_world_texts:="red bottle,trash bin"

```

## 结果分析

在运行终端输出如下信息：

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

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/render_yolo_world.jpeg)


## 进阶用法
如果您想更改本地的文本特征，可以利用相应的工具在本地生成

```bash
# 从tros.b的安装路径中拷贝出运行示例需要的工具文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_yolo_world/tool/ .
```

```bash
cd tool/

# 安装依赖
pip install -r requirements.txt

# 修改class.list里的词汇

# 生成本地词汇
python main.py

#拷贝新的词汇特征
mv offline_vocabulary_embeddings.json ../config/
```
