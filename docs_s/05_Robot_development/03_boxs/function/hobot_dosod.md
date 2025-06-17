---
sidebar_position: 19
---
# DOSOD

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

DOSOD (Decoupled Open-Set Object Detector)[https://github.com/D-Robotics-AI-Lab/DOSOD] 是地瓜自研的开放词汇目标检测方法，利用输入文本特征进行重参数化，可以实现以零样本的方式更改目标检测类别。此为与常规检测器最大的差别。

代码仓库： (https://github.com/D-Robotics/hobot_dosod)

应用场景：DOSOD强大的零样本检测能力使得其具有更强的泛化能力，可以应用在智能驾驶、智能家居、地质检测等领域。


## 支持平台

| 平台                             | 运行方式     | 示例功能                                                 |
| -------------------------------- | ------------ | -------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头/本地回灌，并通过Web展示推理渲染结果 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

DOSOD (hobot_dosod) package订阅sensor package发布的图片，并输入模型推理，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示sensor发布的图片和对应的算法结果。


### RDK平台

**使用MIPI摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件, 通过dosod_model_file_name配置对应模型, dosod_vocabulary_file_name配置对应词汇本。注意词汇本与模型一一对应。
ros2 launch hobot_dosod dosod.launch.py dosod_model_file_name:=config/dosod_mlp3x_l_rep-int8.bin dosod_vocabulary_file_name:=config/offline_vocabulary.json
```

</TabItem>

</Tabs>

**使用USB摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件, 通过dosod_model_file_name配置对应模型, dosod_vocabulary_file_name配置对应词汇本。注意词汇本与模型一一对应。
ros2 launch hobot_dosod dosod.launch.py dosod_model_file_name:=config/dosod_mlp3x_l_rep-int8.bin dosod_vocabulary_file_name:=config/offline_vocabulary.json
```

</TabItem>

</Tabs>

**使用本地回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_dosod/config/ .

# 配置本地回灌图片
export CAM_TYPE=fb

# 启动launch文件, 通过dosod_model_file_name配置对应模型, dosod_vocabulary_file_name配置对应词汇本。注意词汇本与模型一一对应。
ros2 launch hobot_dosod dosod.launch.py dosod_model_file_name:=config/dosod_mlp3x_l_rep-int8.bin dosod_vocabulary_file_name:=config/offline_vocabulary.json
```

</TabItem>

</Tabs>

## 结果分析

在运行终端输出如下信息：

```shell
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-01-08-11-03-34-125542-ubuntu-9125
[INFO] [launch]: Default logging verbosity is set to INFO
camera_type is  fb
using feedback
Hobot shm pkg enables zero-copy with fastrtps profiles file: /userdata/install/lib/hobot_shm/config/shm_fastdds.xml
Hobot shm pkg sets RMW_FASTRTPS_USE_QOS_FROM_XML: 1
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [9128]
[INFO] [hobot_codec_republish-2]: process started with pid [9130]
[INFO] [hobot_dosod-3]: process started with pid [9132]
[INFO] [websocket-4]: process started with pid [9134]
[hobot_codec_republish-2] [WARN] [1736305415.448727260] [hobot_codec_encoder]: Parameters:
[hobot_codec_republish-2] sub_topic: /hbmem_img
[hobot_codec_republish-2] pub_topic: /image
[hobot_codec_republish-2] channel: 1
[hobot_codec_republish-2] in_mode: shared_mem
[hobot_codec_republish-2] out_mode: ros
[hobot_codec_republish-2] in_format: nv12
[hobot_codec_republish-2] out_format: jpeg
[hobot_codec_republish-2] enc_qp: 10
[hobot_codec_republish-2] jpg_quality: 60
[hobot_codec_republish-2] input_framerate: 30
[hobot_codec_republish-2] output_framerate: -1
[hobot_codec_republish-2] dump_output: 0
[hobot_codec_republish-2] [WARN] [1736305415.455977260] [HobotCodecImpl]: platform x5
[hobot_codec_republish-2] [WARN] [1736305415.456186677] [hobot_codec_encoder]: Enabling zero-copy
[hobot_dosod-3] [WARN] [1736305415.687929557] [hobot_dosod]: This is hobot dosod!
[websocket-4] [WARN] [1736305415.794630560] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: hobot_dosod
[websocket-4]  output_fps: 0
[hobot_dosod-3] [WARN] [1736305415.835729185] [hobot_dosod]: Parameter:
[hobot_dosod-3]  model_file_name: config/dosod_mlp3x_l_rep-int8.bin
[hobot_dosod-3]  vocabulary_file_name: config/offline_vocabulary.json
[hobot_dosod-3]  feed_type(0:local, 1:sub): 1
[hobot_dosod-3]  image: config/000000160864.jpg
[hobot_dosod-3]  dump_ai_result: 0
[hobot_dosod-3]  dump_raw_img: 0
[hobot_dosod-3]  dump_render_img: 0
[hobot_dosod-3]  dump_ai_path: .
[hobot_dosod-3]  dump_raw_path: .
[hobot_dosod-3]  dump_render_path: .
[hobot_dosod-3]  is_shared_mem_sub: 1
[hobot_dosod-3]  score_threshold: 0.2
[hobot_dosod-3]  iou_threshold: 0.5
[hobot_dosod-3]  nms_top_k: 50
[hobot_dosod-3]  is_homography: 0
[hobot_dosod-3]  trigger_mode: 0
[hobot_dosod-3]  class_mode: 0
[hobot_dosod-3]  task_num: 2
[hobot_dosod-3]  roi: 0
[hobot_dosod-3]  y_offset: 950
[hobot_dosod-3]  ai_msg_pub_topic_name: /hobot_dosod
[hobot_dosod-3]  ros_img_sub_topic_name: /image
[hobot_dosod-3] [WARN] [1736305415.836617477] [hobot_dosod]: model_file_name_: config/dosod_mlp3x_l_rep-int8.bin, task_num: 2
[hobot_dosod-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[hobot_dosod-3] [HBRT] set log level as 0. version = 3.15.54.0
[hobot_dosod-3] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[hobot_image_pub-1] [WARN] [1736305416.129590859] [image_pub_node]: parameter:
[hobot_image_pub-1] image_source: ./config/000000160864.jpg
[hobot_image_pub-1] source_image_w: 960
[hobot_image_pub-1] source_image_h: 544
[hobot_image_pub-1] output_image_w: 1920
[hobot_image_pub-1] output_image_h: 1080
[hobot_image_pub-1] fps: 10
[hobot_image_pub-1] is_shared_mem: 1
[hobot_image_pub-1] is_loop: 1
[hobot_image_pub-1] is_compressed_img_pub: 0
[hobot_image_pub-1] image_format: jpg
[hobot_image_pub-1] pub_encoding: nv12pub_name_mode: 0
[hobot_image_pub-1] msg_pub_topic_name: /hbmem_img
[hobot_image_pub-1] [WARN] [1736305416.130613275] [hobot_image_pub]: Enabling zero-copy
[hobot_codec_republish-2] [WARN] [1736305416.348757530] [hobot_codec_encoder]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1736305416.349073988] [HobotVenc]: init_pic_w_: 1920, init_pic_h_: 1080, alined_pic_w_: 1920, alined_pic_h_: 1088, aline_w_: 16, aline_h_: 16
[hobot_dosod-3] [A][DNN][packed_model.cpp:247][Model](2025-01-08,11:03:36.664.601) [HorizonRT] The model builder version = 1.24.3
[hobot_dosod-3] [WARN] [1736305417.323044552] [hobot_dosod]: Get model name: 3x-l_epoch_100_rep-coco80-without-nms from load model.
[hobot_dosod-3] [WARN] [1736305417.323560635] [hobot_dosod]: Create ai msg publisher with topic_name: /hobot_dosod
[hobot_dosod-3] [WARN] [1736305417.350238969] [hobot_dosod]: Create img hbmem_subscription with topic_name: /hbmem_img
[hobot_dosod-3] [WARN] [1736305417.445453471] [dosod]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1736305417.453916388] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 11.4504, pub jpeg, fps: 11.4504, comm delay [0.0833]ms, codec delay [13.5833]ms
[hobot_dosod-3] [W][DNN]bpu_model_info.cpp:491][Version](2025-01-08,11:03:37.311.128) Model: 3x-l_epoch_100_rep-coco80-without-nms. Inconsistency between the hbrt library version 3.15.54.0 and the model build version 3.15.55.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[hobot_dosod-3] [WARN] [1736305418.846408168] [hobot_dosod]: Sub img fps: 12.95, Smart fps: 12.67, pre process time ms: 12, infer time ms: 78, post process time ms: 8
[hobot_dosod-3] [WARN] [1736305419.857350566] [hobot_dosod]: Sub img fps: 9.97, Smart fps: 10.88, pre process time ms: 11, infer time ms: 91, post process time ms: 8
[hobot_dosod-3] [WARN] [1736305420.858769504] [hobot_dosod]: Sub img fps: 10.04, Smart fps: 9.99, pre process time ms: 13, infer time ms: 100, post process time ms: 7
[hobot_dosod-3] [WARN] [1736305421.860964318] [hobot_dosod]: Sub img fps: 9.99, Smart fps: 9.99, pre process time ms: 14, infer time ms: 100, post process time ms: 7
```

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_dosod.jpeg)


## 进阶使用
如果您希望修改自定义类别, 请参考[模型重参数化方法] （链接即将上线）