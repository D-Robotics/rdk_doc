---
sidebar_position: 21
---
# EdgeSAM分割一切

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

mono_edgesam package 是基于 [EdgeSAM](https://github.com/chongzhou96/EdgeSAM) 量化部署的使用示例。图像数据来源于本地图片回灌和订阅到的image msg。SAM 依赖检测框输入进行分割, 并分割检测框中的目标, 无需指定目标的类别信息, 仅需提供框。最终将算法信息通过话题发布, 同时在Web页面渲染可视化。

本示例中, 我们提供了两种部署展示方式:
- 固定框分割：固定了检测框（图片中央）用以分割。
- 订阅框分割：订阅上游检测网络输出的检测框信息, 对框中的信息进行分割。

代码仓库： (https://github.com/D-Robotics/mono_edgesam.git)

应用场景：结合检测框进行障碍物分割、水渍区域分割等。

## 支持平台

| 平台                  | 运行方式     | 示例功能                                                     |
| --------------------- | ------------ | ------------------------------------------------------------ |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | · 启动MIPI/USB摄像头/本地回灌, 推理渲染结果在Web显示/保存在本地 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

## 使用介绍

package对外发布包含语义分割和目标检测信息的算法msg, 用户可以订阅 "/perception/segmentation/edgesam" 话题用于应用开发。

### RDK平台

**mipi摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch mono_edgesam sam.launch.py 
```

</TabItem>

</Tabs>

**使用usb摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch mono_edgesam sam.launch.py 
```

</TabItem>

</Tabs>

**使用单张回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 配置回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch mono_edgesam sam.launch.py 
```

</TabItem>

</Tabs>

## 结果分析

**使用回灌工具发布图片**

package初始化后, 在运行终端输出如下信息：

```
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-07-28-19-51-28-488985-ubuntu-107175
[INFO] [launch]: Default logging verbosity is set to INFO
mono_edgesam basic_path is  /root/install/lib/mono_edgesam/config
camera_type is  fb
using feedback
Hobot shm pkg enables zero-copy with fastrtps profiles file: /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
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
[INFO] [hobot_image_pub-1]: process started with pid [107191]
[INFO] [hobot_codec_republish-2]: process started with pid [107193]
[INFO] [mono_edgesam-3]: process started with pid [107195]
[INFO] [websocket-4]: process started with pid [107197]
[hobot_codec_republish-2] [WARN] [1753703489.112526140] [hobot_codec_encoder]: Parameters:
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
[hobot_codec_republish-2] [WARN] [1753703489.118608825] [HobotCodecImpl]: platform x5
[hobot_codec_republish-2] [WARN] [1753703489.118873534] [hobot_codec_encoder]: Enabling zero-copy
[websocket-4] [WARN] [1753703489.499688698] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: perception/segmentation/edgesam
[websocket-4]  output_fps: 0
[hobot_image_pub-1] [WARN] [1753703489.864104228] [image_pub_node]: parameter:
[hobot_image_pub-1] image_source: /root/install/lib/mono_edgesam/config/4.jpg
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
[hobot_image_pub-1] [WARN] [1753703489.864484396] [hobot_image_pub]: Enabling zero-copy
[mono_edgesam-3] [WARN] [1753703489.948943404] [mono_edgesam]: Parameter:
[mono_edgesam-3]  cache_len_limit: 8
[mono_edgesam-3]  dump_render_img: 0
[mono_edgesam-3]  feed_type(0:local, 1:sub): 1
[mono_edgesam-3]  image: ./config/4.jpg
[mono_edgesam-3]  encoder_model_file_name: /root/install/lib/mono_edgesam/config/edgesam_encoder_1024.bin
[mono_edgesam-3]  decoder_model_file_name: /root/install/lib/mono_edgesam/config/edgesam_decoder_1024.bin
[mono_edgesam-3]  is_regular_box: 1
[mono_edgesam-3]  is_padding_seg: 0
[mono_edgesam-3]  is_shared_mem_sub: 1
[mono_edgesam-3]  is_sync_mode: 0
[mono_edgesam-3]  ai_msg_pub_topic_name: /perception/segmentation/edgesam
[mono_edgesam-3]  ai_msg_sub_topic_name: /hobot_dnn_detection
[mono_edgesam-3]  ros_img_sub_topic_name: /image
[mono_edgesam-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono_edgesam-3] [HBRT] set log level as 0. version = 3.15.55.0
[mono_edgesam-3] [DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[hobot_codec_republish-2] [WARN] [1753703490.160037466] [hobot_codec_encoder]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[hobot_codec_republish-2] [WARN] [1753703490.160368467] [HobotVenc]: init_pic_w_: 1920, init_pic_h_: 1080, alined_pic_w_: 1920, alined_pic_h_: 1088, aline_w_: 16, aline_h_: 16
[mono_edgesam-3] [A][DNN][packed_model.cpp:247][Model](2025-07-28,19:51:31.34.979) [HorizonRT] The model builder version = 1.24.3
[hobot_codec_republish-2] [WARN] [1753703491.369838914] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 11.9485, pub jpeg, fps: 11.9485, comm delay [1.7692]ms, codec delay [15.5385]ms
[mono_edgesam-3] [A][DNN][packed_model.cpp:247][Model](2025-07-28,19:51:33.829.850) [HorizonRT] The model builder version = 1.24.3
[mono_edgesam-3] [WARN] [1753703494.035526230] [mono_edgesam]: Create hbmem_subscription with topic_name: /hbmem_img
[mono_edgesam-3] [WARN] [1753703494.057170296] [edgesam_node]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[mono_edgesam-3] [WARN] [1753703494.213043065] [mono_edgesam]: Smart fps: 9.00, pre process time ms: 14, infer time ms: 92, post process time ms: 48
[mono_edgesam-3] [WARN] [1753703494.342125459] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 12, infer time ms: 81, post process time ms: 41
[hobot_codec_republish-2] [WARN] [1753703494.362260146] [hobot_codec_encoder]: Pub img fps [8.45]
[mono_edgesam-3] [WARN] [1753703494.471613064] [mono_edgesam]: Smart fps: 10.00, pre process time ms: 11, infer time ms: 88, post process time ms: 39
[mono_edgesam-3] [WARN] [1753703494.592498850] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 12, infer time ms: 81, post process time ms: 38
[mono_edgesam-3] [WARN] [1753703494.724020335] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 11, infer time ms: 84, post process time ms: 46
[mono_edgesam-3] [WARN] [1753703494.849518302] [mono_edgesam]: Smart fps: 10.00, pre process time ms: 12, infer time ms: 85, post process time ms: 38
[mono_edgesam-3] [WARN] [1753703494.973585807] [mono_edgesam]: Smart fps: 11.00, pre process time ms: 13, infer time ms: 82, post process time ms: 40
```

示例中推理的结果会渲染到Web上, 在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）, 打开界面右上角设置, 选中”Full Image Segmentation“选项, 可以显示渲染效果。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_sam.png)

## 进阶使用

如需调整检测框大小, 可参考下面方法验证。更重要的是可以通过上游检测节点检测结果作为sam输入。

运行sam, 取消固定框模式 sam_is_regular_box:=0
```shell
ros2 launch mono_edgesam sam.launch.py sam_is_regular_box:=0
```

在另一个终端发布ai话题。
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 160, "y_offset": 120, "width": 320, "height": 240}, "type": "anything"}]}] }'
```

说明：这里发布的话题名为"/hobot_dnn_detection", 检测框坐标起点(160, 120), 宽度320, 高度240。这里检测框起止点, 应该不超过输入图像的大小, 在实际使用中需留意。