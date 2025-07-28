---
sidebar_position: 22
---
# 人体实例跟踪

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

Reid package 是基于[Reid](https://github.com/KaiyangZhou/deep-person-reid.git) 量化部署的使用示例。图像数据来源于本地图片回灌和订阅到的image msg。Reid 依赖人体检测框输入进行人体特征提取, 将每个人物特征编码为[1, 512] 大小的特征, 通过特征余弦相似度的对比, 来判断是否是同个人物。这类方法最大特点是当跟随的目标重新出现在画面时, 会根据特征库里的特征对比, 判断是否是同个实例ID。

代码仓库：

 (https://github.com/D-Robotics/reid.git)

 (https://github.com/D-Robotics/mono2d_body_detection)

应用场景：人体跟踪, 人体实例检测。

## 支持平台

| 平台                  | 运行方式     | 示例功能                                                     |
| --------------------- | ------------ | ------------------------------------------------------------ |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | · 启动MIPI/USB摄像头/本地回灌, 推理渲染结果在Web显示 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

## 使用介绍

package对外发布包含实例ID信息的算法msg, 用户可以订阅 "/perception/detection/reid" 话题用于应用开发。

### RDK平台

**mipi摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/reid/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch reid reid.launch.py
```

</TabItem>

</Tabs>

**使用usb摄像头发布图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/reid/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch reid reid.launch.py
```

</TabItem>

</Tabs>

**使用单张回灌图片**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# 配置ROS2环境
source /opt/tros/humble/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/reid/config/ .

# 配置回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch reid reid.launch.py publish_image_source:=config/person_body.jpg publish_image_format:=jpg publish_output_image_w:=1920 publish_output_image_h:=1080
```

</TabItem>

</Tabs>

## 结果分析

**使用图片工具发布图片**

package初始化后, 在运行终端输出如下信息：

```
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-07-28-12-05-31-492990-ubuntu-24131
[INFO] [launch]: Default logging verbosity is set to INFO
camera_type is  fb
using feedback
Hobot shm pkg enables zero-copy with fastrtps profiles file: /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
Hobot shm pkg sets RMW_FASTRTPS_USE_QOS_FROM_XML: 1
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
camera_type is  fb
using feedback
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
webserver has launch
[INFO] [hobot_image_pub-1]: process started with pid [24147]
[INFO] [hobot_codec_republish-2]: process started with pid [24149]
[INFO] [mono2d_body_detection-3]: process started with pid [24151]
[INFO] [websocket-4]: process started with pid [24153]
[INFO] [reid-5]: process started with pid [24155]
[hobot_codec_republish-2] [WARN] [1753675532.073298709] [hobot_codec_decoder]: Parameters:
[hobot_codec_republish-2] sub_topic: /image
[hobot_codec_republish-2] pub_topic: /hbmem_img
[hobot_codec_republish-2] channel: 1
[hobot_codec_republish-2] in_mode: ros
[hobot_codec_republish-2] out_mode: shared_mem
[hobot_codec_republish-2] in_format: jpeg
[hobot_codec_republish-2] out_format: nv12
[hobot_codec_republish-2] enc_qp: 10
[hobot_codec_republish-2] jpg_quality: 60
[hobot_codec_republish-2] input_framerate: 30
[hobot_codec_republish-2] output_framerate: -1
[hobot_codec_republish-2] dump_output: 0
[hobot_codec_republish-2] [WARN] [1753675532.084413614] [HobotCodecImpl]: platform x5
[mono2d_body_detection-3] [WARN] [1753675532.729885121] [mono2d_body_det]: Parameter:
[mono2d_body_detection-3]  is_sync_mode_: 0
[mono2d_body_detection-3]  model_file_name_: config/multitask_body_head_face_hand_kps_960x544.hbm
[mono2d_body_detection-3]  is_shared_mem_sub: 1
[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection
[mono2d_body_detection-3]  ros_img_topic_name: /image_raw
[mono2d_body_detection-3]  image_gap: 1
[mono2d_body_detection-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono2d_body_detection-3] [HBRT] set log level as 0. version = 3.15.55.0
[mono2d_body_detection-3] [DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[websocket-4] [WARN] [1753675532.786307105] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: /perception/detection/reid
[websocket-4]  output_fps: 0
[reid-5] [WARN] [1753675532.797354134] [reid_node]: Parameter:
[reid-5]  feed_type(0:local, 1:sub): 1
[reid-5]  db_file: reid.db
[reid-5]  model_file_name: config/reid.bin
[reid-5]  dump_render_img: 0
[reid-5]  is_sync_mode: 1
[reid-5]  is_shared_mem_sub: 1
[reid-5]  threshold: 0.7
[reid-5]  ai_msg_pub_topic_name: /perception/detection/reid
[reid-5]  ai_msg_sub_topic_name: /hobot_mono2d_body_detection
[reid-5]  ros_img_topic_name: /image_raw
[reid-5]  sharedmem_img_topic_name: /hbmem_img
[reid-5] [BPU_PLAT]BPU Platform Version(1.3.6)!
[reid-5] [HBRT] set log level as 0. version = 3.15.55.0
[reid-5] [DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[hobot_image_pub-1] [WARN] [1753675532.819144234] [image_pub_node]: parameter:
[hobot_image_pub-1] image_source: config/person_body.jpg
[hobot_image_pub-1] source_image_w: 960
[hobot_image_pub-1] source_image_h: 544
[hobot_image_pub-1] output_image_w: 1920
[hobot_image_pub-1] output_image_h: 1080
[hobot_image_pub-1] fps: 10
[hobot_image_pub-1] is_shared_mem: 0
[hobot_image_pub-1] is_loop: 1
[hobot_image_pub-1] is_compressed_img_pub: 1
[hobot_image_pub-1] image_format: jpg
[hobot_image_pub-1] pub_encoding: nv12pub_name_mode: 0
[hobot_image_pub-1] msg_pub_topic_name: /image
[mono2d_body_detection-3] [WARN] [1753675532.965378414] [mono2d_body_det]: Enabling zero-copy
[mono2d_body_detection-3] [WARN] [1753675532.965547956] [mono2d_body_det]: Create hbmem_subscription with topic_name: /hbmem_img
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_euclid_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json
[mono2d_body_detection-3]
[reid-5] [A][DNN][packed_model.cpp:247][Model](2025-07-28,12:05:32.926.316) [HorizonRT] The model builder version = 1.24.4
[reid-5] [WARN] [1753675533.102411862] [reid_node]: Create hbmem_subscription with topic_name: /hbmem_img
[mono2d_body_detection-3] [WARN] [1753675533.132823359] [mono2d_body_det]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[mono2d_body_detection-3] [WARN] [1753675533.133154735] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1088, w: 1920, step: 1920, index: 0, stamp: 1753675533_58950288, data size: 3133440, comm delay [74.1792]ms
[reid-5] [WARN] [1753675533.132964485] [reid_node]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[mono2d_body_detection-3] [W][DNN]bpu_model_info.cpp:491][Version](2025-07-28,12:05:32.951.571) Model: multitask_body_head_face_hand_kps_960x544. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[reid-5] [WARN] [1753675533.492897025] [reid_fet_manage]:
[reid-5] [WARN] [1753675533.503463345] [reid_fet_manage]: Query failed, storage: 1.
[reid-5] [WARN] [1753675533.504189180] [reid_fet_manage]:
[reid-5] id: 0, item similarity: 0.490674
[reid-5] id: 1, item similarity: 0.490674
[reid-5] [WARN] [1753675533.519353762] [reid_fet_manage]: Query failed, storage: 2.
[reid-5] [WARN] [1753675533.520454765] [reid_fet_manage]:
[reid-5] id: 0, item similarity: 0.428227
[reid-5] id: 1, item similarity: 0.428227
[reid-5] id: 2, item similarity: 0.424104
[reid-5] [WARN] [1753675533.534144760] [reid_fet_manage]: Query failed, storage: 3.
[reid-5] [WARN] [1753675533.535162346] [reid_fet_manage]:
[reid-5] id: 0, item similarity: 0.410272
[reid-5] id: 1, item similarity: 0.410272
[reid-5] id: 2, item similarity: 0.519591
[reid-5] id: 3, item similarity: 0.432213
[reid-5] [WARN] [1753675533.547124461] [reid_fet_manage]: Query failed, storage: 4.
[reid-5] [WARN] [1753675533.548487965] [reid_fet_manage]:
[reid-5] id: 0, item similarity: 0.480041
[reid-5] id: 1, item similarity: 0.480041
[reid-5] id: 2, item similarity: 0.369116
[reid-5] id: 3, item similarity: 0.475912
[reid-5] id: 4, item similarity: 0.568766
[reid-5] [WARN] [1753675533.565484010] [reid_fet_manage]: Query failed, storage: 5.
[mono2d_body_detection-3] [WARN] [1753675534.191426925] [mono2d_body_det]: input fps: 11.68, out fps: 11.71, infer time ms: 85, post process time ms: 2
[reid-5] [WARN] [1753675534.696986895] [reid_node]: input fps: 4.00, out fps: 4.15, infer time ms: 240, post process time ms: 0
[mono2d_body_detection-3] [WARN] [1753675536.166313555] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1088, w: 1920, step: 1920, index: 31, stamp: 1753675536_158953660, data size: 3133440, comm delay [7.3511]ms
```

示例中推理的结果会渲染到Web上, 在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）, 其中每个人物上的id为实例ID结果。

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_reid.png)