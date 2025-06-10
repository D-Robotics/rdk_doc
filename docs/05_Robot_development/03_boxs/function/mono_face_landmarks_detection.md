---
sidebar_position: 18
---

# 人脸106关键点检测

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

**人脸106关键点检测示例**订阅图片和包含人脸框信息的智能msg，利用BPU进行算法推理，发布包含人脸106关键点信息的算法msg。

代码仓库：(https://github.com/D-Robotics/face_landmarks_detection)

## 支持平台

| 平台                    | 运行方式                  | 示例功能                         |
|-----------------------|-----------------------|------------------------------|
| RDK X3, RDK X3 Module | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头，并通过Web展示推理渲染结果 |
| RDK X5, RDK X5 Module                | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头，并通过Web展示推理渲染结果 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

**人脸106关键点检测(face_landmarks_detection)package**订阅sensor package发布的图片以及人体检测和跟踪package发布的*
*人脸框检测结果**，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。

**使用MIPI摄像头发布图片**

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch face_landmarks_detection body_det_face_landmarks_det.launch.py
```

**使用USB摄像头发布图片**

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

```shell
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch face_landmarks_detection body_det_face_landmarks_det.launch.py
```

## 结果分析

在运行终端输出如下信息：

```shell
[mono2d_body_detection-3] [WARN] [1731988336.541394391] [example]: This is mono2d body det example!
[face_landmarks_detection-5] [WARN] [1731988336.637554206] [face_landmarks_det_node]: => face_landmarks_det_node params:
[face_landmarks_detection-5] => feed_type: 0
[face_landmarks_detection-5] => is_sync_mode: 0
[face_landmarks_detection-5] => model_file_name: /root/zhikang.zeng/work_humble_ws_x5/tros_ws/install/share/face_landmarks_detection/config/faceLandmark106pts.hbm
[face_landmarks_detection-5] => is_shared_mem_sub: 1
[face_landmarks_detection-5] => dump_render_img: 0
[face_landmarks_detection-5] => ai_msg_pub_topic_name: /hobot_face_landmarks_detection
[face_landmarks_detection-5] [INFO] [1731988336.638429674] [dnn]: Node init.
[face_landmarks_detection-5] [INFO] [1731988336.638482188] [face_landmarks_det_node]: => Set node para.
[face_landmarks_detection-5] [INFO] [1731988336.638589050] [dnn]: Model init.
[mono2d_body_detection-3] [WARN] [1731988336.641041791] [mono2d_body_det]: Parameter:
[mono2d_body_detection-3]  is_sync_mode_: 0
[mono2d_body_detection-3]  model_file_name_: config/multitask_body_head_face_hand_kps_960x544.hbm
[mono2d_body_detection-3]  is_shared_mem_sub: 1
[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection
[mono2d_body_detection-3]  ros_img_topic_name: /image_raw
[mono2d_body_detection-3]  image_gap: 1
[face_landmarks_detection-5] [BPU_PLAT]BPU Platform Version(1.3.6)!
[face_landmarks_detection-5] [HBRT] set log level as 0. version = 3.15.54.0
[face_landmarks_detection-5] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[mono2d_body_detection-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono2d_body_detection-3] [HBRT] set log level as 0. version = 3.15.54.0
[mono2d_body_detection-3] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
```

输出log显示，程序运行成功，推理时算法输入和输出帧率为30fps，每秒钟刷新一次统计帧率。

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/face_landmarks_det_render.png)

