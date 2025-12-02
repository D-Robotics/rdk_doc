---
sidebar_position: 0
---

# TogetheROS.Bot 简介
TogetheROS.Bot是D-Robotics面向机器人厂商和生态开发者推出的机器人操作系统，旨在释放机器人场景的智能潜能，助力生态开发者和商业客户能够高效、便捷的进行机器人开发，打造具有竞争力的智能机器人产品。

TogetheROS.Bot支持在RDK平台上运行，同时提供模拟器版本在X86平台上运行。RDK平台涵盖下图所示的全部功能，X86平台支持以图片回灌方式体验部分功能，提高用户算法开发和验证效率，并能够快速迁移到RDK平台。

![TROS-Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/image/TogetheROS.png)

TogetheROS.Bot源码托管在GitHub [D-Robotics组织](https://github.com/D-Robotics)。

## Communication通信组件

Communication是在ROS2 Foxy/Humble/Jazzy版本通信核心组件基础上进行的功能优化和扩展。

主要特性如下：

其中蓝色部分为优化、新增模块，TogetheROS.Bot主要特性如下：

- 提供“hobot_sensor”适配机器人常用传感器，节省开发时间，聚焦核心竞争力
- 提供“hobot_dnn”简化板端算法模型推理与部署，释放BPU算力，降低智能算法使用门槛
- 提供“hobot_codec”软硬结合加速视频编解码，节省CPU资源，提升并行处理能力
- 提供“hobot_cv”软硬结合提升常见CV算子性能，节省CPU资源，提升运行效率
- 提供“hobot Render”Web端和HDMI动态可视化功能，实时渲染算法结果（仅限Web端），便于展示与调试
- 增加“zero-copy”进程间零拷贝通信机制，降低数据传输时延，减少系统资源消耗
- 丰富中间件软件调试以及性能调优工具，提升问题定位效率，方便系统性能优化
- 与ROS2 Foxy/Humble/Jazzy版本接口保持完全兼容，便于复用ROS丰富工具包，加快原型验证
- 支持最小化和模块化剪裁，方便根据需要部署在资源受限的嵌入式产品中

## Boxs算法仓库

Boxs是D-Robotics面向机器人厂商和生态开发者推出的基于TogetheROS.Bot的智能算法包，旨在提升基于D-Robotics RDK机器人操作系统进行机器人智能算法集成和落地的效率。

- 图像检测算法如FCOS、YOLO、FasterRCNN、Efficientdet、Mobilenet_ssd;
- 图像分类模型如Mobilenet
- 语义分割模型如Unet
- 应用算法模型如人体检测与跟踪、手势识别、人手关键点检测、单目高程网络、单目3D检测、语音处理等

## Apps应用示例

Apps是基于D-Robotics RDK机器人操作系统Communication和Boxs开发的算法应用示例，旨在打通图像输入、感知、策略等完整链路，展示应用效果，加速客户demo开发效率。

## 常见名词解释

| 名词                              | 含义                                                    |
| ----------------------------------| --------------------------------------------------------|
| zero-copy                         | 进程间零拷贝通信方式                                     |
| hobot dnn                         | 基于BPU的模型推理功能封装                                |
| SLAM                              | 定位与地图构建                                          |
| DOA                               | 声源定位                                                |
| ASR                               | 自动语音识别                                            |
| TogetheROS.Bot                    | TogetheROS.Bot机器人操作系统                            |
| tros.b                            | TogetheROS.Bot缩写                                      |


## 功能支持列表

| 功能 | X3 | X5 | S100 | S600 |
|----------------|----------------|----------------|----------------|----------------|
| 数据采集 [hobot_sensor](./02_quick_demo/demo_sensor.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 模型推理 [hobot_dnn](./02_quick_demo/ai_predict.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 图像编解码 [hobot_codec](./02_quick_demo/hobot_codec.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 图像处理加速 [hobot_cv](./02_quick_demo/demo_cv.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 数据展示 [hobot_render](./02_quick_demo/demo_render.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 数据通信 [zero-copy](./02_quick_demo/demo_communication.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 智能语音 [hobot_audio](./03_boxs/audio/hobot_audio.md)以及语音相关示例 | &#10004; | &#10004; | &#10006; | &#10006; |
| 目标检测 | [YOLO](./03_boxs/detection/yolo.md): v2 v3 v5 v8 v10 <br /> [FCOS](./03_boxs/detection/fcos.md) <br /> [MobileNet_SSD](./03_boxs/detection/mobilenet.md) <br /> [EfficientNet_Det](./03_boxs/detection/efficientnet.md) | [YOLO](./03_boxs/detection/yolo.md): v2 v3 v5 v8 v10 <br /> [FCOS](./03_boxs/detection/fcos.md) <br /> [MobileNet_SSD](./03_boxs/detection/mobilenet.md) <br /> [EfficientNet_Det](./03_boxs/detection/efficientnet.md) | [YOLO](./03_boxs/detection/yolo.md): v2 v3 v5 v8 v10 | [YOLO](./03_boxs/detection/yolo.md): v2 v3 v5 v8 v10 |
| 开放词汇目标检测 [YOLO-World](./03_boxs/detection/hobot_yolo_world.md) | &#10006; | &#10004; | &#10006; | &#10006; |
| 开放词汇目标检测 [DOSOD](./03_boxs/detection/hobot_dosod.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 图像分类 [mobilenetv2](./03_boxs/classification/mobilenetv2.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 图像分割 [mobilenet_unet](./03_boxs/segmentation/mobilenet_unet.md) [YOLOv8-Seg](./03_boxs/segmentation/yolov8_seg.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| 分割一切 [mono_edgesam](./03_boxs/segmentation/mono_edgesam.md) | &#10006; | &#10004; | &#10004; | &#10004; |
| 分割一切 [mono_mobilesam](./03_boxs/segmentation/mono_mobilesam.md) | &#10006; | &#10004; | &#10006; | &#10006; |
| [人体检测](./03_boxs/body/mono2d_body_detection.md)、[人手关键点](./03_boxs/body/hand_lmk_detection.md)、[手势识别](./03_boxs/body/hand_gesture_detection.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| [人脸年龄检测](./03_boxs/body/mono_face_age_detection.md)、[人脸106关键点检测](./03_boxs/body/mono_face_landmarks_detection.md)，以及对应APP示例 | &#10004; | &#10004; | &#10006; | &#10006; |
| [人体跟随](./03_boxs/body/reid.md) | &#10006; | &#10004; | &#10004; | &#10004; |
| [BEV](./03_boxs/driver/hobot_bev.md) | &#10006; | &#10004; | &#10004; | &#10006; |
| 激光雷达目标检测算法[CenterPoint](./03_boxs/driver/hobot_centerpoint.md) | &#10006; | &#10004; | &#10004; | &#10006; |
| [双目深度算法](./03_boxs/spatial/hobot_stereonet.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| [双目OCC算法](./03_boxs/spatial/dstereo_occupancy.md) | &#10006; | &#10004; | &#10004; | &#10004; |
| 视觉惯性里程计 [hobot_vio](./03_boxs/spatial/hobot_vio.md) | &#10006; | &#10004; | &#10006; | &#10006; |
| 文本图片特征检索 [hobot_clip](./03_boxs/function/hobot_clip.md) | &#10006; | &#10004; | &#10004; | &#10006; |
| 光流估计 [mono_pwcnet](./03_boxs/function/mono_pwcnet.md)  | &#10006; | &#10004; | &#10006; | &#10006; |
| [2D Lidar SLAM](./04_apps/slam.md) <br /> [Navigation2](./04_apps/navigation2.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| [智能盒子](./04_apps/video_boxs.md) | &#10004; | &#10004; | &#10004; | &#10004; |
| [视觉语音盒子](./04_apps/hobot_llamacpp.md) | &#10006; | &#10004; | &#10004; | &#10004; |
