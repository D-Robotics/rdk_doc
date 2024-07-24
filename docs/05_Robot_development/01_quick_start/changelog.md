---
sidebar_position: 6
---

# 1.6 版本发布记录

## tros-humble

### 版本号：2.2.0

功能变更：

- 基于TROS Foxy 2.1.3版本，适配Ubuntu 22.04系统和ROS2 Humble。
- TROS的安装路径由**`/opt/tros`**变更为**`/opt/tros/humble`**，和ROS2的安装路径层级和命名保持一致。
- 不再提供`tros-ros-base`安装包（包含rclcpp、rclpy、ros2cli等ROS2基础功能包），使用标准的ROS2发行包，安装TROS Humble时自动安装依赖的ROS2 Humble。
- 使用ROS2 fastdds的零拷贝通信功能，涉及到数据采集、图像编解码、算法示例等使用到图像数据的模块。
- 零拷贝通信使用的QoS的Reliability由`RMW_QOS_POLICY_RELIABILITY_RELIABLE`（rclcpp::QoS()）变更为`RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`（rclcpp::SensorDataQoS()），避免使用零拷贝时潜在的稳定性风险。
- 重构`hobot_dnn`，使用更底层的板端推理框架`libdnn`，不再使用`easydnn`。
- `hobot_audio`升级语音算法SDK，使用更底层的板端推理框架`libdnn`，不再使用`easydnn`。
- `hobot_trigger`适配ROS2 Humble版本rosbag2。

新增功能：
- `robot_dev_config`新增bloom编译和打包的脚本，用于ARM平台编译和打包TROS。
- `hobot_mipi_cam` node新增frame_ts_type配置项，支持realtime（用于计算通信延迟）和sensor（默认，用于传感器的时间戳同步）配置参数。
- 新增`hobot_shm` node，用于配置ROS2零拷贝环境。

问题修复：
- 修复编译器升级引入的兼容性问题。
- 修复板端编译部分ROS2 pkg存在的路径依赖问题。

## tros-foxy

### 版本号：2.1.3

功能变更：

- jpeg压缩格式图片使用的数据类型由`sensor_msgs::msg::Image`变更为标准的`sensor_msgs::msg::CompressedImage`，支持使用foxglove和ros2 rqt等工具查看TROS发布的jpeg格式图片。涉及到hobot_websocket, hobot_codec, hobot_image_publisher, hobot_usb_cam模块。
- 统一使用jpeg/mjpeg配置项指定发布/订阅jpeg压缩格式图片，删除jpeg-compressed/mjpeg-compressed配置项，涉及到hobot_codec和hobot_usb_cam模块。
- 引入表示TROS发行版的环境变量TROS_DISTRO，执行`source /opt/tros/setup.bash`/`source /opt/tros/local_setup.bash`命令后，环境变量`TROS_DISTRO`的值为空。hobot_codec, hobot_audio, hobot_mipi_cam, hobot_usb_cam等模块使用的配置文件路径由`/opt/tros/lib`变更为`/opt/tros/${TROS_DISTRO}/lib`。


### 版本号：2.1.2

新增功能：

- 重构`hobot_usb_cam`，支持更多format配置和转码。
- `hobot_audio`更新语音SDK，同时支持2mic和4mic麦克风板;增加micphone_name配置设备ID号。

问题修复：

- `hobot_rgbd_cam` node修复发送数据消息step字段设置错误问题。
- `hobot_tts`更新音频播放函数调用，解决新版本系统播放失败问题。
- `hobot_llm`删除config设备树文件，以及更新README，新版本系统可通过命令工具设置ION内存大小。

### 版本号：2.1.1

新增功能：

- 新增`hobot_chatbot` node，调用智能语音、大语言模型、文本转语音模块，实现板端语音聊天功能。

问题修复：

- 文本转语音`hobot_tts` node，修复某些字符导致应用退出问题。

### 版本号：2.1.0

功能变更：

- `tros-ros-base`更新到最新ROS2 foxy源码，兼容最新ROS2 foxy软件包。
- 使用ROS2 foxy软件包只`source /opt/tros/setup.bash`即可，不再需要使用脚本建立软链接。

新增功能：

- 文本转语音`hobot_tts` node新增参数指定播放音频设备。
- 新增大语言模型`hobot_llm` node，可在端侧体验LLM。
- 图像编解码`hobot_codec` node配置参数`in_format`新增`jpeg-compressed`配置项，同时根据配置项选择订阅的话题数据类型。

问题修复：

- MIPI图像采集`hobot_mipi_cam` node修复发送RGB格式数据消息step字段设置错误问题。


### 版本号：2.0.2

功能变更：

- tros.b安装时配置的ROS2源（`/etc/apt/sources.list.d/ros2.list`）变更为清华镜像源，解决安装ROS2 package速度慢和失败的问题。

新增功能：

- 启动tros.b脚本配置环境时（`source /opt/tros/setup.bash`和`source /opt/tros/local_setup.bash`）新增权限检查的功能。如果当前账户不具有root权限将会自动进入切换到root账户的流程，解决因为权限不够导致的使用tros.b失败的问题。
- 智能语音算法`hobot_audio` node新增音频设备号参数配置功能，方便二次开发​。
- 事件触发`hobot_trigger` node新增通过std_msg话题给Trigger模块发放任务功能，规范Trigger配置方法。

问题修复：

- 修复图像加速处理`hobot_cv` node同时进行crop&resize处理图像时，处理失败的问题。
- 修复MIPI图像采集`hobot_mipi_cam` node启动时输出error log的问题。
- 修复数据可视化消息转换`hobot_visualization` node的launch启动文件配置无效的问题。


### 版本号：2.0-Release（2.0.1）

功能变更：

- 升级语音算法，优化ASR（语音识别）效果。
- 优化算法示例的`model_name`配置项，从模型文件中自动解析`model_name`配置，解决参数配置错误导致的加载模型失败问题，提升算法二次开发的易用性。
- tros.b安装包中不再包含nav2功能包，用户直接在RDK上使用apt命令安装ROS2最新版本的nav2功能包，解决老版本nav2存在的稳定性问题。

新增功能：

- 新增支持`RDK Ultra`平台。
- 新增Trigger事件触发并获取和可视化rosbag数据的`hobot_trigger`和`hobot_visalization`等node，帮助用户定位、复现和可视化机器人场景中的感知、规控等问题。同时用户可以二次开发实现数据触发、录制和实时回传的功能。
- USB图像采集node自适应USB摄像头的设备号，降低用户使用USB摄像头的门槛。
- 新增视觉惯性里程计（Visual Inertial Odometry，VIO）算法node，基于视觉实现低成本、鲁棒性高的机器人高精度定位算法。
- 新增文本转语音的`hobot_tts` node，实现将文本转化为语音进行播报的功能。
- 新增激光雷达目标检测算法`hobot_centerpoint` node。
- 新增BEV感知算法`hobot_bev` node。
- 新增双目深度估计算法`hobot_stereonet` node。

问题修复：

- 升级`RDK X3`的easydnn（版本号1.6.1）和dnn（版本号1.18.4），修复算子crash问题以及支持更多算子。
- 修复RGBD图像采集node发布的深度数据错误的问题。

其他更新：

- 优化人体检测和跟踪算法node，支持根据输入图像分辨率自适应输出的算法感知结果坐标。
- 修复orb_slam3算法编译脚本路径错误导致的编译失败问题。


### 版本号：2.0-Beta（2.0.0）

2.0-Beta（2.0.0）是第一个2.x版本tros.b，建议[1.x版本tros.b](https://developer.horizon.cc/api/v1/fileData/TogetherROS/index.html)的用户升级到2.x版本。

功能变更：

- 代码托管平台从Gitlab更换为GitHub，方便更多开发者进行二次开发。
- 集成更高效的包管理机制，加快版本升级效率，让机器人应用安装更加便捷。

新增功能：

- 支持全新的核心板开发套件RDK X3 Module。
- hobot_audio增加语音ASR识别结果输出，方便用于开发语音应用。

问题修复：

- 修复dnn_node内置的MobileNet_SSD模型后处理在多线程情况下崩溃问题。
- 修复X86平台下dnn_node使用DDR输入模型推理失败问题
- 修复X86平台下hobot_codec和hobot_image_publisher编译失败问题。

其他更新：

- 更新示例的launch启动脚本，应用引用依赖模块的launch脚本并配置参数。
- webscoket更新展示端的地平线logo。