---
sidebar_position: 1
---

# RDK X系列历史发布

## RDK X5

> **注意事项：**  
> - 更新系统后，请使用 `rdk-miniboot-update` 命令更新 NAND 固件至最新版本。  
> - 官方会定期发布新版镜像，这些镜像包含了最新的功能优化和问题修复。用户既可以通过下载最新镜像进行安装，也可以直接通过在线升级方式获取系统更新。

### 版本号：3.3.3

#### 版本信息

- **系统版本**：RDKOS V3.3.3
- **发布日期**：2025 年 10 月
- **适配平台**：RDK X5

#### 版本更新概述

**（1）系统与驱动更新**

- **新增 御光-SC132GS 双目摄像头模组驱动**
- **srpi-config 功能增强**：支持接口自动管脚复用，新增 MIPI 屏幕选择
- **GPU 桌面稳定性 提升**
- **Wi-Fi 驱动 升级至 2025_0410**，提升无线连接稳定性
- **闪连接口增强**：
  - 支持在 macOS 系统 上使用
  - 新增 RDK Studio 链接 U 盘功能
- **多项 Bug 修复与性能优化**

**（2）文档优化**

- **V4L2 使用文档 优化**，补充示例与说明
- **显示屏使用文档 优化**，完善典型屏幕配置说明
- **其他文档细节优化与结构调整**

**（3）生态支持**

以下生态库已同步发布至 PyPI：

| 库名称 | 版本号 | 更新内容 |
|--------|--------|----------|
| hobot-dnn-rdkx5 | 3.0.6 | 增加依赖 numpy >= 1.26.4 |
| hobot-vio-rdkx5 | 3.0.6 | 修复裁剪不生效问题；sp_open_camera 接口默认出图尺寸为 1920×1080 |

#### TROS 更新说明（V2.4.3）

- **单目 MIPI 图像采集**：支持启动多路图像采集
- **双目 MIPI 图像采集**：新增对 SC132GS 双目相机 的支持

#### 获取方式

RDK X5 V3.3.3 版本已同步至：

- 地瓜机器人开发者社区下载中心，下载后升级
- `sudo apt update && sudo apt upgrade`；直接本地升级

---

### TROS算法升级 & 版本号：3.2.3

#### 镜像更新

- **20250610**  
  - 更新 ROS 仓库 GPG 签名密钥  
  - 支持[实时内核切换](../Advanced_development/linux_development/realtime_kernel#x5系列板卡)  
  - 集成最新版 miniboot 固件，使用 `rdk-miniboot-update` 命令烧录 NAND 后，会释放更多内存给系统
- **20250604**  
  - 修复配置幻尔载板系统启动失败的问题

#### 系统层更新

- **桌面显示优化**：Ubuntu 桌面支持 3D GPU 加速渲染，视觉体验更流畅
- **音频子板适配新增**：支持 微雪 WM8960 Audio HAT 与 幻尔载板，加快语音方案集成效率
- **WIFI 驱动升级**：增强弱信号环境下连接稳定性，优化休眠/唤醒过程中的自动重连机制
- **Sensor采集框架支持扩展**：集成 V4L2 框架，已适配 imx477、ov5647、imx219，更多 Sensor 支持持续更新中
- **网络与远程优化**：默认切换至 iptables legacy 模式，VNC 显示流畅性提升，远程桌面不卡顿
- **接口增强**：CAN 接口稳定性优化，解决高速数据丢包问题。`srpi-config` 工具新增 Uart7 支持，提升串口扩展能力
- **存储兼容性优化**：提升对 SD 卡的兼容性，适配更多存储卡型号

#### 应用层更新

- **语音能力增强**：新增 ASR 语音识别方案，语音算法开发更高效
- **双目深度算法升级**：深度估计算法优化，检测速度与精度全面提升
- **多模态示例集成**：内置端侧大模型多模态示例，3 分钟快速构建自定义应用方案

---

### 双目算法升级 & 版本号：3.1.1

#### 核心功能亮点

- **系统备份**：全新 `rdk-backup` 工具，一键备份当前系统，轻松生成可烧录镜像。详见 [rdk-backup介绍](../Appendix/rdk-command-manual/cmd_rdk-backup)
- **配置管理**：支持使用 `config.txt` 配置 40pin 引脚在 U-Boot 阶段的初始化状态，提升系统启动稳定性
- **触摸屏增强**：新增双击和长按操作，长按模拟右键，屏幕控制更加灵活
- **设备树覆盖支持**：增加 1_wire 设备树覆盖（dtoverlay）示例，为定制硬件连接提供更多选择
- **双目算法升级**：StereoNet 深度算法升级，显著提升深度效果；新增 ZED 相机支持，实现双目图像采集，搭配 StereoNet 轻松构建智能视觉系统
- **全新应用方案：智能视频盒子**：`hobot_rtsp_client` 支持 RTSP 拉流、解码、智能推理，并通过 Web 界面展示推理结果，快速集成边缘 AI 解决方案
- **开放词汇检测方案：DOSOD**：`hobot_dosod` 基于地瓜自研的开放性词汇检测算法，提供端侧部署方案，让语音交互更灵活

#### 问题修复与优化

- 修复 efficientnasnet_m_300x300_nv12.bin 模型导入异常问题
- 优化 SD 卡协议支持，提升兼容性
- 修复竖屏显示导致的黑屏问题
- `dnn_node` 修复 YOLOv8-seg 后处理 box 越界崩溃问题
- `hobot_codec` 修复帧率计算错误
- `hobot_stereonet_utils` 删除无法启动的 launch 文件
- 解决多路 I2C 检测问题，增加 LPWM 开关配置

---

### 版本号：3.1.0

#### 新增功能

- 增加按键休眠和唤醒功能
- 开放 40Pin 第二功能

#### 优化改进

- 修复 BUG
- 修正 CAN 频率异常
- 支持更多 sensor 和分辨率

> **注意事项：**  
> 旧版本使用 `apt update && apt upgrade` 升级到该版本时，需要先卸载 `tros-humble-stereonet-model`，再安装 `tros-humble-hobot-stereonet` 包。
>
> ```shell
> sudo apt-get remove tros-humble-stereonet-model
> sudo dpkg --remove --force-all tros-humble-stereonet-model
> sudo apt install -y tros-humble-hobot-stereonet
> ```

---

### 版本号：3.0.1

#### 新增功能

- 提供 Server 版本固件
- 支持 7 款微雪 MIPI DSI LCD 屏幕的桌面显示和触控
- 支持非 root 用户执行示例程序

#### 优化改进

- 修复 BUG
- 支持更多 sensor 和分辨率
- 优化高分辨率显示稳定性

---

### 版本号：3.0.0

首次发布的 RDK X5 固件，基于 Ubuntu 22.04，提供丰富的多媒体示例、算法示例，支持多种应用场景的机器人应用开发。

---

## RDK X3

### 版本号：3.0.0

#### 新增功能

- 支持 Ubuntu 22.04

---

### 版本号：2.1.0

#### 新增功能

- 完善 `srpi-config` 系统配置工具，支持 Wi-Fi 连接，开/关 SSH、VNC，使能/禁用 40pin 上的外设总线，本地化语言配置，设置 CPU 超频，设置 ION 内存大小等系统配置
- 支持 `/boot/config.txt` 系统配置文件，支持设置 dtoverlay、CPU 超频、IO 启动状态配置等选项
- 添加 yolov5s v6/v7 模型示例

#### 优化改进

- 支持在 HDMI 显示器上输出启动日志和进入用户命令行，方便用户使用
- 支持更多的 HDMI 显示分辨率，极大增强兼容性
- 优化 Desktop 和 Server 版本的预装软件清单，删除冗余项，补充必要软件，如添加 VLC
- 优化 Desktop 菜单栏布局，精简选项
- 默认开启蓝牙功能
- 增加后处理的 C++ 接口，提高后处理效率
- 使用 udisk2 自动挂载 U 盘，解决 NTFS 文件系统自动挂载后不能访问的问题
- 支持用户保留 VNC 密码文件
- VNC 服务默认不自动开启，可减少系统资源消耗。用户可通过 `srpi-config` 工具打开
- RDK X3 v2.1 和 RDK Module 开发板 CPU 正常模式下最高运行在 1.5GHz，超频后最高 1.8GHz

#### 问题修复

- 删除 Wi-Fi 驱动的冗余内核日志
- 修改 apt 源域名为 archive.d-robotics.cc

#### 其他更新

- 支持 Chromium 浏览器，用户可通过 `sudo apt install chromium` 安装

---

### 版本号：2.0.0

本次发布带来了许多令人期待的功能和改进，旨在提供更好的开发体验和更广泛的应用场景支持。

#### 开放源代码

- 完全开放操作系统的源代码，包括系统核心模块和功能模块。开发者可自由查看和修改，为定制化和优化提供更大灵活性
- 提供详细代码文档和注释，帮助开发者理解和使用
- 欢迎开发者通过开源社区参与代码贡献和讨论，源码在 [D-Robotics](https://github.com/D-Robotics) 上维护

#### 支持 RDK X3 Module

- 引入全新核心板开发者套件 RDK X3 Module
- RDK X3 Module 拥有更小尺寸，兼容树莓派 CM4 接口
- 开发者可根据需求选择适配的第三方载板，扩展核心板功能和应用场景

#### 其他更新

- 优化已有功能，修复已知问题和漏洞，提升操作系统稳定性和性能
- 修订文档和帮助文档，提供更全面、准确的技术资料和指南
- 提供更低层 API，方便开发者进行二次开发和集成

---

### 版本号：1.0.0

1. `cat /etc/version` 显示 1.x.x
2. 不能使用 `rdkos_info` 命令
3. 老系统，无开源源代码，使用老手册，细小 bug 在新版本镜像中修复
4. 地瓜源域名和密钥有修改

## TogetheROS.Bot

### tros-humble

#### 版本号：2.4.0 (2025-05-12)

**新增功能：**

- 支持 `RDK S100` 平台。

#### 版本号：2.3.3 (2025-04-30)

**新增功能：**

- 支持 `RDK X5 Module` 平台。
- 新增基于 `sensevoice_cpp` 的 [ASR 开源方案](/docs/05_Robot_development/03_boxs/function/sensevoice_ros2.md)，支持命令词和 ASR 数据推送。
- [双目深度估计算法](/docs/05_Robot_development/03_boxs/function/hobot_stereonet.md) 优化后处理耗时，新增 V2.3 版本模型。
- 新增基于 `llama.cpp` 的端侧 [视觉语言模型](/docs/05_Robot_development/02_quick_demo/hobot_llamacpp.md) 算法示例。

#### 版本号：2.3.2 (2025-01-15)

**功能变更：**

- [双目深度估计算法](/docs/05_Robot_development/03_boxs/function/hobot_stereonet.md) 更新双目模型，优化深度估计效果。
- [多路视频分析](/docs/05_Robot_development/04_apps/video_boxs.md) 算法应用示例，优化处理流程及 WEB 端可视化效果。
- [双目辅助功能包](https://github.com/D-Robotics/hobot_stereonet_utils) 删除部分无法启动的 launch 文件。

**新增功能：**

- 新增 [ZED 相机图像采集](/docs/05_Robot_development/02_quick_demo/demo_sensor.md)，用于获取双目图像作为深度估计算法输入。
- 新增 [DOSOD 算法](/docs/05_Robot_development/03_boxs/function/hobot_dosod.md)，提供地瓜自研开放性词汇检测端侧部署功能包。

**问题修复：**

- 修复 [yolov8-seg 图像分割](/docs/05_Robot_development/03_boxs/segmentation/yolov8_seg.md) 算法后处理 box 越界导致的崩溃问题。
- [图像编解码](/docs/05_Robot_development/02_quick_demo/hobot_codec.md) 修复帧率统计错误。
- [双目 MIPI 图像采集](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) 修复 i2c detection 问题，增加 lpwm 开关配置。

#### 版本号：2.3.1 (2024-11-20)

**功能变更：**

- 依赖的 `opencv` 版本由 3.4.5 升级到 4.5.4（Ubuntu 22.04 最新 release）。

**新增功能：**

- [图像发布工具](/docs/05_Robot_development/02_quick_demo/demo_tool.md) 支持发布 `bgr/rgb` 格式消息及配置 frame_id。
- [人体检测和跟踪算法](/docs/05_Robot_development/03_boxs/function/mono2d_body_detection.md) 支持 topic 配置、component 模式、图片缩放推理、压缩图片回灌等。
- [板端算法模型推理与部署框架](https://github.com/D-Robotics/hobot_dnn.git) 修复多线程推理耗时计算错误，支持配置任务数。
- [图像编解码 Node](/docs/05_Robot_development/02_quick_demo/hobot_codec.md) 支持 frame_id 传递及丢帧控制。
- [手势识别算法](/docs/05_Robot_development/03_boxs/function/hand_gesture_detection.md) 支持后处理阈值配置及动态手势识别。
- 新增 [人脸年龄检测算法](/docs/05_Robot_development/03_boxs/function/mono_face_age_detection.md)。
- 新增 [人脸 106 关键点检测算法](/docs/05_Robot_development/03_boxs/function/mono_face_landmarks_detection.md)。
- 新增 [感知消息融合 Node](https://github.com/D-Robotics/tros_perception_fusion)，支持多 topic 感知结果融合。
- 新增 [感知消息滤波 Node](https://github.com/D-Robotics/tros_lowpass_filter)，采用 OneEuroFilter 对检测框/关键点平滑。
- 新增 [双目辅助功能包](https://github.com/D-Robotics/hobot_stereonet_utils)。
- 新增 [多路视频分析](/docs/05_Robot_development/04_apps/video_boxs.md) 算法应用示例。

**问题修复：**

- [MIPI 图像采集](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) 修复 `imx219` 模组启动失败。
- [人手关键点检测算法](/docs/05_Robot_development/03_boxs/function/hand_lmk_detection.md) 前处理增加人手框外扩，修正关键点输出。

#### 版本号：2.3.0 (2024-09-19)

**新增功能：**

- 支持 `RDK X5` 平台。
- 数据采集新增 [双目 MIPI 图像采集](/docs/05_Robot_development/02_quick_demo/demo_sensor.md)。
- 算法仓库新增 `yolov8`、`yolov10` [目标检测](/docs/05_Robot_development/03_boxs/detection/yolo.md)，`yolov8-seg` [图像分割](/docs/05_Robot_development/03_boxs/segmentation/yolov8_seg.md)。
- 新增 [YOLO-World 算法](/docs/05_Robot_development/03_boxs/function/hobot_yolo_world.md)、[光流估计算法](/docs/05_Robot_development/03_boxs/function/mono_pwcnet.md)、[分割一切算法](/docs/05_Robot_development/03_boxs/function/mono_mobilesam.md)、[文本图片特征检索算法](/docs/05_Robot_development/03_boxs/function/hobot_clip.md)、[双目深度估计算法](/docs/05_Robot_development/03_boxs/function/hobot_stereonet.md)。

#### 版本号：2.2.0 (2024-04-11)

**功能变更：**

- 基于 TROS Foxy 2.1.3，适配 Ubuntu 22.04 和 ROS2 Humble。
- TROS 安装路径调整为 `/opt/tros/humble`。
- 不再提供 `tros-ros-base` 安装包，直接依赖标准 ROS2 发行包。
- 使用 ROS2 fastdds 零拷贝通信，QoS Reliability 改为 `BEST_EFFORT`。
- `hobot_dnn`、`hobot_audio` 重构为基于 `libdnn`。
- `hobot_trigger` 适配 ROS2 Humble 版本 rosbag2。

**新增功能：**

- `robot_dev_config` 新增 bloom 编译打包脚本。
- `hobot_mipi_cam` node 新增 frame_ts_type 配置项。
- 新增 `hobot_shm` node 配置 ROS2 零拷贝环境。

**问题修复：**

- 修复编译器升级兼容性问题。
- 修复板端编译部分 ROS2 pkg 路径依赖问题。

---

### tros-foxy

#### 版本号：2.1.3 (2024-03-11)

**功能变更：**

- jpeg 压缩图片类型由 `sensor_msgs::msg::Image` 改为 `sensor_msgs::msg::CompressedImage`，支持 foxglove/ros2 rqt 工具查看。
- 统一使用 jpeg/mjpeg 配置项，删除 jpeg-compressed/mjpeg-compressed 配置项。
- 新增环境变量 `TROS_DISTRO`，配置文件路径调整为 `/opt/tros/${TROS_DISTRO}/lib`。

#### 版本号：2.1.2 (2024-01-19)

**新增功能：**

- 重构 `hobot_usb_cam`，支持更多 format 配置和转码。
- `hobot_audio` 更新语音 SDK，支持 2mic/4mic 麦克风板，增加 micphone_name 配置。

**问题修复：**

- `hobot_rgbd_cam` node 修复 step 字段设置错误。
- `hobot_tts` 修复音频播放失败问题。
- `hobot_llm` 删除 config 设备树文件，更新 README，支持命令工具设置 ION 内存。

#### 版本号：2.1.1 (2023-11-03)

**新增功能：**

- 新增 `hobot_chatbot` node，支持板端语音聊天。

**问题修复：**

- `hobot_tts` node 修复特殊字符导致退出问题。

#### 版本号：2.1.0 (2023-09-14)

**功能变更：**

- `tros-ros-base` 升级到最新 ROS2 foxy 源码。
- 使用 ROS2 foxy 只需 `source /opt/tros/setup.bash`。

**新增功能：**

- `hobot_tts` node 新增音频设备参数。
- 新增 `hobot_llm` node，支持端侧 LLM。
- `hobot_codec` node 新增 `jpeg-compressed` 配置项。

**问题修复：**

- `hobot_mipi_cam` node 修复 RGB 格式 step 字段错误。

#### 版本号：2.0.2 (2023-08-28)

**功能变更：**

- ROS2 源更换为清华镜像源。

**新增功能：**

- tros.b 脚本新增权限检查，自动切换 root。
- `hobot_audio` node 新增音频设备号参数。
- `hobot_trigger` node 新增 std_msg 话题触发功能。

**问题修复：**

- `hobot_cv` node 修复 crop&resize 同时处理失败问题。
- `hobot_mipi_cam` node 启动时 error log 问题。
- `hobot_visualization` node launch 配置无效问题。

#### 版本号：2.0-Release（2.0.1） (2023-06-10)

**功能变更：**

- 升级语音算法，优化 ASR 效果。
- 算法示例 `model_name` 配置自动解析。
- tros.b 安装包不再包含 nav2，建议使用 apt 安装最新版 nav2。

**新增功能：**

- 支持 `RDK Ultra` 平台。
- 新增 Trigger 事件触发、rosbag 数据可视化 node。
- USB 图像采集 node 支持设备号自适应。
- 新增 VIO 算法 node。
- 新增 `hobot_tts` node。
- 新增 `hobot_centerpoint`、`hobot_bev`、`hobot_stereonet` node。

**问题修复：**

- 升级 `RDK X3` easydnn/dnn，修复算子 crash。
- 修复 RGBD 图像采集 node 深度数据错误。

**其他更新：**

- 优化人体检测/跟踪算法 node，支持分辨率自适应。
- 修复 orb_slam3 编译脚本路径错误。

#### 版本号：2.0-Beta（2.0.0） (2023-05-29)

2.0-Beta（2.0.0）为第一个 2.x 版本，建议 1.x 用户升级。

**功能变更：**

- 代码托管平台迁移至 GitHub。
- 集成高效包管理机制。

**新增功能：**

- 支持 RDK X3 Module。
- `hobot_audio` 增加 ASR 识别结果输出。

**问题修复：**

- 修复 dnn_node MobileNet_SSD 多线程崩溃。
- 修复 X86 平台 dnn_node、hobot_codec、hobot_image_publisher 编译失败。

**其他更新：**

- 更新示例 launch 脚本。
- websocket 展示端更新 D-Robotics logo。

---

### TogetheROS-V1.x

#### V1.1.6

- 新增 X86 平台支持，包括 Hobot_DNN、Hobot_codec 等模块。
- 修复音频 codec 驱动自动加载导致的 I2C 占用问题。
- 修复 USB 摄像头 Video 设备节点变化问题。

#### V1.1.5

- 新增 hobot_audio 支持线形/环形麦克风配置。
- 优化 hobot_sensors MIPI Camera 时间戳精度。
- 新增 hobot_image_publisher h265 编码测试视频。
- 修复 TROS 安装包路径依赖问题。
- 修复系统迭代更新后用户文件被覆盖问题。

#### V1.1.4

- 优化 hobot_sensors，支持多种模组。
- 新增 VIO raw/yuv 格式图片接口。
- 新增 hobot_image_publisher MP4/H264/H265 视频发布。
- 修复 hobot_dnn 部分算法后处理无返回值。
- 新增 hobot_cv benchmark 工具。
- 优化工具链模块，新增 bug 修复指导。
- 修复 USB3.0 设备和 HDMI 采集卡兼容问题。

#### V1.1.3

- 优化工具链，一键安装实现模型量化。
- 新增 rtsp 拉流解码 C 语言示例。
- 新增 hobot_cv 图片 padding 与 nv12 处理。
- 优化 hobot_codec 输出日志。
- 优化 hobot_sensor，新增多款摄像头支持。

#### V1.1.2

- 新增板端 docker 支持。
- 新增 2D 垃圾检测算法示例。
- 增加 C/C++ 多媒体与算法开发接口。
- 新增 dnn_node_sample package 使用示例。
- 新增自适应多核模型推理。
- 新增感知结果置信度信息。

#### V1.1.1

- 优化视觉、图像模块报错提示。
- 优化智能语音算法示例。
- 新增声源定位 DOA 应用示例。
- 集成 Navigation2 源码。
- 用户手册新增 AI 工具链说明。

#### V1.1.0

- 开放内核驱动编译环境。
- 优化 TF 卡兼容性。
- 新增小车巡线、自动泊车场景应用示例。
- 新增多款摄像头适配。
- 新增舜宇 RGBD 模组适配。
- hobot_codec 修复异常图片崩溃。
- 支持输出帧率控制。
- 新增 EDID 检测、自适应分辨率。
- 新增 joystick 遥控手柄支持。
- 新增 xfs/ntfs 文件系统支持。

#### V1.0.5

- 新增泊车场景算法示例。
- 新增本地图片发布工具。
- 新增 hobot_cv neon 加速滤波功能。
- hobot_dnn 修复推理输出帧率错误。
- 支持人体检测等算法摄像头类型选择。
- hobot_codec 修复图片 step 参数错误。
- 系统软件新增 restart_network 命令。

#### V1.0.4

- 新增 ORB-SLAM3 并优化。
- hobot_dnn 修复推理异常、升级预测库。
- hobot_cv 支持 pyramid 输出。
- 系统软件新增 rtsp 视频流硬解码、Python API。
- 修复 AI 模型加载 I2C 报错、python pwm 设置周期问题、spi 只写/只读问题。
- 修复以太网和 wifi 同时启动报错。
- 新增 4 路麦克风音频子板说明。

#### V1.0.3

- 新增 ROS2 vision_opencv 适配。
- hobot_audio 优化语音识别，默认唤醒词更换。
- hobot_cv 新增 crop/resize/rotate 接口。
- hobot_sensor 优化 NV12 转 RGB 效率。
- hobot_msgs 优化关键点信息，新增 audio_msg。

#### V1.0.2

- 新增 GC4663 mipi camera、RealSense camera 适配。
- 优化 AI 示例，支持 usb camera 多分辨率。
- 优化首次开机启动时间。
- 修复 Desktop 版本黑屏问题。
- mono2d_body_detection 新增关键点置信度。
- audio_control 新增语音控制机器人运动。
- hobot_audio 新增语音识别功能。
- hobot_websocket 优化 web 显示界面。
- hobot_msgs 优化关键点信息，新增 audio_msg。

#### V1.0.1

- 更新 hobot-arm64-modules，修复 Desktop 鼠标显示问题。
- HobotDNN 优化异步推理性能。
- HobotCV 优化 Log 输出。
- HobotSensor 修复 rgbd sensor 发布消息失败。
- HobotWebsocket 修复内存泄漏。
- HobotCodec 优化编解码 CPU 占用。
- 新增 HobotHDMI 模块。
- mono2d_body_detection 优化性能统计与 Log。
- hand_lmk_detection 修复内存泄漏与推理性能。
- hand_gesture_detection 修复多手检测问题并优化性能。


