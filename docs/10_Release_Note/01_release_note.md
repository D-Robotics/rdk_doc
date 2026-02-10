---
sidebar_position: 1
---

# RDK X系列历史发布

## RDK X5


> **注意事项：**  
> - 更新系统后，请使用 `rdk-miniboot-update` 命令更新 NAND 固件至最新版本。  
> - 官方会定期发布新版镜像，这些镜像包含了最新的功能优化和问题修复。用户既可以通过下载最新镜像进行安装，也可以直接通过在线升级方式获取系统更新。

### 版本号：3.4.1

#### 版本信息

- **系统版本**：RDKOS V3.4.1
- **发布日期**：2025 年 12 月
- **适配平台**：RDK X5

#### 版本更新概述

本次 RDK OS 版本已全面对齐并同步最新底层平台软件 X5 SDK 1.1.1。相比上一版本（基于 X5 SDK 1.0.17），此次更新融合了过去半年内的功能增强、稳定性修复与性能优化。在完成适配、验证与系统裁剪后，各项核心能力均已完整集成至 RDK OS。
在延续 RDK Ubuntu 22.04 与 TROS 机器人操作系统 体验的基础上，本版本为开发者提供了与最新 SDK 保持一致的更完善的平台支持与一致的运行体验。

**（1）系统与启动稳定性**

- **EMMC/SD 启动可靠性增强**：适配多品牌 SD/TF 卡，显著提高冷启动与热重启的成功率与兼容性。
- **系统配置一致性优化**：srpi-config 音频配置流程改进，与 Ubuntu PulseAudio 通道切换保持同步，减少用户额外配置成本。

**（2）媒体链路与显示适配**

- **媒体链路大规模修复**：涵盖 ISP、VIO、编码/解码等模块的稳定性增强和异常场景问题修复。
- **示例程序（Samples）全面清理与补齐**：更新接口调用方式、补齐缺失示例、提高可用性。
- **HDMI 输出能力增强**：改进多分辨率支持与兼容性，提升显示稳定性。
- **LCD 全尺寸适配**：
  - 新增对 11.9 英寸微雪宽屏 的完整适配
  - 修复多个屏幕的触摸坐标异常问题

**（3）外设接口与应用开发能力**

- **WiringPi 兼容性问题修复**：提升脚本与外设控制的易用性。
- **hobot-gpio 功能扩展**：支持多路 PWM 配置。

**（4）文档与开发体验**

- **全面重写《第三章：基础应用开发》**：对章节结构进行重新规划，补充关键概念说明、丰富示例代码与实际操作演示，显著提升文档的可读性与学习路径的连贯性。
- **修复与补充超过 20 项文档问题**：覆盖接口描述不完整、示例缺失、配置步骤不一致等多类问题，使整体文档更加准确、规范、易于使用。

**（5）TogetheROS**

- **修复图像处理加速模块**：hobot_cv 使用 vse 加速图像 resize 的 bug。
- **修复板端模型推理框架**：hobot_dnn 统计推理延迟错误的问题。
- **修复 mipi cam 和算法等模块**：ros component so 安装路径错误导致的运行时加载失败的问题。
- **优化图像编解码模块**：hobot_codec 的配置，删除无效的配置参数，增加用于 debug 的配置参数。

---

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

### TogetheROS-V2.x

详见TogetheROS.Bot[版本发布记录](/docs/05_Robot_development/01_quick_start/changelog.md)。

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


