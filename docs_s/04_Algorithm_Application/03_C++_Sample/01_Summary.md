---
sidebar_position: 1
---

# 示例概述

本项目包含多个基于 C/C++ 编写的 AI 示例程序，适用于 RDK S100平台，覆盖图像分类、目标检测、实例分割、姿态估计、OCR、语音识别等常见 AI 任务。示例使用 `.hbm` 格式的量化模型进行推理，便于开发者快速验证模型效果并开展应用开发。

本项目的板端代码位置：`/app/cdev_demo/bpu`。

## 目录结构总览

```text
|-- 01_classification_sample         # 图像分类示例（如 ResNet18、MobileNet）
|-- 02_detection_sample              # 目标检测示例（YOLO 等）
|-- 03_instance_segmentation_sample  # 实例分割示例
|-- 04_pose_sample                   # 关键点检测示例
|-- 05_open_instance_seg_sample      # 开放实例分割示例
|-- 06_lane_detection_sample         # 车道线检测示例
|-- 07_speech_sample                 # 语音识别示例
|-- 08_OCR_sample                    # 光学字符识别示例
|-- 09_usb_camera_sample             # USB 摄像头实时推理示例
|-- 10_mipi_camera_sample            # MIPI 摄像头实时推理示例
|-- 11_decode_yolov5x_display_sample # 视频解码、推理与显示示例
|-- 12_rtsp_yolov5x_display_sample   # RTSP 流解码、推理与显示示例
|-- utils                            # 通用工具函数
`-- README.md                        # 项目说明文档（当前文件）
```

## 环境要求

在运行示例前，请确保系统环境满足以下要求：

### 硬件
- 支持 BPU 的S100开发板
- 摄像头（USB 或 MIPI）若需运行相关示例

### 系统与工具链
本项目已在以下环境中验证可运行：

- 操作系统

    - Ubuntu 22.04.5 LTS (Jammy Jellyfish)

- 编译工具链

    - CMake: 3.22.1

    - GCC: 11.4.0

    - G++: 11.4.0

### 依赖库
不同的示例依赖不同的开发包，请根据需求安装。

- 通用依赖
    ```bash
    sudo apt update
    sudo apt install libgflags-dev
    ```

- ASR 语音识别示例
    ```bash
    sudo apt update
    sudo apt install libsndfile1-dev
    sudo apt install libsamplerate0-dev
    ```

- OCR 文字识别示例
    ```bash
    sudo apt update
    sudo apt install libpolyclipping-dev
    ```

## 编译方法
以图像分类示例 resnet18 为例：

```bash
cd 01_classification_sample/01_resnet18

mkdir build && cd build

cmake ..

make -j$(nproc)
```

## 运行示例
以图像分类示例 resnet18 为例：
+ 进入sample目录的编译目录
    ```bash
    cd 01_classification_sample/01_resnet18/build
    ```
+ 运行模型
    ``` bash
    ./resnet_18
    ```
+ 查看结果
    ``` bash
    TOP 0: label=zebra, prob=0.99872
    TOP 1: label=cheetah, chetah, Acinonyx jubatus, prob=0.000448407
    TOP 2: label=impala, Aepyceros melampus, prob=0.000398787
    TOP 3: label=gazelle, prob=0.000253181
    TOP 4: label=prairie chicken, prairie grouse, prairie fowl, prob=0.000179423
    ```

## 通用工具说明
utils 目录包含了BPU C/C++ 推理示例中的公共工具函数，用于图像预处理、推理结果后处理、多媒体处理及通用工具函数，方便在不同示例中复用。

```bash

utils
├── inc                          # 头文件目录
│   ├── common_utils.hpp         # 通用工具函数（反量化、结果绘制、常用数据结构等）
│   ├── multimedia_utils.hpp     # 多媒体处理工具（视频帧解码、像素格式转换等）
│   ├── postprocess_utils.hpp    # 推理结果后处理（NMS、解码、mask处理等）
│   └── preprocess_utils.hpp     # 输入数据预处理（图像缩放、归一化、格式转换等）
└── src                          # 源码目录（具体实现）
    ├── common_utils.cc
    ├── multimedia_utils.cc
    ├── postprocess_utils.cc
    └── preprocess_utils.cc


```

## 附加说明
* 所有示例程序均使用`.hbm`格式模型。

* 注意：各子目录下提供的`README.md`会详细介绍对应模型所需环境说明、命令行参数、运行方式等内容。
