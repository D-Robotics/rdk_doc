---
sidebar_position: 1
---

# 示例概述

本项目包含多个基于 Python 编写的 AI 示例程序，适用于 RDK S100平台，覆盖图像分类、目标检测、实例分割、姿态估计、OCR、语音识别等常见 AI 任务。示例使用 `.hbm` 格式的量化模型进行推理，便于开发者快速验证模型效果并开展应用开发。

本项目的板端代码位置：`/app/pydev_demo/`。

## 概述
### 环境要求
本项目基于 Python 编写并依赖多个第三方库。请确保您的环境满足以下要求：

#### Python 环境
- Python 版本：建议使用 Python 3.10.x（当前已在 3.10.12 下测试通过）

#### 依赖库
- 依赖库列表
    | 库名           | 说明                                          | 推荐版本          |
    |----------------|----------------------------------------------|------------------|
    | numpy          | 科学计算库，用于张量、矩阵处理                  | >=1.26.4         |
    | opencv-python  | 图像处理和可视化（cv2）                        | >=4.11.0.86      |
    | scipy          | 包含数学函数库，如 softmax                     | >=1.15.3         |
- 依赖安装
    ```bash
    # 安装依赖
    pip install -r requirements.txt
    ```
- 注意

    **以上依赖库列表和安装文件仅列举了模型运行的基本库，部分示例程序需要额外的三方库，可通过相应示例的README.md文档或此文档的相应章节查看。**

#### 其他组件
- hbm_rumtime：用于加载和运行`.hbm`模型，系统默认已安装，如需自行安装可参考 [4.1 Python 接口](../01_Python_API.md#4.1-python-接口) 部分。


- Hobot VIO：用于访问相机图像流（hobot_vio，如 libsrcampy）


### 目录结构
    ```text
    .
    ├── 01_classification_sample/        # 图像分类样例
    ├── 02_detection_sample/             # 目标检测样例
    ├── 03_instance_segmentation_sample/ # 实例分割样例
    ├── 04_pose_sample/                  # 姿态估计样例
    ├── 05_open_instance_seg_sample/     # 开放词表实例分割样例
    ├── 06_lane_detection_sample/        # 车道线检测样例
    ├── 07_speech_sample/                # 语音识别样例
    ├── 08_OCR_sample/                   # OCR 文字识别样例
    ├── 09_usb_camera_sample/            # USB 摄像头 + 目标检测样例
    ├── 10_mipi_camera_sample/           # MIPI 摄像头 + 目标检测样例
    ├── 11_web_display_camera_sample/    # 摄像头 + Web + 目标检测样例
    ├── utils/                           # 通用预处理、后处理工具模块
    ├── requirements.txt                 # Python 环境依赖
    └── README.md                        # 顶层使用说明文档（本文件）
    ```

### 快速运行
以图像分类示例 resnet18 为例：
- 进入sample目录
    ```bash
    cd 01_classification_sample/01_resnet18
    ```
- 运行模型
    ``` bash
    python3 resnet18.py \
    --model-path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
    --test-img /app/res/assets/zebra_cls.jpg
    ```
- 查看结果
    ``` bash
    Top-5 Predictions:
    zebra: 0.9979
    impala, Aepyceros melampus: 0.0005
    cheetah, chetah, Acinonyx jubatus: 0.0005
    gazelle: 0.0004
    prairie chicken, prairie grouse, prairie fowl: 0.0002
    ```

### 通用工具说明
项目中使用了统一的工具模块以简化样例开发，路径为 utils/：

* preprocess_utils.py：图像预处理，如 resize、颜色格式转换等

* postprocess_utils.py：模型后处理逻辑，如 NMS、框变换等

* draw_utils.py：绘制检测框、关键点、分割掩码等

* common_utils.py：模型信息打印、颜色定义等

### 注意事项
* 所有示例程序均使用`.hbm`格式模型，需配合平台`hbm_runtime`的python推理接口使用。

* 注意：各子目录下提供的`README.md`会详细介绍对应模型所需环境说明、命令行参数、运行方式等内容。
