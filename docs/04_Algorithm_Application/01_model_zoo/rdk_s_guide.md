---
sidebar_position: 4
---

# 4.1.4 RDK S Model Zoo 使用说明

## 分支与系统要求

RDK S 系列（S100 / S600）使用 `rdk_s` 分支作为主交付分支。推荐系统版本 RDK OS >= 4.0.5。该分支的 Python sample 统一使用 `hbm_runtime` 推理接口。

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_s
```

:::tip

RDK S 系列的历史 demo 保留在 [RDK Model Zoo S](https://github.com/d-Robotics/rdk_model_zoo_s) 仓库。`rdk_s` 分支为规范化组织的新版本。

:::

## 仓库目录结构

`rdk_s` 分支采用规范化目录结构，按领域和模型组织：

```bash
rdk_model_zoo/
|-- samples/
|   |-- vision/                  # 视觉模型示例
|   |   |-- lanenet/             # 车道线检测
|   |   |-- mobilenetv2/         # 图像分类
|   |   |-- paddle_ocr/          # OCR 文字识别
|   |   |-- resnet18/            # 图像分类
|   |   |-- unetmobilenet/       # 语义分割
|   |   |-- yolo11/              # YOLO11 检测
|   |   |-- yolo11_pose/         # YOLO11 姿态估计
|   |   |-- yolo11_seg/          # YOLO11 实例分割
|   |   |-- yoloe11_seg/         # YOLOE11 实例分割
|   |   |-- yolov5/              # YOLOv5 检测
|   |   `-- ...
|   `-- speech/                  # 语音模型示例
|       `-- asr/                 # 语音识别
|-- datasets/                    # 公共数据集与示例数据
|-- docs/                        # 项目规范与参考文档
|-- tools/                       # 转换/构建/辅助工具
|-- tros/                        # TROS 集成指南与示例
`-- utils/                       # 公共 Python / C++ 工具
```

## 单个 Sample 结构

每个 RDK S sample 包含以下标准化目录：

```bash
sample_name/
|-- README.md              # 英文说明文档
|-- README_cn.md           # 中文说明文档
|-- conversion/            # ONNX → HBM 转换配置
|-- evaluator/             # 精度与性能评测
|-- model/                 # 预编译 .hbm 模型 + 下载脚本
|-- runtime/
|   |-- python/            # Python 推理（main.py, <model>.py, run.sh）
|   `-- cpp/               # C++ 推理（src/main.cc, CMakeLists.txt, run.sh）
`-- test_data/             # 测试图片与推理结果
```

## 推理接口

RDK S 系列的 Python sample 统一使用 `hbm_runtime` 推理接口，与 RDK X5 的 `hbm_runtime` 接口名称一致，底层依赖不同：RDK S 系列基于 `libhbucp`，RDK X5 基于 `libdnn`。

完整接口参考 👉 [RDK S hbm_runtime Python API 文档](/rdk_s/Algorithm_Application/python-api)

C/C++ 推理接口文档：**UCP（`hb_ucp`）接口文档** 👉 [UCP Overview](https://toolchain.d-robotics.cc/guide/ucp/ucp_overview.html)

### hbm_runtime 基础调用流程

#### 加载模型

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("../../model/yolov5x_672x672_nv12.hbm")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
input_shapes = model.input_shapes[model_name]
```

#### 配置调度参数

`hbm_runtime` 支持指定推理优先级和 BPU 核心：

```python
model.set_scheduling_params(
    priority={model_name: 0},
    bpu_cores={model_name: [0]},
)
```

命令行参数对应：

```bash
--priority 0 --bpu-cores 0
```

#### 准备输入

RDK S 视觉 sample 常见输入为分离的 NV12 格式（Y 平面和 UV 平面分别作为两个输入），与 RDK X5 的 packed NV12 单输入不同：

```python
inputs = {
    model_name: {
        input_names[0]: y_plane,    # Y 平面
        input_names[1]: uv_plane,   # UV 平面
    }
}
```

#### 执行推理

```python
outputs = model.run(inputs)
raw_outputs = outputs[model_name]
output_tensor = raw_outputs[output_names[0]]
```


### Model Zoo wrapper 流程

RDK S sample 封装为 `Config + Model + predict()` 模式：

```python
config = YOLOv5Config(
    model_path="../../model/yolov5x_672x672_nv12.hbm",
    classes_num=80,
    score_thres=0.25,
    nms_thres=0.45,
)

model = YoloV5X(config)
model.set_scheduling_params(priority=0, bpu_cores=[0])
results = model.predict(image)
```

wrapper 内部按以下顺序执行：

1. `pre_process()`：生成模型输入（resize、BGR 转 NV12 分离 Y/UV 平面）
2. `forward()`：调用 `hbm_runtime.run()`
3. `post_process()`：解析检测框、分类结果、分割 mask 或姿态点
4. `predict()`：串联完整流程

## 快速上手

### 运行 YOLOv5 检测 sample

```bash
# 下载模型
cd samples/vision/yolov5/model
bash download_model.sh

# 运行推理
cd ../runtime/python
python3 main.py \
  --model-path ../../model/yolov5x_672x672_nv12.hbm \
  --test-img ../../test_data/kite.jpg \
  --label-file ../../test_data/coco_classes.names \
  --img-save-path result.jpg
```

### 使用 run.sh 一键运行

每个 sample 的 `runtime/python/` 和 `runtime/cpp/` 目录下均提供 `run.sh` 脚本，可一键完成环境设置、模型下载和推理：

```bash
# Python 推理
cd samples/vision/yolov5/runtime/python
bash run.sh

# C++ 推理
cd samples/vision/yolov5/runtime/cpp
bash run.sh
```

## 模型支持范围

### 视觉（Vision）

| 类别 | 模型 | Sample 目录 | 支持平台 |
| :--- | :--- | :--- | :--- |
| 目标检测 | YOLOv5x | `samples/vision/yolov5` | S100 / S600 |
| 目标检测 | YOLO11 | `samples/vision/yolo11` | S100 / S600 |
| 实例分割 | YOLO11-Seg | `samples/vision/yolo11_seg` | S100 / S600 |
| 实例分割 | YOLOe11-Seg | `samples/vision/yoloe11_seg` | S100 |
| 姿态估计 | YOLO11-Pose | `samples/vision/yolo11_pose` | S100 / S600 |
| 图像分类 | ResNet18 | `samples/vision/resnet18` | S100 / S600 |
| 图像分类 | MobileNetV2 | `samples/vision/mobilenetv2` | S100 / S600 |
| 语义分割 | UnetMobileNet | `samples/vision/unetmobilenet` | S100 / S600 |
| 车道线检测 | LaneNet | `samples/vision/lanenet` | S100 |
| 文字识别 | PaddleOCR | `samples/vision/paddle_ocr` | S100 |

### 语音（Speech）

| 类别 | 模型 | Sample 目录 | 支持平台 |
| :--- | :--- | :--- | :--- |
| 语音识别 | ASR | `samples/speech/asr` | S100 / S600 |

## 共享工具（utils/）

`rdk_s` 分支提供以下共享 Python 工具（`utils/py_utils/`）：

| 工具模块 | 功能 |
| :--- | :--- |
| `file_io` | 模型下载、图片加载、类别名加载 |
| `preprocess` | BGR 转 NV12（分离 Y/UV 平面）、resize（direct/letterbox） |
| `postprocess` | NMS、YOLO 检测框/分割/姿态解码、坐标缩放 |
| `visualize` | 检测框、分割 mask、姿态点、分类结果绘制 |
| `inspect` | SoC 名称检测、模型信息打印 |
| `nn_math` | sigmoid、z-score 归一化 |
