---
sidebar_position: 3
---

# 4.1.3 RDK X5 Model Zoo 使用说明

## 分支与系统要求

RDK X5 使用 `rdk_x5` 分支作为主交付分支。推荐系统版本 RDK OS >= 3.5.0。该分支的 Python sample 统一使用 `hbm_runtime` 推理接口。

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x5
```

:::tip

`rdk_x5` 分支是 RDK X5 的主交付分支，推荐优先使用。原 `main` 分支已变更为 `rdk_x5_legacy`，仅用于历史 demo 归档。

:::

## 仓库目录结构

`rdk_x5` 分支采用规范化目录结构，按领域和模型组织：

```bash
rdk_model_zoo/
|-- samples/
|   `-- vision/
|       |-- clip/                 # 图文多模态匹配
|       |-- convnext/             # 图像分类
|       |-- edgenext/             # 图像分类
|       |-- efficientformer/      # 图像分类
|       |-- efficientformerv2/    # 图像分类
|       |-- efficientnet/         # 图像分类
|       |-- efficientvit/         # 图像分类
|       |-- fasternet/            # 图像分类
|       |-- fastvit/              # 图像分类
|       |-- fcos/                 # 目标检测
|       |-- googlenet/            # 图像分类
|       |-- lprnet/               # 车牌识别
|       |-- mobilenetv1/          # 图像分类
|       |-- mobilenetv2/          # 图像分类
|       |-- mobilenetv3/          # 图像分类
|       |-- mobilenetv4/          # 图像分类
|       |-- mobileone/            # 图像分类
|       |-- modnet/               # 图像抠图
|       |-- paddleocr/            # OCR 文字检测与识别
|       |-- repghost/             # 图像分类
|       |-- repvgg/               # 图像分类
|       |-- repvit/               # 图像分类
|       |-- resnet/               # 图像分类
|       |-- resnext/              # 图像分类
|       |-- ultralytics_yolo/     # 检测、分割、姿态、分类
|       |-- ultralytics_yolo26/   # 检测、分割、姿态、OBB、分类
|       |-- vargconvnet/          # 图像分类
|       |-- yoloe/                # 实例分割
|       |-- yolov5/               # 目标检测
|       `-- yoloworld/            # 开放词表目标检测
|-- docs/                         # 项目规范与参考文档
|-- datasets/                     # 数据集与下载脚本
|-- tros/                         # TROS 集成指南与示例
`-- utils/                        # 公共 Python / C++ 工具
```

## 单个 Sample 结构

每个 RDK X5 sample 包含以下标准化目录：

```bash
sample_name/
|-- README.md              # 英文说明文档
|-- README_cn.md           # 中文说明文档
|-- conversion/            # ONNX → HBM/BIN 转换配置
|-- evaluator/             # 精度与性能评测
|-- model/                 # 预编译 .bin 模型 + 下载脚本
|-- runtime/
|   |-- python/            # Python 推理（main.py, <model>.py, run.sh）
|   `-- cpp/               # C++ 推理（src/main.cc, CMakeLists.txt, run.sh）
`-- test_data/             # 测试图片与推理结果
```

## 推理接口

`rdk_x5` 分支的 Python sample 统一使用 `hbm_runtime` 推理接口。完整接口参考 [RDK X5 hbm_runtime Python API 文档](../../03_Basic_Application/06_multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_hbdnn_demo.md)。

C++ 推理接口文档：**`hb_dnn` C/C++ 推理接口文档** 👉 [Runtime 开发文档](https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/runtime/source/runtime_dev.html)

### hbm_runtime 基础调用流程

#### 加载模型

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("../../model/yolo11x_detect_bayese_640x640_nv12.bin")
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

RDK X5 视觉 sample 常见输入为 packed NV12 格式。wrapper 的 `pre_process()` 会完成 resize、BGR 转 NV12、数据打包等步骤：

```python
inputs = {
    model_name: {
        input_names[0]: input_array,
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

RDK X5 sample 封装为 `Config + Model + predict()` 模式：

```python
config = YOLOv5Config(
    model_path="../../model/yolov5n_tag_v7.0_detect_640x640_bayese_nv12.bin",
    classes_num=80,
    score_thres=0.25,
    nms_thres=0.45,
)

model = YOLOv5Detect(config)
model.set_scheduling_params(priority=0, bpu_cores=[0])
results = model.predict(image)
```

wrapper 内部按以下顺序执行：

1. `pre_process()`：生成模型输入
2. `forward()`：调用 `hbm_runtime.run()`
3. `post_process()`：解析检测框、分类结果、分割 mask 或姿态点
4. `predict()`：串联完整流程

## 快速上手

### 运行 Ultralytics YOLO11x 检测 sample

```bash
# 下载模型
cd samples/vision/ultralytics_yolo/model
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/ultralytics_YOLO/yolo11x_detect_bayese_640x640_nv12.bin

# 运行推理
cd ../runtime/python
python3 main.py \
  --task detect \
  --model-path ../../model/yolo11x_detect_bayese_640x640_nv12.bin \
  --test-img ../../../../../datasets/coco/assets/bus.jpg \
  --img-save-path ../../test_data/inference_yolo11x.jpg
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

### 图像分类

| 模型 | Sample 目录 |
| :--- | :--- |
| ConvNeXt | `samples/vision/convnext` |
| EdgeNeXt | `samples/vision/edgenext` |
| EfficientFormer | `samples/vision/efficientformer` |
| EfficientFormerV2 | `samples/vision/efficientformerv2` |
| EfficientNet | `samples/vision/efficientnet` |
| EfficientViT | `samples/vision/efficientvit` |
| FasterNet | `samples/vision/fasternet` |
| FastViT | `samples/vision/fastvit` |
| GoogLeNet | `samples/vision/googlenet` |
| MobileNetV1 | `samples/vision/mobilenetv1` |
| MobileNetV2 | `samples/vision/mobilenetv2` |
| MobileNetV3 | `samples/vision/mobilenetv3` |
| MobileNetV4 | `samples/vision/mobilenetv4` |
| MobileOne | `samples/vision/mobileone` |
| RepGhost | `samples/vision/repghost` |
| RepVGG | `samples/vision/repvgg` |
| RepViT | `samples/vision/repvit` |
| ResNet | `samples/vision/resnet` |
| ResNeXt | `samples/vision/resnext` |
| VargConvNet | `samples/vision/vargconvnet` |

### 目标检测

| 模型 | Sample 目录 |
| :--- | :--- |
| FCOS | `samples/vision/fcos` |
| YOLOv5 | `samples/vision/yolov5` |
| Ultralytics YOLO（YOLOv5u / YOLOv8 / YOLOv9 / YOLOv10 / YOLO11 / YOLO12 / YOLO13） | `samples/vision/ultralytics_yolo` |
| Ultralytics YOLO26 | `samples/vision/ultralytics_yolo26` |

### 实例分割 / 抠图

| 模型 | Sample 目录 |
| :--- | :--- |
| YOLOE | `samples/vision/yoloe` |
| MODNet | `samples/vision/modnet` |

### OCR / 识别

| 模型 | Sample 目录 |
| :--- | :--- |
| PaddleOCR | `samples/vision/paddleocr` |
| LPRNet | `samples/vision/lprnet` |

### 多模态

| 模型 | Sample 目录 |
| :--- | :--- |
| CLIP | `samples/vision/clip` |
| YOLOWorld | `samples/vision/yoloworld` |

## rdk_x5_legacy 分支

RDK X5 使用 `rdk_x5_legacy` 分支时：

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x5_legacy
```

切换到 `rdk_x5_legacy` 后，进入目标 demo 目录，先阅读该目录 README，再按 README 中的命令运行。

该分支使用 `bpu_infer_lib_x5` 的 demo，按以下命令安装：

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/bpu_infer_lib_x5-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x5-1.0.3-py3-none-any.whl
```

该分支使用 `hobot_dnn.pyeasy_dnn` 的 demo 直接使用板端系统自带接口。

:::caution

`rdk_x5_legacy` 为历史归档分支，不再主动维护。新项目请使用 `rdk_x5` 分支。

`bpu_infer_lib_x5` 和 `hobot_dnn.pyeasy_dnn` **对 featuremap 输入模型支持不佳**。如需使用 featuremap 输入模型，请使用 `rdk_x5` 分支的 `hbm_runtime` 推理接口。

:::

## 共享工具（utils/）

`rdk_x5` 分支提供以下共享 Python 工具（`utils/py_utils/`）：

| 工具模块 | 功能 |
| :--- | :--- |
| `file_io` | 模型下载、图片加载、类别名加载 |
| `preprocess` | BGR 转 NV12、resize（direct/letterbox）、NV12 分离 |
| `postprocess` | NMS、YOLO 检测框/分割/姿态/OBB 解码、坐标缩放 |
| `visualize` | 检测框、分割 mask、旋转框、姿态点、分类结果绘制 |
| `inspect` | SoC 名称检测、模型信息打印 |
| `nn_math` | sigmoid、z-score 归一化 |
