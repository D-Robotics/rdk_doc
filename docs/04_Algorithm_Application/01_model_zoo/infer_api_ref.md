---
sidebar_position: 5
---

# 4.1.5 Model Zoo 推理接口参考

## 概述

Model Zoo 的 Python 推理接口按硬件平台和分支选择：

| 平台 | 分支 | 推理接口 | 接口来源 |
| :--- | :--- | :--- | :--- |
| RDK X5 | `rdk_x5` | `hbm_runtime` | 板端系统自带 |
| RDK X5（历史） | `rdk_x5_legacy` | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | 手动安装 / 板端自带 |
| RDK X3 | `rdk_x3` | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | 手动安装 / 板端自带 |
| RDK S 系列 | `rdk_s` | `hbm_runtime` | 板端系统自带 |

## hbm_runtime 接口

`hbm_runtime` 是 RDK X5 和 RDK S 系列统一使用的 Python 推理接口，接口名称一致，但底层依赖、模型格式和输入方式存在差异：

| 项目 | RDK X5 | RDK S 系列 |
| :--- | :--- | :--- |
| 底层依赖 | `libdnn` | `libhbucp` |
| 模型格式 | `.bin` | `.hbm` |
| 安装包 | `hobot-spdev`（板端自带） | `hobot-dnn`（板端自带） |
| NV12 输入方式 | 单个 packed 输入（Y+UV 拼接） | 双输入（Y 与 UV 分离） |

### RDK X5 调用流程

#### 加载模型

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("model.bin")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
```

#### 准备输入（packed NV12）

RDK X5 视觉模型常见输入为 packed NV12 格式，Y 和 UV 拼接为单个输入 tensor：

```python
y, uv = bgr_to_nv12_planes(resized_img)
packed_nv12 = np.concatenate([y.reshape(-1), uv.reshape(-1)]).astype(np.uint8)

inputs = {model_name: {input_names[0]: packed_nv12}}
```

#### 执行推理与读取输出

```python
outputs = model.run(inputs)
result = outputs[model_name][output_names[0]]
```

#### 详细 API 文档

👉 [RDK X5 hbm_runtime Python API 文档](../../03_Basic_Application/06_multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_hbdnn_demo.md)

### RDK S 系列调用流程

#### 加载模型

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("model.hbm")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
```

#### 准备输入（分离 Y/UV）

RDK S 系列视觉模型使用分离的 Y 和 UV 两个输入 tensor：

```python
y, uv = bgr_to_nv12_planes(resized_img)

inputs = {
    model_name: {
        input_names[0]: y,   # Y 平面
        input_names[1]: uv,  # UV 平面
    }
}
```

#### 执行推理与读取输出

```python
outputs = model.run(inputs)
result = outputs[model_name][output_names[0]]
```

#### 详细 API 文档

👉 [RDK S hbm_runtime Python API 文档](/rdk_s/Algorithm_Application/python-api)

## bpu_infer_lib 接口

`bpu_infer_lib_x3` 与 `bpu_infer_lib_x5` 安装后均通过 `import bpu_infer_lib` 导入。

RDK X3 使用 `bpu_infer_lib_x3` wheel，`rdk_x5_legacy` 分支使用 `bpu_infer_lib_x5` wheel。

:::caution

`bpu_infer_lib` **对 featuremap 输入模型支持不佳**。如需使用 featuremap 输入模型，请使用 `hbm_runtime` 推理接口（RDK X5 或 RDK S 系列）。

:::

### 安装

RDK X3：

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x3/bpu_infer_lib_x3-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x3-1.0.3-py3-none-any.whl
```

RDK X5（`rdk_x5_legacy` 分支）：

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/bpu_infer_lib_x5-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x5-1.0.3-py3-none-any.whl
```

### 基础调用流程

```python
import bpu_infer_lib

inf = bpu_infer_lib.Infer(False)
inf.load_model("model.bin")
inf.read_input(input_array, 0)
inf.forward()
inf.get_output()

result = inf.outputs[0].data
```

### 常用接口说明

| 接口 | 说明 |
| :--- | :--- |
| `Infer(debug)` | 创建推理对象。`debug=True` 时输出更多调试信息。 |
| `load_model(model_path)` | 加载 BPU `.bin` 模型。 |
| `read_input(input, index)` | 写入预处理后的 numpy 输入。`index` 为输入节点序号，从 0 开始。 |
| `forward()` | 执行模型推理。 |
| `get_output()` | 获取推理输出。 |
| `outputs[index].data` | 读取指定输出 tensor 的 numpy 数据。 |

## hobot_dnn.pyeasy_dnn 接口

`hobot_dnn.pyeasy_dnn` 是板端系统自带的 Python 推理接口。RDK X3 的 YOLO、FCOS、YOLOv8-Seg 等 demo 使用该接口；`rdk_x5_legacy` 分支的部分 demo 也使用该接口。

:::caution

`hobot_dnn.pyeasy_dnn` **对 featuremap 输入模型支持不佳**。如需使用 featuremap 输入模型，请使用 `hbm_runtime` 推理接口（RDK X5 或 RDK S 系列）。

:::

### 基础调用流程

```python
from hobot_dnn import pyeasy_dnn as dnn

models = dnn.load("models/yolov5s_672x672_nv12.bin")
model = models[0]
outputs = model.forward(input_tensor)
```

模型路径、输入格式、预处理和后处理以目标目录 README 和源码入口为准。

## 接口选择说明

| 平台 | 推荐接口 | 使用说明 |
| :--- | :--- | :--- |
| RDK X5，RDK OS >= 3.5.0 | `hbm_runtime` | `rdk_x5` 分支，按 sample README 使用 |
| RDK X5（历史 demo） | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | `rdk_x5_legacy` 分支，按目标目录 README 使用 |
| RDK X3 | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | `rdk_x3` 分支，按目标目录 README 使用 |
| RDK S 系列 | `hbm_runtime` | `rdk_s` 分支，按 sample README 使用 |
