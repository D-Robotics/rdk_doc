---
sidebar_position: 5
---

# 4.1.5 Model Zoo Inference Interface Reference

## Overview

Model Zoo Python inference interfaces are selected by hardware platform and branch:

| Platform | Branch | Inference Interface | Interface Source |
| :--- | :--- | :--- | :--- |
| RDK X5 | `rdk_x5` | `hbm_runtime` | Board-builtin |
| RDK X5 (legacy) | `rdk_x5_legacy` | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | Manual install / Board-builtin |
| RDK X3 | `rdk_x3` | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | Manual install / Board-builtin |
| RDK S Series | `rdk_s` | `hbm_runtime` | Board-builtin |

## hbm_runtime Interface

`hbm_runtime` is the unified Python inference interface used by both RDK X5 and RDK S series. The interface name is identical, but the underlying dependencies, model formats, and input handling differ:

| Item | RDK X5 | RDK S Series |
| :--- | :--- | :--- |
| Underlying Dependency | `libdnn` | `libhbucp` |
| Model Format | `.bin` | `.hbm` |
| Package | `hobot-spdev` (board-builtin) | `hobot-dnn` (board-builtin) |
| NV12 Input Style | Single packed input (Y+UV concatenated) | Dual inputs (Y and UV separated) |

### RDK X5 Call Flow

#### Load Model

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("model.bin")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
```

#### Prepare Input (Packed NV12)

RDK X5 vision models commonly use packed NV12 format, where Y and UV planes are concatenated into a single input tensor:

```python
y, uv = bgr_to_nv12_planes(resized_img)
packed_nv12 = np.concatenate([y.reshape(-1), uv.reshape(-1)]).astype(np.uint8)

inputs = {model_name: {input_names[0]: packed_nv12}}
```

#### Run Inference and Read Output

```python
outputs = model.run(inputs)
result = outputs[model_name][output_names[0]]
```

#### Detailed API Documentation

👉 [RDK X5 hbm_runtime Python API Documentation](../../03_Basic_Application/06_multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_hbdnn_demo.md)

### RDK S Series Call Flow

#### Load Model

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("model.hbm")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
```

#### Prepare Input (Separated Y/UV)

RDK S series vision models use separated Y and UV input tensors:

```python
y, uv = bgr_to_nv12_planes(resized_img)

inputs = {
    model_name: {
        input_names[0]: y,   # Y plane
        input_names[1]: uv,  # UV plane
    }
}
```

#### Run Inference and Read Output

```python
outputs = model.run(inputs)
result = outputs[model_name][output_names[0]]
```

#### Detailed API Documentation

👉 [RDK S hbm_runtime Python API Documentation](/rdk_s/Algorithm_Application/python-api)

## bpu_infer_lib Interface

After installing `bpu_infer_lib_x3` or `bpu_infer_lib_x5`, import the runtime package with `import bpu_infer_lib`.

RDK X3 uses the `bpu_infer_lib_x3` wheel, and the `rdk_x5_legacy` branch uses the `bpu_infer_lib_x5` wheel.

:::caution

`bpu_infer_lib` **has poor support for featuremap input models**. If you need to use featuremap input models, please use the `hbm_runtime` inference interface (RDK X5 or RDK S series).

:::

### Installation

RDK X3:

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x3/bpu_infer_lib_x3-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x3-1.0.3-py3-none-any.whl
```

RDK X5 (`rdk_x5_legacy` branch):

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/bpu_infer_lib_x5-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x5-1.0.3-py3-none-any.whl
```

### Basic Call Flow

```python
import bpu_infer_lib

inf = bpu_infer_lib.Infer(False)
inf.load_model("model.bin")
inf.read_input(input_array, 0)
inf.forward()
inf.get_output()

result = inf.outputs[0].data
```

### Common Interface Reference

| Interface | Description |
| :--- | :--- |
| `Infer(debug)` | Create an inference object. `debug=True` prints more debug information. |
| `load_model(model_path)` | Load a BPU `.bin` model. |
| `read_input(input, index)` | Write a pre-processed numpy input. `index` is the input node index, starting from 0. |
| `forward()` | Run model inference. |
| `get_output()` | Fetch inference outputs. |
| `outputs[index].data` | Read the numpy data of the specified output tensor. |

## hobot_dnn.pyeasy_dnn Interface

`hobot_dnn.pyeasy_dnn` is a board-builtin Python inference interface. RDK X3 YOLO, FCOS, and YOLOv8-Seg demos use this interface; some demos in the `rdk_x5_legacy` branch also use this interface.

:::caution

`hobot_dnn.pyeasy_dnn` **has poor support for featuremap input models**. If you need to use featuremap input models, please use the `hbm_runtime` inference interface (RDK X5 or RDK S series).

:::

### Basic Call Flow

```python
from hobot_dnn import pyeasy_dnn as dnn

models = dnn.load("models/yolov5s_672x672_nv12.bin")
model = models[0]
outputs = model.forward(input_tensor)
```

The target directory README and source entry point define the model path, input format, pre-processing, and post-processing.

## Interface Selection Guide

| Platform | Recommended Interface | Usage Notes |
| :--- | :--- | :--- |
| RDK X5, RDK OS >= 3.5.0 | `hbm_runtime` | `rdk_x5` branch, follow sample README |
| RDK X5 (legacy demos) | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | `rdk_x5_legacy` branch, follow target directory README |
| RDK X3 | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | `rdk_x3` branch, follow target directory README |
| RDK S Series | `hbm_runtime` | `rdk_s` branch, follow sample README |
