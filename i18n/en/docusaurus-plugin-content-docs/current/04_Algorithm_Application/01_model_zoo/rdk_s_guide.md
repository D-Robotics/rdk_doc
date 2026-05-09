---
sidebar_position: 4
---

# 4.1.4 RDK S Model Zoo Usage Guide

## Branch and System Requirements

RDK S series (S100 / S600) uses the `rdk_s` branch as the main delivery branch. Recommended system version: RDK OS >= 4.0.5. Python samples in this branch uniformly use the `hbm_runtime` inference interface.

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_s
```

:::tip

Legacy demos for the RDK S series are retained in the [RDK Model Zoo S](https://github.com/d-Robotics/rdk_model_zoo_s) repository. The `rdk_s` branch is the reorganized new version.

:::

## Repository Directory Structure

The `rdk_s` branch uses a standardized directory structure, organized by domain and model:

```bash
rdk_model_zoo/
|-- samples/
|   |-- vision/                  # Vision model examples
|   |   |-- lanenet/             # Lane detection
|   |   |-- mobilenetv2/         # Image classification
|   |   |-- paddle_ocr/          # OCR text recognition
|   |   |-- resnet18/            # Image classification
|   |   |-- unetmobilenet/       # Semantic segmentation
|   |   |-- yolo11/              # YOLO11 detection
|   |   |-- yolo11_pose/         # YOLO11 pose estimation
|   |   |-- yolo11_seg/          # YOLO11 instance segmentation
|   |   |-- yoloe11_seg/         # YOLOE11 instance segmentation
|   |   |-- yolov5/              # YOLOv5 detection
|   |   `-- ...
|   `-- speech/                  # Speech model examples
|       `-- asr/                 # Speech recognition
|-- datasets/                    # Public datasets and sample data
|-- docs/                        # Project specs and reference documentation
|-- tools/                       # Conversion/build/utility tools
|-- tros/                        # TROS integration guides and examples
`-- utils/                       # Shared Python / C++ utilities
```

## Individual Sample Structure

Each RDK S sample contains the following standardized directories:

```bash
sample_name/
|-- README.md              # English documentation
|-- README_cn.md           # Chinese documentation
|-- conversion/            # ONNX → HBM conversion configs
|-- evaluator/             # Accuracy and performance evaluation
|-- model/                 # Pre-compiled .hbm models + download scripts
|-- runtime/
|   |-- python/            # Python inference (main.py, <model>.py, run.sh)
|   `-- cpp/               # C++ inference (src/main.cc, CMakeLists.txt, run.sh)
`-- test_data/             # Test images and inference results
```

## Inference Interface

The RDK S series Python samples uniformly use the `hbm_runtime` inference interface, which shares the same interface name as RDK X5's `hbm_runtime`, but with different underlying dependencies: RDK S series is based on `libhbucp`, while RDK X5 is based on `libdnn`.

For the complete interface reference 👉 [RDK S hbm_runtime Python API Documentation](/rdk_s/Algorithm_Application/python-api)

C/C++ inference interface documentation: **UCP (`hb_ucp`) Interface Documentation** 👉 [UCP Overview](https://toolchain.d-robotics.cc/guide/ucp/ucp_overview.html)

### hbm_runtime Basic Call Flow

#### Load Model

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("../../model/yolov5x_672x672_nv12.hbm")
model_name = model.model_names[0]
input_names = model.input_names[model_name]
output_names = model.output_names[model_name]
input_shapes = model.input_shapes[model_name]
```

#### Configure Scheduling Parameters

`hbm_runtime` supports specifying inference priority and BPU core:

```python
model.set_scheduling_params(
    priority={model_name: 0},
    bpu_cores={model_name: [0]},
)
```

Command-line parameter equivalents:

```bash
--priority 0 --bpu-cores 0
```

#### Prepare Inputs

RDK S vision samples commonly use separated NV12 format (Y plane and UV plane as two separate inputs), unlike RDK X5's single packed NV12 input:

```python
inputs = {
    model_name: {
        input_names[0]: y_plane,    # Y plane
        input_names[1]: uv_plane,   # UV plane
    }
}
```

#### Run Inference

```python
outputs = model.run(inputs)
raw_outputs = outputs[model_name]
output_tensor = raw_outputs[output_names[0]]
```


### Model Zoo Wrapper Flow

RDK S samples follow the `Config + Model + predict()` pattern:

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

The wrapper executes in the following order:

1. `pre_process()`: Generate model inputs (resize, BGR-to-NV12 with separated Y/UV planes)
2. `forward()`: Call `hbm_runtime.run()`
3. `post_process()`: Parse detection boxes, classification results, segmentation masks, or pose keypoints
4. `predict()`: Chain the complete flow

## Quick Start

### Run the YOLOv5 Detection Sample

```bash
# Download model
cd samples/vision/yolov5/model
bash download_model.sh

# Run inference
cd ../runtime/python
python3 main.py \
  --model-path ../../model/yolov5x_672x672_nv12.hbm \
  --test-img ../../test_data/kite.jpg \
  --label-file ../../test_data/coco_classes.names \
  --img-save-path result.jpg
```

### Using run.sh for One-Click Execution

Each sample provides a `run.sh` script in its `runtime/python/` and `runtime/cpp/` directories for one-click environment setup, model download, and inference:

```bash
# Python inference
cd samples/vision/yolov5/runtime/python
bash run.sh

# C++ inference
cd samples/vision/yolov5/runtime/cpp
bash run.sh
```

## Model Coverage

### Vision

| Category | Model | Sample Directory | Supported Platforms |
| :--- | :--- | :--- | :--- |
| Object Detection | YOLOv5x | `samples/vision/yolov5` | S100 / S600 |
| | YOLO11 | `samples/vision/yolo11` | S100 / S600 |
| Instance Segmentation | YOLO11-Seg | `samples/vision/yolo11_seg` | S100 / S600 |
| | YOLOe11-Seg | `samples/vision/yoloe11_seg` | S100 |
| Pose Estimation | YOLO11-Pose | `samples/vision/yolo11_pose` | S100 / S600 |
| Image Classification | ResNet18 | `samples/vision/resnet18` | S100 / S600 |
| | MobileNetV2 | `samples/vision/mobilenetv2` | S100 / S600 |
| Semantic Segmentation | UnetMobileNet | `samples/vision/unetmobilenet` | S100 / S600 |
| Lane Detection | LaneNet | `samples/vision/lanenet` | S100 |
| Text Recognition | PaddleOCR | `samples/vision/paddle_ocr` | S100 |

### Speech

| Category | Model | Sample Directory | Supported Platforms |
| :--- | :--- | :--- | :--- |
| Speech Recognition | ASR | `samples/speech/asr` | S100 / S600 |

## Shared Utilities (utils/)

The `rdk_s` branch provides the following shared Python utilities (`utils/py_utils/`):

| Utility Module | Function |
| :--- | :--- |
| `file_io` | Model download, image loading, class name loading |
| `preprocess` | BGR to NV12 (separated Y/UV planes), resize (direct/letterbox) |
| `postprocess` | NMS, YOLO box/mask/keypoint decoding, coordinate scaling |
| `visualize` | Detection box, segmentation mask, pose keypoint, classification result rendering |
| `inspect` | SoC name detection, model info printing |
| `nn_math` | Sigmoid, z-score normalization |
