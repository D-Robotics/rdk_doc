---
sidebar_position: 3
---

# 4.1.3 RDK X5 Model Zoo Usage Guide

## Branch and System Requirements

RDK X5 uses the `rdk_x5` branch as the main delivery branch. Recommended system version: RDK OS >= 3.5.0. Python samples in this branch uniformly use the `hbm_runtime` inference interface.

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x5
```

:::tip

The `rdk_x5` branch is the main delivery branch for RDK X5 and is recommended for priority use. The original `main` branch has been renamed to `rdk_x5_legacy` and is used only for historical demo archiving.

:::

## Repository Directory Structure

The `rdk_x5` branch uses a standardized directory structure, organized by domain and model:

```bash
rdk_model_zoo/
|-- samples/
|   `-- vision/
|       |-- clip/                 # Image-text multimodal matching
|       |-- convnext/             # Image classification
|       |-- edgenext/             # Image classification
|       |-- efficientformer/      # Image classification
|       |-- efficientformerv2/    # Image classification
|       |-- efficientnet/         # Image classification
|       |-- efficientvit/         # Image classification
|       |-- fasternet/            # Image classification
|       |-- fastvit/              # Image classification
|       |-- fcos/                 # Object detection
|       |-- googlenet/            # Image classification
|       |-- lprnet/               # License plate recognition
|       |-- mobilenetv1/          # Image classification
|       |-- mobilenetv2/          # Image classification
|       |-- mobilenetv3/          # Image classification
|       |-- mobilenetv4/          # Image classification
|       |-- mobileone/            # Image classification
|       |-- modnet/               # Image matting
|       |-- paddleocr/            # OCR text detection and recognition
|       |-- repghost/             # Image classification
|       |-- repvgg/               # Image classification
|       |-- repvit/               # Image classification
|       |-- resnet/               # Image classification
|       |-- resnext/              # Image classification
|       |-- ultralytics_yolo/     # Detection, segmentation, pose, classification
|       |-- ultralytics_yolo26/   # Detection, segmentation, pose, OBB, classification
|       |-- vargconvnet/          # Image classification
|       |-- yoloe/                # Instance segmentation
|       |-- yolov5/               # Object detection
|       `-- yoloworld/            # Open-vocabulary object detection
|-- docs/                         # Project specs and reference documentation
|-- datasets/                     # Datasets and download scripts
|-- tros/                         # TROS integration guides and examples
`-- utils/                        # Shared Python / C++ utilities
```

## Individual Sample Structure

Each RDK X5 sample contains the following standardized directories:

```bash
sample_name/
|-- README.md              # English documentation
|-- README_cn.md           # Chinese documentation
|-- conversion/            # ONNX → HBM/BIN conversion configs
|-- evaluator/             # Accuracy and performance evaluation
|-- model/                 # Pre-compiled .bin models + download scripts
|-- runtime/
|   |-- python/            # Python inference (main.py, <model>.py, run.sh)
|   `-- cpp/               # C++ inference (src/main.cc, CMakeLists.txt, run.sh)
`-- test_data/             # Test images and inference results
```

## Inference Interface

The `rdk_x5` branch Python samples uniformly use the `hbm_runtime` inference interface. For the complete interface reference, see [RDK X5 hbm_runtime Python API Documentation](../../03_Basic_Application/06_multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_hbdnn_demo.md).

C++ inference interface documentation: **`hb_dnn` C/C++ Inference Interface Documentation** 👉 [Runtime Development Docs](https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/runtime/source/runtime_dev.html)

### hbm_runtime Basic Call Flow

#### Load Model

```python
import hbm_runtime

model = hbm_runtime.HB_HBMRuntime("../../model/yolo11x_detect_bayese_640x640_nv12.bin")
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

RDK X5 vision samples commonly use packed NV12 format input. The wrapper's `pre_process()` handles resize, BGR-to-NV12 conversion, data packing, etc.:

```python
inputs = {
    model_name: {
        input_names[0]: input_array,
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

RDK X5 samples follow the `Config + Model + predict()` pattern:

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

The wrapper executes in the following order:

1. `pre_process()`: Generate model inputs
2. `forward()`: Call `hbm_runtime.run()`
3. `post_process()`: Parse detection boxes, classification results, segmentation masks, or pose keypoints
4. `predict()`: Chain the complete flow

## Quick Start

### Run the Ultralytics YOLO11x Detection Sample

```bash
# Download model
cd samples/vision/ultralytics_yolo/model
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/ultralytics_YOLO/yolo11x_detect_bayese_640x640_nv12.bin

# Run inference
cd ../runtime/python
python3 main.py \
  --task detect \
  --model-path ../../model/yolo11x_detect_bayese_640x640_nv12.bin \
  --test-img ../../../../../datasets/coco/assets/bus.jpg \
  --img-save-path ../../test_data/inference_yolo11x.jpg
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

### Image Classification

| Model | Sample Directory |
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

### Object Detection

| Model | Sample Directory |
| :--- | :--- |
| FCOS | `samples/vision/fcos` |
| YOLOv5 | `samples/vision/yolov5` |
| Ultralytics YOLO (YOLOv5u / YOLOv8 / YOLOv9 / YOLOv10 / YOLO11 / YOLO12 / YOLO13) | `samples/vision/ultralytics_yolo` |
| Ultralytics YOLO26 | `samples/vision/ultralytics_yolo26` |

### Instance Segmentation / Matting

| Model | Sample Directory |
| :--- | :--- |
| YOLOE | `samples/vision/yoloe` |
| MODNet | `samples/vision/modnet` |

### OCR / Recognition

| Model | Sample Directory |
| :--- | :--- |
| PaddleOCR | `samples/vision/paddleocr` |
| LPRNet | `samples/vision/lprnet` |

### Multimodal

| Model | Sample Directory |
| :--- | :--- |
| CLIP | `samples/vision/clip` |
| YOLOWorld | `samples/vision/yoloworld` |

## rdk_x5_legacy Branch

When using RDK X5 with the `rdk_x5_legacy` branch:

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x5_legacy
```

After switching to `rdk_x5_legacy`, enter the target demo directory, read its README first, then follow the commands in that README.

For demos that use `bpu_infer_lib_x5`, install with:

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x5/bpu_infer_lib_x5-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x5-1.0.3-py3-none-any.whl
```

Demos that use `hobot_dnn.pyeasy_dnn` use the board-builtin interface directly.

:::caution

`rdk_x5_legacy` is a legacy archive branch and is no longer actively maintained. New projects should use the `rdk_x5` branch.

`bpu_infer_lib_x5` and `hobot_dnn.pyeasy_dnn` **have poor support for featuremap input models**. If you need to use featuremap input models, please use the `hbm_runtime` inference interface from the `rdk_x5` branch.

:::

## Shared Utilities (utils/)

The `rdk_x5` branch provides the following shared Python utilities (`utils/py_utils/`):

| Utility Module | Function |
| :--- | :--- |
| `file_io` | Model download, image loading, class name loading |
| `preprocess` | BGR to NV12, resize (direct/letterbox), NV12 splitting |
| `postprocess` | NMS, YOLO box/mask/keypoint/OBB decoding, coordinate scaling |
| `visualize` | Detection box, segmentation mask, rotated box, pose keypoint, classification result rendering |
| `inspect` | SoC name detection, model info printing |
| `nn_math` | Sigmoid, z-score normalization |
