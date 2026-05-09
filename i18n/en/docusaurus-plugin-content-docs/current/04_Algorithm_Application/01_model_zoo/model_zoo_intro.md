---
sidebar_position: 1
---

# 4.1.1 Model Zoo Overview

## Product Introduction

RDK Model Zoo is a collection of BPU (Brain Processing Unit) model examples and tools provided by D-Robotics for the RDK series development boards, designed for model deployment and intelligent application development, helping developers quickly get started with BPU and run through the model inference workflow.

The repository includes BPU-runnable models covering multiple intelligent application domains such as image classification, object detection, instance segmentation, pose estimation, OCR, and multimodal, and provides complete reference implementations from **original model (PyTorch/ONNX) → quantization conversion → inference execution → result parsing → example verification**, helping users understand and use BPU capabilities at minimal cost.

:::tip

Model Zoo GitHub repository: https://github.com/D-Robotics/rdk_model_zoo

:::

:::info

RDK Model Zoo is a community-driven open-source project. We highly welcome developers to contribute new model examples, optimize existing code, or improve documentation. If you have any suggestions for improvement, please join us by submitting a Pull Request (PR)!

:::

## Branch and Hardware Platform Mapping

Model Zoo provides corresponding branches by hardware platform. Directory structure, inference interfaces, and system requirements differ across branches:

| Target Hardware | Branch | Python Inference Interface | Entry Point |
| :--- | :--- | :--- | :--- |
| RDK X5 | [`rdk_x5`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x5) | `hbm_runtime` | `samples/vision/<sample>/README.md` |
| RDK X5 (legacy demos) | [`rdk_x5_legacy`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x5_legacy) | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | Target demo directory README |
| RDK X3 | [`rdk_x3`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x3) | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | `demos/<task>/<demo>/README.md` |
| RDK S Series | [`rdk_s`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_s) | `hbm_runtime` | `samples/<domain>/<sample>/README.md` |

## Branch Descriptions

### rdk_x5

The main delivery branch for RDK X5, requiring system version RDK OS >= 3.5.0.

- Python samples uniformly use the `hbm_runtime` interface, C++ samples use the `hb_dnn` interface
- Organized by `samples/vision/<model>` in a standardized structure
- Includes model download, conversion configs, Python/C++ runtime, evaluation tools, and test data

### rdk_x5_legacy

The legacy RDK X5 demo archive branch, used only for historical compatibility and old demo lookup.

- Organized by `demos/<task>/<demo>`
- Different demos use different inference interfaces (`bpu_infer_lib_x5` or the board-builtin `hobot_dnn.pyeasy_dnn`), as specified in each target directory's README

:::caution

`bpu_infer_lib_x5` and `hobot_dnn.pyeasy_dnn` have poor support for featuremap input models. If you need to use featuremap input models, please use `hbm_runtime` from the `rdk_x5` branch.

:::

### rdk_x3

Dedicated branch for RDK X3 devices.

- Organized by `demos/<task>/<demo>`
- Includes image classification, object detection, and instance segmentation examples
- The inference interface is chosen based on the target directory README, including `bpu_infer_lib_x3` and the board-builtin `hobot_dnn.pyeasy_dnn`

:::caution

`bpu_infer_lib_x3` and `hobot_dnn.pyeasy_dnn` have poor support for featuremap input models.

:::

Some demos support Jupyter Notebook interactive experience.

### rdk_s

Dedicated branch for RDK S100 / S600 series boards, requiring system version RDK OS >= 4.0.5.

- Organized by domain, including vision and speech examples
- The inference interface is `hbm_runtime` (same name as RDK X5, different underlying dependency: S series is based on `libhbucp`, X5 is based on `libdnn`)
- Legacy demos for the RDK S series are retained in the [RDK Model Zoo S](https://github.com/d-Robotics/rdk_model_zoo_s) repository

## Detailed Usage Guides by Platform

- RDK X3 usage guide: [4.1.2 RDK X3 Model Zoo Usage Guide](./rdk_x3_guide.md)
- RDK X5 usage guide: [4.1.3 RDK X5 Model Zoo Usage Guide](./rdk_x5_guide.md)
- RDK S Series usage guide: [4.1.4 RDK S Model Zoo Usage Guide](./rdk_s_guide.md)
- Inference interface reference: [4.1.5 Inference Interface Reference](./infer_api_ref.md)
