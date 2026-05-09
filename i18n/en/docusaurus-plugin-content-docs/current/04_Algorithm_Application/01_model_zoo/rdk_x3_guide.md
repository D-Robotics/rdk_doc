---
sidebar_position: 2
---

# 4.1.2 RDK X3 Model Zoo Usage Guide

## Branch and System Requirements

RDK X3 uses the `rdk_x3` branch. Recommended system version: RDK OS >= 3.0.0.

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x3
```

## Repository Directory Structure

The `rdk_x3` branch is organized by demo directory. Main structure:

```bash
rdk_model_zoo/
|-- demos/
|   |-- classification/          # Image classification
|   |   |-- GoogLeNet/
|   |   |-- MobileNetV1/
|   |   |-- MobileNetV2/
|   |   |-- MobileNetV4/
|   |   |-- MobileOne/
|   |   |-- RepGhost/
|   |   |-- RepVGG/
|   |   |-- RepViT/
|   |   `-- ResNet/
|   |-- detect/                  # Object detection
|   |   |-- FCOS/
|   |   |-- PaddleOCR/
|   |   |-- YOLOv10/
|   |   |-- YOLOv5/
|   |   `-- YOLOv8/
|   `-- Instance_Segmentation/   # Instance segmentation
|       `-- YOLOv8-Seg/
`-- resource/                    # Documentation resources
```

## Python Inference Interface

The RDK X3 branch provides two Python inference interfaces:

- **`bpu_infer_lib_x3`**: A Python inference library that requires manual installation
- **`hobot_dnn.pyeasy_dnn`**: A board-builtin Python inference interface

Which interface to use depends on the target demo directory README and source entry point.

:::caution

`bpu_infer_lib_x3` and `hobot_dnn.pyeasy_dnn` **have poor support for featuremap input models**. If you need to run featuremap input models, consider using C++ interfaces or upgrading to the RDK X5 platform with the `hbm_runtime` inference interface.

:::

### Installing bpu_infer_lib_x3

For RDK X3 demos that use `bpu_infer_lib_x3`, install with:

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x3/bpu_infer_lib_x3-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x3-1.0.3-py3-none-any.whl
```

Demos that use `hobot_dnn.pyeasy_dnn` use the board-builtin interface directly, no additional installation required.

## Quick Start

### Running a Demo

```bash
cd demos/<task>/<demo>
less README.md
# Follow the README to prepare models, install dependencies, and run inference.
```

### Task Directory Reference

| Task | Directory | Included Models |
| :--- | :--- | :--- |
| Image classification | `demos/classification` | GoogLeNet, MobileNetV1, MobileNetV2, MobileNetV4, MobileOne, RepGhost, RepVGG, RepViT, ResNet |
| Object detection | `demos/detect` | FCOS, PaddleOCR, YOLOv10, YOLOv5, YOLOv8 |
| Instance segmentation | `demos/Instance_Segmentation` | YOLOv8-Seg |

## Using Jupyter

Some RDK X3 demos provide Jupyter Notebooks for interactive execution:

```bash
# Install Jupyter Lab
pip install jupyterlab

# Start Jupyter Lab (replace IP with the actual board IP)
jupyter lab --allow-root --ip 192.168.1.10
```

After launching, open the link in a browser, navigate to the desired demo's notebook, and click Run to try it out.

:::note

All relative paths in programs are calculated from the model's directory.

:::
