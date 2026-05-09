---
sidebar_position: 9
---

# Instance Segmentation - Ultralytics YOLOE11

This example demonstrates how to run the Ultralytics YOLOE11 instance segmentation model on the BPU using `hbm_runtime`. The program implements a complete pipeline from input image preprocessing, model inference, post-processing, to result visualization. The example code is located in the `/app/pydev_demo/05_open_instance_seg_sample/01_yoloe11_seg/` directory.

## Model Description
- **Introduction**:

    Ultralytics YOLOE11 is a high-performance on-device instance segmentation model suitable for open-vocabulary object detection and segmentation tasks. By leveraging multi-scale feature extraction, dense classification, and prototype-based mask generation, this model effectively identifies objects in images and outputs precise instance segmentation results. This example uses a lightweight version of Ultralytics YOLOE11 with an input image size of 640x640, supporting generalized object classification and segmentation across 4,585 categories.

- **HBM Model Name**: `yoloe_11s_seg_pf_nashe_640x640_nv12.hbm`

- **Input Format**: NV12, size 640x640

- **Outputs**:

    - Bounding boxes (in xyxy format)
    - Class IDs and confidence scores
    - Instance segmentation masks (one mask per instance)

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm
    ```

## Functionality Description
- **Model Loading**

    Uses `hbm_runtime` to load the specified quantized model and parse metadata such as input/output names, shapes, and quantization parameters.

- **Input Preprocessing**

    Resizes the BGR image to 640x640, converts it to NV12 format (separated Y and UV planes), and constructs the input tensor for inference.

- **Inference Execution**

    Calls the `.run()` interface to perform forward inference, supporting scheduling strategies such as setting execution priority and binding to specific BPU cores.

- **Result Post-processing**

    Performs post-processing on multi-scale outputs, including:

    - Classification confidence filtering (based on score threshold)
    - DFL bounding box decoding
    - Prototype mask fusion and mask generation
    - NMS filtering and result merging
    - Rescaling bounding boxes and masks back to the original image dimensions
    - Optional morphological opening operation on masks and contour drawing

## Environment Dependencies
This example has no special environment requirements—only the dependencies specified in the pydev environment need to be installed.
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── yoloe11_seg.py              # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter             | Description                                                  | Default Value                                                      |
| --------------------- | ------------------------------------------------------------ | ------------------------------------------------------------------ |
| `--model-path`        | Path to the BPU quantized model (*.hbm)                      | `/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm` |
| `--test-img`          | Path to the input test image                                 | `/app/res/assets/office_desk.jpg`                                  |
| `--label-file`        | Path to the class label file (one class per line)            | `/app/res/labels/coco_extended.names`                              |
| `--img-save-path`     | Path to save the inference result image                      | `result.jpg`                                                       |
| `--priority`          | Model scheduling priority (0–255)                            | `0`                                                                |
| `--bpu-cores`         | BPU core IDs to use (e.g., `--bpu-cores 0 1`)                | `[0]`                                                              |
| `--nms-thres`         | IoU threshold for Non-Maximum Suppression (NMS)              | `0.7`                                                              |
| `--score-thres`       | Object detection confidence threshold                        | `0.25`                                                             |
| `--is-open`           | Whether to apply morphological opening on masks              | `False`                                                            |
| `--is-point`          | Whether to draw contour points on mask edges                 | `False`                                                            |

## Quick Start
- **Run the model**
    - Using default parameters:
        ```bash
        python yoloe11_seg.py
        ```
    - Running with custom parameters:
        ```bash
        python yoloe11_seg.py \
        --model-path /opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --test-img /app/res/assets/office_desk.jpg \
        --label-file /app/res/labels/coco_extended.names \
        --img-save-path result.jpg \
        --nms-thres 0.7 \
        --score-thres 0.25 \
        --is-open False \
        --is-point False
        ```
- **View Results**

    Upon successful execution, the results will be overlaid on the original image and saved to the path specified by `--img-save-path`.
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.

## License
    ```license
    Copyright (C) 2025, XiangshunZhao D-Robotics.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    ```