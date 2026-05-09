---
sidebar_position: 7
---

# Instance Segmentation - Ultralytics YOLO11

This example demonstrates how to run the YOLOv11 instance segmentation model on the BPU using `hbm_runtime`, supporting image preprocessing, inference, and post-processing (parsing outputs and overlaying colored segmentation masks). The example code is located in the `/app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg/` directory.

## Model Description
- **Overview**:

    Ultralytics YOLO11 is a lightweight object detection and instance segmentation model based on the YOLO series, integrating both anchor-free and anchor-based design concepts along with distributional regression strategies. This variant supports simultaneous output of bounding boxes, class probabilities, and high-quality pixel-level masks, making it suitable for real-time multi-object detection and segmentation tasks.

- **HBM Model Name**: `yolo11n_seg_nashe_640x640_nv12.hbm`

- **Input Format**: NV12 format image (separate Y/UV planes), sized 640x640

- **Outputs**:

    - Object detection results (bounding box + class + confidence score)
    - Instance segmentation masks (one mask per detected object)

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_seg_nashe_640x640_nv12.hbm
    ```

## Functionality Description
- **Model Loading**

    Uses `hbm_runtime` to load the quantized Ultralytics YOLO11 instance segmentation model and parse runtime metadata such as input/output tensor names, shapes, and quantization parameters.

- **Input Preprocessing**

    Resizes the input BGR image to 640×640 and converts it to NV12 format (with separate Y and UV planes) to meet the model's input requirements.

- **Inference Execution**

    Triggers forward inference via the `.run()` method, supporting configurable scheduling parameters (BPU core binding and priority). The inference output includes multi-scale class scores, bounding box regression values, mask coefficients, and a global mask prototype tensor.

- **Result Post-processing**

    - Filters detection candidates using a confidence threshold and decodes bounding boxes and mask coefficients;
    - Merges outputs from all scales and applies NMS to obtain final detections;
    - Reconstructs per-object masks using the mask prototype and coefficients;
    - Rescales both masks and bounding boxes back to the original image dimensions;
    - Optionally applies morphological operations (opening) to refine mask edges;
    - Final outputs include bounding boxes, class labels, confidence scores, and pixel-level instance masks.

## Environment Dependencies
This example has no special environment requirements—just ensure that the dependencies listed in `pydev` are installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── ultralytics_yolo11_seg.py   # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter           | Description                                      | Default Value |
|---------------------|--------------------------------------------------|--------------------------------------|
| `--model-path`      | Path to the model file (.hbm format)             | `/opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm` |
| `--test-img`        | Path to the test image                           | `/app/res/assets/office_desk.jpg`        |
| `--label-file`      | Path to the class label file                     | `/app/res/labels/coco_classes.names`     |
| `--img-save-path`   | Path to save the output result image             | `result.jpg`                          |
| `--priority`        | Model inference priority (0~255)                 | `0`                                   |
| `--bpu-cores`       | BPU core IDs                                     | `[0]`                                 |
| `--nms-thres`       | NMS IoU threshold                                | `0.7`                                 |
| `--score-thres`     | Confidence score threshold                       | `0.25`                                |
| `--is-open`         | Whether to apply morphological opening to masks  | `True`                                |
| `--is-point`        | Whether to draw points on mask boundaries        | `True`                                |

## Quick Start
- **Run the Model**
    - Using default parameters:
        ```bash
        python ultralytics_yolo11_seg.py
        ```
    - With custom parameters:
        ```bash
        python ultralytics_yolo11_seg.py \
        --model-path /opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm \
        --test-img /app/res/assets/office_desk.jpg \
        --label-file /app/res/labels/coco_classes.names \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --nms-thres 0.7 \
        --score-thres 0.25 \
        --is-open True \
        --is-point True
        ```

- **View Results**

    Upon successful execution, the results will be overlaid on the original image and saved to the path specified by `--img-save-path`:
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