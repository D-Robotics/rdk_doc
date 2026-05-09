---
sidebar_position: 4
---

# Object Detection - Ultralytics YOLOv5x

This example demonstrates how to perform object detection on images using a quantized Ultralytics YOLOv5x model on the BPU. It supports preprocessing, postprocessing, NMS (Non-Maximum Suppression), drawing bounding boxes, and saving results. The example code is located in the `/app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x/` directory.


## Model Description
- **Overview**:

    Ultralytics YOLOv5x is a high-performance object detection model. The name "YOLO" stands for "You Only Look Once," enabling simultaneous object localization and classification in a single forward pass. YOLOv5x is the largest variant in the YOLOv5 series, featuring more network parameters and delivering high detection accuracy, making it suitable for scenarios demanding high precision. The model divides the input image into a grid and predicts multiple anchor boxes per grid cell, each with associated class probabilities and bounding box coordinates. This model has been quantized into the HBM format optimized for BPU chips, accepting NV12 input images of size 672×672.

- **HBM Model Name**: `yolov5x_672x672_nv12.hbm`

- **Input Format**: NV12, size 672x672 (separate Y and UV planes)

- **Output**: N bounding boxes, each represented as a triplet (class index, confidence score, bounding box coordinates)


## Functionality Description
- **Model Loading**

    Load the quantized Ultralytics YOLOv5x model via `hbm_runtime`, parsing model name, input/output names, shapes, and quantization parameters to prepare for inference.

- **Input Preprocessing**

    Resize the input image to 672x672, convert it to NV12 format (with separate Y and UV planes), and organize the input as a nested dictionary to match the inference interface.

- **Inference Execution**

    Run inference using the `.run()` method, supporting configuration of inference priority and BPU core binding (e.g., core0/core1).

- **Result Postprocessing**

    - Dequantize the output tensors;
    - Decode YOLO outputs to obtain predicted bounding boxes, confidence scores, and class indices;
    - Apply initial filtering based on a confidence score threshold;
    - Perform NMS (Non-Maximum Suppression) to remove redundant boxes;
    - Map predicted bounding box coordinates back to the original image dimensions;
    - Overlay detection boxes and save the resulting image.


## Environment Dependencies
This example has no special environment requirements—just ensure the dependencies from `pydev` are installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure

```text
.
├── ultralytics_yolov5x.py      # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description

| Parameter         | Description                                                  | Default Value                                      |
|-------------------|--------------------------------------------------------------|----------------------------------------------------|
| `--model-path`    | Path to the model file (.hbm format)                         | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--test-img`      | Path to the test image                                       | `/app/res/assets/kite.jpg`                         |
| `--label-file`    | Path to the class label file (one class per line)            | `/app/res/labels/coco_classes.names`               |
| `--img-save-path` | Path to save the detection result image                      | `result.jpg`                                       |
| `--priority`      | Model scheduling priority (0–255)                            | `0`                                                |
| `--bpu-cores`     | List of BPU core IDs to use (e.g., `--bpu-cores 0 1`)        | `[0]`                                              |
| `--nms-thres`     | Non-Maximum Suppression (NMS) threshold                      | `0.45`                                             |
| `--score-thres`   | Confidence score threshold                                   | `0.25`                                             |


## Quick Start
- **Run the model**
    - With default parameters:
        ```bash
        python ultralytics_yolov5x.py
        ```
    - With custom parameters:
        ```bash
        python ultralytics_yolov5x.py \
            --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --test-img /app/res/assets/kite.jpg \
            --label-file /app/res/labels/coco_classes.names \
            --img-save-path result.jpg \
            --priority 0 \
            --bpu-cores 0 \
            --nms-thres 0.45 \
            --score-thres 0.25
        ```
- **View Results**

    Upon successful execution, bounding boxes will be drawn on the input image and saved to the path specified by `--img-save-path`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- If the specified model path does not exist, try checking in `/opt/hobot/model/s100/basic/`.

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