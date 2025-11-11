---
sidebar_position: 4
---

# Object Detection - Ultralytics YOLOv5x

This example demonstrates how to perform image object detection on the BPU using a quantized Ultralytics YOLOv5x model. It supports preprocessing, postprocessing, NMS (Non-Maximum Suppression), drawing of final bounding boxes, and saving results. The example code is located in the directory `/app/cdev_demo/bpu/02_detection_sample/01_ultralytics_yolov5x/`.


## Model Description
- Overview:

    Ultralytics YOLOv5x is a high-performance object detection model. The name "YOLO" stands for "You Only Look Once," enabling simultaneous object localization and classification in a single forward pass. Among the YOLOv5 variants, YOLOv5x is the largest, featuring more network parameters and delivering high detection accuracy, making it suitable for scenarios demanding high precision. The Ultralytics YOLOv5x model divides the input image into multiple grids, with each grid predicting multiple anchors' class probabilities and bounding boxes. This model has been quantized into the HBM format compatible with the BPU chip and accepts NV12 input images of size 672×672.

- HBM Model Name: `yolov5x_672x672_nv12.hbm`

- Input Format: NV12, size 672x672 (separate Y and UV planes)

- Output: N bounding boxes, each represented as a triplet (class index, confidence score, bounding box coordinates)


## Functionality Description
- Model Loading

    Load the quantized Ultralytics YOLOv5x model, parse relevant model information, and prepare configurations for subsequent inference.

- Input Preprocessing

    Resize the input image to 672x672, convert it to NV12 format (separate Y and UV planes), and organize the input as a nested dictionary to match the inference interface.

- Inference Execution

    Run the inference process using the `.infer()` method.

- Result Postprocessing

    - Dequantize the output tensors;
    - Decode YOLO outputs to obtain predicted bounding boxes, confidence scores, and class indices;
    - Apply an initial filter based on a confidence score threshold;
    - Perform NMS (Non-Maximum Suppression) to remove redundant boxes;
    - Map predicted bounding box coordinates back to the original image dimensions;
    - Overlay detection boxes and save the resulting image.


## Environment Dependencies
Ensure the following dependencies are installed on your system:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
├── CMakeLists.txt                 # Build configuration file
├── README.md                      # Usage instructions
├── inc
│   └── ultralytics_yolov5x.hpp     # Class definition for YOLOv5x model wrapper
└── src
    ├── main.cc                     # Inference entry point (argument parsing and execution)
    └── ultralytics_yolov5x.cc      # Implementation of YOLOv5x inference logic
```

## Building the Project
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Description
| Parameter           | Description                                      | Default Value                                                  |
| ------------------- | ------------------------------------------------ | -------------------------------------------------------------- |
| `--model-path`      | Path to the model file (.hbm format)             | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`         |
| `--test-img`        | Path to the test image                           | `/app/res/assets/kite.jpg`                                     |
| `--label-file`      | Path to the class label file                     | `/app/res/labels/coco_classes.names`                           |
| `--score-thres`     | Confidence score threshold (filters low-score boxes) | `0.25`                                                     |
| `--nms-thres`       | IoU threshold for NMS (Non-Maximum Suppression)  | `0.45`                                                         |

## Quick Start
- Running the Model
    - Using default parameters:
        ```bash
        ./ultralytics_yolov5x
        ```
    - Running with custom parameters:
        ```bash
        ./ultralytics_yolov5x \
            --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --test-img /app/res/assets/kite.jpg \
            --label-file /app/res/labels/coco_classes.names \
            --score-thres 0.25 \
            --nms-thres 0.45
        ```
- Viewing Results

    Upon successful execution, detection bounding boxes will be drawn on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`, which users can inspect directly.

- For more information about deployment options or model support, please refer to the official documentation or contact platform technical support.

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