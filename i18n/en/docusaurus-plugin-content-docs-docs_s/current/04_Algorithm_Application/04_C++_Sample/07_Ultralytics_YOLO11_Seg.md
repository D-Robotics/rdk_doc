---
sidebar_position: 7
---

# Instance Segmentation - Ultralytics YOLO11

This example demonstrates how to run the YOLOv11 instance segmentation model on the BPU, supporting functionalities such as image preprocessing, inference, and post-processing (parsing outputs and overlaying colored segmentation masks). The example code is located in the directory `/app/cdev_demo/bpu/03_instance_segmentation_sample/02_ultralytics_yolo11_seg/`.

## Model Description
- **Introduction**:

    Ultralytics YOLO11 is a lightweight object detection and instance segmentation model based on the YOLO series, integrating both anchor-free and anchor-based design concepts along with distributional regression strategies. This variant is tailored for instance segmentation and simultaneously outputs bounding boxes, class probabilities, and high-quality pixel-level masks, making it suitable for real-time multi-object detection and segmentation tasks.

- **HBM Model Name**: yolo11n_seg_nashe_640x640_nv12.hbm

- **Input Format**: NV12 format image (separate Y/UV planes), resolution 640x640

- **Outputs**:

    - Object detection results (bounding box + class + confidence score)

    - Instance segmentation masks (one mask per detected object)

## Functionality Overview
- **Model Loading**

    Loads the quantized Ultralytics YOLO11 instance segmentation model and parses runtime metadata.

- **Input Preprocessing**

    Resizes the input BGR image to 640×640 and converts it to NV12 format (separate Y and UV planes) to meet the model's input requirements.

- **Inference Execution**

    Triggers forward inference via the `.infer()` method. The inference output includes multi-scale class scores, bounding box regressions, mask coefficients, and a global mask prototype tensor.

- **Result Post-processing**

    - Filters detection candidates using a confidence threshold and decodes bounding boxes and mask coefficients;
    
    - Merges outputs from all scales and applies Non-Maximum Suppression (NMS) to obtain final detections;
    
    - Reconstructs each object's mask using the mask prototype and corresponding coefficients;
    
    - Rescales both masks and bounding boxes back to the original image dimensions;
    
    - Optionally applies morphological operations (e.g., opening) to refine mask edges;
    
    - Final outputs include bounding boxes, class labels, confidence scores, and pixel-level instance masks.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                 # CMake build script
|-- README.md                      # Usage instructions (this file)
|-- inc
|   `-- ultralytics_yolo11_seg.hpp # Header file for YOLO11_Seg inference wrapper (interfaces for loading, preprocessing, inference, and post-processing)
`-- src
    |-- main.cc                    # Main entry point: parses arguments, executes full pipeline, and saves visualization results
    `-- ultralytics_yolo11_seg.cc  # Implementation of YOLO11_Seg inference: decoding, NMS, mask generation, scaling, etc.
```

## Building the Project
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found at runtime, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_seg_nashe_640x640_nv12.hbm
```

## Parameter Description
| Parameter          | Description                                      | Default Value                                                       |
| ------------------ | ------------------------------------------------ | ------------------------------------------------------------------- |
| `--model_path`     | Path to the model file (`.hbm`)                  | `/opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm`   |
| `--test_img`       | Path to the input test image                     | `/app/res/assets/office_desk.jpg`                                  |
| `--label_file`     | Path to the class label file                     | `/app/res/labels/coco_classes.names`                               |
| `--score_thres`    | Confidence threshold (detections below this are discarded) | `0.25`                                                    |
| `--nms_thres`      | IoU threshold for class-wise NMS (to remove duplicate detections) | `0.7`                                             |

## Quick Start
- **Running the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./ultralytics_yolo11_seg
        ```
    - Run with custom parameters:
        ```bash
        ./ultralytics_yolo11_seg \
            --model_path /opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm \
            --test_img /app/res/assets/office_desk.jpg \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.7
        ```

- **Viewing Results**

    Upon successful execution, results are overlaid on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`; users can inspect it directly.

- For more information on deployment options or model support, please refer to the official documentation or contact platform technical support.

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