---
sidebar_position: 5
---

# Object Detection - Ultralytics YOLO11

This example is based on the Ultralytics YOLO11 model and performs object detection on images using C/C++. It supports image preprocessing, inference, post-processing (including decoding, confidence filtering, and NMS), and saving the result image. The example code is located in the `/app/cdev_demo/bpu/02_detection_sample/02_ultralytics_yolo11/` directory.

## Model Description
- **Overview**:

    Ultralytics YOLO11 is a lightweight anchor-based object detection model that integrates both anchor-free and anchor-based concepts, offering fast inference and precise localization. During the regression phase, this model employs discrete regression bins combined with softmax classification and a decoding mechanism to enhance localization accuracy. Ultralytics YOLO11 is suitable for deploying small models in real-time scenarios, such as security surveillance and industrial inspection.

- **HBM Model Name**: yolo11n_detect_nashe_640x640_nv12.hbm

- **Input Format**: NV12 format, sized 640x640 (separated Y and UV planes)

- **Output**: Multi-scale feature maps, each containing a class score tensor and a discrete bounding box regression tensor. The final output includes bounding box coordinates, class IDs, and confidence scores.

## Functionality Description
- **Model Loading**

   Load the quantized Ultralytics YOLO11 model and extract model metadata for subsequent inference steps.

- **Input Preprocessing**

    Resize the original BGR image to 640×640, convert it to NV12 format (separated Y and UV planes), and construct a nested input tensor dictionary compatible with the inference interface.

- **Inference Execution**

    Run forward inference using the `.infer()` method, producing classification and regression tensors from multiple scale branches.

- **Result Post-processing**

    - Dequantize the quantized output back to float32;
    - Filter classification scores for each scale branch, retaining candidate boxes with confidence scores above a specified threshold;
    - Decode bounding boxes using a multi-bin regression algorithm;
    - Merge candidate boxes from all scales and apply NMS (Non-Maximum Suppression) to remove redundant boxes;
    - Map detected boxes from the model input coordinate system back to the original image dimensions;
    - Optionally draw detection results and save the output image file.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure

```text
.
├── CMakeLists.txt               # CMake build script
├── README.md                    # Usage instructions
├── inc
│   └── ultralytics_yolo11.hpp   # YOLO11 model wrapper header file
└── src
    ├── main.cc                  # Main program entry point
    └── ultralytics_yolo11.cc    # YOLO11 model implementation
```

## Building the Project
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found during program execution, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_detect_nashe_640x640_nv12.hbm
```

## Parameter Description

| Parameter          | Description                                                  | Default Value                                                                 |
| ------------------ | ------------------------------------------------------------ | ----------------------------------------------------------------------------- |
| `--model_path`     | Path to the model file (.hbm format)                         | `/opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm`           |
| `--test_img`       | Path to the input test image                                 | `/app/res/assets/kite.jpg`                                                    |
| `--label_file`     | Path to the class label file (one class name per line)       | `/app/res/labels/coco_classes.names`                                          |
| `--score_thres`    | Confidence threshold for filtering (objects below this value will be discarded) | `0.25`                                                                        |
| `--nms_thres`      | IoU threshold for Non-Maximum Suppression (NMS)              | `0.7`                                                                         |


## Quick Start
- **Running the Model**
    - Using default parameters:
        ```bash
        ./ultralytics_yolo11
        ```
    - Running with custom parameters:
        ```bash
        ./ultralytics_yolo11 \
            --model_path /opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm \
            --test_img /app/res/assets/kite.jpg \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.7
        ```
- **Viewing Results**

    Upon successful execution, bounding boxes will be drawn on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`, which users can inspect directly.

- For more information on deployment methods or model support, please refer to the official documentation or contact platform technical support.

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