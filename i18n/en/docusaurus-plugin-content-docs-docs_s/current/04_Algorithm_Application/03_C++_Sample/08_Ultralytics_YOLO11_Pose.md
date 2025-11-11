---
sidebar_position: 8
---

# Pose Estimation - Ultralytics YOLO11

This example demonstrates how to run the Ultralytics YOLO11 pose estimation model on the BPU to perform human keypoint detection and visualization. It supports model preprocessing, inference execution, and post-processing (including keypoint decoding, bounding box drawing, and keypoint annotation). The example code is located in the `/app/cdev_demo/bpu/04_pose_sample/01_ultralytics_yolo11_pose/` directory.

## Model Description
- **Overview**:

    Ultralytics YOLO11 Pose is an efficient, lightweight human keypoint detection model capable of simultaneous object detection and pose estimation (multi-keypoint prediction). It integrates Distribution Focal Loss (DFL) to enhance the localization accuracy of bounding boxes and keypoints, making it suitable for real-time multi-person pose estimation tasks.

- **HBM Model Name**: `yolo11n_pose_nashe_640x640_nv12.hbm`

- **Input Format**: NV12 format image (separated Y and UV planes), with resolution 640×640

- **Outputs**:

    - Bounding box coordinates (xyxy) for each detected person

    - Keypoint locations (K×2, x/y coordinates)

    - Confidence scores for each keypoint

    - Supports COCO human keypoint format (commonly 17 keypoints)

## Functionality Description
- **Model Loading**

    Loads the specified Ultralytics YOLO11 pose estimation model and automatically parses its metadata.

- **Input Preprocessing**

    Resizes the input BGR image to 640×640 and converts it to NV12 format (separated Y and UV planes) for model inference.

- **Inference Execution**

    Calls the `.infer()` interface to perform inference.

- **Result Post-processing**

    - Decodes bounding boxes from multi-scale outputs (using DFL bin decoding);

    - Decodes keypoint positions and confidence scores (K×2 + K);

    - Applies NMS to remove redundant detection boxes;

    - Maps keypoint coordinates and bounding boxes back to the original image dimensions;

    - Provides threshold control to display only high-confidence keypoints;

    - Supports image visualization, including drawing detection boxes and keypoints.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                     # CMake build script: target/dependency/include path configuration
|-- README.md                          # Usage instructions (this file)
|-- inc
|   `-- ultralytics_yolo11_pose.hpp    # Model wrapper header: declarations for load/preprocess/infer/postprocess interfaces
`-- src
    |-- main.cc                        # Program entry point: parses arguments → full pipeline → saves visualization results
    `-- ultralytics_yolo11_pose.cc     # Model implementation: decoding, NMS, keypoint post-processing, and coordinate restoration
```

## Build Project
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found during program execution, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_pose_nashe_640x640_nv12.hbm
```

## Parameter Description
| Parameter             | Description                                              | Default Value                                                   |
| --------------------- | -------------------------------------------------------- | --------------------------------------------------------------- |
| `--model_path`        | Path to the model file (`.hbm`)                          | `/opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm` |
| `--test_img`          | Path to the input test image                             | `/app/res/assets/bus.jpg`                                       |
| `--label_file`        | Class label file (one class name per line)               | `/app/res/labels/coco_classes.names`                            |
| `--score_thres`       | Confidence threshold (detections below this are filtered) | `0.25`                                                          |
| `--nms_thres`         | IoU threshold for class-wise NMS deduplication           | `0.7`                                                           |
| `--kpt_conf_thres`    | Keypoint visualization confidence threshold (points below this are hidden) | `0.5`                                                           |

## Quick Run
- **Run the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./ultralytics_yolo11_pose
        ```
    - Run with custom parameters:
        ```bash
        ./ultralytics_yolo11_pose \
        --model_path /opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm \
        --test_img   /app/res/assets/bus.jpg \
        --label_file /app/res/labels/coco_classes.names \
        --score_thres 0.25 \
        --nms_thres   0.7 \
        --kpt_conf_thres 0.5
        ```
- **View Results**

    Upon successful execution, results are overlaid on the original image and saved as `build/result.jpg`:
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