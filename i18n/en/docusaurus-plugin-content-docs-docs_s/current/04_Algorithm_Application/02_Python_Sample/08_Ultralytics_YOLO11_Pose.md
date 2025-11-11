---
sidebar_position: 8
---

# Pose Estimation - Ultralytics YOLO11

This example demonstrates how to run the Ultralytics YOLO11 pose estimation model on the BPU using `hbm_runtime`, enabling human keypoint detection and visualization. It supports model preprocessing, inference execution, and post-processing (including keypoint decoding, bounding box drawing, and keypoint annotation). The example code is located in the `/app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose/` directory.

## Model Description
- Introduction:

    Ultralytics YOLO11 Pose is an efficient and lightweight human keypoint detection model capable of performing object detection and pose estimation (multi-keypoint prediction) simultaneously. It integrates Distribution Focal Loss (DFL) to enhance the localization accuracy of bounding boxes and keypoints, making it suitable for multi-person pose recognition tasks in real-time scenarios.

- HBM Model Name: yolo11n_pose_nashe_640x640_nv12.hbm

- Input Format: NV12 format image (separated Y and UV planes), resolution 640×640

- Output:

    - Bounding box coordinates (xyxy) for each person

    - Keypoint locations (K×2, x/y coordinates)

    - Confidence scores for each keypoint

    - Supports COCO human keypoint format (typically 17 keypoints)

- Model Download URL (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_pose_nashe_640x640_nv12.hbm
    ```

## Functionality Description
- Model Loading

    Uses `hbm_runtime` to load the specified Ultralytics YOLO11 pose estimation model and automatically parses the model's input/output tensor names, shapes, and quantization parameters.

- Input Preprocessing

    Resizes the input BGR image to 640×640 and converts it to NV12 format (separated Y and UV planes) for model inference.

- Inference Execution

    Calls the `.run()` interface to perform inference, supporting configuration of scheduling priority and BPU core binding via `set_scheduling_params()`.

- Result Post-processing

    - Decodes bounding boxes from multi-scale outputs (using DFL binning decoding);

    - Decodes keypoint locations and confidence scores (K×2 + K);

    - Applies Non-Maximum Suppression (NMS) to remove redundant detection boxes;

    - Maps keypoint coordinates and bounding boxes back to the original image dimensions;

    - Provides threshold control to display only high-confidence keypoints;

    - Supports image visualization, including drawing detection boxes and keypoints.

## Environment Dependencies
This example has no special environment requirements; ensure that the dependencies in pydev are installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── ultralytics_yolo11_pose.py    # Main inference script
└── README.md                     # Usage instructions
```

## Parameter Description
| Parameter             | Description                                                  | Default Value                                |
| --------------------- | ------------------------------------------------------------ | -------------------------------------------- |
| `--model-path`        | Path to the model file (`.hbm` format)                       | `/opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm` |
| `--test-img`          | Path to the test image                                       | `/app/res/assets/bus.jpg`                    |
| `--label-file`        | Path to the class label file (one class name per line)       | `/app/res/labels/coco_classes.names`         |
| `--img-save-path`     | Path to save the detection result                            | `result.jpg`                                 |
| `--priority`          | Model scheduling priority (0–255; higher value = higher priority) | `0`                                          |
| `--bpu-cores`         | List of BPU core IDs to use for inference (e.g., `--bpu-cores 0 1`) | `[0]`                                        |
| `--nms-thres`         | IoU threshold for Non-Maximum Suppression (NMS)              | `0.7`                                        |
| `--score-thres`       | Object confidence threshold (objects below this are filtered out) | `0.25`                                       |
| `--kpt-conf-thres`    | Keypoint visualization confidence threshold (keypoints below this are not displayed) | `0.5`                                        |

## Quick Start
- Run the model
    - With default parameters:
        ```bash
        python ultralytics_yolo11_pose.py
        ```
    - With custom parameters:
        ```bash
        python ultralytics_yolo11_pose.py \
        --model-path /opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm \
        --test-img /app/res/assets/bus.jpg \
        --label-file /app/res/labels/coco_classes.names \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --score-thres 0.25 \
        --nms-thres 0.7 \
        --kpt-conf-thres 0.5
        ```
- View results

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