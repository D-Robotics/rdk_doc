---
sidebar_position: 14
---

# MIPI Camera YOLOv5x Inference

This is a real-time inference example based on `hbm_runtime` using Ultralytics YOLOv5x. It supports reading frames from a MIPI camera, performing object detection, and visualizing detection results in full-screen mode. The sample code is located in the `/app/pydev_demo/10_mipi_camera_sample/` directory.

## Features

- **Model Loading**  
  Load a `.hbm` format model via `hbm_runtime` and initialize input/output information.

- **Camera Capture**  
  Initialize the VIO camera using `srcampy.Camera()` and capture NV12 images at 1920×1080 resolution.

- **HDMI Display**  
  Bind the image output channel using `srcampy.Display()` to enable real-time display.

- **Image Preprocessing**  
  Separate, resize, and convert the NV12-formatted image into the tensor format required by the BPU.

- **BPU Inference**  
  Invoke the BPU to execute inference tasks via the `run()` method.

- **Post-processing**  
  Includes output decoding, confidence thresholding, NMS suppression, and coordinate scaling.

- **Visualization**  
  Draw detection bounding boxes and class labels onto an overlay layer.


## Model Description
Refer to [Ultralytics YOLOv5x Object Detection Example Summary](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).


## Environment Dependencies
- Ensure the dependencies in pydev are installed:
    ```bash
    pip install -r ../requirements.txt
    ```

## Hardware Requirements
- The MIPI camera interface uses auto-detection mode. Only one MIPI camera (connected to any MIPI port) is allowed when running this sample; connecting multiple cameras will cause an error.
- Currently, this sample only supports the following MIPI sensors: IMX219, SC230AI.
- For MIPI camera installation instructions, refer to the section [Camera Expansion Board – MIPI Camera Interface](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md).

## Directory Structure

```text
.
├── 01_mipi_camera_yolov5x.py       # Real-time object detection and display using YOLOv5X model with camera input
├── 02_mipi_camera_dump.py          # Save captured camera frames to YUV files (unrelated to model inference)
├── 03_mipi_camera_scale.py         # Resize local YUV images (unrelated to model inference)
├── 04_mipi_camera_crop_scale.py    # Crop and resize local YUV images (unrelated to model inference)
├── 05_mipi_camera_streamer.py      # Stream camera images to HDMI display in real time (streaming test, unrelated to model inference)
└── README.md                       # This file, containing script descriptions, parameter details, and usage instructions
```

## Parameter Description
| Parameter        | Description                                      | Default Value                                              |
| ---------------- | ------------------------------------------------ | ---------------------------------------------------------- |
| `--model-path`   | Path to the BPU quantized model (`.hbm`)         | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`     |
| `--priority`     | Inference priority (0–255, where 255 is highest)  | `0`                                                        |
| `--bpu-cores`    | List of BPU core indices (e.g., `0 1`)           | `[0]`                                                      |
| `--label-file`   | Path to class label file                         | `/app/res/labels/coco_classes.names`                       |
| `--nms-thres`    | IoU threshold for Non-Maximum Suppression (NMS)  | `0.45`                                                     |
| `--score-thres`  | Detection confidence threshold                   | `0.25`                                                     |


## Quick Start
Note: This program must be run in a desktop environment.

- **Run the model**
    - With default parameters:
        ```bash
        python 01_mipi_camera_yolov5x.py
        ```
    - With custom parameters:
        ```bash
        python 01_mipi_camera_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```

- **Exit the program**  
  Press `Ctrl+C` in the terminal.

- **View results**  
  Upon successful execution, the screen will display real-time object detection results.

## Notes
- This program must be run in a desktop environment.
- If the specified model path does not exist, try checking the directory `/opt/hobot/model/s100/basic/`.

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