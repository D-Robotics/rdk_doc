---
sidebar_position: 14
---

# MIPI Camera YOLOv5x Inference

This example demonstrates real-time inference of Ultralytics YOLOv5x on the BPU, supporting image capture from a MIPI camera for object detection and full-screen visualization of detection results. The sample code is located in the `/app/cdev_demo/bpu/10_mipi_camera_sample/` directory.

## Feature Description

- **Model Loading**  
  Load models in `.hbm` format and initialize input/output information.

- **Camera Capture**  
  Initialize the VIO camera to capture NV12 images at 1920×1080 resolution.

- **HDMI Display**  
  Bind the image output channel to enable real-time display.

- **Image Preprocessing**  
  Split, resize, and convert NV12-format images into tensor formats required by the BPU.

- **BPU Inference**  
  Invoke BPU inference tasks using the `.infer()` method.

- **Post-processing**  
  Includes output decoding, confidence thresholding, NMS suppression, and coordinate rescaling.

- **Visualization**  
  Draw bounding boxes and class labels onto the overlay layer.


## Model Description  
Refer to the [Ultralytics YOLOv5x Object Detection Example section](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).


## Environment Dependencies  
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Hardware Requirements
- The MIPI camera interface uses auto-detection mode. Only one MIPI camera (connected to any MIPI port) can be used when running this sample; connecting multiple cameras will cause errors.
- Currently, this sample only supports MIPI sensors: IMX219 and SC230AI.
- For instructions on installing the MIPI camera, refer to the [Camera Expansion Board – MIPI Camera Interface](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md) section.

## Directory Structure
```text
.
|-- CMakeLists.txt                     # CMake build script: targets/dependencies/includes and linking
|-- README.md                          # Usage instructions (current file)
|-- inc
|   `-- ultralytics_yolov5x.hpp        # YOLOv5x inference wrapper header: interfaces for loading/preprocessing/inference/post-processing
`-- src
    |-- main.cc                        # Program entry point: VIO stream → inference → drawing on Display layer
    `-- ultralytics_yolov5x.cc         # Inference implementation: letterbox, NV12 tensor writing, decoding, NMS, coordinate restoration
```

## Build Instructions
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Description
| Parameter        | Description                                           | Default Value                                               |
| ---------------- | ----------------------------------------------------- | ---------------------------------------------------------- |
| `--width`        | Sensor original width (used for VIO parameters/display scaling) | `1920`                                                 |
| `--height`       | Sensor original height (used for VIO parameters/display scaling) | `1080`                                                 |
| `--model_path`   | Path to BPU quantized model (`.hbm`)                  | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`  |
| `--label_file`   | Class label file (one class name per line)            | `/app/res/labels/coco_classes.names`                      |
| `--score_thres`  | Confidence threshold                                  | `0.25`                                                 |
| `--nms_thres`    | IoU threshold for NMS                                 | `0.45`                                                |

## Quick Start  
Note: This program must run in a desktop environment.

- **Run the model**
  - Ensure you are in the `build` directory.
  - Run with default parameters:
    ```bash
    ./mipi_camera
    ```
  - Run with custom parameters:
    ```bash
    ./mipi_camera \
        --width 1920 --height 1080 \
        --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --label_file /app/res/labels/coco_classes.names \
        --score_thres 0.25 \
        --nms_thres 0.45
    ```

- **Terminate Execution**  
  Press `Ctrl+C` in the terminal.

- **View Results**  
  Upon successful execution, the screen will display real-time object detection results.

## Notes
- This program must run in a desktop environment.
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