---
sidebar_position: 13
---

# USB Camera YOLOv5x Inference

A real-time inference example of Ultralytics YOLOv5x based on the BPU, supporting reading frames from a USB camera, performing object detection, and visualizing detection results in full-screen mode. The sample code is located in the `/app/cdev_demo/bpu/09_usb_camera_sample/` directory.

## Feature Description
- **Model Loading**

    Load the specified `.hbm` model file and extract model-related metadata.

- **Camera Capture**

    Automatically scan devices under `/dev/video*`, open the first available USB camera, and configure it to use MJPEG encoding, 1080p resolution, and 30 FPS.

- **Image Preprocessing**

    Resize the BGR image to the model's input resolution (using letterbox mode or standard scaling) and convert it to NV12 format.

- **Inference Execution**

    Submit the input tensor via the `infer()` method and perform forward computation on the BPU.

- **Post-processing**

    Includes decoding quantized outputs, filtering candidate boxes (based on a score threshold), applying NMS for deduplication, and mapping bounding box coordinates back to the original image dimensions.

- **Visualization**

    Draw detection boxes along with their class labels and confidence scores onto the image, and display the result in a full-screen window with support for real-time processing and exit control.

## Model Description
    Refer to the [Ultralytics YOLOv5x Object Detection Example section](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                 # CMake build script: target/dependency/include/link configuration
|-- README.md                      # Usage instructions (current file)
|-- inc
|   `-- ultralytics_yolov5x.hpp    # YOLOv5x inference wrapper header: interfaces for loading/preprocessing/inference/postprocessing
`-- src
    |-- main.cc                    # Program entry point: camera detection → frame capture → inference → drawing → display (full-screen window)
    `-- ultralytics_yolov5x.cc     # Inference implementation: letterbox, NV12 tensor writing, decoding, NMS, and box coordinate restoration
```

## Build the Project
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Description
| Parameter            | Description                                          | Default Value                                                |
| -------------------- | ---------------------------------------------------- | ------------------------------------------------------------ |
| `--video_device`     | Specify video device (e.g., `/dev/video0`; leave empty for auto-detection) | `""` (empty: automatically detect the first openable device under `/dev/video*`) |
| `--model_path`       | Path to the BPU quantized model (`.hbm`)             | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`       |
| `--label_file`       | Class label file (one class name per line)           | `/app/res/labels/coco_classes.names`                         |
| `--score_thres`      | Confidence score threshold                           | `0.25`                                                       |
| `--nms_thres`        | IoU threshold for NMS                                | `0.45`                                                       |

## Quick Start
Note: This program must run in a desktop environment.
- **Run the model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./usb_camera
        ```
    - Run with custom parameters:
        ```bash
        ./usb_camera \
            --video_device /dev/video0 \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.45
        ```
- **Exit the program**

    Place your mouse cursor inside the display window and press the `q` key to exit.

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