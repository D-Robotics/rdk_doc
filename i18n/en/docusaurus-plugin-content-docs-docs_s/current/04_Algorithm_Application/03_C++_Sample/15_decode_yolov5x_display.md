---
sidebar_position: 15
---

# Video Decoding and YOLOv5x Inference

This example demonstrates an end-to-end pipeline on platforms such as the RDK S100, combining SP decoding/display/VIO with BPU to achieve:
Local H.264 file → Hardware decoding (NV12) → YOLOv5x inference → Overlay bounding boxes onto the display layer.  
The example code is located in the `/app/cdev_demo/bpu/11_decode_yolov5x_display_sample` directory.

## Functionality Overview

- **Model Loading**  
  Load the model and obtain input/output metadata.

- **Preprocessing**  
  Convert the NV12 frame obtained from VIO to BGR (`cv::cvtColor`), apply letterboxing/resizing, and write the result into the NV12 input tensor.

- **Model Inference**  
  Call `infer()` to execute forward computation on the BPU.

- **Postprocessing**  
  Call `yolov5x.post_process(score_thres, nms_thres, W, H)` to decode outputs, filter by confidence threshold, perform NMS, and map bounding box coordinates back to the original resolution.

- **Camera Management (VIO)**  
  Open the sensor channel via `sp_open_camera_v2` and fetch NV12 frames using `sp_vio_get_yuv`.

- **On-Screen Overlay (SP Display)**  
  Initialize the display channel with `sp_start_display`; render detection results onto the screen via `draw_detections_on_disp`.

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
|-- CMakeLists.txt                     # CMake build script (targets/dependencies/includes/links)
|-- README.md                          # This documentation
|-- inc
|   `-- ultralytics_yolov5x.hpp        # YOLOv5x wrapper header: interfaces for loading, preprocessing, inference, and postprocessing
`-- src
    |-- main.cc                        # Program entry point: H.264 decoding → inference → on-screen overlay (exit with Ctrl+C)
    `-- ultralytics_yolov5x.cc         # YOLOv5x implementation: letterboxing, NV12 tensor writing, decoding, NMS, coordinate rescaling
```

## Build Instructions
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Reference
| Parameter        | Description                                                  | Default Value                                                |
| ---------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `--width`        | Desired width of source stream/decoded frames (in pixels)      | `1920`                                                       |
| `--height`       | Desired height of source stream/decoded frames (in pixels)     | `1080`                                                       |
| `--input_path`   | Path to input H.264 file (example uses a local file; can be extended to support stream pipelines) | `/app/res/assets/1080P_test.h264`                            |
| `--model_path`   | Path to quantized YOLOv5x model (`.hbm`)                       | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`       |
| `--label_file`   | Class name list file (one class per line)                      | `/app/res/labels/coco_classes.names`                         |
| `--score_thres`  | Confidence threshold (filters out low-confidence detections)   | `0.25`                                                       |
| `--nms_thres`    | NMS IoU threshold                                              | `0.45`                                                       |

## Quick Start
- **Run the model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./decode_yolov5x_display
        ```
    - Run with custom parameters:
        ```bash
        ./decode_yolov5x_display \
            --width 1920 --height 1080 \
            --input_path /app/res/assets/1080P_test.h264 \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.45
        ```

- **Terminating Execution**  
  Press **Ctrl+C** in the terminal.

- **Viewing Results**  
  Upon successful execution, the screen will display real-time object detection results.

## Notes
- This program must run in a desktop environment.

- For more information on deployment options or model compatibility, please consult the official documentation or contact platform technical support.

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