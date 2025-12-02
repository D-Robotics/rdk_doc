---
sidebar_position: 13
---

# USB Camera YOLOv5x Inference

A real-time inference example of Ultralytics YOLOv5x based on `hbm_runtime`, which supports reading frames from a USB camera for object detection and visualizing the detection results in full-screen mode. The sample code is located under the directory `/app/pydev_demo/09_usb_camera_sample/`.

## Feature Description
- **Model Loading**

    Load the specified `.hbm` model file via `hbm_runtime`, and extract model metadata such as model name, input/output shapes, and quantization information.

- **Camera Capture**

    Automatically scan devices under `/dev/video*`, open the first available USB camera, and configure it to MJPEG encoding, 1080p resolution, and 30 FPS.

- **Image Preprocessing**

    Resize the BGR image to the model's input resolution (using letterbox or standard scaling) and convert it to NV12 format.

- **Inference Execution**

    Submit the input tensor via the `run()` method and perform forward computation on the BPU.

- **Post-processing**

    Includes decoding quantized outputs, filtering candidate boxes by confidence score threshold, applying Non-Maximum Suppression (NMS) to remove duplicates, and mapping bounding box coordinates back to the original image dimensions.

- **Visualization**

    Draw detection boxes along with their class labels and confidence scores onto the image, and display the result in a full-screen window, supporting real-time processing and exit control.

## Model Description
    Refer to [Ultralytics YOLOv5x Object Detection Example Summary](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).

## Environment Dependencies
- Ensure that the required dependencies in pydev are installed:
    ```bash
    pip install -r ../requirements.txt
    ```

## Directory Structure
```text
.
├── usb_camera_yolov5x.py       # Main program
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter         | Description                                      | Default Value                                               |
| ----------------- | ------------------------------------------------ | -------------------------------------------------------- |
| `--model-path`    | Path to the BPU quantized model (`.hbm`)         | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`   |
| `--priority`      | Inference priority (0–255, where 255 is highest) | `0`                                                      |
| `--bpu-cores`     | List of BPU core indices (e.g., `0 1`)           | `[0]`                                                    |
| `--label-file`    | Path to the class label file                     | `/app/res/labels/coco_classes.names`                     |
| `--nms-thres`     | IoU threshold for Non-Maximum Suppression (NMS)  | `0.45`                                                   |
| `--score-thres`   | Detection confidence threshold                   | `0.25`                                                   |

## Quick Start
Note: This program must run in a desktop environment.
- **Run the model**
    - With default parameters:
        ```bash
        python usb_camera_yolov5x.py
        ```
    - With custom parameters:
        ```bash
        python usb_camera_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```
- **Exit the program**

    Place your mouse cursor inside the display window and press the `q` key to quit.

- **View Results**

    Upon successful execution, the screen will display real-time object detection results.

## Notes
- This program must run in a desktop environment.

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