---
sidebar_position: 15
---

# WebSocket YOLOv5x Inference

This example demonstrates how to perform object detection using the Ultralytics YOLOv5x model on an embedded platform (e.g., RDK S100) equipped with an HBM accelerator and a VIO camera module, and stream JPEG images along with detection bounding boxes in real time via WebSocket. The example code is located in the `/app/pydev_demo/11_web_display_camera_sample/` directory.

## Feature Description

- **Model Loading**

    Initialize `hbm_runtime`, load the model, and obtain input/output names and dimensions.

- **Preprocessing**

    Split the raw NV12 image into Y/UV channels, resize it to the required model input dimensions, and generate input tensors in the correct format.

- **Model Inference**

    Invoke `.run()` to execute BPU inference.

- **Postprocessing**

    Decode inference results, filter out low-confidence detections, apply Non-Maximum Suppression (NMS), and scale the results back to the original image dimensions.

- **Camera Management (`CameraManager`)**

    Open the camera, acquire raw or model-sized images, and encode them into JPEG format.

- **WebSocket Server**

    Accept connections from web clients, continuously fetch camera images, perform detection, and return results to the web client using Protocol Buffers.

## Model Description

Refer to [Ultralytics YOLOv5x Object Detection Example Summary](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).

## Environment Dependencies

- Ensure the dependencies in `pydev` are installed:
    ```bash
    pip install -r ../requirements.txt
    ```
- Install WebSocket-related packages:
    ```bash
    pip install websockets==15.0.1 protobuf==3.20.3
    ```

## Hardware Environment

- The MIPI camera interface uses auto-detection mode. Only one MIPI camera (connected to any MIPI port) is supported during sample execution; connecting multiple cameras simultaneously will cause errors.
- This sample currently supports only the following MIPI sensors: IMX219, SC230AI.
- For MIPI camera installation instructions, refer to the section [Camera Expansion Board – MIPI Camera Interface](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md).

## Directory Structure

```text
.
├── mipi_camera_web_yolov5x.py      # Main program
└── README.md                       # Usage instructions
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

- **Start the Web Service**
    ```bash
    # 1. Navigate to the webservice directory
    cd webservice/

    # 2. Start the service
    sudo ./sbin/nginx -p .
    ```

- **Run the Model**
    - Return to the current directory:
        ```bash
        cd ..
        ```
    - Run with default parameters:
        ```bash
        python mipi_camera_web_yolov5x.py
        ```
    - Run with custom parameters:
        ```bash
        python mipi_camera_web_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```

- **View Results**

    After successful execution, open the web display page by visiting: http://IP  
    **Note: Do not include a port number.**

- **Terminate Execution**

    Press `Ctrl + C` in the terminal.

## Notes

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