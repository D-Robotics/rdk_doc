---
sidebar_position: 16
---

# RTSP Video Streaming & YOLOv5x Inference

This example demonstrates how to integrate SP hardware modules (Decoder, VIO, Display) and BPU on platforms like the RDK S100 to achieve:
RTSP/H.264 video stream → Hardware Decoding (NV12) → YOLOv5x Inference → Detection Box Overlay → Real-time Display. The sample code is located in the `/app/pydev_demo/12_rtsp_yolov5x_display_sample/` directory.

## Functional Description

- **Model Loading**
    Uses `hbm_runtime.HB_HBMRuntime(model_path)` to load the YOLOv5x model and read input/output information. BPU priority and core binding can be set via `set_scheduling_params()`.

- **Preprocessing**
    Retrieves NV12 frames from the decoding thread, separates Y/UV planes, resizes to the model input dimensions, and packs them into the BPU input format.

- **Inference**
    Calls `self.model.run()` to perform forward inference and generate detection results.

- **Postprocessing**
    Performs dequantization, decoding, filtering, NMS (Non-Maximum Suppression) on the inference results, and maps them to display coordinates, outputting detection boxes and class labels.

- **RTSP Decoding (RTSP + HW Decoder)**
    A child thread uses `cv2.VideoCapture` to pull the H.264 stream, which is then hardware-decoded into NV12 frames via `srcampy.Decoder`.

- **Resolution & Display (VPS + Display)**
    Calls `srcampy.Display()` and `srcampy.Camera().open_vps()` to establish a VPS → HDMI display pipeline.

- **Drawing Detection Results**
    Uses `draw.draw_detections_on_disp()` to draw detection boxes and class labels onto the display layer.

- **Signal Handling & Exit**
    Catches SIGINT (Ctrl+C), sets `is_stop=True`, and safely exits the main loop and child threads, sequentially closing VPS, display, and decoder.

- **Multithreading & Frame Buffering**
    `DecodeRtspStream` inherits `threading.Thread` and maintains a frame queue. The main thread retrieves the latest frame via `get_frame()`.

- **Argument Parsing**
    Uses `argparse` to provide parameters: RTSP source, model path, BPU cores, priority, label file, NMS threshold, and confidence threshold.

- **HDMI Resolution Detection**
    Calls `/usr/bin/get_hdmi_res` to get the current HDMI resolution; defaults to 1920×1080 if unavailable.

## Model Description
    Refer to the [Ultralytics YOLOv5x Object Detection Example Section](04_Ultralytics_YOLOv5x.md#object-detection-ultralytics-yolov5x).


## Environment Dependencies
This sample has no special environment requirements. Simply ensure the dependencies for `pydev` are installed.
```bash
pip install -r ../requirements.txt
```

## Directory Structure

```text
.
├── README.md               # Usage Guide
└── rtsp_yolov5x_display.py # Main Program
```

## Parameter Description
| Parameter            | Description                                                                                                | Default Value                                                 |
| -------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `--rtsp-urls` / `-u` | RTSP video stream address (Multiple streams can be separated by semicolons, e.g., `rtsp://192.168.1.10/stream1;rtsp://192.168.1.11/stream2`) | `rtsp://127.0.0.1/1080P_test.h264`                           |
| `--model-path`       | BPU quantized model path (`.hbm`)                                                                          | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`       |
| `--priority`         | Inference priority (0~255, 255 is highest)                                                                 | `0`                                                           |
| `--bpu-cores`        | List of BPU core indices (e.g., `0 1`)                                                                     | `[0]`                                                         |
| `--label-file`       | Path to the class label file                                                                                | `/app/res/labels/coco_classes.names`                          |
| `--nms-thres`        | IoU threshold for Non-Maximum Suppression                                                                  | `0.45`                                                        |
| `--score-thres`      | Detection confidence threshold                                                                              | `0.25`                                                        |


## Quick Start
- **Prepare RTSP Stream**
    Use the system's pre-installed streaming service to prepare an RTSP stream as the input source. This service processes the `1080P_test.h264` video file into an RTSP stream available at `rtsp://127.0.0.1/assets/1080P_test.h264`. Start the streaming service with the following commands:
    ```bash
    cd /app/res
    sudo chmod +x live555MediaServer
    sudo ./live555MediaServer &
    ```
- **Run the Model**
    - Using default parameters:
        ```bash
        python rtsp_yolov5x_display.py
        ```
    - Running with specified parameters:
        ```bash
        python rtsp_yolov5x_display.py \
        --rtsp-urls rtsp://127.0.0.1/assets/1080P_test.h264 \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```

- **Exit the Program**
    Press `Ctrl + C` in the command line.

- **View Results**
    Upon successful execution, the target detection image will be displayed in real-time on the screen.

## Important Notes
- This program must be run in a desktop environment.

- For more deployment methods or model support details, please refer to the official documentation or contact platform technical support.

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