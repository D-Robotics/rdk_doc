---
sidebar_position: 16
---

# RTSP Video Streaming and YOLOv5x Inference

This example demonstrates how to combine SP hardware modules (decoder, VIO, display) and BPU on platforms such as RDK S100 to achieve the following pipeline:  
RTSP/H.264 video stream → Hardware decoding (NV12) → YOLOv5x inference → Overlay detection boxes → Real-time display. The example code is located in the directory `/app/cdev_demo/bpu/12_rtsp_yolov5x_display_sample/`.

## Feature Description

- **Model Loading**  
  Load the BPU model using `YOLOv5x(model_path)` and obtain the class name list via `load_linewise_labels` for subsequent inference.

- **Preprocessing**  
  Retrieve NV12 frames from the SP decoder (`sp_decoder_get_image`), convert them to BGR (`cv::cvtColor`), perform scaling/letterbox processing, and write the result into the YOLOv5x input tensor (`pre_process`).

- **Model Inference**  
  Call `yolov5x.infer()` to execute forward computation on the BPU and generate raw detection results.

- **Postprocessing**  
  Invoke `yolov5x.post_process` to apply confidence thresholding, Non-Maximum Suppression (NMS), and map detection box coordinates back to the display resolution.

- **RTSP Streaming and Decoding (SP Decoder / FFmpeg)**  
  Initialize the network stack using FFmpeg (`avformat_network_init`), open the RTSP stream (`avformat_open_input`), and pull H.264 video frames via the SP module (`sp_start_decode`, `sp_decoder_get_image`).

- **Resolution Adaptation and Scaling (VPS)**  
  If the display resolution differs from the video stream resolution, use the SP VPS module for scaling (`sp_open_vps`) and bind the decoder, VPS, and display modules into a pipeline via `sp_module_bind`.

- **Screen Display (SP Display)**  
  Initialize the display channel via `sp_start_display`; overlay detection results onto the screen using `draw_detections_on_disp`; if resolutions match, directly display YUV frames via `sp_display_set_image`.

- **Signal Handling**  
  Register `signal_handler_func` to capture signals like SIGINT, set the global flag `is_stop`, and allow the main loop to exit safely.

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
|-- CMakeLists.txt
|-- README.md
|-- inc
|   `-- ultralytics_yolov5x.hpp       # YOLOv5x wrapper header file
`-- src
    |-- main.cc                       # Main program entry: RTSP decoding → YOLOv5x inference → Display
    `-- ultralytics_yolov5x.cc        # YOLOv5x implementation: preprocessing/inference/postprocessing/NMS
```

## Build Instructions
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Description
| Parameter         | Description                                      | Default Value                                               |
| ----------------- | ------------------------------------------------ | -------------------------------------------------------- |
| `--rtsp_url`      | RTSP stream URL                                  | `rtsp://127.0.0.1/assets/1080P_test.h264`                |
| `--transfer_type` | RTSP transport protocol (tcp/udp)                | `tcp`                                                    |
| `--model_path`    | Path to quantized YOLOv5x BPU model (.hbm)       | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`   |
| `--label_file`    | Class label file (one class name per line)       | `/app/res/labels/coco_classes.names`                     |
| `--score_thres`   | Confidence threshold (filters low-score boxes)   | `0.25`                                                   |
| `--nms_thres`     | NMS IoU threshold                                | `0.45`                                                   |

## Quick Start
- **Prepare RTSP Stream**  
  Use the pre-installed streaming service to prepare an RTSP stream as the input source. This service converts the `1080P_test.h264` video file into an RTSP stream accessible at `rtsp://127.0.0.1/assets/1080P_test.h264`. Start the streaming service with the following commands:
    ```bash
    cd /app/res
    sudo chmod +x live555MediaServer
    sudo ./live555MediaServer &
    ```
- **Run the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./rtsp_yolov5x_display
        ```
    - Run with custom parameters:
        ```bash
        ./rtsp_yolov5x_display \
            --rtsp_url rtsp://127.0.0.1/assets/1080P_test.h264 \
            --transfer_type tcp \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.3 \
            --nms_thres 0.5
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