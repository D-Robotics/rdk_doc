---
sidebar_position: 5
---

# Object Detection - Ultralytics YOLO11

This example uses the Ultralytics YOLO11 model and leverages the `hbm_runtime` interface to perform object detection on images. It supports image preprocessing, inference, post-processing (including decoding, confidence filtering, and NMS), and saving result images. The example code is located in the `/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11/` directory.

## Model Description
- **Overview**:

    Ultralytics YOLO11 is a lightweight anchor-based object detection model that integrates both anchor-free and anchor-based concepts, offering fast inference and precise localization capabilities. During regression, it adopts a discrete binning approach combined with softmax classification and a decoding mechanism to enhance localization accuracy. Ultralytics YOLO11 is suitable for deploying small models in real-time scenarios such as security surveillance and industrial inspection.

- **HBM Model Name**: `yolo11n_detect_nashe_640x640_nv12.hbm`

- **Input Format**: NV12 format, sized at 640x640 (separate Y and UV planes)

- **Output**: Multi-scale feature maps; each scale includes a class score tensor and a discrete bounding box regression tensor. The final output consists of bounding box coordinates, class IDs, and confidence scores.

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_detect_nashe_640x640_nv12.hbm
    ```

## Functionality Description
- **Model Loading**

    Load the quantized Ultralytics YOLO11 model using the `hbm_runtime` interface, extract model metadata such as input/output names, shapes, and quantization information for subsequent inference steps.

- **Input Preprocessing**

    Resize the original BGR image to 640×640, convert it to NV12 format (separate Y and UV planes), and construct a nested dictionary of input tensors compatible with the inference interface.

- **Inference Execution**

    Run forward inference via the `.run()` method, optionally specifying scheduling parameters (inference priority and BPU core binding). The output includes classification and regression tensors from multiple scale branches.

- **Result Post-processing**

    - Dequantize quantized outputs back to float32;
    
    - Filter classification scores per scale branch, retaining candidate boxes exceeding a specified confidence threshold;
    
    - Decode bounding boxes using a multi-bin regression algorithm;
    
    - Merge candidate boxes across all scales and apply NMS (Non-Maximum Suppression) to remove redundant detections;
    
    - Map detected boxes from the model input coordinate system back to the original image dimensions;
    
    - Optionally draw detection results and save the annotated image file.

## Environment Dependencies
This example has no special environment requirements—only ensure that dependencies listed in pydev are installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── ultralytics_yolo11.py       # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description
| Argument             | Description                                                                 | Default Value                                         |
|----------------------|-----------------------------------------------------------------------------|-------------------------------------------------------|
| `--model-path`        | Path to the model file (.hbm format)                                        | `/opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm`        |
| `--test-img`          | Path to the input test image                                                | `/app/res/assets/kite.jpg`                        |
| `--label-file`        | Path to the class label file (one class name per line)                      | `/app/res/labels/coco_classes.names`              |
| `--img-save-path`     | Path to save the detection result image                                     | `result.jpg`                                       |
| `--priority`          | Model scheduling priority (0–255; higher values indicate higher priority)   | `0`                                                |
| `--bpu-cores`         | List of BPU core IDs to use (e.g., `--bpu-cores 0 1`)                       | `[0]`                                              |
| `--nms-thres`         | IoU threshold for Non-Maximum Suppression (NMS)                             | `0.45`                                             |
| `--score-thres`       | Confidence threshold for filtering detections (boxes below this are discarded) | `0.25`                                           |

## Quick Start
- **Run the Model**
    - With default arguments:
        ```bash
        python ultralytics_yolo11.py
        ```
    - With custom arguments:
        ```bash
        python ultralytics_yolo11.py \
            --model-path /opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm \
            --test-img /app/res/assets/kite.jpg \
            --label-file /app/res/labels/coco_classes.names \
            --img-save-path result.jpg \
            --priority 0 \
            --bpu-cores 0 \
            --nms-thres 0.45 \
            --score-thres 0.25
        ```
- **View Results**

    Upon successful execution, detection bounding boxes will be drawn on the original image and saved to the path specified by `--img-save-path`:
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