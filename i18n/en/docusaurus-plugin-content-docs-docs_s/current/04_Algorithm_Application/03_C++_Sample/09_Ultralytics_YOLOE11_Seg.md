---
sidebar_position: 9
---

# Instance Segmentation - Ultralytics YOLOE11

This example demonstrates how to run the Ultralytics YOLOE11 instance segmentation model on the BPU. The program implements a complete pipeline from input image preprocessing, model inference, post-processing, to result visualization. The example code is located in the `/app/cdev_demo/bpu/05_open_instance_seg_sample/01_yoloe11_seg/` directory.

## Model Description
- Introduction:

    Ultralytics YOLOE11 is a high-performance on-device instance segmentation model suitable for open-vocabulary object detection and segmentation tasks. Through multi-scale feature extraction, dense classification, and prototype mask generation, this model effectively identifies objects in images and outputs precise instance segmentation results. This example uses a lightweight version of Ultralytics YOLOE11, which takes 640x640 input images and supports generalized object classification and segmentation across 4,585 categories.

- HBM Model Name: yoloe_11s_seg_pf_nashe_640x640_nv12.hbm

- Input Format: NV12, resolution 640x640

- Outputs:

    - Bounding boxes (in xyxy format)
    - Class IDs and confidence scores
    - Instance segmentation masks (one independent mask per instance)

## Functionality Description
- Model Loading

    Loads the specified quantized model and parses partial model metadata.

- Input Preprocessing

    Resizes the BGR image to 640x640, converts it to NV12 format (separated Y and UV planes), and constructs the input tensor for inference.

- Inference Execution

    Calls the `.infer()` interface to perform forward inference.

- Result Post-processing

    Performs post-processing on multi-scale outputs, including:

    - Classification confidence filtering (based on score threshold)
    - DFL bounding box decoding
    - Mask prototype fusion and mask generation
    - NMS filtering and result merging
    - Rescaling bounding boxes and masks back to the original image dimensions
    - Optional morphological opening operation on masks and contour drawing

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                      # CMake build script: target/dependencies/include paths/link libraries configuration
|-- README.md                           # Usage instructions (current file)
|-- inc
|   `-- ultralytics_yoloe11_seg.hpp     # YOLOE11_Seg wrapper header: declarations for load/preprocess/infer/postprocess interfaces
`-- src
    |-- main.cc                         # Program entry point: parse arguments → full pipeline → render and save results
    `-- ultralytics_yoloe11_seg.cc      # Inference implementation: decoding, score filtering, class-wise NMS, mask generation and restoration
```

## Build the Project
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found during program execution, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm
```

## Parameter Description
| Parameter Name     | Description                                      | Default Value                                                       |
| ------------------ | ------------------------------------------------ | ------------------------------------------------------------------- |
| `--model_path`     | Path to the model file (`.hbm`)                  | `/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm` |
| `--test_img`       | Path to the input test image                     | `/app/res/assets/office_desk.jpg`                                   |
| `--label_file`     | Class label file (one class name per line)       | `/app/res/labels/coco_extended.names`                               |
| `--score_thres`    | Confidence threshold (detections below this are filtered out) | `0.25`                                                              |
| `--nms_thres`      | IoU threshold for class-wise NMS deduplication   | `0.7`                                                               |

## Quick Start
- Run the Model
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./ultralytics_yoloe11_seg
        ```
    - Run with custom parameters:
        ```bash
        ./ultralytics_yoloe11_seg \
            --model_path /opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm \
            --test_img   /app/res/assets/office_desk.jpg \
            --label_file /app/res/labels/coco_extended.names \
            --score_thres 0.25 \
            --nms_thres   0.7
        ```
- View Results

    Upon successful execution, the results will be overlaid on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`, which users can inspect directly.

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

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.