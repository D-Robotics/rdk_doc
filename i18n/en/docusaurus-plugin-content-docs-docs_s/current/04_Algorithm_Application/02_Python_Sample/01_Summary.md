---
sidebar_position: 1
---

# Example Overview

This project contains multiple AI example programs written in Python, designed for the RDK S100 platform, covering common AI tasks such as image classification, object detection, instance segmentation, pose estimation, OCR, and speech recognition. The examples perform inference using quantized models in `.hbm` format, enabling developers to quickly validate model performance and accelerate application development.

The on-device code for this project is located at: `/app/pydev_demo/`.

## Overview
### Environment Requirements
This project is written in Python and relies on several third-party libraries. Please ensure your environment meets the following requirements:

#### Python Environment
- Python version: Python 3.10.x is recommended (currently tested and verified on Python 3.10.12)

#### Dependencies
- Dependency list

    | Library Name       | Description                                      | Recommended Version |
    |--------------------|--------------------------------------------------|---------------------|
    | numpy              | Scientific computing library for tensor and matrix operations | >=1.26.4            |
    | opencv-python      | Image processing and visualization (cv2)         | >=4.11.0.86         |
    | scipy              | Mathematical functions library (e.g., softmax)   | >=1.15.3            |

- Installing dependencies
    ```bash
    # Install dependencies
    pip install -r requirements.txt
    ```

- Note:

    **The above dependency list and installation file only include the basic libraries required to run the models. Some example programs require additional third-party libraries. Please refer to the corresponding example's README.md or the relevant section of this document for details.**

#### Other Components
- hbm_runtime: Used to load and run `.hbm` models. This is pre-installed by default on the system. For manual installation instructions, refer to section [4.1 Python Interface](../01_Python_API.md#4.1-python-interface).

- Hobot VIO: Used to access camera image streams (e.g., hobot_vio, such as libsrcampy).

### Directory Structure
    ```text
    .
    ├── 01_classification_sample/        # Image classification example
    ├── 02_detection_sample/             # Object detection example
    ├── 03_instance_segmentation_sample/ # Instance segmentation example
    ├── 04_pose_sample/                  # Pose estimation example
    ├── 05_open_instance_seg_sample/     # Open-vocabulary instance segmentation example
    ├── 06_lane_detection_sample/        # Lane detection example
    ├── 07_speech_sample/                # Speech recognition example
    ├── 08_OCR_sample/                   # OCR text recognition example
    ├── 09_usb_camera_sample/            # USB camera + object detection example
    ├── 10_mipi_camera_sample/           # MIPI camera + object detection example
    ├── 11_web_display_camera_sample/    # Camera + Web + object detection example
    ├── utils/                           # Common preprocessing and postprocessing utility modules
    ├── requirements.txt                 # Python environment dependencies
    └── README.md                        # Top-level usage documentation (this file)
    ```

### Quick Start
Taking the ResNet18 image classification example as an illustration:
- Enter the sample directory
    ```bash
    cd 01_classification_sample/01_resnet18
    ```
- Run the model
    ```bash
    python3 resnet18.py \
    --model-path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
    --test-img /app/res/assets/zebra_cls.jpg
    ```
- View results
    ```bash
    Top-5 Predictions:
    zebra: 0.9979
    impala, Aepyceros melampus: 0.0005
    cheetah, chetah, Acinonyx jubatus: 0.0005
    gazelle: 0.0004
    prairie chicken, prairie grouse, prairie fowl: 0.0002
    ```

### Common Utilities
The project uses a unified set of utility modules to simplify example development, located under `utils/`:

* preprocess_utils.py: Image preprocessing functions, such as resize and color format conversion.

* postprocess_utils.py: Model postprocessing logic, such as NMS and bounding box transformations.

* draw_utils.py: Functions for drawing detection boxes, keypoints, segmentation masks, etc.

* common_utils.py: Utilities for printing model information, defining colors, etc.

### Notes
* All example programs use models in `.hbm` format and must be used with the platform's `hbm_runtime` Python inference interface.

* Note: Each subdirectory includes a `README.md` file that provides detailed instructions regarding required environment setup, command-line arguments, and execution methods for the corresponding model.