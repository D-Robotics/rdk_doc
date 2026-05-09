---
sidebar_position: 1
---

# Example Overview

This project contains multiple AI example programs written in C/C++, designed for the RDK S100 platform, covering common AI tasks such as image classification, object detection, instance segmentation, pose estimation, OCR, and speech recognition. The examples perform inference using quantized models in `.hbm` format, enabling developers to quickly validate model performance and accelerate application development.

The on-device code for this project is located at: `/app/cdev_demo/bpu`.

## Directory Structure Overview

```text
|-- 01_classification_sample         # Image classification examples (e.g., ResNet18, MobileNet)
|-- 02_detection_sample              # Object detection examples (e.g., YOLO)
|-- 03_instance_segmentation_sample  # Instance segmentation examples
|-- 04_pose_sample                   # Keypoint detection examples
|-- 05_open_instance_seg_sample      # Open-vocabulary instance segmentation examples
|-- 06_lane_detection_sample         # Lane detection examples
|-- 07_speech_sample                 # Speech recognition examples
|-- 08_OCR_sample                    # Optical Character Recognition (OCR) examples
|-- 09_usb_camera_sample             # Real-time inference examples using USB camera
|-- 10_mipi_camera_sample            # Real-time inference examples using MIPI camera
|-- 11_decode_yolov5x_display_sample # Video decoding, inference, and display examples
|-- 12_rtsp_yolov5x_display_sample   # RTSP stream decoding, inference, and display examples
|-- utils                            # Common utility functions
`-- README.md                        # Project documentation (this file)
```

## Environment Requirements

Before running the examples, please ensure your system meets the following requirements:

### Hardware
- S100 development board with BPU support
- Camera (USB or MIPI) if running camera-related examples

### System and Toolchain
This project has been verified to run in the following environment:

- Operating System
    - Ubuntu 22.04.5 LTS (Jammy Jellyfish)

- Build Toolchain
    - CMake: 3.22.1
    - GCC: 11.4.0
    - G++: 11.4.0

### Dependencies
Different examples require different development packages. Please install the necessary dependencies based on your needs.

- Common Dependencies
    ```bash
    sudo apt update
    sudo apt install libgflags-dev
    ```

- ASR (Automatic Speech Recognition) Example
    ```bash
    sudo apt update
    sudo apt install libsndfile1-dev
    sudo apt install libsamplerate0-dev
    ```

- OCR (Optical Character Recognition) Example
    ```bash
    sudo apt update
    sudo apt install libpolyclipping-dev
    ```

## Build Instructions
Taking the ResNet18 image classification example as an illustration:

```bash
cd 01_classification_sample/01_resnet18

mkdir build && cd build

cmake ..

make -j$(nproc)
```

## Running Examples
Taking the ResNet18 image classification example as an illustration:
+ Navigate to the build directory of the sample:
    ```bash
    cd 01_classification_sample/01_resnet18/build
    ```
+ Run the model:
    ```bash
    ./resnet_18
    ```
+ View the results:
    ```bash
    TOP 0: label=zebra, prob=0.99872
    TOP 1: label=cheetah, chetah, Acinonyx jubatus, prob=0.000448407
    TOP 2: label=impala, Aepyceros melampus, prob=0.000398787
    TOP 3: label=gazelle, prob=0.000253181
    TOP 4: label=prairie chicken, prairie grouse, prairie fowl, prob=0.000179423
    ```

## Common Utilities Description
The `utils` directory contains shared utility functions used across BPU C/C++ inference examples, including image preprocessing, post-processing of inference results, multimedia processing, and general-purpose utilities, facilitating code reuse across different examples.

```bash
utils
├── inc                          # Header files
│   ├── common_utils.hpp         # General utility functions (dequantization, result visualization, common data structures, etc.)
│   ├── multimedia_utils.hpp     # Multimedia processing utilities (video frame decoding, pixel format conversion, etc.)
│   ├── postprocess_utils.hpp    # Inference result post-processing (NMS, decoding, mask handling, etc.)
│   └── preprocess_utils.hpp     # Input data preprocessing (image resizing, normalization, format conversion, etc.)
└── src                          # Source code implementations
    ├── common_utils.cc
    ├── multimedia_utils.cc
    ├── postprocess_utils.cc
    └── preprocess_utils.cc
```

## Additional Notes
* All example programs use models in `.hbm` format.

* Note: Each subdirectory includes a `README.md` file that provides detailed information about the required environment, command-line arguments, and execution instructions for the corresponding model.