---
sidebar_position: 6
---

# Semantic Segmentation - UNetMobileNet

This example demonstrates how to run the UNet-MobileNet semantic segmentation model on the BPU, supporting functionalities such as image preprocessing, inference, and post-processing (parsing outputs and overlaying color segmentation masks). The example code is located in the `/app/cdev_demo/bpu/03_instance_segmentation_sample/01_unetmobilenet/` directory.

## Model Description
- **Overview**:

    UNet is a classic semantic segmentation network architecture that adopts an encoder-decoder structure and excels in fields such as medical image analysis. In this example, MobileNet is used as the encoder backbone to reduce model complexity and accelerate inference speed, making it suitable for real-time segmentation tasks on edge devices. The model outputs a class label for each pixel, enabling applications like urban street scene segmentation.

- **HBM Model Name**: unet_mobilenet_1024x2048_nv12.hbm

- **Input Format**: NV12, with resolution 1024x2048 (separate Y and UV planes)

- **Output**: A segmentation map with the same dimensions as the input, where each pixel corresponds to a class label ranging from 0 to 18 (19 classes in total)

## Functionality Description
- **Model Loading**

    Loads the quantized semantic segmentation model and extracts model metadata.

- **Input Preprocessing**

    The original image is loaded in BGR format, resized to 1024×2048, converted to NV12 format (with separate Y/UV planes), and packaged into the input dictionary structure required by the inference interface.

- **Inference Execution**

    Executes forward inference using the `.infer()` method, producing a logits tensor for each class.

- **Result Post-processing**

    - Applies argmax to the output tensor to obtain the predicted class for each pixel;
    - Resizes the prediction map to match the original input image size;
    - Restores it to the original image dimensions and maps classes to a specified color palette;
    - Blends the result with the original image using a configurable alpha blending coefficient to generate a visual segmentation overlay;
    - The final image contains an intuitive overlay of the segmentation result and can be saved or displayed.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
├── CMakeLists.txt             # CMake build configuration file
├── README.md                  # Usage documentation (this file)
├── inc
│   └── unet_mobilenet.hpp      # Header file for the UnetMobileNet class (declares interfaces for model loading, preprocessing, inference, and post-processing)
└── src
    ├── main.cc                 # Main program entry point, orchestrating the inference pipeline
    └── unet_mobilenet.cc       # Implementation of the UnetMobileNet class
```

## Building the Project
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Parameter Description

| Parameter        | Description                                               | Default Value                                                       |
| ---------------- | --------------------------------------------------------- | ------------------------------------------------------------------- |
| `--model_path`   | Path to the model file (`.hbm` format)                    | `/opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm`     |
| `--test_img`     | Path to the input test image                              | `/app/res/assets/segmentation.png`                                  |
| `--alpha_f`      | Visualization blending factor: `0.0=mask only`, `1.0=original image only` | `0.75`                                                              |

## Quick Start
- **Running the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./unet_mobilenet
        ```
    - Run with custom parameters:
        ```bash
        ./unet_mobilenet \
        --model_path /opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm \
        --test_img /app/res/assets/segmentation.png \
        --alpha_f 0.75
        ```
- **Viewing Results**

    Upon successful execution, the result will be overlaid on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`; users can inspect it directly.

- For more information about deployment options or model support, please refer to the official documentation or contact platform technical support.