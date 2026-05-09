---
sidebar_position: 3
---

# Image Classification - MobileNetV2

This example demonstrates how to perform image classification using a BPU-deployed `MobileNetV2` model with `C/C++` inference. The example code is located in the directory `/app/cdev_demo/bpu/01_classification_sample/02_mobilenetv2/`.

## Model Description
- **Overview**:

    MobileNetV2 is a lightweight convolutional neural network introduced by Google in 2018, designed for efficient image recognition on mobile devices. It incorporates the Inverted Residual and Linear Bottleneck structures to reduce computational cost while improving performance. MobileNetV2 is highly suitable for deployment on edge devices and in resource-constrained scenarios for tasks such as image classification and detection. The MobileNetV2 model used in this example accepts 224×224 input and is a BPU quantized model supporting the NV12 format.

- **HBM Model Name**: mobilenetv2_224x224_nv12.hbm

- **Input Format**: NV12, size 224x224 (separated Y and UV planes)

- **Output**: Softmax probability distribution over 1000 classes (conforming to the ImageNet 1000-class standard)

## Functionality Description
- **Model Loading**

    Load the model file and extract information such as model name, number of inputs and outputs, etc.

- **Input Preprocessing**

    Resize the input image from BGR format to 224x224, then convert it to the hardware-required NV12 format (with Y and UV separated), and structure it into a dictionary for inference interface compatibility.

- **Inference Execution**

    Call the `.infer()` method to perform inference.

- **Result Post-processing**

    Retrieve the model output tensor, parse the probabilities, and display the top-K (default top-5) prediction results, including corresponding class names and probabilities.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure

```text
.
├── CMakeLists.txt              # CMake build script
├── README.md                   # Usage instructions
├── inc
│   └── mobilenetv2.hpp         # Model inference header file
└── src
    ├── main.cc                 # Main program entry point
    └── mobilenetv2.cc          # Model implementation
```
## Build the Project
- Build the project
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found during program execution, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/MobileNet/mobilenetv2_224x224_nv12.hbm
```

## Parameter Description
| Parameter        | Description                              | Default Value                                                     |
| ---------------- | ---------------------------------------- | ----------------------------------------------------------------- |
| `--model_path`   | Path to the model file (.hbm format)     | `/opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm`       |
| `--test_img`     | Path to the test image                   | `/app/res/assets/zebra_cls.jpg`                                   |
| `--label_file`   | Path to the class label mapping file (dict format) | `/app/res/labels/imagenet1000_clsidx_to_labels.txt`        |
| `--top_k`        | Number of top-k classification results to output | `5`                                                        |

## Quick Start
- Run the model
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./mobilenetv2
        ```
    - Run with custom parameters:
        ```bash
        ./mobilenetv2 \
        --model_path /opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm \
        --test_img /app/res/assets/zebra_cls.jpg \
        --label_file /app/res/labels/imagenet1000_clsidx_to_labels.txt \
        --top_k 5
        ```
- View results
    ```bash
    TOP 0: label=zebra, prob=0.992246
    TOP 1: label=tiger, Panthera tigris, prob=0.00404656
    TOP 2: label=hartebeest, prob=0.00133707
    TOP 3: label=tiger cat, prob=0.000722661
    TOP 4: label=impala, Aepyceros melampus, prob=0.000539704
    ```

## Notes
- The output displays the top-K classes with the highest probabilities.

- For more information on deployment methods or model support, please refer to the official documentation or contact platform technical support.