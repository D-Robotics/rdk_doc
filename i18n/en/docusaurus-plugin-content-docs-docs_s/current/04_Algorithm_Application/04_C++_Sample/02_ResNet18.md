---
sidebar_position: 2
---

# Image Classification - ResNet18

This example demonstrates how to deploy the `ResNet18` model using `C/C++` for image classification inference on RDK S100 devices equipped with a BPU chip. The example code is located in the `/app/cdev_demo/bpu/01_classification_sample/01_resnet18/` directory.

## Model Description
- **Introduction**:

  ResNet (Residual Network) is a deep convolutional neural network architecture proposed by Microsoft Research. Its core idea is the introduction of "residual connections," which alleviate the vanishing gradient problem in deep networks through shortcut connections across layers, enabling effective training of networks with dozens or even hundreds of layers. The ResNet18 used in this example is a lightweight variant with an 18-layer structure, widely applied in tasks such as image classification and feature extraction.

- **HBM Model Name**: resnet18_224x224_nv12.hbm

- **Input Format**: NV12, size 224x224

- **Output**: Softmax probability distribution over 1000 classes

## Functionality Overview
- **Model Loading**

    Loads the specified model and parses input/output names and shapes for subsequent inference.

- **Input Preprocessing**

    Resizes the BGR image to 224x224 and converts it to NV12 format (separated Y and UV planes).

- **Inference Execution**

    Performs forward inference using the `.infer()` method.

- **Result Post-processing**

    Reads the output tensor, parses the top-K classification results (Top-5), and displays class labels along with their probabilities.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure

```text
.
|-- CMakeLists.txt         # CMake build script
|-- README.md              # Project documentation
|-- inc/                   # Header files directory
|   `-- resnet18.hpp       # Definition of ResNet18 inference class
`-- src/                   # Source code directory
    |-- main.cc            # Program entry point, invoking ResNet18 inference pipeline
    `-- resnet18.cc        # Implementation of ResNet18 inference class
```

## Build Instructions
- **Configuration and Compilation**
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found at runtime, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ResNet/resnet18_224x224_nv12.hbm
```

## Parameter Description
| Parameter          | Description                                                  | Default Value                                               |
| ------------------ | ------------------------------------------------------------ | ----------------------------------------------------------- |
| `--model_path`     | Path to the model file (`.hbm` format)                       | `/opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm`     |
| `--test_img`       | Path to the test image                                       | `/app/res/assets/zebra_cls.jpg`                             |
| `--label_file`     | ImageNet class mapping (dictionary, each line: `index\tlabel`) | `/app/res/labels/imagenet1000_clsidx_to_labels.txt`         |
| `--top_k`          | Number of top-K classification results to output             | `5`                                                         |

## Quick Start
- **Run the Model**
    - Ensure you are in the `build` directory.
    - Run with default parameters:
        ```bash
        ./resnet_18
        ```
    - Run with custom parameters:
        ```bash
        ./resnet_18 \
        --model_path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
        --test_img   /app/res/assets/zebra_cls.jpg \
        --label_file /app/res/labels/imagenet1000_clsidx_to_labels.txt \
        --top_k 5
        ```

- **View Results**
    ```bash
    Top-5 Predictions:
    zebra: 0.9979
    impala, Aepyceros melampus: 0.0005
    cheetah, chetah, Acinonyx jubatus: 0.0005
    gazelle: 0.0004
    prairie chicken, prairie grouse, prairie fowl: 0.0002
    ```

## Notes
- The output displays the top-K classes with the highest probabilities.

- For more information on deployment methods or model support, please refer to the official documentation or contact platform technical support.