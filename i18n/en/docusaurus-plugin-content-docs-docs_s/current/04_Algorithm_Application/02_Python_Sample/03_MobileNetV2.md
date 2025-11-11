---
sidebar_position: 3
---

# Image Classification - MobileNetV2

This example demonstrates how to perform image classification using a BPU-deployed `MobileNetV2` model with inference executed via `hbm_runtime`. The example code is located in the `/app/pydev_demo/01_classification_sample/02_mobilenetv2/` directory.

## Model Description
- **Overview**:

    MobileNetV2 is a lightweight convolutional neural network proposed by Google in 2018, designed for efficient image recognition on mobile devices. It introduces the Inverted Residual and Linear Bottleneck structures to reduce computational cost while improving performance. MobileNetV2 is highly suitable for deployment on edge devices and in resource-constrained scenarios for tasks such as image classification and detection. The MobileNetV2 model used in this example accepts 224×224 input and is a BPU quantized model supporting the NV12 format.

- **HBM Model Name**: `mobilenetv2_224x224_nv12.hbm`

- **Input Format**: NV12, with resolution 224x224 (separated Y and UV planes)

- **Output**: Softmax probability distribution over 1000 classes (conforming to the ImageNet 1000-class standard)

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/MobileNet/mobilenetv2_224x224_nv12.hbm
    ```

## Functionality Description
- **Model Loading**

    Load the model file using `hbm_runtime`, and extract the model name, input/output names, and their corresponding shapes.

- **Input Preprocessing**

    Resize the input image from BGR format to 224x224, then convert it to the hardware-required NV12 format (with separated Y and UV planes). The processed data is structured as a dictionary to match the inference interface.

- **Inference Execution**

    Call the `.run()` method to perform inference, supporting configuration of BPU core(s) (e.g., `core0`/`core1`) and inference priority (0–255).

- **Result Post-processing**

    Retrieve the model output tensor, parse the softmax probabilities, and display the top-K predictions (default: top-5), including class names and corresponding probabilities.

## Environment Dependencies
This example has no special environment requirements—only the dependencies from `pydev` need to be installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure

```text
.
├── mobilenetv2.py              # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter        | Description                                                  | Default Value                                      |
|------------------|--------------------------------------------------------------|----------------------------------------------------|
| `--model-path`   | Path to the model file (`.hbm` format)                       | `/opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm` |
| `--test-img`     | Path to the test image                                       | `/app/res/assets/zebra_cls.jpg`                    |
| `--label-file`   | Path to the class label mapping file                         | `/app/res/labels/imagenet1000_clsidx_to_labels.txt`|
| `--priority`     | Model inference priority (0–255; higher value = higher priority) | `0`                                             |
| `--bpu-cores`    | List of BPU core IDs to use for inference (e.g., `--bpu-cores 0 1`) | `[0]`                                          |

## Quick Start
- **Run the Model**
    - With default parameters:
        ```bash
        python mobilenetv2.py
        ```
    - With custom parameters:
        ```bash
        python mobilenetv2.py \
        --model-path /opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm \
        --test-img /app/res/assets/zebra_cls.jpg \
        --label-file /app/res/labels/imagenet1000_clsidx_to_labels.txt
        ```
- **View Results**
    ```bash
    Top-5 Predictions:
    zebra: 0.8916
    tiger, Panthera tigris: 0.0028
    hartebeest: 0.0018
    jaguar, panther, Panthera onca, Felis onca: 0.0016
    tiger cat: 0.0016
    ```

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.