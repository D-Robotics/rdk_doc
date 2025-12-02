---
sidebar_position: 2
---

# Image Classification - ResNet18

This example demonstrates how to deploy the `ResNet18` model for image classification inference using the Python API of `hbm_runtime`. It is applicable to RDK S100 devices equipped with a BPU chip. The example code is located in the `/app/pydev_demo/01_classification_sample/01_resnet18/` directory.

## Model Description
- **Introduction**:

  ResNet (Residual Network) is a deep convolutional neural network architecture proposed by Microsoft Research. Its core idea is to introduce "residual connections," which alleviate the vanishing gradient problem in deep networks through shortcut connections across layers, enabling effective training of networks with dozens or even hundreds of layers. The ResNet18 used in this example is a lightweight variant with 18 layers, widely applied in tasks such as image classification and feature extraction.

- **HBM Model Name**: `resnet18_224x224_nv12.hbm`

- **Input Format**: NV12, size 224x224

- **Output**: Softmax probability distribution over 1000 classes

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ResNet/resnet18_224x224_nv12.hbm
    ```

## Functionality Description
- **Model Loading**

    Use `hbm_runtime` to load the specified model and parse input/output names and shapes for subsequent inference.

- **Input Preprocessing**

    Resize the BGR image to 224x224 and convert it to NV12 format (separated Y and UV planes).

- **Inference Execution**

    Perform forward inference using the `.run()` method, supporting optional scheduling parameters (priority, core binding).

- **Result Post-processing**

    Read the output tensor, parse the top-K classification results (Top-5), and display class labels along with their probabilities.

## Environment Dependencies
This example has no special environment requirements; simply ensure that the dependencies listed in pydev are installed.
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── resnet18.py                 # Main program: uses `hbm_runtime` to run ResNet18 classification
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter        | Description                                                  | Default Value                                      |
|------------------|--------------------------------------------------------------|----------------------------------------------------|
| `--model-path`   | Path to the model file (.hbm format)                         | `/opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm` |
| `--test-img`     | Path to the test image                                       | `/app/res/assets/zebra_cls.jpg`                    |
| `--label-file`   | Path to the class label mapping file                         | `/app/res/labels/imagenet1000_clsidx_to_labels.txt` |
| `--priority`     | Model priority (0–255; higher value means higher priority)    | `0`                                                |
| `--bpu-cores`    | List of BPU core IDs to use for inference (e.g., `--bpu-cores 0 1`) | `[0]`                                        |

## Quick Start
- **Run the Model**
    - Using default parameters:
        ```bash
        python resnet18.py
        ```
    - Running with custom parameters:
        ```bash
        python resnet18.py \
        --model-path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
        --test-img /app/res/assets/zebra_cls.jpg \
        --label-file /app/res/labels/imagenet1000_clsidx_to_labels.txt
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
- If the specified model path does not exist, the program will attempt to download the model automatically.