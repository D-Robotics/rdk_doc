---
sidebar_position: 6
---

# Semantic Segmentation - UNetMobileNet

This example demonstrates how to run the UNet-MobileNet semantic segmentation model on the BPU using `hbm_runtime`, supporting functionalities such as image preprocessing, inference, and post-processing (parsing outputs and overlaying colored segmentation masks). The example code is located in the `/app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet/` directory.

## Model Description
- **Introduction**:

    UNet is a classic semantic segmentation network architecture featuring an encoder-decoder structure, widely recognized for its excellent performance in medical image analysis and similar fields. In this example, MobileNet is used as the encoder backbone to reduce model complexity and accelerate inference speed, making it suitable for real-time segmentation tasks on edge devices. The model outputs a class label for each pixel, enabling applications such as urban street scene segmentation.

- **HBM Model Name**: `unet_mobilenet_1024x2048_nv12.hbm`

- **Input Format**: NV12, with resolution 1024x2048 (separate Y and UV planes)

- **Output**: A segmentation map matching the input dimensions, where each pixel corresponds to a class label ranging from 0 to 18 (19 classes in total)

## Functionality Overview
- **Model Loading**

    The quantized semantic segmentation model is loaded using `hbm_runtime`, extracting metadata such as model name, input/output tensor names, shapes, and quantization parameters.

- **Input Preprocessing**

    The original image is loaded in BGR format, resized to 1024×2048, converted to NV12 format (with separated Y/UV planes), and packaged into the input dictionary structure required by the inference interface.

- **Inference Execution**

    Model forward inference is executed using the `.run()` method, supporting scheduling parameters such as BPU core assignment and priority. The output is a logits tensor representing class scores.

- **Result Post-processing**

    - Apply argmax to the output tensor to obtain the predicted class for each pixel;
    - Resize the prediction map to match the original input image size;
    - Restore it to the original image dimensions and map classes to a specified color palette;
    - Blend the segmentation mask with the original image using a configurable alpha blending coefficient to generate a visualization;
    - The final image contains an intuitive overlay of the segmentation result, which can be saved or displayed.

## Environment Dependencies
This example has no special environment requirements—only the dependencies installed in the pydev environment are needed.
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── unet_mobilenet.py           # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description

| Parameter             | Description                                                      | Default Value                                           |
| --------------------- | ---------------------------------------------------------------- | ------------------------------------------------------- |
| `--model-path`        | Path to the model file (.hbm format)                             | `/opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm` |
| `--test-img`          | Path to the input test image                                     | `/app/res/assets/segmentation.png`                      |
| `--img-save-path`     | Path to save the inference result image                          | `result.jpg`                                            |
| `--priority`          | Model priority (0–255; higher value = higher priority)           | `0`                                                     |
| `--bpu-cores`         | List of BPU core IDs to run the model on (e.g., `--bpu-cores 0 1`) | `[0]`                                                   |
| `--alpha-f`           | Visualization blending factor: `0.0=mask only`, `1.0=original image only` | `0.75`                                                  |


## Quick Start
- **Run the Model**
    - Using default parameters:
        ```bash
        python unet_mobilenet.py
        ```
    - Running with custom parameters:
        ```bash
        python unet_mobilenet.py \
        --model-path /opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm \
        --test-img /app/res/assets/segmentation.png \
        --img-save-path result.jpg \
        --alpha-f 0.75 \
        --priority 0 \
        --bpu-cores 0
        ```
- **View Results**

    Upon successful execution, the segmentation result will be overlaid on the original image and saved to the path specified by `--img-save-path`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```