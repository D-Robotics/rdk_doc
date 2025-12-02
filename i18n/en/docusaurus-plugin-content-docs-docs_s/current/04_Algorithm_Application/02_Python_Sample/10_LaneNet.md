---
sidebar_position: 10
---

# Lane Detection - LaneNet

This example runs the LaneNet model based on `hbm_runtime` to perform instance segmentation and binary segmentation of lane lines, and saves the resulting images locally. The example code is located in the `/app/pydev_demo/06_lane_detection_sample/01_lanenet/` directory.

## Model Description
- **Introduction**:

    LaneNet is a semantic segmentation model designed for real-time lane detection. It employs normalization and standardization during image preprocessing, making it suitable for road scene analysis in autonomous driving and ADAS systems. This example uses the quantized model `lanenet256x512.hbm`, which supports BPU inference acceleration.

- **HBM Model Name**: `lanenet256x512.hbm`

- **Input Format**: RGB, sized 256×512, normalized to [0,1] and then standardized.

- **Outputs**:

    - `instance_seg_logits`: a 3-channel map used to distinguish different lane instances.
    - `binary_seg_pred`: binary segmentation result indicating lane regions.

- **Model Download URL** (automatically downloaded by the program):

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/Lanenet/lanenet256x512.hbm
    ```

## Functionality Description
- **Model Loading**

    Load the LaneNet model using `hbm_runtime`, which automatically parses input/output information and quantization parameters.

- **Input Preprocessing**

    Convert the input image to RGB format, resize it to 256×512, apply normalization and standardization using ImageNet mean and standard deviation, convert to NCHW format, and add a batch dimension.

- **Inference Execution**

    Perform inference using the `.run()` method. The outputs include the instance feature map and binary mask, with support for setting inference priority and BPU core scheduling.

- **Post-processing**

    Reshape and normalize the output tensors:

    - `instance_seg_logits`: output a 3-channel image for visualizing each lane instance.
    - `binary_seg_pred`: output a single-channel binary image for extracting lane regions.

## Environment Dependencies
This example has no special environment requirements—just ensure the dependencies listed in `pydev` are installed:
```bash
pip install -r ../../requirements.txt
```

## Directory Structure
```text
.
├── lanenet.py                  # Main inference script
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter         | Description                                                  | Default Value                                      |
| ----------------- | ------------------------------------------------------------ | -------------------------------------------------- |
| `--model-path`    | Path to the model file in `.hbm` format                      | `/opt/hobot/model/s100/basic/lanenet256x512.hbm`   |
| `--priority`      | Inference priority (range: 0–255; higher value = higher priority) | `0`                                                |
| `--bpu-cores`     | BPU core IDs to use for model execution                      | `[0]`                                              |
| `--test-img`      | Path to the test image                                       | `/app/res/assets/input.jpg`                        |

## Quick Start
- **Run the Model**
    - With default parameters:
        ```bash
        python lanenet.py
        ```
    - With custom parameters:
        ```bash
        python lanenet.py \
        --model-path /opt/hobot/model/s100/basic/lanenet256x512.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --test-img /app/res/assets/input.jpg
        ```
- **View Results**

    Upon successful execution, the results will be saved as `instance_pred.png` and `binary_pred.png`:
    ```bash
    Results saved to: instance_pred.png and binary_pred.png
    ```

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.