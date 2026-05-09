---
sidebar_position: 10
---


# Lane Detection - LaneNet

This example runs the LaneNet model on the BPU to perform instance segmentation and binary segmentation of lane lines, and saves the resulting images locally. The example code is located in the `/app/cdev_demo/bpu/06_lane_detection_sample/01_lanenet/` directory.

## Model Description
- Overview:

    LaneNet is a semantic segmentation model designed for real-time lane detection. It employs normalization and standardization during image preprocessing, making it suitable for road scene analysis in autonomous driving and ADAS systems. This example uses the quantized model `lanenet256x512.hbm`, which supports BPU inference acceleration.

- HBM Model Name: lanenet256x512.hbm

- Input Format: RGB, sized 256x512, normalized to [0,1] and then standardized

- Outputs:

    - `instance_seg_logits`: a 3-channel map used to distinguish different lane instances

    - `binary_seg_pred`: binary segmentation result indicating lane regions

## Functionality Description
- Model Loading

    Loads the LaneNet model and automatically parses partial model metadata.

- Input Preprocessing

    Converts the input image to RGB format, resizes it to 256x512, applies normalization and standardization using ImageNet mean and standard deviation, converts it to NCHW format, and adds a batch dimension.

- Inference Execution

    Performs inference using the `.infer()` method, producing an instance feature map and a binary mask.

- Post-processing

    Reshapes and normalizes the output tensors:

    - `instance_seg_logits`: outputs a three-channel image for visualizing individual lane instances

    - `binary_seg_pred`: outputs a single-channel binary image for extracting lane regions

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install libgflags-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt            # CMake build script: target/dependencies/include and link configuration
|-- README.md                 # Usage instructions (this file)
|-- inc
|   `-- lanenet.hpp           # LaneNet inference wrapper header: interfaces for loading/preprocessing/inference/post-processing
`-- src
    |-- lanenet.cc            # LaneNet inference implementation: pre/post-processing and inference calls
    `-- main.cc               # Program entry point: parse arguments → full pipeline → save instance/binary results
```

## Build the Project
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If the model is not found at runtime, download it using the following command:
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/Lanenet/lanenet256x512.hbm
```

## Parameter Description
| Parameter        | Description                          | Default Value                                           |
| ---------------- | ------------------------------------ | ------------------------------------------------------- |
| `--model_path`   | Path to the model file (`.hbm` format) | `/opt/hobot/model/s100/basic/lanenet256x512.hbm`        |
| `--test_img`     | Path to the input test image         | `/app/res/assets/input.jpg`                             |


## Quick Start
- Run the Model
    - Ensure you are in the `build` directory
    - Run with default parameters
        ```bash
        ./lanenet
        ```
    - Run with specified parameters
        ```bash
        ./lanenet \
            --model_path /opt/hobot/model/s100/basic/lanenet256x512.hbm \
            --test_img   /app/res/assets/input.jpg
        ```
- View Results

    Upon successful execution, the results will be saved as `instance_pred.png` and `binary_pred.png`:
    ```bash
    Results saved to: instance_pred.png and binary_pred.png
    ```

## Notes
- Output results are saved as `instance_pred.png` and `binary_pred.png`; users can inspect them directly.

- For more information on deployment options or model support, please refer to the official documentation or contact platform technical support.