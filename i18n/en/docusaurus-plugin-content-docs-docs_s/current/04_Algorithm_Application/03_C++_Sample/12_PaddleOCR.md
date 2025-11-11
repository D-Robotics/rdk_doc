---
sidebar_position: 12
---

# Text Detection and Recognition - PaddleOCR

This example runs the PaddleOCR model on the BPU inference engine for text detection and recognition, supporting OCR recognition and visualization in Chinese scenarios. The example code is located in the `/app/cdev_demo/bpu/08_OCR_sample/01_paddleOCR/` directory.


## Model Description
- Overview:

    This example implements Chinese text detection and recognition (two-stage OCR) based on PaddleOCR v3. The overall pipeline includes detecting text regions (detection model) and recognizing text content region by region (recognition model).

- HBM Model Names:

- Detection Model: `cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm`

- Recognition Model: `cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`

- Input Format:

    - Detection Model: BGR image → resized to 640×640 and converted to NV12 format (separated Y and UV planes)

    - Recognition Model: Rotated and cropped BGR text patch → resized to 48×320, normalized, and converted to RGB format

- Output:

    - Detection Model: Segmentation probability map (1×1×H×W); post-processing yields bounding box coordinates of text regions

    - Recognition Model: Character token logits; decoded via CTC to obtain recognized text strings

## Functionality Description
- Model Loading

    Load the text detection and recognition models and parse their input/output specifications.

- Input Preprocessing

    - Detection Model: Resize the original image to 640×640 and convert it to NV12 format (for BPU inference).

    - Recognition Model: Resize each rotated and cropped text patch to 48×320, convert to RGB format, normalize, and finally reshape into NCHW layout.

- Inference Execution

    Call the `.infer()` method to perform forward inference, producing a probability map (detection) and logits (recognition).

- Post-processing

    - Detection Model:

        - Binarize the probability map using a predefined threshold

        - Find contours of text regions and dilate them

        - Extract rotated bounding boxes and crop corresponding image regions

    - Recognition Model:

        - Decode logits using `CTCLabelDecode` to map them into text strings

    Finally, overlay the recognition results as red text on a blank canvas and concatenate it with the original image for visualization.

## Environment Dependencies
Before compiling and running, ensure the following dependencies are installed:
```bash
sudo apt update
sudo apt install -y libgflags-dev libpolyclipping-dev
```

## Directory Structure
```text
.
|-- CMakeLists.txt                 # CMake build script: targets/dependencies/include paths/link libraries
|-- FangSong.ttf                   # Chinese font (used to render recognized text on the visualization canvas)
|-- README.md                      # Usage instructions (this file)
|-- inc
|   `-- paddleOCR.hpp              # OCR wrapper header: detection/recognition class interfaces (loading/preprocessing/inference/post-processing)
`-- src
    |-- main.cc                    # Program entry point: parse arguments → detect → crop → recognize → visualize → save
    `-- paddleOCR.cc               # Implementation details: polygon generation, cropping, CTC decoding, text rendering
```

## Build Project
- Configuration and Compilation
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## Model Download
If models are not found during runtime, download them using the following commands:
```bash
# Detection model
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm
# Recognition model
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm
```

## Parameter Description
| Parameter             | Description                                           | Default Value                                                                      |
| --------------------- | ----------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `--det_model_path`    | Text detection model (`.hbm`)                          | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm`        |
| `--rec_model_path`    | Text recognition model (`.hbm`)                        | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`         |
| `--test_image`        | Path to input test image                              | `/app/res/assets/gt_2322.jpg`                                                      |
| `--label_file`        | Recognition label file                                | `/app/res/labels/ppocr_keys_v1.txt`                                                |
| `--threshold`         | Binarization threshold for text regions (used in detection post-processing) | `0.5`                                                              |
| `--ratio_prime`       | Text box expansion factor (used in detection post-processing, affects polygon dilation) | `2.7`                                                  |



## Quick Start
- Run the model
    - Ensure you are in the `build` directory
    - Run with default parameters
        ```bash
        ./paddleOCR
        ```
    - Run with custom parameters
        ```bash
        ./paddleOCR \
            --det_model_path /opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm \
            --rec_model_path /opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm \
            --test_image     /app/res/assets/gt_2322.jpg \
            --label_file     /app/res/labels/ppocr_keys_v1.txt \
            --threshold 0.5 \
            --ratio_prime 2.7
        ```
- View Results

    Upon successful execution, results will be overlaid on the original image and saved as `build/result.jpg`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- The output result is saved as `result.jpg`, which users can inspect directly.

- For more information about deployment options or model support, please refer to the official documentation or contact platform technical support.