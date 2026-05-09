---
sidebar_position: 12
---

# Text Detection and Recognition - PaddleOCR

This example runs the PaddleOCR model using the `hbm_runtime` inference engine for text detection and recognition, supporting OCR recognition and visualization in Chinese scenarios. The example code is located in the `/app/pydev_demo/08_OCR_sample/01_paddleOCR/` directory. The example code is also located in the `/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11/` directory.


## Model Description
- **Overview**:

    This example implements a two-stage OCR task for Chinese text detection and recognition based on PaddleOCR v3. The overall pipeline includes detecting text regions (using a detection model) and recognizing text content region by region (using a recognition model).

- **HBM Model Names**:

    - **Detection Model**: `cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm`
    - **Recognition Model**: `cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`

- **Input Format**:

    - **Detection Model**: BGR image → resized to 640×640 and converted to NV12 format (separated Y and UV planes)
    - **Recognition Model**: Rotated and cropped BGR text patch → resized to 48×320, normalized, and converted to RGB format

- **Output**:

    - **Detection Model**: Segmentation probability map (1×1×H×W); post-processing yields text bounding box coordinates
    - **Recognition Model**: Character token logits; decoded via CTC to produce recognized text strings

- **Model Download URLs** (automatically downloaded by the program):

    ```bash
    # Detection model
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm
    # Recognition model
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/paddle_ocr/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm
    ```

## Functionality Description
- **Model Loading**

    Uses `hbm_runtime` to load the text detection and recognition models separately, parses input/output names and shapes, and supports setting inference priority and BPU core binding.

- **Input Preprocessing**

    - **Detection Model**: Resizes the original image to 640×640 and converts it to NV12 format (for BPU inference).
    - **Recognition Model**: Resizes each rotated and cropped text patch to 48×320, converts to RGB format, normalizes the pixel values, and reshapes to NCHW layout.

- **Inference Execution**

    Calls the `.run()` method to perform forward inference, producing a probability map (detection) and logits (recognition).

- **Post-processing**

    - **Detection Model**:
        - Binarizes the probability map using a specified threshold
        - Finds contours of text regions and dilates them
        - Extracts rotated bounding boxes and crops corresponding image regions

    - **Recognition Model**:
        - Decodes logits using `CTCLabelDecode` to map them to text strings

    Finally, the recognized text is annotated in red on a blank canvas and stitched with the original image for visualization.

## Environment Dependencies
- Ensure the dependencies in `pydev` are installed:
    ```bash
    pip install -r ../../requirements.txt
    ```
- Install packages required for OCR processing:
    ```bash
    pip install pyclipper==1.3.0.post6 Pillow==9.0.1 paddlepaddle
    ```

## Directory Structure
```text
.
├── FangSong.ttf                # Font for Chinese character display
├── paddle_ocr.py               # Main program implementing text detection and recognition
├── postprocess/                # Post-processing logic (sorting, merging, decoding, etc.)
└── README.md                   # Usage instructions
```

## Parameter Description
| Parameter             | Default Value                                                           | Description                                      |
| --------------------- | ----------------------------------------------------------------------- | ------------------------------------------------ |
| `--det-model-path`    | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm` | Path to the text detection model                 |
| `--rec-model-path`    | `/opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm`   | Path to the text recognition model               |
| `--priority`          | `0`                                                                     | Inference priority (higher value = higher priority) |
| `--bpu-cores`         | `[0]`                                                                   | Indices of BPU cores to run inference on         |
| `--test-img`          | `/app/res/assets/gt_2322.jpg`                                              | Input image path                                 |
| `--label-file`        | `/app/res/labels/ppocr_keys_v1.txt`                                        | Path to label file required for text recognition |
| `--img-save-path`     | `result.jpg`                                                            | Path to save the inference result image          |
| `--threshold`         | `0.5`                                                                   | Threshold for binarizing text regions            |
| `--ratio-prime`       | `2.7`                                                                   | Expansion factor for text bounding boxes         |

## Quick Start
- **Run the model**
    - With default parameters:
        ```bash
        python paddle_ocr.py
        ```
    - With custom parameters:
        ```bash
        python paddle_ocr.py \
        --det-model-path /opt/hobot/model/s100/basic/cn_PP-OCRv3_det_infer-deploy_640x640_nv12.hbm \
        --rec-model-path /opt/hobot/model/s100/basic/cn_PP-OCRv3_rec_infer-deploy_48x320_rgb.hbm \
        --test-img /app/res/assets/gt_2322.jpg \
        --label-file /app/res/labels/ppocr_keys_v1.txt \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --threshold 0.5 \
        --ratio-prime 2.7
        ```
- **View Results**

    Upon successful execution, the results will be overlaid on the original image and saved to the path specified by `--img-save-path`:
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## Notes
- If the specified model path does not exist, the program will attempt to download the model automatically.