---
sidebar_position: 2
---

# Object Detection

## Example Introduction
The object detection examples are a set of **Python interface** development code examples located in `/app/pydev_demo/02_detection_sample/`, demonstrating how to use the `hbm_runtime` module for object detection tasks. These examples implement the same object detection functionality based on different YOLO series models.
Included model examples:

```
root@ubuntu:/app/pydev_demo/02_detection_sample$ tree -L 1
.
├── 01_ultralytics_yolov5x
├── 02_ultralytics_yolo11
├── 03_ultralytics_yolov8
└── 04_ultralytics_yolo10
```

## Effect Demonstration
The four examples have the same effect, only the models called are different. Here is the effect demonstration of the YOLOv5X model. The effects of other YOLO series models are similar.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_running_yolov5x.png)

## Hardware Preparation

### Hardware Connection
This example only requires the RDK development board itself, no additional peripheral connections are needed. Ensure the development board is properly powered on and the system is started.
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_hw_connect.png)

## Quick Start

### Code and Board Location
Navigate to `/app/pydev_demo/02_detection_sample/`, where you will see four example folders:

```
root@ubuntu:/app/pydev_demo/02_detection_sample# tree
.
├── 01_ultralytics_yolov5x
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolov5x.py
│   └── result.jpg
├── 02_ultralytics_yolo11
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolo11.py
│   └── result.jpg
├── 03_ultralytics_yolov8
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolov8.py
│   └── result.jpg
└── 04_ultralytics_yolo10
    ├── coco_classes.names
    ├── kite.jpg
    ├── ultralytics_yolo10.py
    └── result.jpg
```

### Compilation and Execution
Python examples do not require compilation; run them directly:
```
cd /app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x
python ultralytics_yolov5x.py 

cd /app/pydev_demo/02_detection_sample/02_ultralytics_yolo11
python ultralytics_yolo11.py

cd /app/pydev_demo/02_detection_sample/03_ultralytics_yolov8
python ultralytics_yolov8.py

cd /app/pydev_demo/02_detection_sample/04_ultralytics_yolo10
python ultralytics_yolo10.py
```

### Execution Effect
<!-- After running, the program will load the pre-trained YOLOv3 model, perform object detection on the kite.jpg image, and generate a result image output_image.jpg with detection boxes. -->

#### YOLOv5X Execution Effect

```
root@ubuntu:/app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x# python ultralytics_yolov5x.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:07:20.971.89) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-26,17:07:21.81.905) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Detected 18 objects: person(0.86), person(0.79), person(0.67), person(0.65), person(0.55), person(0.54), person(0.51), person(0.45), person(0.44), person(0.34), person(0.33), kite(0.88), kite(0.84), kite(0.71), kite(0.70), kite(0.57), kite(0.44), kite(0.31)
[Saved] Result saved to: result.jpg
```

#### YOLOv11 Execution Effect

```
root@ubuntu:/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11# python ultralytics_yolo11.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:10:52.565.485) [HorizonRT] The model builder version = 1.24.3

...

[Saved] Result saved to: result.jpg
```

#### YOLOv8 Execution Effect
```
root@ubuntu:/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8# python ultralytics_yolov8.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:12:13.322.476) [HorizonRT] The model builder version = 1.24.3

...

Detected 13 objects: person(0.88), person(0.87), person(0.72), person(0.50), person(0.45), person(0.34), person(0.30), kite(0.90), kite(0.84), kite(0.76), kite(0.71), kite(0.62), kite(0.61)
[Saved] Result saved to: result.jpg
```

#### YOLOv10 Execution Effect
```
root@ubuntu:~/app/pydev_demo/02_detection_sample/04_ultralytics_yolo10# python ultralytics_yolo10.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:13:38.644.334) [HorizonRT] The model builder version = 1.24.3

...

Detected 14 objects: kite(0.91), kite(0.81), kite(0.51), kite(0.39), kite(0.78), kite(0.41), kite(0.80), person(0.27), person(0.58), person(0.60), person(0.76), person(0.26), person(0.91), person(0.90)
[Saved] Result saved to: result.jpg
```

## Detailed Introduction

### Example Program Parameter Options Description
The object detection examples support command-line parameter configuration. The parameter descriptions for the four examples are essentially the same, but the default model paths differ for each example. If no parameters are specified, the program will automatically use default values to load the test image in the same directory for inference.

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | Varies by model, see table below |
| `--test-img` | Input test image path | str | `kite.jpg` |
| `--label-file` | Class label path (one class per line) | str | `coco_classes.names` |
| `--img-save-path` | Detection result image save path | str | `result.jpg` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |
| `--nms-thres` | Non-Maximum Suppression (NMS) threshold | float | `0.45` |
| `--score-thres` | Confidence threshold | float | `0.25` |

**Default model paths for each example:**

| Model | Default Model Path |
|-------|---------------------|
| YOLOv5X | `/opt/hobot/model/x5/basic/yolov5x_672x672_nv12.bin` |
| YOLOv11 | `/opt/hobot/model/x5/basic/yolo11n_detect_bayese_640x640_nv12.bin` |
| YOLOv8  | `/opt/hobot/model/x5/basic/yolov8x_detect_bayese_640x640_nv12.bin` |
| YOLOv10 | `/opt/hobot/model/x5/basic/yolov10x_detect_bayese_640x640_nv12.bin` |

### Software Architecture Description

This section introduces the software architecture and workflow of the object detection examples, explaining the complete execution process from model loading to result output. It helps understand the overall code structure and data flow. Since the architecture of the four examples is the same, only one software architecture diagram is provided here.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_detect_sample_software_arch.png)
</center>

1. **Model Loading** - Load the pre-compiled model file using the `hbm_runtime` module
2. **Image Loading** - Read the input test image
3. **Image Preprocessing** - Resize the input image to the model input size, convert BGR to NV12 format
4. **Model Inference** - Execute model forward computation on the BPU
5. **Result Post-processing** - Use the post_utils library to parse inference results, perform decoding, confidence filtering, and NMS (Non-Maximum Suppression)
6. **Result Output** - Draw detection boxes on the image and save the result image

### API Flow Description

This section lists the main API interfaces used in the example programs, explaining the function, input parameters, and return value of each interface. It helps developers quickly understand the code implementation details and interface calling methods. Since the calling interfaces of the four examples are essentially the same, only one API flow diagram is provided here.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_detect_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   Load a pre-compiled model. Input: model file path.

2. `load_image(image_path)`

   Load a test image. Input: image file path.

3. `resized_image(img, input_W, input_H, resize_type)`

   Resize the image. Input: image, target width, target height, resize type. Returns: resized image.

4. `bgr_to_nv12_planes(image)`

   Convert BGR to NV12 format. Input: BGR image. Returns: y plane, uv plane.

5. `model.run(input_data)`

   Execute model inference. Input: preprocessed image data. Returns: model output.

6. `dequantize_outputs(outputs, output_quants)`

   Dequantize the results. Input: model output, specified type. Returns: dequantized results.

7. `decode_outputs(output_names, fp32_outputs, STRIDES, ANCHORS, classes_num)`

   Decode model output. Input: list of output names, model output, strides, anchors, number of classes. Returns: prediction results.

8. `filter_predictions(pred, score_thres)`

   Filter prediction results. Input: prediction results, confidence threshold. Returns: detection boxes, confidences, classes.

9. `NMS(detections, iou_threshold)`

   Perform Non-Maximum Suppression. Input: detection boxes, confidences, classes, IoU threshold. Returns: indices to keep.

10. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    Scale detection boxes back to the original image size. Input: detection boxes, original image size, model input size, interpolation method. Returns: scaled detection boxes.

11. `draw_boxes(image, detections, class_names)`

    Draw detection results. Input: image, detection boxes, class IDs, confidences, class list, color mapping.

### FAQ

Q: What should I do if I get a "No module named 'hobot_dnn'" error when running the example?  
A: Please ensure that the RDK Python environment is properly installed, including official dedicated inference libraries such as `hobot_dnn`.

Q: How can I change the test image?  
A: Use the `--test-img` parameter to specify the image path when running the command, for example: `python ultralytics_yolov5x.py --test-img your_image.jpg`

<!-- Q: What if the detection results are inaccurate?  
A: The YOLOv3 model is trained on the COCO dataset. For specific scenarios, fine-tuning or using a more suitable model may be necessary. -->

Q: How can I adjust the detection threshold?  
A: Modify the value of `--score-thres` in the code. For example, changing it to 0.5 can increase detection sensitivity.

Q: Can it process video streams in real-time?  
A: The current example is designed for single images, but the code can be modified to achieve real-time object detection on video streams.

Q: How can I obtain other pre-trained YOLO models?  
A: You can refer to the [model_zoo repository](https://github.com/D-Robotics/rdk_model_zoo) or the [basic model repository of the toolchain](https://github.com/D-Robotics/hobot_model).