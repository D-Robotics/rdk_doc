---
sidebar_position: 9
---

# USB Camera Real-Time Detection

## Example Introduction
The USB camera real-time detection example is located at `/app/pydev_demo/07_usb_camera_sample/`, providing a **Python interface** for real-time object detection. It demonstrates how to use `hbm_runtime` with the YOLOv5X model to perform inference on USB camera footage and display the detection results in a window in real-time.

Included model example:
```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample$ tree -L 1
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

## Effect Demonstration
> **Note**: Since this example pops up a window to display detection results, it needs to be run in a graphical environment. For server version images, it is recommended to use remote tools such as **MobaXterm** that support X11 forwarding to connect to the development board, allowing the display window to appear on your local computer.

The example reads the USB camera feed in real-time, detects objects in the画面, and overlays bounding boxes, class labels, and confidence scores on the window in real-time.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_07_usb_camera_sample_running.png)

## Hardware Preparation

### Hardware Connection
- One RDK development board
- One USB camera (connected to the USB port of the development board)
- Connect power cable and network cable

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_hw_connect.png)

## Quick Start

### Code Location on Board
Enter `/app/pydev_demo/07_usb_camera_sample/` to see the example files:

```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample# tree
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

### Compilation and Execution
Python examples do not require compilation; just run them directly:

```
cd /app/pydev_demo/07_usb_camera_sample
python usb_camera_yolov5x.py
```

Press the `q` key to exit (make sure the mouse focus is on the display window).

### Execution Effect
After running, the program will automatically find an available USB camera device and start real-time object detection. The detection results will be displayed in a window. Place the mouse focus on the display window and press `q` to exit the entire program.

```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample# python usb_camera_yolov5x.py
Opening video device: /dev/video0
Open USB camera successfully
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-20,19:02:15.744.442) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-20,19:02:15.859.240) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Place the mouse in the display window and press 'q' to quit
```

## Detailed Introduction

### Example Program Parameter Options
| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/yolov5s_672x672_nv12.bin` |
| `--priority` | Inference priority (0~255, 255 highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |
| `--label-file` | Class label path | str | `coco_classes.names` |
| `--resize-type` | Preprocessing resize method (0 direct scaling, 1 letterbox scaling) | int | `1` |
| `--classes-num` | Number of detection classes | int | `80` |
| `--nms-thres` | NMS IoU threshold | float | `0.45` |
| `--score-thres` | Confidence threshold | float | `0.25` |

### Software Architecture Description
This section introduces the software architecture and workflow of the USB camera example, explaining the entire execution process from model loading to result output, helping developers understand the overall code structure and data flow.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_usb_camera_software_arch.png)
</center>

1. **Camera Detection and Opening** - Automatically search `/dev/video*` and open the first available USB camera, set resolution and frame rate
2. **Model Loading** - Load the model file using the `hbm_runtime` module
3. **Image Loading** - Read real-time image frames
4. **Image Preprocessing** - Scale camera frames to model input size, convert BGR to NV12, and organize tensors according to model input format
5. **Model Inference** - Execute YOLOv5X forward inference on the BPU
6. **Result Post-processing** - Dequantize outputs, decode prediction boxes, confidence filtering and NMS, map coordinates back to original image
7. **Visualization Output** - Draw bounding boxes, class labels, and confidence scores on the image, display in real-time window, exit with `q`

### API Flow Description
This section lists the main API interfaces used in the example program, explaining the functionality, input parameters, and return values of each interface, helping developers quickly understand the code implementation details and interface calling methods. Interfaces prefixed with `cv2.` are general interfaces from the OpenCV library, while the others are custom interfaces from D-Robotics.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_usb_camera_api.png)
</center>

1. `find_first_usb_camera()`

   Automatically search for USB camera

2. `cv2.VideoCapture(device)`  
    
   Open the camera

3. `hbm_runtime.HB_HBMRuntime(model_path)`  

   Load the quantized model, input: model path

4. `load_class_names(label_file)`  
   
   Load class names, input: class file path

5. `cap.read()`

   Get image frame

6. `resized_image(img, input_W, input_H, resize_type)`

   Image scaling, input: image, target width, target height, resize type, return: scaled image

7. `bgr_to_nv12_planes(image)`

   Convert BGR to NV12 format, input: BGR image, return: y plane, uv plane

8. `model.run(input_data)`

   Execute model inference, input: preprocessed image data, return: model output

9. `dequantize_outputs(outputs, output_quants)`

   Dequantize results, input: model output, specified type, return: dequantized results

10. `decode_outputs(output_names, fp32_outputs, STRIDES, ANCHORS, classes_num)`

      Decode model output, input: output name list, model output, strides, anchors, number of classes, return: prediction results

11. `filter_classification(cls_output, conf_thres)`

      Filter prediction results, input: classification output, confidence threshold, return: confidence scores, class IDs, valid indices

12. `NMS(dbboxes, scores, ids, nms_thresh)`

      Non-maximum suppression, input: detection boxes, confidence scores, classes, IoU threshold, return: retained indices

13. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

      Scale detection boxes back to original image size, input: detection boxes, original image width, original image height, model input width, model input height, resize method, return: scaled detection boxes

14. `draw_boxes(img, boxes, ids, scores, coco_names, rdk_colors)`

      Draw detection results, input: image, detection boxes, class IDs, confidence scores, class list, color mapping

15. `cv2.imshow(...)` / `cv2.waitKey(1)`  
      Display and exit with key press

### FAQ

Q: What to do if the error "No USB camera found" occurs?  
A: Make sure the camera is plugged in and a `video*` device exists under `/dev`; try `v4l2-ctl --list-devices` to check.  

<!-- Q: What to do if there is no image or the error "Failed to open video device" occurs?  
A: Ensure a desktop environment and camera permissions; try a different USB port or camera.   -->

Q: What if the model file does not exist?  
A: Check if `--model-path` is correct, and look for the corresponding model under `/app/model/basic`.  

Q: Video lag or stuttering?  
A: Reduce the camera resolution or frame rate, or decrease the model input size.
<!-- ; Ensure BPU core configuration is correct.   -->

Q: How to exit the program?  
A: Place the mouse focus on the display window and press the `q` key to exit.