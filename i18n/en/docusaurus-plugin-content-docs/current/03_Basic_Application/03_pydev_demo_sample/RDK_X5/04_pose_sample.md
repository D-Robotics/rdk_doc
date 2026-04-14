---
sidebar_position: 4
---

# Pose Estimation

## Example Introduction
The pose estimation example is a set of **Python interface** code examples located in `/app/pydev_demo/04_pose_sample/`, designed to demonstrate how to use the `hbm_runtime` module for human pose estimation tasks. The example implements human keypoint detection functionality based on the YOLO11 model.

Included model example:
```
root@ubuntu:/app/pydev_demo/04_pose_sample$ tree -L 1
.
└── 01_ultralytics_yolo11_pose
```

## Effect Demonstration
The example detects humans in an image and annotates human keypoints (such as head, shoulders, elbows, wrists, knees, ankles, etc.), while also drawing bounding boxes and confidence scores.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_running_yolov11_pose.png)

## Hardware Preparation

### Hardware Connection
This example only requires the RDK development board itself, with no additional peripherals needed. Ensure the development board is properly powered and the system is booted.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## Quick Start

### Code and Board Location

Navigate to `/app/pydev_demo/04_pose_sample/` to find the folder containing the pose estimation example:

```
root@ubuntu:/app/pydev_demo/04_pose_sample# tree
.
└── 01_ultralytics_yolo11_pose
    ├── bus.jpg
    ├── coco_classes.names
    ├── result.jpg
    ├── ultralytics_yolo11_pose.py
    └── yolo11n_pose_bayese_640x640_nv12.bin
```

### Compilation and Execution
Python examples do not require compilation and can be run directly.

```
cd /app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose
python ultralytics_yolo11_pose.py 
```

### Execution Result

```
root@ubuntu:/app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose# python ultralytics_yolo11_pose.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,10:16:20.686.495) [HorizonRT] The model builder version = 1.24.3
...
[Saved] Result saved to: result.jpg
```

## Detailed Description

### Example Program Parameter Options
The pose estimation example supports command-line parameter configuration. If no parameters are specified, the program will automatically load the test image in the same directory for inference using default values.

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/yolo11n_pose_bayese_640x640_nv12.bin` |
| `--test-img` | Input test image path | str | `bus.jpg` |
| `--label-file` | Class label path (one class per line) | str | `coco_classes.names` |
| `--img-save-path` | Detection result image save path | str | `result.jpg` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |
| `--nms-thres` | Non-Maximum Suppression (NMS) threshold | float | `0.7` |
| `--score-thres` | Confidence threshold | float | `0.25` |
| `--kpt-conf-thres` | Keypoint display confidence threshold | float | `0.5` |

### Software Architecture Description

This section describes the software architecture and workflow of the pose estimation example, explaining the complete execution process from model loading to result output, helping to understand the overall code structure and data flow.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_pose_sample_software_arch.png)
</center>

1. **Model Loading** - Load the precompiled model file using the `hbm_runtime` module
2. **Image Loading** - Read the input test image
3. **Image Preprocessing** - Resize the input image to the model input size, convert BGR to NV12 format
4. **Model Inference** - Execute model forward computation on the BPU
5. **Result Post-processing** - Parse inference results using the post_utils library, perform decoding, confidence filtering, and NMS (Non-Maximum Suppression), extract bounding boxes and keypoint coordinates
6. **Result Output** - Draw bounding boxes and keypoints on the image and save the result image

### API Flow Description

This section lists the main API interfaces used in the example program, describing the functionality, input parameters, and return values of each interface to help developers quickly understand code implementation details and interface calling methods. Interfaces prefixed with `cv2.` are general OPENCV interfaces, while the rest are `hbm_runtime` and related interfaces.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_pose_yolo11_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   Load a precompiled model. Input: model file path

2. `load_image(image_path)`

   Load a test image. Input: image file path

3. `resized_image(img, input_W, input_H, resize_type)`

   Resize an image. Input: image, target width, target height, resize type. Returns: resized image

4. `bgr_to_nv12_planes(image)`

   Convert BGR to NV12 format. Input: BGR image. Returns: y plane, uv plane

5. `model.run(input_data)`

   Execute model inference. Input: preprocessed image data. Returns: model output

6. `dequantize_outputs(outputs, output_quants)`

   Dequantize results. Input: model output, specified type. Returns: dequantized results

7. `filter_classification(cls_output, conf_thres)`

   Filter prediction results. Input: classification output, confidence threshold. Returns: confidence scores, class IDs, valid indices

8. `decode_boxes(box_output, valid_indices, anchor_size, stride, weights)`

   Decode bounding boxes. Input: box output, valid indices, anchor size, stride, weights. Returns: bounding boxes

9. `decode_kpts(kpts_output, valid_indices, anchor_size, stride)`

   Decode keypoints. Input: keypoint output, valid indices, anchor size, stride. Returns: keypoint coordinates and confidence scores

10. `NMS(dbboxes, scores, ids, nms_thresh)`

    Perform Non-Maximum Suppression. Input: bounding boxes, confidence scores, class IDs, IoU threshold. Returns: retained indices

11. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    Scale bounding boxes back to original image size. Input: bounding boxes, original image width, original image height, model input width, model input height, resize method. Returns: scaled bounding boxes

12. `scale_keypoints_to_original_image(kpts_xy, kpts_score, boxes, img_w, img_h, input_W, input_H, resize_type)`

    Scale keypoint coordinates back to original image size. Input: keypoint coordinates, keypoint confidence scores, bounding boxes, original image width, original image height, model input width, model input height, resize method. Returns: scaled keypoint coordinates and confidence scores

13. `draw_boxes(img, boxes, ids, scores, coco_names, rdk_colors)`

    Draw detection results. Input: image, bounding boxes, class IDs, confidence scores, class list, color mapping

14. `draw_keypoints(image, kpts_xy, kpt_score, kpt_conf_thresh)`

    Draw keypoint skeleton. Input: image, keypoint coordinates, keypoint confidence scores, keypoint confidence threshold

15. `cv2.imwrite(image_path, image)`
    
    Save the result image. Input: save path, image

### FAQ

Q: What should I do if I get a "No module named 'hbm_runtime'" error when running the example?  
A: Ensure that the RDK Python environment is correctly installed, including the official inference libraries such as the `hbm_runtime` module.

Q: How do I change the test image?  
A: Use the `--test-img` parameter when running the command to specify the image path, e.g., `python ultralytics_yolo11_pose.py --test-img your_image.jpg`

Q: What if the detection results are inaccurate?  
A: Try adjusting the `--score-thres` and `--nms-thres` parameters, or use a higher quality input image.

Q: How do I adjust the detection threshold?  
A: Modify the value of `--score-thres` in the code. For example, changing it to 0.5 can improve detection sensitivity.

Q: What if the keypoints are not fully displayed?  
A: Adjust the `--kpt-conf-thres` parameter. Lowering the threshold can display more keypoints, but may include points with low confidence.

Q: Can it process video streams in real-time?  
A: The current example is designed for single images, but the code can be modified to achieve real-time pose estimation on video streams.

Q: How can I obtain other pre-trained YOLO pose estimation models?  
A: Refer to the [model_zoo repository](https://github.com/D-Robotics/rdk_model_zoo) or the [toolchain base model repository](https://github.com/D-Robotics/hobot_model).