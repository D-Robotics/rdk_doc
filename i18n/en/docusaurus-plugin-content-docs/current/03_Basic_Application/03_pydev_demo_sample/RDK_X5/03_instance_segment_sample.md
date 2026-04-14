---
sidebar_position: 3
---

# Semantic Segmentation

## Example Introduction
The semantic segmentation examples are a set of **Python interface** development code examples located in `/app/pydev_demo/03_instance_segmentation_sample/`, used to demonstrate how to perform semantic segmentation tasks using the `hbm_runtime` module. These examples implement semantic segmentation functionality based on different neural network models.

Included model examples:
```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample$ tree -L 1
.
├── 01_unetmobilenet
└── 02_ultralytics_yolo11_seg
```

## Effect Demonstration
Both examples achieve semantic segmentation, but with different effects. Below are the effect demonstrations of the two models.

### unetmobilenet Effect Demonstration

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_running_unetmobilenet.png)

### yolov11 Effect Demonstration
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_running_yolov11.png)

## Hardware Preparation

### Hardware Connection
This example only requires the RDK development board itself, no additional peripheral connections are needed. Ensure the development board is properly powered and the system is booted.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## Quick Start

### Code and Board Location

Navigate to `/app/pydev_demo/03_instance_segmentation_sample/` to see the two example folders:

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample# tree
.
├── 01_unetmobilenet
│   ├── segmentation.png
│   ├── unet_mobilenet.py
│   └── result.jpg
└── 02_ultralytics_yolo11_seg
    ├── bus.jpg
    ├── coco_classes.names
    ├── result.jpg
    ├── ultralytics_yolo11_seg.py
    └── yolo11n_seg_bayese_640x640_nv12.bin
```

### Compilation and Execution
Python examples do not require compilation; run them directly:

```
cd /app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet
python unet_mobilenet.py 

cd /app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg
python ultralytics_yolo11_seg.py
```

### Execution Results

#### UNetMobileNet Execution Results

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet# python unet_mobilenet.py 
[BPU_PLAT]BPU Platform Version(1.3.6)! soc info(x5)
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-02,02:54:24.731.966) [HorizonRT] The model builder version = 1.23.8

[W][DNN]bpu_model_info.cpp:491][Version](2000-01-02,02:54:24.831.251) Model: unet_mobilenet_1024x2048_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
=== Model Name List ===
['unet_mobilenet_1024x2048_nv12']

...

=== Scheduling Parameters ===
unet_mobilenet_1024x2048_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
[Saved] Result saved to: result.jpg
```

#### YOLOv11 Instance Segmentation Execution Results

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg# python ultralytics_yolo11_seg.py
[BPU_PLAT]BPU Platform Version(1.3.6)! soc info(x5)
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-02,02:51:30.798.145) [HorizonRT] The model builder version = 1.24.3
=== Model Name List ===
['yolo11n_seg_bayese_640x640_nv12']

=== Model Count ===
1

=== Input Counts ===
yolo11n_seg_bayese_640x640_nv12: 1

...

=== Scheduling Parameters ===
yolo11n_seg_bayese_640x640_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
[Saved] Result saved to: result.jpg

```

## Detailed Description

### Example Program Parameter Options
The semantic segmentation examples support command-line parameter configuration. The parameter descriptions differ between the two examples, and each example has its own default model path. If no parameters are specified, the program will automatically use default values to load the test image in the same directory for inference.

#### UNetMobileNet Parameters

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/mobilenet_unet_1024x2048_nv12.bin` |
| `--test-img` | Input test image path | str | `segmentation.png` |
| `--img-save-path` | Segmentation result image save path | str | `result.jpg` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |
| `--alpha-f` | Transparency factor for blending segmentation mask with original image (0.0=mask only, 1.0=original only) | float | `0.75` |

#### YOLOv11 Parameters

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/yolo11n_seg_bayese_640x640_nv12.bin` |
| `--test-img` | Input test image path | str | `bus.jpg` |
| `--label-file` | Class label path (one class per line) | str | `coco_classes.names` |
| `--img-save-path` | Segmentation result image save path | str | `result.jpg` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |
| `--nms-thres` | Non-Maximum Suppression (NMS) threshold | float | `0.7` |
| `--score-thres` | Confidence threshold | float | `0.25` |
| `--is-open` | Whether to perform morphological operations on masks | bool | `True` |
| `--is-point` | Whether to draw mask edge contours | bool | `True` |

### Software Architecture Description

This section describes the software architecture and workflow of the semantic segmentation examples, explaining the complete execution process from model loading to result output, helping to understand the overall code structure and data flow. Since the architecture of the two examples is roughly the same, a software architecture diagram is provided below:

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_software_arch.png)
</center>

1. **Model Loading** - Load the pre-compiled model file using the `hbm_runtime` module
2. **Image Loading** - Read the input test image
3. **Image Preprocessing** - Resize the input image to the model's input dimensions, convert BGR to NV12 format
4. **Model Inference** - Execute model forward computation on the BPU
5. **Result Post-processing** - Parse segmentation output, map class predictions to color masks, blend with original image
6. **Result Output** - Save the result image with segmentation mask overlay

### API Flow Description

This section lists the main API interfaces used in the example programs, describing the functionality, input parameters, and return values of each interface to help developers quickly understand code implementation details and interface calling methods. Since the two examples use different API interfaces, the corresponding API flow diagrams are provided separately. Interfaces starting with `cv2.` are general OPENCV interfaces, those starting with `np.` are general Numpy interfaces, and the rest are D-Robotics custom interfaces.

#### UNetMobileNet API Flow

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_unet_sample_api_flow.png)
</center>

<!-- <center>
<img src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_unet_sample_api_flow.png" width="1200" height="780" alt="API_Flow" />
</center> -->

1. `hbm_runtime.HB_HBMRuntime(model_path)`
   
   Load the pre-compiled model, input: model file path

2. `load_image(image_path)` 
   
   Load the test image, input: image file path

3. `resized_image(img, input_W, input_H, resize_type)`
   
   Resize the image, input: image, target width, target height, resize type

4. `bgr_to_nv12_planes(image)` 
   
   Convert BGR to NV12 format, input: BGR image, returns: y plane, uv plane

5. `model.run(input_data)`
   
   Execute model inference, input: preprocessed image data

6. `np.argmax(logits, axis=-1)` 
   
   Get the index of maximum value, input: model output, specified dimension, returns: predicted class

7. `cv2.resize(pred_class, (input_W, input_H), interpolation=cv2.INTER_NEAREST)`
   
   Resize predicted class to original image dimensions, input: predicted class, (model input width, model input height), interpolation type

8. `recover_to_original_size(seg_mask, img_w, img_h, resize_type)`
   
   Restore segmentation mask to original image dimensions, input: segmentation mask, original image width, original image height, resize type

9. `cv2.addWeighted(origin_image, alpha, parsing_img, 1-alpha, 0.0)`
   
   Blend original image with segmentation mask, input: original image, original image weight, segmentation mask color image, segmentation mask weight, brightness offset

#### YOLOv11 Instance Segmentation API Flow

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_yolo11_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`
   
   Load the pre-compiled model, input: model file path

2. `load_image(image_path)`
   
   Load the test image, input: image file path

3. `resized_image(img, input_W, input_H, resize_type)`
   
   Resize the image, input: image, target width, target height, resize type, returns: resized image

4. `bgr_to_nv12_planes(image)`
   
   Convert BGR to NV12 format, input: BGR image, returns: y plane, uv plane

5. `model.run(input_data)`
   
   Execute model inference, input: preprocessed image data, returns: model output

6. `dequantize_outputs(outputs, output_quants)`
   
   Dequantize the results, input: model output, specified type, returns: dequantized results

7. `filter_classification(cls_output, conf_thres)`
   
   Filter classification results, input: classification output, confidence threshold, returns: confidence, class ID, valid indices

8. `decode_boxes(box_output, valid_indices, anchor_size, stride, weights)`
   
   Decode detection boxes, input: box regression output, valid indices, anchor size, stride, weights, returns: detection boxes

9. `filter_mces(mces_output, valid_indices)`
   
   Filter mask coefficients, input: mask coefficient output, valid indices, returns: mask coefficients

10. `NMS(dbboxes, scores, ids, nms_thresh)`
    
    Non-Maximum Suppression, input: detection boxes, confidence scores, class IDs, IoU threshold, returns: retained indices

11. `decode_masks(mces, boxes, protos, input_W, input_H, Mask_W, Mask_H, mask_thresh)`
    
    Decode segmentation masks, input: mask coefficients, detection boxes, mask prototypes, input width, input height, mask width, mask height, threshold, returns: masks

12. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`
    
    Resize detection boxes to original image dimensions, input: detection boxes, original image width, original image height, model input width, model input height, resize type, returns: resized detection boxes

13. `resize_masks_to_boxes(masks, boxes, img_w, img_h, do_morph)`
    
    Resize masks to detection box regions and scale to original image dimensions, input: masks, detection boxes, original image width, original image height, whether to perform morphological operations, returns: resized masks

14. `draw_boxes(image, boxes, cls_ids, scores, class_names, rdk_colors)`
    
    Draw detection boxes, input: image, detection boxes, class IDs, confidence scores, class list, color mapping

15. `draw_masks(image, boxes, masks, cls_ids, rdk_colors, alpha)`
    
    Draw segmentation masks, input: image, detection boxes, masks, class IDs, color mapping, transparency

16. `draw_contours(image, boxes, masks, cls_ids, rdk_colors, thickness)`
    
    Draw mask edge contours, input: image, detection boxes, masks, class IDs, color mapping, line width

17. `cv2.imwrite(image_path, image)`
    
    Save the result image, input: save path, image

### FAQ

Q: What should I do if I get a "ModuleNotFoundError: No module named 'hbm_runtime'" error when running the example?  
A: Please ensure that the RDK Python environment is properly installed, including official proprietary inference libraries such as the `hbm_runtime` module.

Q: How do I change the test image?  
A: Specify the image path using the `--test-img` parameter when running the command, for example: `python unet_mobilenet.py --test-img your_image.jpg`

Q: What is the difference between UNetMobileNet and YOLOv11 instance segmentation?  
A: UNetMobileNet is a semantic segmentation model that performs pixel-level classification of the image without distinguishing different instances of the same class; YOLOv11 is an instance segmentation model that can simultaneously detect objects and segment precise contours of each instance.

Q: How do I adjust the transparency of the segmentation results?  
A: For UNetMobileNet, you can adjust the `--alpha-f` parameter (0.0=mask only, 1.0=original only); for YOLOv11, you can modify the `alpha` parameter of the `draw_masks` function in the code.

Q: What if the detection results are inaccurate?  
A: You can try adjusting the `--score-thres` and `--nms-thres` parameters, or use a model better suited for your specific scenario. The YOLOv11 model is trained on the COCO dataset and may require fine-tuning for specific scenarios.

Q: How can I obtain other pre-trained segmentation models?  
A: You can refer to the [model_zoo repository](https://github.com/D-Robotics/rdk_model_zoo) or the [toolchain basic model repository](https://github.com/D-Robotics/hobot_model)

Q: Can it process video streams in real-time?  
A: The current examples are designed for single images, but the code can be modified to achieve real-time instance segmentation of video streams.