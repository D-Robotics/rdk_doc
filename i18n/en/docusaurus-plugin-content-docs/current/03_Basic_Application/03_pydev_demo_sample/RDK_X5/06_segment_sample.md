---
sidebar_position: 6
---

# Segmentation Sample Introduction

## Sample Overview
The segmentation sample is a set of **Python interface** development code examples located in `/app/pydev_demo/06_segment_sample/`, used to demonstrate how to perform segmentation tasks using the `hbm_runtime` module. This sample uses the MobileNet-UNet model to achieve segmentation of categories such as people, vehicles, road surfaces, and traffic signs.

Included model example:
```
root@ubuntu:/app/pydev_demo/06_segment_sample$ tree -L 1
.
└── 01_mobilenet_unet
```

## Effect Display
The sample performs pixel-level classification on images, using different colors to identify different categories (people, vehicles, road surfaces, traffic signs, etc.), and overlays the segmentation results on the original image.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_running_seg.png)

## Hardware Preparation

### Hardware Connection
This sample only requires the RDK development board itself, with no additional peripheral connections needed. Ensure the development board is properly powered on and the system is started.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## Quick Start

### Code and Board Location

Navigate to `/app/pydev_demo/06_segment_sample/` to find the folder containing the semantic segmentation sample:

```
root@ubuntu:/app/pydev_demo/06_segment_sample# tree
.
└── 01_mobilenet_unet
    ├── mobilenet_unet.py
    ├── segmentation.png
    └── segmentation_result.png
```

### Compilation and Execution
Python samples do not require compilation; they can be run directly.

```
cd /app/pydev_demo/06_segment_sample/01_mobilenet_unet
python mobilenet_unet.py
```

### Execution Effect

```
root@ubuntu:/app/pydev_demo/06_segment_sample/01_mobilenet_unet# python mobilenet_unet.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,11:51:43.693.476) [HorizonRT] The model builder version = 1.23.8
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,11:51:43.813.309) Model: unet_mobilenet_1024x2048_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...


=== Output Tensor Stride ===
unet_mobilenet_1024x2048_nv12:
  seg_head_seg_pred_4/FusedBatchNorm:0 -> stride: [9961472, 38912, 76, 4]

=== Scheduling Parameters ===
unet_mobilenet_1024x2048_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
root@ubuntu:/app/pydev_demo/06_segment_sample/01_mobilenet_unet#
```
The visualization result is an image named segmentation_result.png in the same directory.

## Detailed Introduction

### Sample Program Parameter Options
The semantic segmentation sample supports command-line parameter configuration. If no parameters are specified, the program will automatically load the test image in the same directory for inference using default values.

| Parameter | Description | Type | Default Value |
|-----------|-------------|------|----------------|
| `--model-path` | BPU quantized model path (`.bin`) | str | `/app/model/basic/mobilenet_unet_1024x2048_nv12.bin` |
| `--test-img` | Input test image path | str | `segmentation.png` |
| `--save-path` | Segmentation result image save path | str | `segmentation_result.png` |
| `--priority` | Inference priority (0~255, 255 is highest) | int | `0` |
| `--bpu-cores` | BPU core index list (e.g., `0 1`) | int list | `[0]` |

### Software Architecture Description

This section introduces the software architecture and workflow of the semantic segmentation sample, explaining the complete execution process from model loading to result output, helping to understand the overall code structure and data flow.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_seg_sample_software_arch.png)
</center>

1. **Model Loading** - Load the model file using the `hbm_runtime` module
2. **Image Loading** - Read the input test image
3. **Image Preprocessing** - Resize the input image to the model input size, convert BGR to NV12 format
4. **Model Inference** - Perform forward computation of the model on the BPU
5. **Result Post-processing** - Perform argmax operation on the model output to obtain category predictions for each pixel
6. **Result Output** - Draw segmentation results on the original image, using different colors to identify different categories, and save the result image

### API Flow Description

This section lists the main API interfaces used in the sample program, describing the functionality, input parameters, and return values of each interface to help developers quickly understand code implementation details and interface calling methods. Interfaces starting with `np.` are general Numpy library interfaces, while the rest are custom interfaces from D-Robotics.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_seg_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   Load a precompiled model. Input: model file path

2. `load_image(image_path)`

   Load the test image. Input: image file path

3. `resized_image(img, input_W, input_H, resize_type)`

   Resize the image. Input: image, target width, target height, resize type. Returns: resized image

4. `bgr_to_nv12_planes(image)`

   Convert BGR to NV12 format. Input: BGR image. Returns: y plane, uv plane

5. `model.run(input_data)`

   Execute model inference. Input: preprocessed image data. Returns: model output

6. `np.argmax(outputs, axis)`

   Get the index of the maximum value. Input: model output, specified axis. Returns: category index for each pixel

7. `draw_seg_result(image, pred_result, save_path)`

   Draw the segmentation result. Input: original image, prediction result, save path

### FAQ

Q: What should I do if I get a "ModuleNotFoundError: No module named 'hbm_runtime'" error when running the sample?  
A: Please ensure that the RDK Python environment is properly installed, including the official proprietary inference libraries such as `hbm_runtime`.

Q: How do I change the test image?  
A: Use the `--test-img` parameter to specify the image path when running the command, for example: `python mobilenet_unet.py --test-img your_image.jpg`

<!-- Q: What if the segmentation results are inaccurate?  
A: Ensure the input image is of good quality and its dimensions meet the model requirements. The MobileNet-UNet model input size is 1024x2048; the program will automatically resize the input image. -->

Q: How do I change the result save path?  
A: Use the `--save-path` parameter to specify the save path, for example: `--save-path /path/to/result.png`

Q: What if the model file does not exist?  
A: If the specified model path does not exist, look for the model file in the `/app/model/basic` directory or download the corresponding model as prompted.

<!-- Q: Can I process images of other sizes?  
A: Yes, the program will automatically resize the input image to the model input size (1024x2048), but it is recommended to use images close to this size for better results. -->

Q: How can I view the segmentation categories?  
A: The segmentation results use different colors to identify different categories (people, vehicles, road surfaces, traffic signs, etc.), which can be viewed in the saved result image.

Q: How can I obtain other pretrained semantic segmentation models?  
A: You can refer to the [model_zoo repository](https://github.com/D-Robotics/rdk_model_zoo) or the [toolchain base model repository](https://github.com/D-Robotics/hobot_model)