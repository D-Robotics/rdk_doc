---
sidebar_position: 7
---

# 3.3.7 YOLOv5s v6/v7 Sample Introduction

## Sample Overview
The YOLOv5s v6/v7 object detection sample is a **Python interface** development code example located in `/app/pydev_demo/12_yolov5s_v6_v7_sample/`, demonstrating how to use different versions of the YOLOv5s model (v6 and v7) for object detection tasks. This sample shows how to load and use different versions of the YOLOv5s model, allowing developers to compare differences in detection accuracy and performance across versions.

## Demo Display
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_runing_v6.png)

## Hardware Preparation

### Hardware Connection
This sample only requires the RDK development board itself, without additional peripheral connections. Ensure the development board is properly powered and the system is booted.
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_hw_connect.png)


## Quick Start

### Code and Board Location
Navigate to the `/app/pydev_demo/12_yolov5s_v6_v7_sample/` directory. The YOLOv5s v6/v7 sample contains the following files:
```
root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# tree
.
├── coco_classes.names
├── kite.jpg
├── test_yolov5s_v6.py
└── test_yolov5s_v7.py
```

### Compilation and Execution
The Python sample does not require compilation and can be run directly:

```bash
# Run YOLOv5s v6 sample
python3 test_yolov5s_v6.py

# Run YOLOv5s v7 sample
python3 test_yolov5s_v7.py
```

Execution Result
After running, the program will load the corresponding version of the YOLOv5s model, perform object detection on the kite.jpg image, and generate a result image with detection boxes named output_image.jpg.
```
root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# ./test_yolov5s_v6.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,09:13:46.814.163) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,09:13:46.956.776) Model: yolov5s_v6_640x640_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 640, 640)
3
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 80, 80, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 40, 40, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 20, 20, 255)
bbox: [593.067139, 80.756065, 671.163269, 153.011993], score: 0.897454, id: 33, name: kite
bbox: [113.915047, 612.207764, 167.110794, 762.353516], score: 0.859162, id: 0, name: person
bbox: [214.223053, 699.533386, 271.895355, 858.340942], score: 0.830993, id: 0, name: person
bbox: [279.538818, 237.951126, 306.371399, 280.617645], score: 0.827493, id: 33, name: kite
bbox: [1082.881104, 393.765442, 1100.385864, 423.442505], score: 0.670609, id: 33, name: kite
bbox: [576.906311, 346.061401, 600.630432, 370.330414], score: 0.666767, id: 33, name: kite
bbox: [81.075279, 506.947998, 109.796234, 565.465027], score: 0.654038, id: 0, name: person
bbox: [466.722168, 339.011902, 488.579193, 360.203003], score: 0.630985, id: 33, name: kite
bbox: [176.781357, 540.801941, 194.842041, 574.298706], score: 0.60232, id: 0, name: person
bbox: [28.475588, 511.213043, 54.388405, 559.522705], score: 0.547704, id: 0, name: person
bbox: [534.540955, 513.663635, 556.496826, 535.459045], score: 0.520755, id: 0, name: person
bbox: [518.65332, 503.918335, 536.506348, 530.772156], score: 0.459669, id: 0, name: person
bbox: [344.917938, 486.057465, 357.588165, 504.467285], score: 0.450269, id: 0, name: person
bbox: [307.091553, 374.980927, 325.684875, 403.705261], score: 0.409444, id: 33, name: kite
draw result time is : 0.03637397289276123



root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# ./test_yolov5s_v7.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,09:14:54.942.317) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,09:14:55.56.435) Model: yolov5s_v7_640x640_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 640, 640)
3
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 80, 80, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 40, 40, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 20, 20, 255)
bbox: [593.067139, 80.756065, 671.163269, 153.011993], score: 0.897454, id: 33, name: kite
bbox: [113.915047, 612.207764, 167.110794, 762.353516], score: 0.859162, id: 0, name: person
bbox: [214.223053, 699.533386, 271.895355, 858.340942], score: 0.830993, id: 0, name: person
bbox: [279.538818, 237.951126, 306.371399, 280.617645], score: 0.827493, id: 33, name: kite
bbox: [1082.881104, 393.765442, 1100.385864, 423.442505], score: 0.670609, id: 33, name: kite
bbox: [576.906311, 346.061401, 600.630432, 370.330414], score: 0.666767, id: 33, name: kite
bbox: [81.075279, 506.947998, 109.796234, 565.465027], score: 0.654038, id: 0, name: person
bbox: [466.722168, 339.011902, 488.579193, 360.203003], score: 0.630985, id: 33, name: kite
bbox: [176.781357, 540.801941, 194.842041, 574.298706], score: 0.60232, id: 0, name: person
bbox: [28.475588, 511.213043, 54.388405, 559.522705], score: 0.547704, id: 0, name: person
bbox: [534.540955, 513.663635, 556.496826, 535.459045], score: 0.520755, id: 0, name: person
bbox: [518.65332, 503.918335, 536.506348, 530.772156], score: 0.459669, id: 0, name: person
bbox: [344.917938, 486.057465, 357.588165, 504.467285], score: 0.450269, id: 0, name: person
bbox: [307.091553, 374.980927, 325.684875, 403.705261], score: 0.409444, id: 33, name: kite
draw result time is : 0.037317872047424316
root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# 
```

## Detailed Introduction

### Sample Program Parameter Options
The YOLOv5s v6/v7 object detection sample does not require command-line parameters and can be run directly. The program will automatically load the kite.jpg image in the same directory for detection processing.

### Software Architecture Description
- The software architecture of the YOLOv5s v6/v7 object detection sample includes the following core components:

- Model Loading: Use the pyeasy_dnn module to load the pre-trained YOLOv5s model (v6 or v7 version)

-  Image Preprocessing: Convert the input image to the NV12 format and specified size (640x640) required by the model

- Model Inference: Call the model for forward computation to generate feature maps

- Post-processing: Use the libpostprocess library to parse the model output and generate detection results

- Result Visualization: Draw detection boxes and category information on the original image

- Result Saving: Save the visualized results as an image file

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_software_arch.png)
</center>

### API Flow Description
Model Loading:

- v6: models = dnn.load('../models/yolov5s_v6_640x640_nv12.bin')

- v7: models = dnn.load('../models/yolov5s_v7_640x640_nv12.bin')

- Image Preprocessing: Adjust image size and convert to NV12 format

- Model Inference: outputs = models[0].forward(nv12_data)

- Post-processing Configuration: Set post-processing parameters (size, thresholds, etc.)

- Result Parsing: Call the post-processing library to parse output tensors

- Result Visualization: Draw detection boxes and label information on the original image

- Result Saving: Save the results as an image file

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_api_flow.png)
</center>

### YOLOv5s v6 vs v7 Version Comparison
Main Improvements
YOLOv5 v7 brings the following main improvements compared to v6:

1. Architecture Optimization: Improved network structure and training strategies

2. Accuracy Improvement: Increased object detection accuracy

3. Speed Optimization: Optimized inference speed and improved efficiency

4. Training Improvements: Enhanced data augmentation and training strategies

5. Deployment Optimization: Better support for various deployment environments

### FAQ
**Q:** What's the difference between YOLOv5s v6 and v7?  
**A:** The v7 version has optimizations in architecture, training strategies, and inference speed, typically offering higher detection accuracy and faster inference speed.  

**Q:** What should I do if I encounter a "No module named 'hobot_dnn'" error when running the sample?  
**A:** Please ensure the RDK Python environment is correctly installed, including the hobot_dnn module and other official dedicated inference libraries.   

**Q:** How can I change the test image?  
**A:** Place the new image file in the sample directory and modify img_file = cv2.imread('your_image_path') in the code.  

**Q:** How to choose between v6 and v7 versions?  
**A:** You can test based on specific application requirements. The v7 version typically performs better, but you can also choose the most suitable version based on actual test results.  

**Q:** How to adjust the detection threshold?  
**A:** Modify the value of yolov5_postprocess_info.score_threshold in the code. For example, changing it to 0.5 can increase detection sensitivity.  

**Q:** Can it process video streams in real-time?  
**A:** The current sample is designed for single images, but the code can be modified to achieve real-time object detection for video streams.  

**Q:** How to further improve detection speed?  
**A:** You can try using smaller input sizes (if supported by the model) or utilize hardware acceleration features.  

**Q:** What specific improvements does v7 have over v6?  
**A:** The v7 version improves network architecture, training strategies, and data augmentation methods, enhances detection accuracy and inference speed, and optimizes model deployment compatibility.

