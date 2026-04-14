---
sidebar_position: 7
---

# YOLOv5s v6 / v7 Sample

## Introduction

The sample under `/app/pydev_demo/12_yolov5s_v6_v7_sample/` shows how to run YOLOv5s v6 and v7 checkpoints for object detection so you can compare accuracy and speed.

## Demo

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_runing_v6.png)

## Hardware setup

### Connections
Only the RDK board is required.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_hw_connect.png)

## Quick start

### Code location on device

```
root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# tree
.
├── coco_classes.names
├── kite.jpg
├── test_yolov5s_v6.py
└── test_yolov5s_v7.py
```

### Build and run

```bash
python3 test_yolov5s_v6.py
python3 test_yolov5s_v7.py
```

### Sample output

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

## Details

### Command-line options
No arguments; uses `kite.jpg`.

### Software architecture

Load YOLOv5s (v6 or v7) → preprocess NV12 640×640 → forward → post-process → draw → save.

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_software_arch.png)
</center>

### API flow

- v6: `dnn.load('../models/yolov5s_v6_640x640_nv12.bin')`  
- v7: `dnn.load('../models/yolov5s_v7_640x640_nv12.bin')`  

Then preprocess → `forward` → post-process → visualize → save.

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_api_flow.png)
</center>

### YOLOv5s v6 vs v7

YOLOv5 v7 improves over v6 with architecture and training updates—typically better accuracy and faster inference, plus better deployment characteristics.

### FAQ

**Q:** v6 vs v7?\
**A:** v7 is usually more accurate and faster; benchmark on your workload.

**Q:** `hobot_dnn`?\
**A:** Install RDK Python environment.

**Q:** Change image?\
**A:** Update `cv2.imread` path.

**Q:** Which to use?\
**A:** Prefer v7 unless testing shows v6 fits better.

**Q:** Threshold?\
**A:** `yolov5_postprocess_info.score_threshold`.

**Q:** Video?\
**A:** Extend for video capture.

**Q:** More speed?\
**A:** Smaller input if supported, or hardware tuning.

**Q:** v7 changes in detail?\
**A:** Network, training, augmentations, and deployment tweaks for better accuracy/speed.

