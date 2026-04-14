---
sidebar_position: 4
---

# YOLOv5 Model

## Introduction

The YOLOv5 sample under `/app/pydev_demo/07_yolov5_sample/` demonstrates object detection with YOLOv5. Compared with YOLOv3, YOLOv5 typically offers better accuracy and speed. The sample runs on a static image and draws boxes with confidence scores.

## Demo

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_07_runing.png)

## Hardware setup

### Connections
Only the RDK board is required.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_07_hw_connect.png)

## Quick start

### Code location on device

```
root@ubuntu:/app/pydev_demo/07_yolov5_sample# tree
.
├── coco_classes.names
├── kite.jpg
└── test_yolov5.py
```

### Build and run

```
python3 test_yolov5.py
```

### Sample output

```
root@ubuntu:/app/pydev_demo/07_yolov5_sample# ./test_yolov5.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-09-10,10:48:04.337.848) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2025-09-10,10:48:04.479.654) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 672, 672)
3
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 84, 84, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 42, 42, 255)
tensor type: float32
data type: float32
layout: NHWC
shape: (1, 21, 21, 255)
bbox: [593.949768, 80.819038, 672.215027, 147.131607], score: 0.856997, id: 33, name: kite
bbox: [215.716019, 696.537476, 273.653442, 855.298706], score: 0.852251, id: 0, name: person
bbox: [278.934448, 236.631256, 305.838867, 281.294922], score: 0.834647, id: 33, name: kite
bbox: [115.184196, 615.987, 167.202667, 761.042542], score: 0.781627, id: 0, name: person
bbox: [577.261719, 346.008453, 601.795349, 370.308624], score: 0.705358, id: 33, name: kite
bbox: [1083.22998, 394.714569, 1102.146729, 422.34787], score: 0.673642, id: 33, name: kite
bbox: [80.515938, 511.157104, 107.181572, 564.28363], score: 0.662, id: 0, name: person
bbox: [175.470078, 541.949219, 194.192871, 572.981812], score: 0.623189, id: 0, name: person
bbox: [518.504333, 508.224396, 533.452759, 531.92926], score: 0.597822, id: 0, name: person
bbox: [469.970398, 340.634796, 486.181305, 358.508972], score: 0.5593, id: 33, name: kite
bbox: [32.987705, 512.65033, 57.665741, 554.898804], score: 0.508812, id: 0, name: person
bbox: [345.142609, 486.988464, 358.24762, 504.551331], score: 0.50672, id: 0, name: person
bbox: [530.825439, 513.695679, 555.200256, 536.498352], score: 0.459818, id: 0, name: person
draw result time is : 0.03627920150756836
```

## Details

### Command-line options
No arguments; uses `kite.jpg` by default.

### Software architecture

1. Load YOLOv5  
2. Preprocess to NV12 672×672  
3. Forward pass  
4. Post-process with libpostprocess  
5. Draw and save  

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_07_yolov5_sample_software_arch.png)
</center>

### API flow

1. `models = dnn.load('../models/yolov5s_672x672_nv12.bin')`  
2. Resize → NV12  
3. `outputs = models[0].forward(nv12_data)`  
4. Post-process config  
5. Parse tensors  
6. Draw and save  

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_07_yolov5_sample_api_flow.png)
</center>

### FAQ

**Q:** Difference between YOLOv5 and YOLOv3?\
**A:** YOLOv5 improves architecture, training, and post-processing—usually better accuracy and speed.

**Q:** `hobot_dnn` missing?\
**A:** Install the RDK Python stack and official packages.

**Q:** Change image?\
**A:** Set `img_file = cv2.imread('...')`.

**Q:** Poor results?\
**A:** COCO-trained; fine-tune or switch models for your scene.

**Q:** Threshold?\
**A:** `yolov5_postprocess_info.score_threshold` (e.g. `0.5`).

**Q:** Video?\
**A:** Extend for video loops.

**Q:** Other YOLOv5 sizes?\
**A:** [model_zoo](https://github.com/D-Robotics/rdk_model_zoo) or [hobot_model](https://github.com/D-Robotics/hobot_model).

**Q:** More speed?\
**A:** Try lighter variants (YOLOv5n/s) or lower input resolution.

