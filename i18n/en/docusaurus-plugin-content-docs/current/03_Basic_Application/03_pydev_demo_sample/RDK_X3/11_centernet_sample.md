---
sidebar_position: 6
---

# CenterNet Sample

## Introduction

The CenterNet sample under `/app/pydev_demo/11_centernet_sample/` demonstrates anchor-free detection via center points. It can be more accurate on small objects than anchor-based detectors.

## Demo

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_runing.png)

## Hardware setup

### Connections
Only the RDK board is required.

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_hw_connect.png)

## Quick start

### Code location on device

```
root@ubuntu:/app/pydev_demo/11_centernet_sample# tree
.
├── kite.jpg
└── test_centernet.py
```

### Build and run

```
python3 test_centernet.py
```

### Sample output

```
root@ubuntu:/app/pydev_demo/11_centernet_sample# ./test_centernet.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,09:04:41.531.16) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,09:04:41.900.505) Model: centernet_resnet101_512x512_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 512, 512)
3
tensor type: int16
data type: int16
layout: NCHW
shape: (1, 80, 128, 128)
tensor type: int32
data type: int32
layout: NCHW
shape: (1, 2, 128, 128)
tensor type: int32
data type: int32
layout: NCHW
shape: (1, 2, 128, 128)
inferece time is : 0.038387179374694824
postprocess time is : 0.008000016212463379
bbox: [535.099487, 518.289795, 552.85321, 533.168884], score: 0.411767, id: 0, name: person
bbox: [1205.362183, 452.914368, 1213.579956, 462.972992], score: 0.416783, id: 0, name: person
bbox: [37.22504, 512.7771, 55.708, 551.758057], score: 0.479478, id: 0, name: person
bbox: [302.082428, 373.24588, 326.903992, 406.137909], score: 0.481639, id: 33, name: kite
bbox: [79.558655, 511.698425, 104.828987, 561.18573], score: 0.483801, id: 0, name: person
bbox: [763.340332, 381.275391, 773.633484, 388.304504], score: 0.49954, id: 33, name: kite
bbox: [512.4505, 506.076019, 535.645386, 527.521606], score: 0.50862, id: 0, name: person
bbox: [1083.63208, 398.408325, 1101.694458, 420.525391], score: 0.560764, id: 33, name: kite
bbox: [578.292786, 346.042908, 599.692627, 366.190063], score: 0.561831, id: 33, name: kite
bbox: [470.628357, 341.963165, 484.707916, 356.737732], score: 0.599044, id: 33, name: kite
bbox: [176.473038, 539.143616, 190.889175, 567.11084], score: 0.602763, id: 0, name: person
bbox: [116.152634, 617.276489, 164.758057, 756.843872], score: 0.655859, id: 0, name: person
bbox: [345.088379, 485.373199, 357.569305, 505.430756], score: 0.656233, id: 0, name: person
bbox: [593.67334, 80.689156, 670.185425, 148.085022], score: 0.668426, id: 33, name: kite
bbox: [214.575424, 696.642883, 276.78363, 853.193604], score: 0.709791, id: 0, name: person
bbox: [278.955109, 234.4608, 304.618103, 279.500824], score: 0.716334, id: 33, name: kite
draw result time is : 0.036167144775390625
det.size(): 16root@ubuntu:/app/pydev_demo/11_centernet_sample# 
```

## Details

### Command-line options
No arguments; uses `kite.jpg`.

### Software architecture

1. Load CenterNet-ResNet101  
2. Preprocess NV12 512×512  
3. Forward → heatmaps / size / offset  
4. Post-process  
5. Draw and save  

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_centernet_sample_software_arch.png)
</center>

### API flow

1. `models = dnn.load('../models/centernet_resnet101_512x512_nv12.bin')`  
2. Preprocess  
3. `outputs = models[0].forward(nv12_data)`  
4. Post-process  
5. Visualize and save  

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_centernet_sample_api_flow.png)
</center>

### FAQ

**Q:** CenterNet vs YOLO?\
**A:** CenterNet predicts centers instead of anchors—often better localization, especially for small objects.

**Q:** `hobot_dnn`?\
**A:** Install RDK Python stack.

**Q:** Change image?\
**A:** Update `cv2.imread` path.

**Q:** Poor results?\
**A:** COCO-trained; fine-tune for your domain.

**Q:** Threshold?\
**A:** `centernet_postprocess_info.score_threshold`.

**Q:** Video?\
**A:** Extend for frame loop.

**Q:** Small objects?\
**A:** Center points + offsets help avoid anchor–scale mismatch.

**Q:** Higher accuracy?\
**A:** Larger input if supported, fine-tuning, or post-process tuning.

