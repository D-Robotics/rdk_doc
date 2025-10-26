---
sidebar_position: 7
---

# 3.3.7 yolov5s_v6_v7 示例介绍

## 示例简介
YOLOv5s v6/v7 目标检测示例是一个位于 `/app/pydev_demo/12_yolov5s_v6_v7_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 YOLOv5s 模型的不同版本（ v6 和 v7 ）进行目标检测任务。该示例展示了如何加载和使用不同版本的 YOLOv5s 模型，让开发者能够比较不同版本在检测精度和性能上的差异。

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_runing_v6.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_hw_connect.png)


## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/12_yolov5s_v6_v7_sample/` 位置，可以看到 YOLOv5s v6/v7 示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/12_yolov5s_v6_v7_sample# tree
.
├── coco_classes.names
├── kite.jpg
├── test_yolov5s_v6.py
└── test_yolov5s_v7.py
```

### 编译以及运行
Python 示例无需编译，直接运行即可：

```bash
# 运行 YOLOv5s v6 示例
python3 test_yolov5s_v6.py

# 运行 YOLOv5s v7 示例
python3 test_yolov5s_v7.py
```

### 执行效果
运行后，程序会加载相应版本的 YOLOv5s 模型，对 kite.jpg 图像进行目标检测，并生成带有检测框的结果图像 output_image.jpg
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

## 详细介绍

### 示例程序参数选项说明
YOLOv5s v6/v7 目标检测示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 kite.jpg 图像进行检测处理。

### 软件架构说明
YOLOv5s v6/v7 目标检测示例的软件架构包含以下几个核心部分：

模型加载：使用 pyeasy_dnn 模块加载预训练的 YOLOv5s 模型（ v6 或 v7 版本）

图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸（ 640x640 ）

模型推理：调用模型进行前向计算，生成特征图

后处理：使用 libpostprocess 库解析模型输出，生成检测结果

结果可视化：在原图上绘制检测框和类别信息

结果保存：将可视化结果保存为图像文件

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_software_arch.png)
</center>

### API 流程说明
模型加载：

v6: models = dnn.load('../models/yolov5s_v6_640x640_nv12.bin')

v7: models = dnn.load('../models/yolov5s_v7_640x640_nv12.bin')

图像预处理：调整图像尺寸并转换为 NV12 格式

模型推理： outputs = models[0].forward(nv12_data)

后处理配置：设置后处理参数（尺寸、阈值等）

结果解析：调用后处理库解析输出张量

结果可视化：在原图上绘制检测框和标签信息

结果保存：将结果保存为图像文件

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_12_yolov5s_v6_v7_sample_api_flow.png)
</center>

### YOLOv5s v6 与 v7 版本对比
主要改进
YOLOv5 v7 相比 v6 版本带来了以下主要改进：

1. 架构优化：改进了网络结构和训练策略

2. 精度提升：提高了目标检测的准确率

3. 速度优化：优化了推理速度，提高了效率

4. 训练改进：改进了数据增强和训练策略

5. 部署优化：更好地支持各种部署环境

### FAQ
Q: YOLOv5s v6 和 v7 有什么区别？\
A: v7 版本在架构、训练策略和推理速度上都有优化，通常具有更高的检测精度和更快的推理速度。

Q: 运行示例时出现 "No module named 'hobot_dnn'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 你的图片路径 ')。

Q: 如何选择使用 v6 还是 v7 版本？\
A: 可以根据具体应用需求进行测试， v7 版本通常性能更好，但也可以根据实际测试结果选择最适合的版本。

Q: 如何调整检测阈值？\
A: 在代码中修改 yolov5_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时目标检测。

Q: 如何进一步提高检测速度？\
A: 可以尝试使用更小的输入尺寸（如果模型支持），或者使用硬件加速特性。

Q: v7 版本相比 v6 有哪些具体改进？\
A: v7 版本改进了网络架构、训练策略和数据增强方法，提高了检测精度和推理速度，同时优化了模型部署的兼容性。

