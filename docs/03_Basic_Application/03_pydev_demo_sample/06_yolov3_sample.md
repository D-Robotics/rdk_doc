---
sidebar_position: 3
---

# 3.3.3 yolov3  模型示例介绍

## 示例简介
YOLOv3 目标检测示例是一个位于 `/app/pydev_demo/06_yolov3_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 YOLOv3 模型进行目标检测任务。该示例展示了如何对静态图像进行目标检测，识别图像中的多种对象，并在图像上绘制检测框和置信度信息。

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_06_runing.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_06_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/06_yolov3_sample/` 位置，可以看到 YOLOv3 示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/06_yolov3_sample# tree
.
├── coco_classes.names
├── kite.jpg
└── test_yolov3.py

```

### 编译以及运行
Python 示例无需编译，直接运行即可：

### 执行效果
运行后，程序会加载预训练的 YOLOv3 模型，对 kite.jpg 图像进行目标检测，并生成带有检测框的结果图像 output_image.jpg。

```
root@ubuntu:/app/pydev_demo/06_yolov3_sample# ./test_yolov3.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-09-10,10:35:11.677.325) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2025-09-10,10:35:12.46.71) Model: yolov3_vargdarknet_416x416_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 416, 416)
3
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 13, 13, 255)
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 26, 26, 255)
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 52, 52, 255)
inferece time is : 0.020566940307617188
channel_valid: 255
channel_aligned: 256
channel_valid: 255
channel_aligned: 256
channel_valid: 255
channel_aligned: 256
postprocess time is : 0.01925802230834961
bbox: [112.40535, 612.55957, 167.20108, 763.162781], score: 0.988552, id: 0, name: person
bbox: [590.504944, 80.579185, 668.850891, 152.585663], score: 0.979709, id: 33, name: kite
bbox: [215.867859, 697.843567, 271.909821, 853.475281], score: 0.976957, id: 0, name: person
bbox: [347.447327, 486.957092, 355.643707, 505.142273], score: 0.971156, id: 0, name: person
bbox: [576.44989, 345.108185, 599.307068, 369.19931], score: 0.963799, id: 33, name: kite
bbox: [278.754852, 236.495605, 305.968567, 280.169434], score: 0.939796, id: 33, name: kite
bbox: [468.2388, 339.285095, 485.910553, 358.016907], score: 0.930308, id: 33, name: kite
bbox: [178.86084, 540.466187, 192.142792, 572.972656], score: 0.896047, id: 0, name: person
bbox: [304.221893, 375.879303, 326.426636, 399.683228], score: 0.889891, id: 33, name: kite
bbox: [541.845886, 516.654236, 554.71283, 535.644592], score: 0.849118, id: 0, name: person
bbox: [28.978374, 525.916199, 41.111099, 555.264893], score: 0.845627, id: 0, name: person
bbox: [523.410095, 505.26712, 533.049866, 526.813965], score: 0.82792, id: 0, name: person
bbox: [762.491577, 381.598389, 774.152283, 390.471924], score: 0.769005, id: 33, name: kite
bbox: [528.595642, 516.481018, 540.382019, 531.763367], score: 0.690236, id: 0, name: person
bbox: [34.753479, 512.438354, 51.458652, 552.005005], score: 0.651956, id: 0, name: person
bbox: [1082.991089, 395.685669, 1099.050781, 423.359985], score: 0.617869, id: 33, name: kite
bbox: [84.095276, 514.01886, 103.595062, 566.563904], score: 0.605346, id: 0, name: person
bbox: [1206.786499, 451.33432, 1215.150146, 462.050873], score: 0.363324, id: 0, name: person
draw result time is : 0.0428011417388916
root@ubuntu:/app/pydev_demo/06_yolov3_sample# 
```

## 详细介绍

### 示例程序参数选项说明
YOLOv3 目标检测示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 kite.jpg 图像进行检测处理。

### 软件架构说明
YOLOv3 目标检测示例的软件架构包含以下几个核心部分：

1. 模型加载：使用 pyeasy_dnn 模块加载预训练的 YOLOv3 模型

2. 图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸（ 416x416 ）

3. 模型推理：调用模型进行前向计算，生成特征图

4. 后处理：使用 libpostprocess 库解析模型输出，生成检测结果

5. 结果可视化：在原图上绘制检测框和类别信息

6. 结果保存：将可视化结果保存为图像文件

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_06_yolov3_sample_software_arch.png)
</center>

### API 流程说明
1. 模型加载 :models = dnn.load('../models/yolov3_416x416_nv12.bin')
2. 图像预处理 : 调整图像尺寸并转换为 NV12 格式

3. 模型推理 : outputs = models[0].forward(nv12_data)

4. 后处理配置 : 设置后处理参数 ( 尺寸、阈值等 )

5. 结果解析 : 调用后处理库解析输出张量

6. 结果可视化 : 在原图上绘制检测框和标签信息

7. 结果保存 : 将结果保存为图像文件

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_06_yolov3_sample_api_flow.png)
</center>

### FAQ

Q: 运行示例时出现 "No module named 'hobot_dnn'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 你的图片路径 ')。

Q: 检测结果不准确怎么办？\
A: YOLOv3 模型是针对 COCO 数据集训练的，对于特定场景可能需要进行微调或使用更适合的模型。

Q: 如何调整检测阈值？\
A: 在代码中修改 yolov3_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时目标检测。

Q: 如何获取其他预训练的 YOLO 模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

