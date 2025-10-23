---
sidebar_position: 6
---

# 3.2.6 centernet 示例介绍

## 示例简介
CenterNet 目标检测示例是一个位于 `/app/pydev_demo/11_centernet_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 CenterNet 模型进行高效的目标检测任务。 CenterNet 是一种基于中心点预测的目标检测算法，相比传统的锚框 -based 方法，具有更简洁的架构和更高的检测精度，特别适合需要精确定位和识别小目标的场景。

## 效果展示

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_runing.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/11_centernet_sample/` 位置，可以看到 CenterNet 示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/11_centernet_sample# tree
.
├── kite.jpg
└── test_centernet.py
```

### 编译以及运行
Python 示例无需编译，直接运行即可：

```
python3 test_centernet.py
```

### 执行效果
运行后，程序会加载预训练的 CenterNet 模型，对 kite.jpg 图像进行目标检测，并生成带有检测框的结果图像 output_image.jpg。
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

## 详细介绍

### 示例程序参数选项说明
CenterNet 目标检测示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 kite.jpg 图像进行检测处理。

### 软件架构说明
1. CenterNet 目标检测示例的软件架构包含以下几个核心部分：

2. 模型加载：使用 pyeasy_dnn 模块加载预训练的 CenterNet-ResNet101 模型

3. 图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸（ 512x512 ）

4. 模型推理：调用模型进行前向计算，生成热力图、尺寸和偏移量预测

5. 后处理：使用 libpostprocess 库解析模型输出，生成检测结果

6. 结果可视化：在原图上绘制检测框和类别信息

7. 结果保存：将可视化结果保存为图像文件

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_centernet_sample_software_arch.png)
</center>

### API 流程说明
1. 模型加载： models = dnn.load('../models/centernet_resnet101_512x512_nv12.bin')

2. 图像预处理：调整图像尺寸并转换为 NV12 格式

3. 模型推理： outputs = models[0].forward(nv12_data)

4. 后处理配置：设置后处理参数（尺寸、阈值等）

5. 结果解析：调用后处理库解析输出张量

6. 结果可视化：在原图上绘制检测框和标签信息

7. 结果保存：将结果保存为图像文件

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_11_centernet_sample_api_flow.png)
</center>

### FAQ
Q: CenterNet 和 YOLO 有什么区别？\
A: CenterNet 使用中心点预测而不是锚框，具有更简洁的架构和更精确的定位能力，特别适合小目标检测。

Q: 运行示例时出现 "No module named 'hobot_dnn'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 你的图片路径 ')。

Q: 检测结果不准确怎么办？\
A: CenterNet 模型是针对 COCO 数据集训练的，对于特定场景可能需要进行微调或使用更适合的模型。

Q: 如何调整检测阈值？\
A: 在代码中修改 centernet_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时目标检测。

Q: CenterNet 在处理小目标方面有什么优势？\
A: CenterNet 通过中心点预测和精细的偏移量调整，能够更精确地定位小目标，避免了锚框方法中锚框与小目标不匹配的问题。

Q: 如何进一步提高检测精度？\
A: 可以尝试使用更大的输入尺寸（如果模型支持），或者对特定场景进行模型微调，也可以调整后处理参数来优化检测结果。

