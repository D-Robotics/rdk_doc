---
sidebar_position: 5
---

# 3.2.5 yolov5x 模型示例介绍

## 示例简介
YOLOv5X 目标检测示例是一个位于 `/app/pydev_demo/09_yolov5x_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 YOLOv5X 模型进行高精度目标检测任务。 YOLOv5X 是 YOLOv5 系列中最大、最精确的模型变体，相比 YOLOv5s 具有更高的检测精度，适用于对检测准确性要求更高的应用场景。

## 效果展示

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_09_runing.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_09_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/09_yolov5x_sample/` 位置，可以看到 YOLOv5X 示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/09_yolov5x_sample# tree
.
├── coco_classes.names
├── kite.jpg
└── test_yolov5x.py
```

### 编译以及运行
Python 示例无需编译，直接运行即可：
```
python3 test_yolov5x.py
```

### 执行效果
运行后，程序会加载预训练的 YOLOv5X 模型，对 kite.jpg 图像进行目标检测，并生成带有检测框的结果图像 output_image.jpg。
```
root@ubuntu:/app/pydev_demo/09_yolov5x_sample# ./test_yolov5x.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,08:32:56.277.320) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,08:32:56.956.224) Model: yolov5x_672x672_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 672, 672)
3
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 84, 84, 255)
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 42, 42, 255)
tensor type: int32
data type: int32
layout: NHWC
shape: (1, 21, 21, 255)
inferece time is : 0.10300564765930176
postprocess time is : 0.060691237449645996
draw result time is : 0.048194289207458496
```

## 详细介绍

### 示例程序参数选项说明
YOLOv5X 目标检测示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 kite.jpg 图像进行检测处理。

### 软件架构说明
YOLOv5X 目标检测示例的软件架构包含以下几个核心部分：

1. 模型加载：使用 pyeasy_dnn 模块加载预训练的 YOLOv5X 模型

2. 图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸（ 672x672 ）

3. 模型推理：调用模型进行前向计算，生成特征图

4. 后处理：使用 libpostprocess 库解析模型输出，生成检测结果

5. 结果可视化：在原图上绘制检测框和类别信息

6. 结果保存：将可视化结果保存为图像文件

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_09_yolov5x_sample_software_arch.png)
</center>

### API 流程说明
模型加载： models = dnn.load('../models/yolov5x_672x672_nv12.bin')

图像预处理：调整图像尺寸并转换为 NV12 格式

模型推理： outputs = models[0].forward(nv12_data)

后处理配置：设置后处理参数（尺寸、阈值等）

结果解析：调用后处理库解析输出张量

结果可视化：在原图上绘制检测框和标签信息

结果保存：将结果保存为图像文件

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_09_yolov5x_sample_api_flow.png)
</center>

### FAQ

Q: YOLOv5X 和 YOLOv5s 有什么区别？\
A: YOLOv5X 是 YOLOv5 系列中最大、最精确的模型变体，参数量是 YOLOv5s 的约 4 倍，检测精度更高，但推理速度较慢。

Q: 运行示例时出现 "No module named 'hobot_dnn'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 你的图片路径 ')。

Q: 检测结果不准确怎么办？\
A: YOLOv5X 模型是针对 COCO 数据集训练的，对于特定场景可能需要进行微调或使用更适合的模型。

Q: 如何调整检测阈值？\
A: 在代码中修改 yolov5_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时目标检测。由于 YOLOv5X 模型较大，实时处理可能需要降低帧率或分辨率。

Q: 如何选择合适的 YOLOv5 模型变体？\
A: 根据应用需求选择：如果需要最高精度，选择 YOLOv5X；如果需要平衡精度和速度，选择 YOLOv5s 或 YOLOv5m；如果资源受限，选择 YOLOv5n。

Q: 如何处理不同尺寸的输入图像？\
A: YOLOv5X 模型需要固定尺寸的输入（ 672x672 ），程序会自动将输入图像调整到该尺寸。

Q: 如何进一步提高检测精度？\
A: 可以尝试使用更大的输入尺寸（如果模型支持），或者对特定场景进行模型微调。


