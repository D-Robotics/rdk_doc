---
sidebar_position: 1
---

# 3.2.1 基础图像分类示例介绍

## 示例简介

基础图像分类示例是一组位于 `/app/pydev_demo/01_basic_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hobot_dnn` 模块进行图像分类任务。这些示例基于不同的神经网络模型实现相同的图像分类功能。

包含的模型示例：
- test_resnet18.py - 使用 ResNet18 模型进行图像分类
- test_efficientnasnet_m.py - 使用 EfficientNasNet 模型进行图像分类  
- test_googlenet.py - 使用 GoogleNet 模型进行图像分类
- test_mobilenetv1.py - 使用 MobileNetV1 模型进行图像分类
- test_vargconvnet.py - 使用 VargConvNet 模型进行图像分类

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_01_running.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/01_basic_sample/` 位置，可以看到基础示例包含了多个模型测试文件：

```
root@ubuntu:/app/pydev_demo/01_basic_sample# tree
.
├── imagenet1000_clsidx_to_labels.txt
├── test_efficientnasnet_m.py
├── test_googlenet.py
├── test_mobilenetv1.py
├── test_resnet18.py
├── test_vargconvnet.py
└── zebra_cls.jpg

```

### 编译以及运行
Python 示例无需编译，直接运行即可 .


### 执行效果


```
root@ubuntu:/app/pydev_demo/01_basic_sample# ./test_resnet18.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-09-09,19:46:43.279.404) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2025-09-09,19:46:43.415.165) Model: resnet18_224x224_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
========== inputs[0] properties ==========
tensor type: NV12
data type: uint8
layout: NCHW
shape: (1, 3, 224, 224)
inputs[0] name is: data
========== outputs[0] properties ==========
tensor type: float32
data type: float32
layout: NCHW
shape: (1, 1000, 1, 1)
outputs[0] name is: prob
postprocess time is : 0.0007224082946777344
cls id: 340, Confidence: 0.98893, class_name: zebra
root@ubuntu:/app/pydev_demo/01_basic_sample# 
```

## 详细介绍

### 示例程序参数选项说明
基础分类示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 zebra_cls.jpg 图像进行推理。

### 软件架构说明
本示例通过不同 python 代码示例展示了不同的模型执行效果，但软件架构上基本一致，所以这里统一说明，示例程序的软件架构包含以下几个核心部分：

1. 模型加载：使用 hobot_dnn.pyeasy_dnn 模块加载预编译的模型文件

2. 图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸

3. 模型推理：调用模型进行前向计算

4. 后处理：使用 libpostprocess 库解析推理结果

5. 结果输出：输出分类结果和置信度
<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_01_basic_software_arch.png)
</center>

### API 流程说明
<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_01_basic_api_flow.png)
</center>

### FAQ
Q： 运行示例时出现 "ModuleNotFoundError: No module named 'hobot_dnn'" 错误怎么办？\
A： 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q： 如何更换测试图片？\
A： 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 图片路径 ')。

Q： 不同的模型文件有什么区别？\
A： 不同的模型在准确率、推理速度和模型大小上有所区别， ResNet18 精度较高但速度稍慢， MobileNetV1 速度较快但精度稍低，可根据实际需求选择合适的模型。

Q： 如何获取其他预训练模型？\
A： 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

