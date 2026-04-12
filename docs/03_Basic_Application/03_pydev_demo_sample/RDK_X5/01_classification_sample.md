---
sidebar_position: 1
---

# 图像分类

## 示例简介

图像分类示例是一组位于 `/app/pydev_demo/01_classification_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行图像分类任务。这些示例基于不同的神经网络模型实现相同的图像分类功能。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/01_classification_sample$ tree -L 1
.
├── 01_resnet18
└── 02_mobilenetv2
```

## 效果展示
两个示例效果一致只是调用的模型不一致,这里给出 resnet18 模型的效果展示, MobileNetV2 的效果类似。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_running.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/01_classification_sample/` 位置，可以看到包含了两个示例文件夹：

```
root@ubuntu:/app/pydev_demo/01_classification_sample# tree
.
├── 01_resnet18
│   ├── imagenet1000_clsidx_to_labels.txt
│   ├── resnet18.py
│   └── zebra_cls.jpg
└── 02_mobilenetv2
    ├── imagenet1000_clsidx_to_labels.txt
    ├── mobilenetv2.py
    └── zebra_cls.jpg
```

### 编译以及运行
Python 示例无需编译，进入到示例目录直接运行即可 .

```
cd /app/pydev_demo/01_classification_sample/01_resnet18
python resnet18.py 

cd /app/pydev_demo/01_classification_sample/02_mobilenetv2
python mobilenetv2.py
```

### 执行效果

#### resnet18执行效果

```
root@ubuntu:/app/pydev_demo/01_classification_sample/01_resnet18# python resnet18.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-20,18:14:57.93.241) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-20,18:14:57.223.307) Model: resnet18_224x224_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Top-5 Predictions:
zebra: 0.9853
tiger, Panthera tigris: 0.0009
prairie chicken, prairie grouse, prairie fowl: 0.0008
gazelle: 0.0006
warthog: 0.0004
root@ubuntu:/app/pydev_demo/01_classification_sample/01_resnet18# 
```

#### mobilenetv2执行效果

```
root@ubuntu:/app/pydev_demo/01_classification_sample/02_mobilenetv2# python mobilenetv2.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-21,17:24:56.88.461) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-21,17:24:56.172.366) Model: mobilenetv2_224x224_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Top-5 Predictions:
zebra: 0.9936
tiger, Panthera tigris: 0.0039
hartebeest: 0.0007
tiger cat: 0.0007
impala, Aepyceros melampus: 0.0003
```

## 详细介绍

### 示例程序参数选项说明
分类示例支持命令行参数配置，两个示例的参数说明基本一致，但各示例的默认模型路径不同。如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | 各示例不同，见下表"各示例默认模型路径" |
| `--test-img` | 输入测试图像路径 | str | `zebra_cls.jpg` |
| `--label-file` | 类别标签映射文件路径（dict 格式） | str | `imagenet1000_clsidx_to_labels.txt` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |

**各示例默认模型路径：**

| 示例 | 默认模型路径 |
|------|-------------|
| ResNet18 | `/opt/hobot/model/x5/basic/resnet18_224x224_nv12.bin` |
| MobileNetV2 | `/opt/hobot/model/x5/basic/mobilenetv2_224x224_nv12.bin` |

### 软件架构说明

本部分介绍基础图像分类示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_basic_software_arch1.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载预编译的模型文件
2. **图像加载** - 读取输入测试图像
3. **图像预处理** - 将输入图像缩放至模型输入尺寸，BGR 转 NV12 格式
4. **模型推理** - 在 BPU 上执行模型前向计算
5. **结果后处理** - 使用 post_utils 库解析推理结果，获取分类概率
6. **结果输出** - 输出 Top-5 分类结果及对应置信度


### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_basic_api_flow1.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`
    
    加载预编译模型，输入：模型文件路径

2. `load_image(image_path)` 

    加载测试图像，输入：图像文件路径

3. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型

4. `bgr_to_nv12_planes(image)` 

    BGR 转 NV12 格式，输入：BGR 图像, 返回: y 平面, uv 平面

5. `model.run(input_data)`

    执行模型推理，输入：预处理后的图像数据

6. `print_topk_predictions(outputs, idx2label)` 

    后处理并输出 Top-K 结果，输入：模型输出 tensor、类别名称列表


### FAQ
Q： 运行示例时出现 "ModuleNotFoundError: No module named 'hbm_runtime'" 错误怎么办？\
A： 请确保已正确安装 RDK 的 Python 环境， hbm_runtime 模块等官方提供的专用推理库。

Q： 如何更换测试图片？\
A： 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python resnet18.py --test-img your_image.jpg`

Q： 不同的模型文件有什么区别？\
A： 不同的模型在准确率、推理速度和模型大小上有所区别， ResNet18 精度较高但速度稍慢， MobileNetV2 速度较快但精度稍低，可根据实际需求选择合适的模型。

Q： 如何获取其他预训练模型？\
A： 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

