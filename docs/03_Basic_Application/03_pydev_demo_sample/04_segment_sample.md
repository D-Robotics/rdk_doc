---
sidebar_position: 2
---

# 3.2.2 segment 模型示例介绍

## 示例简介
图像分割示例是一个位于 `/app/pydev_demo/04_segment_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 MobileNet-UNet 模型进行语义分割任务。该示例展示了如何对输入图像进行像素级分类，将图像中的不同对象分割出来，并以可视化方式展示分割结果。

## 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_04_runing.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_04_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/04_segment_sample/` 位置，可以看到图像分割示例包含以下文件：
```
root@ubuntu:/app/pydev_demo/04_segment_sample# tree 
.
├── segmentation.png
└── test_mobilenet_unet.py
```

### 编译以及运行
Python 示例无需编译，直接运行即可：

```bash
python3 test_mobilenet_unet.py
```

### 执行效果

运行后，程序会加载预训练的 MobileNet-UNet 模型，对 segmentation.png 图像进行分割处理，并生成分割结果图像 segment_result.png。

```
root@ubuntu:/app/pydev_demo/04_segment_sample# ./test_mobilenet_unet.py 
Matplotlib is building the font cache; this may take a moment.
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-09-10,10:07:36.611.352) [HorizonRT] The model builder version = 1.23.8
[W][DNN]bpu_model_info.cpp:491][Version](2025-09-10,10:07:36.671.453) Model: unet_mobilenet_1024x2048_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
========== Model load successfully. ==========
========== Model forward finished. ==========
========== Postprocess successfully. ==========
========== Waiting for drawing image  ..........
Saving predicted image with name segment_result.png 
========== Dump result image segment_result.png successfully. ==========
root@ubuntu:/app/pydev_demo/04_segment_sample# 
```

## 详细介绍

### 示例程序参数选项说明
图像分割示例不需要命令行参数，直接运行即可。程序会自动加载同目录下的 segmentation.png 图像进行分割处理。

### 软件架构说明
图像分割示例的软件架构包含以下几个核心部分：

- 模型加载：使用 pyeasy_dnn 模块加载预训练的 MobileNet-UNet 模型

- 图像预处理：将输入图像转换为模型需要的 NV12 格式和指定尺寸

- 模型推理：调用模型进行前向计算，生成分割掩码

- 后处理：对模型输出进行 argmax 操作，获取每个像素的类别

- 结果可视化：使用颜色调色板将分割结果可视化，并叠加到原图上

- 结果保存：将可视化结果保存为图像文件

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_04_segment_sample_software_arch.png)
</center>

### API 流程说明
- 模型加载： models = pyeasy_dnn.load('../models/mobilenet_unet_1024x2048_nv12.bin')

- 图像预处理：调整图像尺寸并转换为 NV12 格式

- 模型推理： outputs = models[0].forward(nv12_data)

- 后处理： pred_result = np.argmax(model_output[0], axis=-1)

- 结果可视化：使用调色板将分割掩码转换为彩色图像

- 结果保存：将分割结果叠加到原图并保存

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_04_segment_sample_api_flow.png)
</center>

### FAQ
Q: 运行示例时出现 "ModuleNotFoundError: No module named 'matplotlib'" 错误怎么办？\
A: 请确保已安装 matplotlib 库，可以使用 pip3 install matplotlib 命令安装。

Q: 如何更换测试图片？\
A: 将新的图片文件放置在示例目录下，并在代码中修改 img_file = cv2.imread(' 你的图片路径 ')。

Q: 分割结果不准确怎么办？\
A: MobileNet-UNet 模型是针对城市街景场景训练的，对于其他场景可能效果不佳。可以尝试使用更适合特定场景的分割模型。

Q: 如何修改输出图像的分辨率？\
A: 模型输入分辨率是固定的 1024x2048 ，输出图像的分辨率会自动调整为原始图像的分辨率。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时分割处理。

Q: 如何获取其他预训练的分割模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

