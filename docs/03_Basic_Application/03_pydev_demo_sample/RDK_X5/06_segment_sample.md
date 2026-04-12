---
sidebar_position: 6
---

# 分割示例介绍

## 示例简介
分割示例是一组位于 `/app/pydev_demo/06_segment_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行分割任务。该示例使用 MobileNet-UNet 模型实现人、车辆、路面、路标等类别的分割。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/06_segment_sample$ tree -L 1
.
└── 01_mobilenet_unet
```

## 效果展示
示例会对图像进行像素级别的分类，使用不同颜色标识不同类别（人、车辆、路面、路标等），并将分割结果叠加在原图上。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_running_seg.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/06_segment_sample/` 位置，可以看到包含了语义分割示例文件夹：

```
root@ubuntu:/app/pydev_demo/06_segment_sample# tree
.
└── 01_mobilenet_unet
    ├── mobilenet_unet.py
    ├── segmentation.png
    └── segmentation_result.png
```

### 编译以及运行
Python 示例无需编译，直接运行即可。

```
cd /app/pydev_demo/06_segment_sample/01_mobilenet_unet
python mobilenet_unet.py
```

### 执行效果

```
root@ubuntu:/app/pydev_demo/06_segment_sample/01_mobilenet_unet# python mobilenet_unet.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,11:51:43.693.476) [HorizonRT] The model builder version = 1.23.8
[W][DNN]bpu_model_info.cpp:491][Version](2000-01-01,11:51:43.813.309) Model: unet_mobilenet_1024x2048_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...


=== Output Tensor Stride ===
unet_mobilenet_1024x2048_nv12:
  seg_head_seg_pred_4/FusedBatchNorm:0 -> stride: [9961472, 38912, 76, 4]

=== Scheduling Parameters ===
unet_mobilenet_1024x2048_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
root@ubuntu:/app/pydev_demo/06_segment_sample/01_mobilenet_unet#
```
可视化结果为同级目录下名为 segmentation_result.png 的图片。

## 详细介绍

### 示例程序参数选项说明
语义分割示例支持命令行参数配置，如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/mobilenet_unet_1024x2048_nv12.bin` |
| `--test-img` | 输入测试图像路径 | str | `segmentation.png` |
| `--save-path` | 分割结果图像保存路径 | str | `segmentation_result.png` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |

### 软件架构说明

本部分介绍语义分割示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_seg_sample_software_arch.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载模型文件
2. **图像加载** - 读取输入测试图像
3. **图像预处理** - 将输入图像缩放至模型输入尺寸，BGR 转 NV12 格式
4. **模型推理** - 在 BPU 上执行模型前向计算
5. **结果后处理** - 对模型输出进行 argmax 操作，获取每个像素的类别预测
6. **结果输出** - 将分割结果绘制在原图上，使用不同颜色标识不同类别，并保存结果图像

### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。其中，`np.` 开头的是 Numpy 库的通用接口，其余的为地瓜自定义的接口。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_seg_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载预编译模型，输入：模型文件路径

2. `load_image(image_path)`

   加载测试图像，输入：图像文件路径

3. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型，返回:缩放后的图像

4. `bgr_to_nv12_planes(image)`

   BGR 转 NV12 格式，输入：BGR 图像，返回: y 平面, uv 平面

5. `model.run(input_data)`

   执行模型推理，输入：预处理后的图像数据,返回：模型输出

6. `np.argmax(outputs, axis)`

   获取最大值索引，输入：模型输出、指定维度，返回：每个像素的类别索引

7. `draw_seg_result(image, pred_result, save_path)`

   绘制分割结果，输入：原始图像、预测结果、保存路径

### FAQ

Q: 运行示例时出现 "ModuleNotFoundError: No module named 'hbm_runtime'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， `hbm_runtime` 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python mobilenet_unet.py --test-img your_image.jpg`

<!-- Q: 分割结果不准确怎么办？\
A: 确保输入图像质量良好，图像尺寸符合模型要求。MobileNet-UNet 模型输入尺寸为 1024x2048，程序会自动缩放输入图像。 -->

Q: 如何修改结果保存路径？\
A: 使用 `--save-path` 参数指定保存路径，例如：`--save-path /path/to/result.png`

Q: 模型文件不存在怎么办？\
A: 若指定模型路径不存在，请在 `/app/model/basic` 目录下查找模型文件，或根据提示下载对应的模型。

<!-- Q: 可以处理其他尺寸的图像吗？\
A: 可以，程序会自动将输入图像缩放至模型输入尺寸（1024x2048），但建议使用接近该尺寸的图像以获得更好的效果。 -->

Q: 如何查看分割的类别？\
A: 分割结果使用不同颜色标识不同类别（人、车辆、路面、路标等），可以在保存的结果图像中查看。

Q: 如何获取其他预训练的语义分割模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)
