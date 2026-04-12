---
sidebar_position: 2
---

# 目标检测

## 示例简介
目标检测示例是一组位于 `/app/pydev_demo/02_detection_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行目标检测任务。这些示例基于不同的 YOLO 系列模型实现相同的目标检测功能。
包含的模型示例：

```
root@ubuntu:/app/pydev_demo/02_detection_sample$ tree -L 1
.
├── 01_ultralytics_yolov5x
├── 02_ultralytics_yolo11
├── 03_ultralytics_yolov8
└── 04_ultralytics_yolo10
```

## 效果展示
四个示例效果一致只是调用的模型不一致,这里给出 YOLOv5X 模型的效果展示,其他 YOLO 系列模型的效果类似。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_running_yolov5x.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。
![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_06_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入到 `/app/pydev_demo/02_detection_sample/` 位置，可以看到包含了四个示例文件夹：

```
root@ubuntu:/app/pydev_demo/02_detection_sample# tree
.
├── 01_ultralytics_yolov5x
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolov5x.py
│   └── result.jpg
├── 02_ultralytics_yolo11
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolo11.py
│   └── result.jpg
├── 03_ultralytics_yolov8
│   ├── coco_classes.names
│   ├── kite.jpg
│   ├── ultralytics_yolov8.py
│   └── result.jpg
└── 04_ultralytics_yolo10
    ├── coco_classes.names
    ├── kite.jpg
    ├── ultralytics_yolo10.py
    └── result.jpg
```

### 编译以及运行
Python 示例无需编译，直接运行即可：
```
cd /app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x
python ultralytics_yolov5x.py 

cd /app/pydev_demo/02_detection_sample/02_ultralytics_yolo11
python ultralytics_yolo11.py

cd /app/pydev_demo/02_detection_sample/03_ultralytics_yolov8
python ultralytics_yolov8.py

cd /app/pydev_demo/02_detection_sample/04_ultralytics_yolo10
python ultralytics_yolo10.py
```

### 执行效果
<!-- 运行后，程序会加载预训练的 YOLOv3 模型，对 kite.jpg 图像进行目标检测，并生成带有检测框的结果图像 output_image.jpg。 -->

#### YOLOv5X执行效果

```
root@ubuntu:/app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x# python ultralytics_yolov5x.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:07:20.971.89) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-26,17:07:21.81.905) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

检测到 18 个目标: person(0.86), person(0.79), person(0.67), person(0.65), person(0.55), person(0.54), person(0.51), person(0.45), person(0.44), person(0.34), person(0.33), kite(0.88), kite(0.84), kite(0.71), kite(0.70), kite(0.57), kite(0.44), kite(0.31)
[Saved] Result saved to: result.jpg
```

#### YOLOv11执行效果

```
root@ubuntu:/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11# python ultralytics_yolo11.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:10:52.565.485) [HorizonRT] The model builder version = 1.24.3

...

[Saved] Result saved to: result.jpg
```

#### YOLOv8执行效果
```
root@ubuntu:/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8# python ultralytics_yolov8.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:12:13.322.476) [HorizonRT] The model builder version = 1.24.3

...

检测到 13 个目标: person(0.88), person(0.87), person(0.72), person(0.50), person(0.45), person(0.34), person(0.30), kite(0.90), kite(0.84), kite(0.76), kite(0.71), kite(0.62), kite(0.61)
[Saved] Result saved to: result.jpg
```


#### YOLOv10执行效果
```
root@ubuntu:~/app/pydev_demo/02_detection_sample/04_ultralytics_yolo10# python ultralytics_yolo10.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-26,17:13:38.644.334) [HorizonRT] The model builder version = 1.24.3

...

检测到 14 个目标: kite(0.91), kite(0.81), kite(0.51), kite(0.39), kite(0.78), kite(0.41), kite(0.80), person(0.27), person(0.58), person(0.60), person(0.76), person(0.26), person(0.91), person(0.90)
[Saved] Result saved to: result.jpg
```

## 详细介绍

### 示例程序参数选项说明
目标检测示例支持命令行参数配置，四个示例的参数说明基本一致，但各示例的默认模型路径不同。如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | 各模型不同，见下表 |
| `--test-img` | 输入测试图像路径 | str | `kite.jpg` |
| `--label-file` | 类别标签路径（每行一个类别） | str | `coco_classes.names` |
| `--img-save-path` | 检测结果图像保存路径 | str | `result.jpg` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--nms-thres` | 非极大值抑制（NMS）阈值 | float | `0.45` |
| `--score-thres` | 置信度阈值 | float | `0.25` |

**各示例默认模型路径：**

| 模型 | 默认模型路径 |
|------|-------------|
| YOLOv5X | `/opt/hobot/model/x5/basic/yolov5x_672x672_nv12.bin` |
| YOLOv11 | `/opt/hobot/model/x5/basic/yolo11n_detect_bayese_640x640_nv12.bin` |
| YOLOv8  | `/opt/hobot/model/x5/basic/yolov8x_detect_bayese_640x640_nv12.bin` |
| YOLOv10 | `/opt/hobot/model/x5/basic/yolov10x_detect_bayese_640x640_nv12.bin` |

### 软件架构说明

本部分介绍目标检测示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向,由于四个示例的架构相同，这里只给出一个软件架构图。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_detect_sample_software_arch.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载预编译的模型文件
2. **图像加载** - 读取输入测试图像
3. **图像预处理** - 将输入图像缩放至模型输入尺寸，BGR 转 NV12 格式
4. **模型推理** - 在 BPU 上执行模型前向计算
5. **结果后处理** - 使用 post_utils 库解析推理结果，进行解码、置信度过滤和 NMS 非极大值抑制
6. **结果输出** - 在图像上绘制检测框并保存结果图像



### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。由于四个示例的调用接口基本一致，这里只给出一个 API 流程说明图

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_02_detect_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载预编译模型，输入：模型文件路径

2. `load_image(image_path)`

   加载测试图像，输入：图像文件路径

3. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型,返回:缩放后的图像

4. `bgr_to_nv12_planes(image)`

   BGR 转 NV12 格式，输入：BGR 图像,返回: y 平面, uv 平面

5. `model.run(input_data)`

   执行模型推理，输入：预处理后的图像数据,返回：模型输出

6. `dequantize_outputs(outputs, output_quants)`

   结果反量化，输入：模型输出，指定类型，返回：反量化后的结果

7. `decode_outputs(output_names, fp32_outputs, STRIDES, ANCHORS, classes_num)`

   解码模型输出，输入：输出名称列表、模型输出、步长、锚框、类别数量, 返回：预测结果

8. `filter_predictions(pred, score_thres)`

   过滤预测结果，输入：预测结果、置信度阈值, 返回：检测框、置信度、类别

9. `NMS(detections, iou_threshold)`

   非极大值抑制，输入：检测框、置信度、类别、IoU 阈值, 返回：保留的索引

10. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    调整检测框至原图尺寸，输入：检测框、原图尺寸、模型输入尺寸、插值方法, 返回：缩放后的检测框

11. `draw_boxes(image, detections, class_names)`

    绘制检测结果，输入：图像、检测框、类别ID、置信度、类别列表、颜色映射


### FAQ

Q: 运行示例时出现 "No module named 'hobot_dnn'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hobot_dnn 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python ultralytics_yolov5x.py --test-img your_image.jpg`

<!-- Q: 检测结果不准确怎么办？\
A: YOLOv3 模型是针对 COCO 数据集训练的，对于特定场景可能需要进行微调或使用更适合的模型。 -->

Q: 如何调整检测阈值？\
A: 在代码中修改 `--score-thres` 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时目标检测。

Q: 如何获取其他预训练的 YOLO 模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

