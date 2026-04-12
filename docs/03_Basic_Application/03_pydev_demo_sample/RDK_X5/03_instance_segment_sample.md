---
sidebar_position: 3
---

# 语义分割

## 示例简介
语义分割示例是一组位于 `/app/pydev_demo/03_instance_segmentation_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行语义分割任务。这些示例基于不同的神经网络模型实现语义分割功能。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample$ tree -L 1
.
├── 01_unetmobilenet
└── 02_ultralytics_yolo11_seg
```

## 效果展示
两个示例均实现了语义分割的效果，但二者效果不同，这里给出两个模型的效果展示。
### unetmobilenet 效果展示

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_running_unetmobilenet.png)

### yolov11 效果展示
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_running_yolov11.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/03_instance_segmentation_sample/` 位置，可以看到包含了两个示例文件夹：

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample# tree
.
├── 01_unetmobilenet
│   ├── segmentation.png
│   ├── unet_mobilenet.py
│   └── result.jpg
└── 02_ultralytics_yolo11_seg
    ├── bus.jpg
    ├── coco_classes.names
    ├── result.jpg
    ├── ultralytics_yolo11_seg.py
    └── yolo11n_seg_bayese_640x640_nv12.bin
```

### 编译以及运行
Python 示例无需编译，直接运行即可：

```
cd /app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet
python unet_mobilenet.py 

cd /app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg
python ultralytics_yolo11_seg.py
```

### 执行效果

#### UNetMobileNet执行效果

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet# python unet_mobilenet.py 
[BPU_PLAT]BPU Platform Version(1.3.6)! soc info(x5)
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-02,02:54:24.731.966) [HorizonRT] The model builder version = 1.23.8

[W][DNN]bpu_model_info.cpp:491][Version](2000-01-02,02:54:24.831.251) Model: unet_mobilenet_1024x2048_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.54.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
=== Model Name List ===
['unet_mobilenet_1024x2048_nv12']

...

=== Scheduling Parameters ===
unet_mobilenet_1024x2048_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
[Saved] Result saved to: result.jpg
```

#### YOLOv11实例分割执行效果

```
root@ubuntu:/app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg# python ultralytics_yolo11_seg.py
[BPU_PLAT]BPU Platform Version(1.3.6)! soc info(x5)
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-02,02:51:30.798.145) [HorizonRT] The model builder version = 1.24.3
=== Model Name List ===
['yolo11n_seg_bayese_640x640_nv12']

=== Model Count ===
1

=== Input Counts ===
yolo11n_seg_bayese_640x640_nv12: 1

...

=== Scheduling Parameters ===
yolo11n_seg_bayese_640x640_nv12:
  priority    : 0
  customId    : 0
  bpu_cores   : [0]
  deviceId    : 0
[Saved] Result saved to: result.jpg

```

## 详细介绍

### 示例程序参数选项说明
语义分割示例支持命令行参数配置，两个示例的参数说明有所不同，各示例的默认模型路径也不同。如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

#### UNetMobileNet 参数说明

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/mobilenet_unet_1024x2048_nv12.bin` |
| `--test-img` | 输入测试图像路径 | str | `segmentation.png` |
| `--img-save-path` | 分割结果图像保存路径 | str | `result.jpg` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--alpha-f` | 分割掩码与原图混合的透明度因子（0.0=仅掩码，1.0=仅原图） | float | `0.75` |

#### YOLOv11 参数说明

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/yolo11n_seg_bayese_640x640_nv12.bin` |
| `--test-img` | 输入测试图像路径 | str | `bus.jpg` |
| `--label-file` | 类别标签路径（每行一个类别） | str | `coco_classes.names` |
| `--img-save-path` | 分割结果图像保存路径 | str | `result.jpg` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--nms-thres` | 非极大值抑制（NMS）阈值 | float | `0.7` |
| `--score-thres` | 置信度阈值 | float | `0.25` |
| `--is-open` | 是否对掩码进行形态学操作 | bool | `True` |
| `--is-point` | 是否绘制掩码边缘轮廓 | bool | `True` |

### 软件架构说明

本部分介绍语义分割示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向。由于两个示例的架构大致相同，这里给出一个软件架构图
<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_software_arch.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载预编译的模型文件
2. **图像加载** - 读取输入测试图像
3. **图像预处理** - 将输入图像缩放至模型输入尺寸，BGR 转 NV12 格式
4. **模型推理** - 在 BPU 上执行模型前向计算
5. **结果后处理** - 解析分割输出，将类别预测映射为颜色掩码，与原图混合
6. **结果输出** - 保存带有分割掩码叠加的结果图像

### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。由于两个示例采用的 API 接口有所不同，这里分别给出对应的 API 流程图。其中， `cv2.` 开头的是 OPENCV 的通用接口，`np.` 开头的是 Numpy 的通用接口，其余为地瓜自定义接口。

#### UNetMobileNet API 流程

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_unet_sample_api_flow.png)
</center>

<!-- <center>
<img src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_unet_sample_api_flow.png" width="1200" height="780" alt="API_Flow" />
</center> -->

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

6. `np.argmax(logits, axis=-1)` 

   获取最大值索引，输入：模型输出、指定维度，返回：预测类别

7. `cv2.resize(pred_class, (input_W, input_H), interpolation=cv2.INTER_NEAREST)`
   
   缩放预测类别至原图尺寸，输入：预测类别、（模型输入的宽、模型输入的高）、插值类型

8. `recover_to_original_size(seg_mask, img_w, img_h, resize_type)`

   将分割掩码恢复到原图尺寸，输入：分割掩码、原图宽、原图高、缩放类型

9. `cv2.addWeighted(origin_image, alpha, parsing_img, 1-alpha, 0.0)`

   混合原图与分割掩码，输入：原图、原图权重 、分割掩码彩色图、分割掩码权重、亮度偏移量

#### YOLOv11实例分割 API 流程

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_instance_seg_yolo11_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载预编译模型，输入：模型文件路径

2. `load_image(image_path)`

   加载测试图像，输入：图像文件路径

3. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型, 返回:缩放后的图像

4. `bgr_to_nv12_planes(image)`

   BGR 转 NV12 格式，输入：BGR 图像,返回: y 平面, uv 平面

5. `model.run(input_data)`

   执行模型推理，输入：预处理后的图像数据,返回：模型输出

6. `dequantize_outputs(outputs, output_quants)`

   结果反量化，输入：模型输出，指定类型，返回：反量化后的结果

7. `filter_classification(cls_output, conf_thres)`

   过滤分类结果，输入：分类输出、置信度阈值, 返回：置信度、类别ID、有效索引

8. `decode_boxes(box_output, valid_indices, anchor_size, stride, weights)`

   解码检测框，输入：框回归输出、有效索引、锚框大小、步长、权重, 返回：检测框

9. `filter_mces(mces_output, valid_indices)`

   过滤掩码系数，输入：掩码系数输出、有效索引, 返回：掩码系数

10. `NMS(dbboxes, scores, ids, nms_thresh)`

    非极大值抑制，输入：检测框、置信度、类别、IoU 阈值, 返回：保留的索引

11. `decode_masks(mces, boxes, protos, input_W, input_H, Mask_W, Mask_H, mask_thresh)`

    解码分割掩码，输入：掩码系数、检测框、掩码原型、输入宽、输入高、 掩码宽、掩码宽、阈值, 返回：掩码

12. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    调整检测框至原图尺寸，输入：检测框、原图宽、 原图高、 模型输入宽、 模型输入高、缩放类型, 返回：缩放后的检测框

13. `resize_masks_to_boxes(masks, boxes, img_w, img_h, do_morph)`

    将掩码调整到检测框区域并缩放到原图尺寸，输入：掩码、检测框、原图宽、原图高、是否形态学操作, 返回：调整后的掩码

14. `draw_boxes(image, boxes, cls_ids, scores, class_names, rdk_colors)`

    绘制检测框，输入：图像、检测框、类别ID、置信度、类别列表、颜色映射

15. `draw_masks(image, boxes, masks, cls_ids, rdk_colors, alpha)`

    绘制分割掩码，输入：图像、检测框、掩码、类别ID、颜色映射、透明度

16. `draw_contours(image, boxes, masks, cls_ids, rdk_colors, thickness)`

    绘制掩码边缘轮廓，输入：图像、检测框、掩码、类别ID、颜色映射、线宽
17. `cv2.imwrite(image_path, image)`

    保存结果图像，输入：保存路径、图像

### FAQ

Q: 运行示例时出现 "ModuleNotFoundError: No module named 'hbm_runtime'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境， hbm_runtime 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python unet_mobilenet.py --test-img your_image.jpg`

Q: UNetMobileNet 和 YOLOv11 实例分割有什么区别？\
A: UNetMobileNet 是语义分割模型，对图像进行像素级分类，不区分同一类别的不同实例；YOLOv11 是实例分割模型，可以同时检测目标并分割出每个实例的精确轮廓。

Q: 如何调整分割结果的透明度？\
A: 对于 UNetMobileNet，可以通过 `--alpha-f` 参数调整（0.0=仅掩码，1.0=仅原图）；对于 YOLOv11，可以通过修改代码中 `draw_masks` 函数的 `alpha` 参数。

Q: 检测结果不准确怎么办？\
A: 可以尝试调整 `--score-thres` 和 `--nms-thres` 参数，或者使用更适合特定场景的模型。YOLOv11 模型是针对 COCO 数据集训练的，对于特定场景可能需要进行微调。

Q: 如何获取其他预训练的分割模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时实例分割。
