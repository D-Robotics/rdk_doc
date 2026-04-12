---
sidebar_position: 4
---

# 姿态估计

## 示例简介
姿态估计示例是一组位于 `/app/pydev_demo/04_pose_sample/` 中的 **Python 接口** 开发代码示例，用于演示如何使用 `hbm_runtime` 模块进行人体姿态估计任务。示例基于 YOLO11 模型实现人体关键点检测功能。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/04_pose_sample$ tree -L 1
.
└── 01_ultralytics_yolo11_pose
```

## 效果展示
示例会检测图像中的人体，并标注出人体关键点（如头部、肩膀、手肘、手腕、膝盖、脚踝等），同时绘制检测框和置信度。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_running_yolov11_pose.png)

## 硬件准备

### 硬件连接
该示例只需要 RDK 开发板本身，无需额外的外设连接。确保开发板正常供电并启动系统。

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_01_hw_connect.png)

## 快速开始

### 代码以及板端位置

进入到 `/app/pydev_demo/04_pose_sample/` 位置，可以看到包含了姿态估计示例文件夹：

```
root@ubuntu:/app/pydev_demo/04_pose_sample# tree
.
└── 01_ultralytics_yolo11_pose
    ├── bus.jpg
    ├── coco_classes.names
    ├── result.jpg
    ├── ultralytics_yolo11_pose.py
    └── yolo11n_pose_bayese_640x640_nv12.bin
```

### 编译以及运行
Python 示例无需编译，直接运行即可。

```
cd /app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose
python ultralytics_yolo11_pose.py 
```

### 执行效果

```
root@ubuntu:/app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose# python ultralytics_yolo11_pose.py 
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2000-01-01,10:16:20.686.495) [HorizonRT] The model builder version = 1.24.3
...
[Saved] Result saved to: result.jpg
```

## 详细介绍

### 示例程序参数选项说明
姿态估计示例支持命令行参数配置，如果不指定参数，程序会使用默认值自动加载同目录下的测试图像进行推理。

| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/yolo11n_pose_bayese_640x640_nv12.bin` |
| `--test-img` | 输入测试图像路径 | str | `bus.jpg` |
| `--label-file` | 类别标签路径（每行一个类别） | str | `coco_classes.names` |
| `--img-save-path` | 检测结果图像保存路径 | str | `result.jpg` |
| `--priority` | 推理优先级（0~255，255为最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--nms-thres` | 非极大值抑制（NMS）阈值 | float | `0.7` |
| `--score-thres` | 置信度阈值 | float | `0.25` |
| `--kpt-conf-thres` | 关键点显示置信度阈值 | float | `0.5` |

### 软件架构说明

本部分介绍姿态估计示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_pose_sample_software_arch.png)
</center>

1. **模型加载** - 使用 `hbm_runtime` 模块加载预编译的模型文件
2. **图像加载** - 读取输入测试图像
3. **图像预处理** - 将输入图像缩放至模型输入尺寸，BGR 转 NV12 格式
4. **模型推理** - 在 BPU 上执行模型前向计算
5. **结果后处理** - 使用 post_utils 库解析推理结果，进行解码、置信度过滤和 NMS 非极大值抑制，提取检测框和关键点坐标
6. **结果输出** - 在图像上绘制检测框、关键点并保存结果图像


### API 流程说明

本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。其中， `cv2.` 开头的是 OPENCV 的通用接口，其余为 hbm_runtime 及其相关接口。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_04_pose_yolo11_sample_api_flow.png)
</center>

1. `hbm_runtime.HB_HBMRuntime(model_path)`

   加载预编译模型，输入：模型文件路径

2. `load_image(image_path)`

   加载测试图像，输入：图像文件路径

3. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型, 返回:缩放后的图像

4. `bgr_to_nv12_planes(image)`

   BGR 转 NV12 格式，输入：BGR 图像, 返回: y 平面, uv 平面

5. `model.run(input_data)`

   执行模型推理，输入：预处理后的图像数据, 返回：模型输出

6. `dequantize_outputs(outputs, output_quants)`

   结果反量化，输入：模型输出，指定类型，返回：反量化后的结果

7. `filter_classification(cls_output, conf_thres)`

   过滤预测结果，输入：分类输出、置信度阈值, 返回：置信度、类别ID、有效索引

8. `decode_boxes(box_output, valid_indices, anchor_size, stride, weights)`

   解码检测框，输入：框输出、有效索引、锚框大小、步长、权重, 返回：检测框

9. `decode_kpts(kpts_output, valid_indices, anchor_size, stride)`

   解码关键点，输入：关键点输出、有效索引、锚框大小、步长, 返回：关键点坐标和置信度

10. `NMS(dbboxes, scores, ids, nms_thresh)`

    非极大值抑制，输入：检测框、置信度、类别、IoU 阈值, 返回：保留的索引

11. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    调整检测框至原图尺寸，输入：检测框、原图宽、原图高、模型输入宽、模型输入高、缩放方法, 返回：缩放后的检测框

12. `scale_keypoints_to_original_image(kpts_xy, kpts_score, boxes, img_w, img_h, input_W, input_H, resize_type)`

    调整关键点坐标至原图尺寸，输入：关键点坐标、关键点置信度、检测框、原图宽、原图高、模型输入宽、模型输入高、缩放方法, 返回：缩放后的关键点坐标和置信度

13. `draw_boxes(img, boxes, ids, scores, coco_names, rdk_colors)`

    绘制检测结果，输入：图像、检测框、类别ID、置信度、类别列表、颜色映射

14. `draw_keypoints(image, kpts_xy, kpt_score, kpt_conf_thresh)`

    绘制关键点骨架，输入：图像、关键点坐标、关键点置信度、关键点置信度阈值

15. `cv2.imwrite(image_path, image)`
    
    保存结果图像，输入：保存路径、图像

### FAQ

Q: 运行示例时出现 "No module named 'hbm_runtime'" 错误怎么办？\
A: 请确保已正确安装 RDK 的 Python 环境，`hbm_runtime` 模块等官方提供的专用推理库。

Q: 如何更换测试图片？\
A: 在运行命令时使用 `--test-img` 参数指定图片路径，例如：`python ultralytics_yolo11_pose.py --test-img your_image.jpg`

Q: 检测结果不准确怎么办？\
A: 可以尝试调整 `--score-thres` 和 `--nms-thres` 参数，或者使用更高质量的输入图像。

Q: 如何调整检测阈值？\
A: 在代码中修改 `--score-thres` 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 关键点显示不完整怎么办？\
A: 可以调整 `--kpt-conf-thres` 参数，降低阈值可以显示更多关键点，但可能包含低置信度的点。

Q: 可以实时处理视频流吗？\
A: 当前示例是针对单张图像设计的，但可以修改代码以实现视频流的实时姿态估计。

Q: 如何获取其他预训练的 YOLO 姿态估计模型？\
A: 可以参考 [model_zoo 仓库 ](https://github.com/D-Robotics/rdk_model_zoo) 或者 [ 工具链的基础模型仓库 ](https://github.com/D-Robotics/hobot_model)
