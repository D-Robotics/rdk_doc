---
sidebar_position: 9
---

# USB 摄像头实时检测

## 示例简介
USB 摄像头实时检测示例位于 `/app/pydev_demo/07_usb_camera_sample/`，提供 **Python 接口** 的实时目标检测示例，演示如何使用 `hbm_runtime` 结合 YOLOv5X 模型对 USB 摄像头画面进行推理，并在窗口中实时展示检测结果。

包含的模型示例：
```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample$ tree -L 1
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

## 效果展示
> **注意**：由于本示例会弹出一个窗口来显示检测结果，因此需要在有图形界面的环境下运行。Server 版本的镜像建议使用 **MobaXterm** 等支持 X11 转发的远程工具连接到开发板，这样可以在本地电脑上看到弹出的显示窗口。

实时读取 USB 摄像头画面，对画面中的目标进行检测，并在窗口实时叠加检测框、类别与置信度。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_07_usb_camera_sample_running.png)

## 硬件准备

### 硬件连接
- RDK 开发板一套
- USB 摄像头一只（连接至开发板 USB 口）
- 连接电源线和网线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_hw_connect.png)

## 快速开始

### 代码以及板端位置
进入 `/app/pydev_demo/07_usb_camera_sample/` 可见示例文件：

```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample# tree
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

### 编译以及运行
Python 示例无需编译, 直接运行即可：

```
cd /app/pydev_demo/07_usb_camera_sample
python usb_camera_yolov5x.py
```

按 `q` 键退出（需将鼠标聚焦到显示窗口）。

### 执行效果
运行后，程序会自动查找可用的 USB 摄像头设备，并开始实时目标检测。检测结果会通过窗口显示，将鼠标焦点放到显示窗口按下 q 退出整体程序。

```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample# python usb_camera_yolov5x.py
Opening video device: /dev/video0
Open USB camera successfully
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2026-01-20,19:02:15.744.442) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2026-01-20,19:02:15.859.240) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.

...

Place the mouse in the display window and press 'q' to quit
```

## 详细介绍

### 示例程序参数选项说明
| 参数 | 说明 | 类型 | 默认值 |
|------|------|------|--------|
| `--model-path` | BPU 量化模型路径（`.bin`） | str | `/app/model/basic/yolov5s_672x672_nv12.bin` |
| `--priority` | 推理优先级（0~255，255最高） | int | `0` |
| `--bpu-cores` | BPU 核心索引列表（如 `0 1`） | int list | `[0]` |
| `--label-file` | 类别标签路径 | str | `coco_classes.names` |
| `--resize-type` | 预处理缩放方式（0直接缩放，1信箱缩放） | int | `1` |
| `--classes-num` | 检测类别数 | int | `80` |
| `--nms-thres` | NMS IoU 阈值 | float | `0.45` |
| `--score-thres` | 置信度阈值 | float | `0.25` |

### 软件架构说明
本部分介绍usb 摄像头示例的软件架构和工作流程，说明程序从模型加载到结果输出的完整执行过程，帮助开发者理解代码的整体结构和数据流向。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_usb_camera_software_arch.png)
</center>

1. **摄像头检测与打开** - 自动搜索 `/dev/video*` 并打开首个可用 USB 摄像头，设置分辨率与帧率
2. **模型加载** - 使用 `hbm_runtime` 模块加载模型文件
3. **图片加载** - 读取实时的图像帧
3. **图像预处理** - 将摄像头帧缩放至模型输入尺寸，BGR 转 NV12，并按模型输入格式组织张量
4. **模型推理** - 在 BPU 上执行 YOLOv5X 前向推理
5. **结果后处理** - 反量化输出，解码预测框，置信度过滤与 NMS，坐标映射回原图
6. **可视化输出** - 在图像上绘制检测框、类别与置信度，窗口实时显示，按 `q` 退出

### API 流程说明
本部分列出示例程序中使用的主要 API 接口，说明各接口的功能、输入参数和返回值，帮助开发者快速了解代码实现细节和接口调用方式。其中，`cv2.` 开头的是 OPENCV 库的通用接口，其余的为地瓜自定义的接口。

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_usb_camera_api.png)
</center>

1. `find_first_usb_camera()`

   自动搜索 usb 摄像头

2. `cv2.VideoCapture(device)`  
    
    开启摄像头

3. `hbm_runtime.HB_HBMRuntime(model_path)`  

   加载量化模型，输入：模型路径

4. `load_class_names(label_file)`  
   
   加载类别名称，输入：类别文件路径
5. `cap.read()`

    获取图像帧

6. `resized_image(img, input_W, input_H, resize_type)`

   图像缩放，输入：图像、目标宽、目标高、缩放类型, 返回:缩放后的图像

7. `bgr_to_nv12_planes(image)`

   BGR 转 NV12 格式，输入：BGR 图像, 返回: y 平面, uv 平面

8. `model.run(input_data)`

   执行模型推理，输入：预处理后的图像数据, 返回：模型输出

9. `dequantize_outputs(outputs, output_quants)`

   结果反量化，输入：模型输出，指定类型，返回：反量化后的结果

10. `decode_outputs(output_names, fp32_outputs, STRIDES, ANCHORS, classes_num)`

      解码模型输出，输入：输出名称列表、模型输出、步长、锚框、类别数量, 返回：预测结果

11. `filter_classification(cls_output, conf_thres)`

      过滤预测结果，输入：分类输出、置信度阈值, 返回：置信度、类别ID、有效索引

12. `NMS(dbboxes, scores, ids, nms_thresh)`

    非极大值抑制，输入：检测框、置信度、类别、IoU 阈值, 返回：保留的索引

13. `scale_coords_back(boxes, img_w, img_h, input_W, input_H, resize_type)`

    调整检测框至原图尺寸，输入：检测框、原图宽、原图高、模型输入宽、模型输入高、缩放方法, 返回：缩放后的检测框

14. `draw_boxes(img, boxes, ids, scores, coco_names, rdk_colors)`

    绘制检测结果，输入：图像、检测框、类别ID、置信度、类别列表、颜色映射

15. `cv2.imshow(...)` / `cv2.waitKey(1)`  
    显示与按键退出

### FAQ

Q: 运行时报 “No USB camera found” 怎么办？  
A: 确认摄像头已插入且在 `/dev` 下存在 `video*` 设备；可尝试 `v4l2-ctl --list-devices` 检查。  

<!-- Q: 运行无画面或报 “Failed to open video device” 怎么办？  
A: 确认有桌面环境并具备摄像头权限；更换 USB 口或摄像头。   -->

Q: 模型文件不存在怎么办？  
A: 检查 `--model-path` 是否正确，可在 `/app/model/basic` 下查找对应模型。  

Q: 画面延迟或卡顿？  
A: 降低摄像头分辨率或帧率，或减小模型输入尺寸
<!-- ；确保 BPU 核心配置正确。   -->

Q: 如何退出程序？  
A: 将鼠标焦点放在显示窗口，按 `q` 键退出。  
