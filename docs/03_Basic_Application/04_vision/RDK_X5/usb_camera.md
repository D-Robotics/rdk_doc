---
sidebar_position: 2
---

# USB 摄像头使用
USB 摄像头的使用，可以直接参考 python 示例中的 “USB 摄像头实时检测” 示例

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
[更多详细内容可以直接查看示例](../../03_pydev_demo_sample/RDK_X5/07_usb_camera_sample.md)
