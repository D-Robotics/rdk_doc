---
sidebar_position: 2
---

# USB Camera Usage

For using a USB camera, you can directly refer to the "USB Camera Real-time Detection" example in the Python samples.

## Example Introduction

The USB camera real-time detection example is located in `/app/pydev_demo/07_usb_camera_sample/`. It provides a **Python interface** for real-time object detection, demonstrating how to use `hbm_runtime` with the YOLOv5X model to perform inference on USB camera feeds and display the detection results in a window in real time.

Included model example:
```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample$ tree -L 1
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

## Effect Demonstration
> **Note**: Since this example pops up a window to display detection results, it needs to be run in a graphical environment. For server version images, it is recommended to use remote tools that support X11 forwarding, such as **MobaXterm**, to connect to the development board, allowing the pop-up display window to appear on your local computer.

The example reads the USB camera feed in real time, detects objects in the画面, and overlays bounding boxes, class labels, and confidence scores on the window in real time.

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_07_usb_camera_sample_running.png)

## Hardware Preparation

### Hardware Connection
- One RDK development board
- One USB camera (connected to the USB port of the development board)
- Connect the power cable and network cable

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_09_hw_connect.png)

## Quick Start

### Code and Board Location
Navigate to `/app/pydev_demo/07_usb_camera_sample/` to see the example files:

```
root@ubuntu:/app/pydev_demo/07_usb_camera_sample# tree
.
├── coco_classes.names
└── usb_camera_yolov5x.py
```

### Compilation and Execution
Python examples do not require compilation; you can run them directly:

```
cd /app/pydev_demo/07_usb_camera_sample
python usb_camera_yolov5x.py
```

Press the `q` key to exit (you need to focus the mouse on the display window).

### Execution Effect
After running, the program will automatically find an available USB camera device and start real-time object detection. The detection results will be displayed in a window. Focus the mouse on the display window and press `q` to exit the entire program.

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
[For more detailed content, you can directly refer to the example](../../03_pydev_demo_sample/RDK_X5/07_usb_camera_sample.md)