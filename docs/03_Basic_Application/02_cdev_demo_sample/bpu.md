---
sidebar_position: 1
---

# 3.2.1 bpu 示例介绍

## 示例简介
bpu 是一个位于 /app/cdev_demo 目录中的 **C 语言接口** 开发代码示例，用于演示如何用 c 语言来调用 bpu 已经支持的模型。参考这个示例，用户可以理解并开发相关应用。


## 效果展示
bpu 示例中支持两个场景，一种是带摄像头的 , 固定是使用的 yolo 的模型，一种是不带摄像头的，推理回灌的数据。

如下是使用摄像头搭配 yolov5 进行推理的效果，显示器上可以看到识别到了水瓶

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_running_yolo5v.png)

如下是使用 fcos 进行推理的效果，显示器上显示的是使用 h264 文件作为输入数据，进行推理的效果。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_running_fcos.png)


## 硬件准备

### 硬件连接

(1) 搭配摄像头进行 yolov5 的推理

该示例不需要鼠标和键盘，所以这里连接了摄像头 HDMI 显示屏，网线接口，电源线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/vio_display_hardware_connect.png)


(2) 不使用摄像头，通过 h264 的码流进行推理，结果呈现在显示器上。

该示例不需要鼠标和键盘，所以这里连接了 HDMI 显示屏，网线接口，电源线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/hardware-connect.png)


## 快速开始

### 代码以及板端位置
进入到 `/app/cdev_demo/bpu` 位置，可以看到两个目录和一个 README.md 其中 include 包含了示例模型所需要的头文件，而 src 目录中则是程序入口和各类模型的前处理、推理、后处理等实现。
```
root@ubuntu:/app/cdev_demo/bpu# tree -L 1
.
├── include
├── README.md
└── src
```

### 编译以及运行
我们需要进入到 src 目录下执行 make，产物会出现在 src/bin 目录下
```

root@ubuntu:/app/cdev_demo/bpu/src/bin# tree
.
├── 1080p_.h264
└── sample

```
我们需要进入到 /app/cdev_demo/bpu/src/bin 目录下执行，这里我们以接摄像头的 yolov5 推理作为执行效果。

### 执行效果
(1)
通过 `systemctl stop lightdm` 命令关闭图形界面服务。

(2) 搭配摄像头进行 yolov5 的推理
在 `root@ubuntu:/app/cdev_demo/bpu/src/bin#` 目录使用 `./sample -f /app/model/basic/yolov5s_672x672_nv12.bin -m 0 ` 命令。
```
root@ubuntu:/app/cdev_demo/bpu/src/bin# ./sample -f /app/model/basic/yolov5s_672x672_nv12.bin -m 0
Opened DRM device: /dev/dri/card0
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-10-25,23:45:13.56.570) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2025-10-25,23:45:13.222.713) Model: yolov5s_v2_672x672_bayese_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
Model info:
model_name: yolov5s_v2_672x672_bayese_nv12Input count: 1input[0]: tensorLayout: 2 tensorType: 1 validShape:(1, 3, 672, 672, ), alignedShape:(1, 3, 672, 672, )
Output count: 3Output[0]: tensorLayout: 0 tensorType: 13 validShape:(1, 84, 84, 255, ), alignedShape:(1, 84, 84, 255, )
Output[1]: tensorLayout: 0 tensorType: 13 validShape:(1, 42, 42, 255, ), alignedShape:(1, 42, 42, 255, )
Output[2]: tensorLayout: 0 tensorType: 13 validShape:(1, 21, 21, 255, ), alignedShape:(1, 21, 21, 255, )
2025/10/25 23:45:13.229 !INFO [OpenCamera][0447]hbn module
set camera fps: -1,width: 1920,height: 1080
Camera 0:
        mipi_host: 0
......
......
......

```

就能看到文档开头时候展示的图片一样的效果：
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_running_yolo5v.png)

(3) 解码 h264 之后，通过 fcos 模型进行推理
在 `root@ubuntu:/app/cdev_demo/bpu/src/bin#` 目录使用 `./sample -f /app/model/basic/fcos_512x512_nv12.bin -m 1 -i 1080p_.h264 -w 1920 -h 1080 ` 命令。
```
root@ubuntu:/app/cdev_demo/bpu/src/bin# ./sample -f /app/model/basic/fcos_512x512_nv12.bin -m 1 -i 1080p_.h264 -w 1920 -h 1080
Opened DRM device: /dev/dri/card0
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-10-25,23:46:08.985.465) [HorizonRT] The model builder version = 1.23.5
[W][DNN]bpu_model_info.cpp:491][Version](2025-10-25,23:46:09.127.913) Model: fcos_efficientnetb0_512x512_nv12. Inconsistency between the hbrt library version 3.15.55.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
Model info:
model_name: fcos_efficientnetb0_512x512_nv12Input count: 1input[0]: tensorLayout: 2 tensorType: 1 validShape:(1, 3, 512, 512, ), alignedShape:(1, 3, 512, 512, )
Output count: 15Output[0]: tensorLayout: 0 tensorType: 14 validShape:(1, 64, 64, 80, ), alignedShape:(1, 64, 64, 80, )
......
......
......

```

就能看到文档开头时候展示的图片一样的效果：
![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_running_fcos.png)


## 详细是介绍
这个示例代码相对较多，但执行的时候也有固定的对应关系，我们的 yolov5 是搭配摄像头做的。

### 示例程序参数选项说明

```

root@ubuntu:/app/cdev_demo/bpu/src/bin# ./sample
Usage: sample [OPTION...]
bpu sample -- An C++ example of using bpu

  -d, --debug                Print lots of debugging information.
  -f, --file=modle_file      path of model file
  -h, --video_height=height  height of video
  -i, --input_video=video path   path of video
  -m, --mode=type            0:yolov5;1:fcos
  -w, --video_width=width    width of video
  -?, --help                 Give this help list
      --usage                Give a short usage message

Mandatory or optional arguments to long options are also mandatory or optional
for any corresponding short options.
root@ubuntu:/app/cdev_demo/bpu/src/bin#

```
其中 \
-d 代表打印 debug 信息。\
-f 代表模型的位置\
-i 代表输入给程序的视频文件路径，注意这里是没有使用摄像头的时候才输入，运行有摄像头的示例时候，不需要输入。\
-m 代表选择的模型\
-w 代表输出视频的宽度\
-h 代表输出视频的高度\
-? 代表打印帮助信息


### 软件架构说明
本 Sample 是基于 spcdev 接口实现的， 解析输入给 main 的参数，通过 libspcdev.so API，获取到显示屏的分辨率之后，初始化模型模块，显示模块，视频输入模块，根据适配的分辨率和显示器的分辨率，判断是否需要使用 VPS 进行缩放 ， 通过适当的前处理和后处理线程，将推理结果转换成坐标，通过显示屏呈现出来。由于这个示例代码中包含了多个模型的推理案例，我们在软件架构图中抽取了主要的核心逻辑进行了暂时。

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_single_software_arch.png)
</center>

### API 流程说明

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/02_cdev_demo_sample/image/cdev_bpu_api_flow.png)
</center>

### FAQ
Q：不同的模型是否前处理不一样？\
A：需要根据模型的特性来决定，不同的模型一般是不一样的。
