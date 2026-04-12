---
sidebar_position: 1
---

# MIPI 摄像头使用
MIPI 摄像头的使用，可以直接参考 python 示例中的 “MIPI 摄像头实时检测” 示例

## 示例简介
MIPI 摄像头实时检测示例是一个位于 `/app/pydev_demo/08_mipi_camera_sample` 中的 **Python 接口** 开发代码示例，用于演示如何使用板载 MIPI 摄像头进行实时目标检测。该示例使用 YOLOv5x 目标检测模型对 MIPI 摄像头采集的视频流进行实时推理，并将检测结果通过 HDMI 显示，输出检测框信息。

包含的示例：
```
root@ubuntu:/app/pydev_demo/08_mipi_camera_sample$ tree
.
├── 01_mipi_camera_yolov5s.py
├── 02_mipi_camera_dump.py
├── 03_mipi_camera_scale.py
├── 04_mipi_camera_crop_scale.py
├── 05_mipi_camera_streamer.py
└── coco_classes.names
```

## 效果展示

### 01实时目标检测效果

:::info 可视化检测效果

如果需要通过显示屏查看实时的摄像头画面和检测结果可视化效果，需要：

1. **外接显示屏**：通过 HDMI 线缆将开发板连接到显示器
2. **Desktop 版特殊处理**：如果使用的是 Desktop 版本系统，需要先执行以下命令关闭桌面服务：
   ```bash
   sudo systemctl stop lightdm
   ```
3. **远程连接**：通过 SSH 远程连接到板端
4. **运行代码**：执行示例程序后，即可在连接的显示屏上看到实时的检测结果

:::

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_yolov5s_runing.jpg)

### 02图像采集保存效果

运行后会在脚本同级目录下保存多个 YUV 格式的图像文件，默认是 1920x1080 的分辨率。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_dump.png)

### 03图像缩放处理效果

运行后会在脚本同级目录下保存缩放后的 YUV 图像文件，默认是 640x360 的分辨率。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_scale.png)

### 04图像裁剪缩放效果

运行后会在脚本同级目录下保存裁剪并缩放后的 YUV 图像文件（NV12 格式），默认是裁剪缩放画面中心，我们调整一下裁剪位置，可以得到如下 YUV 图像。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_crop_scale.png)

### 05实时推流显示效果

运行后通过 HDMI 屏幕实时显示摄像头画面（推流测试）, 注意 Desktop 版本需要先执行 `sudo systemctl stop lightdm` 关闭桌面服务。

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_08_mipi_camera_stream.gif)



## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 连接官方适配的 MIPI 摄像头
3. 通过 HDMI 线连接显示器和开发板
4. 连接电源线和网线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/RDK_X5_Newly/pydev_03_hw_connect.png)

## 快速开始

### 代码以及板端位置
示例文件位于 `/app/pydev_demo/08_mipi_camera_sample`

### 编译以及运行
Python 示例无需编译，直接运行即可：

01_mipi_camera_yolov5s.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 01_mipi_camera_yolov5s.py
```

02_mipi_camera_dump.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 02_mipi_camera_dump.py -f 30 -c 10 -w 1920 -h 1080
```

03_mipi_camera_scale.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample

# 该示例需要 input.yuv 作为输入，这里我们以上个运行示例中的 output0.yuv 作为输入，执行拷贝命令
cp output0.yuv input.yuv

# 然后运行示例
python 03_mipi_camera_scale.py -i input.yuv -o output_640x360.yuv -w 640 -h 360 --iwidth 1920 --iheight 1080
```

04_mipi_camera_crop_scale.py 运行:

```
cd /app/pydev_demo/08_mipi_camera_sample
python 04_mipi_camera_crop_scale.py -i input.yuv -o output_640x480.yuv -w 640 -h 480 --iwidth 1920 --iheight 1080 -x 304 -y 304 --crop_w 896 --crop_h 592
```

05_mipi_camera_streamer.py 运行:
```
cd /app/pydev_demo/08_mipi_camera_sample
python 05_mipi_camera_streamer.py -w 1920 -h 1080
```

<!-- ### 执行效果
01_mipi_camera_yolov5s.py 执行效果:
运行后，程序会初始化 MIPI 摄像头和 HDMI 显示，并开始实时目标检测。检测结果会通过 HDMI 显示。

02_mipi_camera_dump.py 执行效果:
运行成功后，脚本所在目录会存放多个yuv文件。

03_mipi_camera_scale.py 执行效果:
运行成功后，脚本所在目录会存放缩放后的yuv文件。

04_mipi_camera_crop_scale.py 执行效果:
运行成功后，脚本所在目录会存放缩放后的yuv文件。注意:裁剪宽度必须是 16 的整数倍（即对齐到 16 字节）。

05_mipi_camera_streamer.py 执行效果:
运行成功后，屏幕会显示实时画面。 -->


[更多详细内容可以直接查看示例](../../03_pydev_demo_sample/RDK_X5/08_mipi_camera_sample.md)
