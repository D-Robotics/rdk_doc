---
sidebar_position: 9
---

# 3.3.9 mipi camera 示例介绍

## 示例简介
MIPI 摄像头示例是一个位于 `/app/pydev_demo/03_mipi_camera_sample` 中的 **Python 接口** 开发代码示例，用于演示如何使用板载 MIPI 摄像头进行实时目标检测。该示例使用 FCOS 目标检测模型对 MIPI 摄像头采集的视频流进行实时推理，并将检测结果通过 HDMI 显示，同时输出检测框信息和 FPS 性能数据。

## 效果展示

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_runing.png)

## 硬件准备

### 硬件连接
1. 准备一个 RDK 开发板
2. 连接官方适配的 MIPI 摄像头
3. 通过 HDMI 线连接显示器和开发板
4. 连接电源线和网线

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_hw_connect.png)

## 快速开始

### 代码以及板端位置
MIPI 摄像头示例文件位于 `/app/pydev_demo/03_mipi_camera_sample/mipi_camera.py`

### 编译以及运行
Python 示例无需编译，直接运行即可：

### 执行效果

运行后，程序会初始化 MIPI 摄像头和 HDMI 显示，并开始实时目标检测。检测结果会通过 HDMI 显示，控制台会打印检测到的目标信息及 FPS。

## 详细介绍

### 示例程序参数选项说明
MIPI 摄像头示例不需要命令行参数，直接运行即可。程序会自动检测并使用板载 MIPI 摄像头。

### 软件架构说明
MIPI 摄像头示例的软件架构包含以下几个核心部分：

1. 模型加载：加载 FCOS 目标检测模型

2. 摄像头初始化：使用 srcampy.Camera() 初始化 MIPI 摄像头

3. 显示初始化：初始化 HDMI 显示

4. 摄像头与显示绑定：将摄像头输出直接绑定到显示

5. 实时推理循环：

- 从摄像头获取图像

- 模型推理

- 后处理（解析检测结果）

- 绘制检测框并显示

- 计算并打印 FPS

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_software_arch.png)
</center>

### API 流程说明

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_api_flow.png)
</center>

### FAQ

Q: 运行示例时提示摄像头初始化失败怎么办？\
A: 请检查 MIPI 摄像头是否正确连接，并确保摄像头驱动已正确加载。可以尝试重启设备。

Q: HDMI 显示不正常或没有输出怎么办？\
A: 请检查 HDMI 连接，并确保显示服务已停止（如使用 systemctl stop lightdm）。

Q: 如何调整检测阈值？\
A: 在代码中修改 fcos_postprocess_info.score_threshold 的值，例如改为 0.5 可以提高检测灵敏度。

Q: 如何修改显示的分辨率？\
A: 修改代码中的 sensor_width 和 sensor_height 变量，但需要注意显示设备是否支持该分辨率。

Q: 运行示例时帧率很低怎么办？\
A: 可以尝试使用更轻量的模型，或者调整摄像头的采集分辨率。

Q: 如何保存检测结果图像？\
A: 可以在代码中添加图像保存逻辑，例如使用 cv2.imwrite() 保存处理后的图像。

Q: 如何添加新的检测类别？\
A: 需要修改 get_classes() 函数，添加新的类别名称，并重新训练模型或使用支持新类别的模型。


