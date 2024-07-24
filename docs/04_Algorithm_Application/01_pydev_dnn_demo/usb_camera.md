---
sidebar_position: 4
---

# 4.1.4 基于USB摄像头推理{#usb}

## 目标检测算法—fcos

本示例主要实现以下功能：

1. 加载 `fcos` 目标检测算法模型（基于COCO数据集训练的80个类别的目标检测）
2. 从USB摄像头读取视频流，并进行推理
3. 解析模型输出并将结果渲染到原始视频流
4. 通过`HDMI`接口输出渲染后的视频流

### 运行方法

请查阅 [USB摄像头AI推理](/first_application/usb_camera) 了解如何快速运行本示例。

### 示例代码解析
- 导入算法推理模块hobot_dnn、视频输出模块hobot_vio、numpy、opencv、colorsys等模块

    ```
    from hobot_dnn import pyeasy_dnn as dnn
    from hobot_vio import libsrcampy as srcampy
    import numpy as np
    import cv2
    import colorsys
    ```

- 加载模型文件

    调用[load](/python_development/pydev_dnn_api)方法加载模型文件，并返回一个 `hobot_dnn.pyeasy_dnn.Model` 类的 list。

    ```shell
    models = dnn.load('../models/fcos_512x512_nv12.bin')
    ```

    `fcos`模型的输入是`1x3x512x512`数据，格式为`NCHW`。输出为15组数据，用来表示检测到的物体检测框。示例中定义了`print_properties`函数用来输出模型的输入、输出参数：

    ```python
    # print properties of input tensor
    print_properties(models[0].inputs[0].properties)
    # print properties of output tensor
    print(len(models[0].outputs))
    for output in models[0].outputs:
        print_properties(output.properties)
    ```

- 数据预处理

    使用opencv打开USB摄像头设备节点`/dev/video8`，获取实时图像，并把图像缩放到符合模型输入tensor的尺寸

    ```python
    # open usb camera: /dev/video8
    cap = cv2.VideoCapture(8)
    if(not cap.isOpened()):
        exit(-1)
    print("Open usb camera successfully")
    # set the output of usb camera to MJPEG, solution 640 x 480
    codec = cv2.VideoWriter_fourcc( 'M', 'J', 'P', 'G' )
    cap.set(cv2.CAP_PROP_FOURCC, codec)
    cap.set(cv2.CAP_PROP_FPS, 30) 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ```

    然后把bgr格式的图像转换成符合模型输入的 NV12 格式
    ```python
    nv12_data = bgr2nv12_opencv(resized_data)
    ```

- 模型推理

    调用 [Model](../pydev_dnn_api#model) 类的 `forward` 接口进行推理，模型输出15组数据，用来表示检测到的物体检测框。

    ```python
    outputs = models[0].forward(nv12_data)
    ```

- 算法后处理

    示例中的后处理函数`postprcess`，会处理模型输出的物体类别、检测框、置信度等信息。
    ```python
    prediction_bbox = postprocess(outputs, input_shape, origin_img_shape=(1080,1920))
    ```

- 检测结果可视化

    示例对算法结果和原始视频流进行了渲染，并通过`HDMI`接口输出，用户可在显示器上实时预览效果。显示部分用到了hobot_vio模块的Display功能，该模块详细信息请查看 [Display章节](../pydev_multimedia_api_x3/object_display.md)。

    ```python
    # create display object
    disp = srcampy.Display()
    # set solution to 1920 x 1080
    disp.display(0, 1920, 1080)
    
    # if the solution of image is not 1920 x 1080, do resize
    if frame.shape[0]!=1080 and frame.shape[1]!=1920:
        frame = cv2.resize(frame, (1920,1080), interpolation=cv2.INTER_AREA)
    
    # render the detection results to image
    box_bgr = draw_bboxs(frame, prediction_bbox)
    
    # convert BGR to NV12
    box_nv12 = bgr2nv12_opencv(box_bgr)
    # do display
    disp.set_img(box_nv12.tobytes())
    ```

