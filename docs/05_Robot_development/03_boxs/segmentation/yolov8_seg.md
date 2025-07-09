---
sidebar_position: 2
---
# Ultralytics YOLOv8-Seg

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

Ultralytics YOLOv8-Seg实例分割算法示例使用图片作为输入，利用BPU进行算法推理，发布包含检测和分割结果msg。

YOLOv8-Seg是使用[COCO128-seg数据集](http://cocodataset.org/)训练出来的Onnx模型，模型来源： https://github.com/D-Robotics/hobot_model 。
支持对人、动物、水果、交通工具等共80种类型进行实例分割。

代码仓库： https://github.com/D-Robotics/hobot_dnn

应用场景：YOLOv8-Seg能够识别图像中的单个物体并对其进行精确分割。这种技术可以应用在自动驾驶、遥感图像分析、医疗影像分析等领域。


## 支持平台

| 平台    | 运行方式      | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X5, RDK X5 Module| Ubuntu 22.04 (Humble) | · 启动MIPI/USB摄像头/本地回灌，渲染结果保存在本地 |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头，无摄像头的情况下通过回灌本地JPEG/PNG格式图片的方式体验算法效果。


## 使用介绍

### RDK平台

#### 使用摄像头发布图片

##### 使用MIPI摄像头发布图片

YOLOv8-Seg实例分割示例订阅sensor package发布的图片, 经过推理后发布算法msg。默认不保存渲染图片, 如需保存, 需要在运行时设置 dnn_example_dump_render_img 为1, 会在运行路径下自动保存渲染后的图片，命名方式为render_frameid_时间戳秒_时间戳纳秒.jpg。

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=0 dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

##### 使用USB摄像头发布图片

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_dump_render_img:=0 dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image_width:=1920 dnn_example_image_height:=1080
```

#### 使用本地图片回灌

YOLOv8-Seg分割示例使用本地JPEG/PNG格式图片回灌，经过推理后将算法结果渲染后的图片存储在本地的运行路径下。

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>


```shell
# 启动launch文件
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/yolov8segworkconfig.json dnn_example_image:=config/test.jpg
```

## 结果分析

### 使用摄像头发布图片

在运行终端输出如下信息：

```shell
[example-3] [WARN] [0000001244.489045384] [example]: Sub img fps: -1.00, Smart fps: 6.00, infer time ms: 12, post process time ms: 31
[example-3] [WARN] [0000001245.524813052] [example]: Sub img fps: 5.84, Smart fps: 4.99, infer time ms: 8, post process time ms: 64
[example-3] [WARN] [0000001246.526635344] [example]: Sub img fps: 4.96, Smart fps: 5.00, infer time ms: 8, post process time ms: 66
[example-3] [WARN] [0000001247.528846136] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001248.528474095] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001249.528576345] [example]: Sub img fps: 5.00, Smart fps: 5.00, infer time ms: 8, post process time ms: 68
[example-3] [WARN] [0000001250.493265846] [example]: Sub img fps: 5.02, Smart fps: 5.00, infer time ms: 8, post process time ms: 32
[example-3] [WARN] [0000001251.528909346] [example]: Sub img fps: 4.98, Smart fps: 5.00, infer time ms: 8, post process time ms: 67
```

输出log显示，发布算法推理结果的topic为`hobot_dnn_detection`，订阅图片的topic为`/hbmem_img`，其中图片发布的帧率根据会根据算法推理输出帧率自适应。此外，RDK上会渲染实例分割结果并存储图片在运行路径下，会使帧率下降。

原始图片：
![raw](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/test.jpg)

渲染后的图片：
![render_web](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/web.jpeg)

### 使用本地图片回灌

在运行终端输出如下信息：

```shell
[INFO] [0000001744.811779665] [example]: Dnn node feed with local image: /userdata/config/test.jpg
[INFO] [0000001746.237111249] [example]: Output from frame_id: feedback, stamp: 0.0
[INFO] [0000001746.266157040] [PostProcessBase]: out box size: 6
[INFO] [0000001746.266340040] [PostProcessBase]: det rect: 90.4946 58.2675 192.103 351.403, det type: person, score:0.927177
[INFO] [0000001746.267129832] [PostProcessBase]: det rect: 455.518 77.1254 536.289 354.541, det type: person, score:0.909735
[INFO] [0000001746.267248457] [PostProcessBase]: det rect: 381.604 103.953 464.446 327.9, det type: person, score:0.898899
[INFO] [0000001746.267331624] [PostProcessBase]: det rect: 204.864 71.6262 303.593 351.835, det type: person, score:0.887814
[INFO] [0000001746.267404540] [PostProcessBase]: det rect: 317.885 108.287 389.773 338.197, det type: person, score:0.866887
[INFO] [0000001746.267486457] [PostProcessBase]: det rect: 181.487 111.093 202.097 132.665, det type: car, score:0.443035
[INFO] [0000001746.267548999] [ClassificationPostProcess]: out cls size: 0
[INFO] [0000001746.267662832] [SegmentationPostProcess]: features size: 14240, width: 160, height: 89, num_classes: 80, step: 1
[INFO] [0000001746.270546040] [ImageUtils]: target size: 7
[INFO] [0000001746.270674082] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.270745915] [ImageUtils]: roi.type: person, x_offset: 90 y_offset: 58 width: 101 height: 293
[INFO] [0000001746.271122207] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271162499] [ImageUtils]: roi.type: person, x_offset: 455 y_offset: 77 width: 80 height: 277
[INFO] [0000001746.271325499] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271362082] [ImageUtils]: roi.type: person, x_offset: 381 y_offset: 103 width: 82 height: 223
[INFO] [0000001746.271491040] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271525249] [ImageUtils]: roi.type: person, x_offset: 204 y_offset: 71 width: 98 height: 280
[INFO] [0000001746.271782749] [ImageUtils]: target type: person, rois.size: 1
[INFO] [0000001746.271819457] [ImageUtils]: roi.type: person, x_offset: 317 y_offset: 108 width: 71 height: 229
[INFO] [0000001746.271947790] [ImageUtils]: target type: car, rois.size: 1
[INFO] [0000001746.271982374] [ImageUtils]: roi.type: car, x_offset: 181 y_offset: 111 width: 20 height: 21
[INFO] [0000001746.272044124] [ImageUtils]: target type: parking_space, rois.size: 0
[WARN] [0000001746.276824624] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg
```

输出log显示，算法使用输入的图片config/test.jpeg推理，存储的渲染图片文件名为render_feedback_0_0.jpeg，渲染图片效果：

![render_feedback](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/segmentation/image/yolov8_seg/local.jpeg)
