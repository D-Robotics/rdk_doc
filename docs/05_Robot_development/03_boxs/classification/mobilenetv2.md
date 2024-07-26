---
sidebar_position: 1
---
# mobilenetv2

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

mobilenetv2图片分类算法示例使用图片作为输入，利用BPU进行算法推理，发布包含物体类别的算法msg。

mobilenetv2是使用[ImageNet data](http://www.image-net.org/)数据集训练出来的caffe模型，模型来源： (https://github.com/shicai/MobileNet-Caffe)。
支持的目标类型包括人、动物、水果、交通工具等共1000种类型。具体支持的类别详见地平线RDK板端文件 /opt/tros/`${TROS_DISTRO}`/lib/dnn_node_example/config/imagenet.list（已安装TogatherROS.Bot）。

代码仓库： (https://github.com/D-Robotics/hobot_dnn)

应用场景：mobilenetv2能够预测给定图片的类别，可实现数字识别、物体识别等功能，主要应用于文字识别、图像检索等领域。

食品类型识别案例： (https://github.com/frotms/Chinese-and-Western-Food-Classification`

## 支持平台

| 平台    | 运行方式      | 示例功能                       |
| ------- | ------------ | ------------------------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | · 启动MIPI/USB摄像头，并通过web展示推理渲染结果<br/>· 使用本地回灌，渲染结果保存在本地 |
| RDK Ultra| Ubuntu 20.04 (Foxy) | · 启动MIPI/USB摄像头，并通过web展示推理渲染结果<br/>· 使用本地回灌，渲染结果保存在本地 |
| X86     | Ubuntu 20.04 (Foxy) | · 使用本地回灌，渲染结果保存在本地 |

## 准备工作

### 地平线RDK平台

1. 地平线RDK已烧录好地平线提供的Ubuntu 20.04/Ubuntu 22.04系统镜像。

2. 地平线RDK已成功安装tros.b。

3. 地平线RDK已安装MIPI或者USB摄像头，无摄像头的情况下通过回灌本地JPEG/PNG格式图片或者MP4、H.264和H.265的视频方式体验算法效果。

4. 确认PC机能够通过网络访问地平线RDK。

### X86平台

1. X86环境已配置好Ubuntu 20.04系统镜像。

2. X86环境系统已成功安装tros.b。

## 使用介绍

### 地平线RDK平台

mobilenetv2图片分类订阅sensor package发布的图片，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示发布的图片和对应的算法结果。

#### 使用MIPI摄像头发布图片


<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# 配置tros.b环境
source /opt/tros/setup.bash
```

</TabItem>

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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### 使用USB摄像头发布图片


<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# 配置tros.b环境
source /opt/tros/setup.bash
```

</TabItem>

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
ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image_width:=480 dnn_example_image_height:=272
```

#### 使用本地图片回灌

mobilenetv2图片分类算法示例使用本地JPEG/PNG格式图片回灌，经过推理后将算法结果渲染后的图片存储在本地的运行路径下。


<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# 配置tros.b环境
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# 配置tros.b环境
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
# 启动launch文件
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image:=config/target_class.jpg
```

### X86平台

#### 使用本地图片回灌

mobilenetv2图片分类算法示例使用本地JPEG/PNG格式图片回灌，经过推理后将算法结果渲染后的图片存储在本地的运行路径下。

```bash
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动launch文件
ros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/mobilenetv2workconfig.json dnn_example_image:=config/target_class.jpg
```

## 结果分析

### 使用摄像头发布图片

在运行终端输出如下信息：

```shell
[example-3] [WARN] [1655095481.707875587] [example]: Create ai msg publisher with topic_name: hobot_dnn_detection
[example-3] [WARN] [1655095481.707983957] [example]: Create img hbmem_subscription with topic_name: /hbmem_img
[example-3] [WARN] [1655095482.985732162] [img_sub]: Sub img fps 31.07
[example-3] [WARN] [1655095482.992031931] [example]: Smart fps 31.31
[example-3] [WARN] [1655095484.018818843] [img_sub]: Sub img fps 30.04
[example-3] [WARN] [1655095484.025123362] [example]: Smart fps 30.04
[example-3] [WARN] [1655095485.051988567] [img_sub]: Sub img fps 30.01
[example-3] [WARN] [1655095486.057854228] [example]: Smart fps 30.07
```

输出log显示，发布算法推理结果的topic为`hobot_dnn_detection`，订阅图片的topic为`/hbmem_img`，订阅到的图片和算法推理输出帧率约为30fps。

在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为地平线RDK的IP地址）：

![render_web](/../static/img/05_Robot_development/03_boxs/classification/image/mobilenetv2/mobilenetv2_render_web.jpeg)

### 使用本地图片回灌

在运行终端输出如下信息：

```shell
[example-1] [INFO] [1654767648.897132079] [example]: The model input width is 224 and height is 224
[example-1] [INFO] [1654767648.897180241] [example]: Dnn node feed with local image: config/target_class.jpg
[example-1] [INFO] [1654767648.935638968] [example]: task_num: 2
[example-1] [INFO] [1654767648.946566665] [example]: Output from image_name: config/target_class.jpg, frame_id: feedback, stamp: 0.0
[example-1] [INFO] [1654767648.946671029] [ClassificationPostProcess]: outputs size: 1
[example-1] [INFO] [1654767648.946718774] [ClassificationPostProcess]: out cls size: 1
[example-1] [INFO] [1654767648.946773602] [ClassificationPostProcess]: class type:window-shade, score:0.776356
[example-1] [INFO] [1654767648.947251721] [ImageUtils]: target size: 1
[example-1] [INFO] [1654767648.947342212] [ImageUtils]: target type: window-shade, rois.size: 1
[example-1] [INFO] [1654767648.947381666] [ImageUtils]: roi.type: , x_offset: 112 y_offset: 112 width: 0 height: 0
[example-1] [WARN] [1654767648.947563731] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg
```

输出log显示，算法使用输入的图片config/target_class.jpg推理出的图片分类结果是window-shade，置信度为0.776356（算法只输出置信度最高的分类结果）。存储的渲染图片文件名为render_feedback_0_0.jpeg，渲染图片效果：

![render_feedback](/../static/img/05_Robot_development/03_boxs/classification/image/mobilenetv2/mobilenetv2_render_feedback.jpeg)
