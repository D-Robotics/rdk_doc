---
sidebar_position: 4
---
# 单目3D室内检测

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

mono3d_indoor_detection package是基于hobot_dnn package开发的室内物体3D检测算法示例，在RDK上使用3D检测模型和室内数据利用BPU进行模型推理，从而得到推理结果。

相比于2D目标检测只能够识别出物体的类别和检测框，3D目标检测能够识别出物体的精确位置和朝向。例如在导航避障应用场景下，3D目标检测算法提供的丰富信息可以帮助规划控制模块实现更好的避障效果。

算法支持的室内物体检测类别包括：充电座、垃圾桶、拖鞋。

每个类别的检测结果包括：

- 长、宽、高：三维物体（即六面体）的长、宽、高，单位为米。

- 转向：物体相对于相机的朝向，单位弧度，取值范围为-π~ π，表示在相机坐标系下物体前进方向与相机坐标系x轴的夹角。

- 深度信息：相机到物体的距离，单位为米。

代码仓库： (https://github.com/D-Robotics/mono3d_indoor_detection)

应用场景：单目3D室内检测算法能够直接识别出图片中物体的确切位置和朝向，可实现物体姿态的识别，主要应用于自动驾驶、智能家居等领域。

单目3D车辆检测案例： (https://github.com/RayXie29/Kaggle-Peking-University-Baidu-Autonomous-Driving-32-place-solution)

## 支持平台

| 平台                  | 运行方式     | 示例功能                                              |
| --------------------- | ------------ | ----------------------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头，并通过Web展示推理渲染结果 |
| RDK X5 | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头，并通过Web展示推理渲染结果 |
| X86                   | Ubuntu 20.04 (Foxy) | · 启动本地回灌，推理渲染结果保存在本地                |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 20.04/Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

### X86平台

1. X86环境已配置Ubuntu 20.04系统镜像。

2. X86环境已成功安装tros.b。

## 使用介绍

因3D检测模型与相机参数相关，不同相机需要进行参数等调整。

单目3D室内检测算法示例package采取读取本地图片的形式进行检测推理，经过算法推理后检测出物体类别和3D定位信息，并且对外发布3D检测信息的算法msg。用户可以订阅3D检测结果msg用于应用开发。

### RDK平台

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
# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono3d_indoor_detection/config/ .

# 启动launch文件
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

### X86平台

```bash
# 配置tros.b环境
source /opt/tros/setup.bash

# 从tros.b的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono3d_indoor_detection/config/ .

# 启动launch文件
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 
```

## 结果分析

mono3d_indoor_detection package处理完一帧图片数据后，在运行终端输出如下信息：

```shell
[mono3d_indoor_detection-1] [INFO] [1662612553.868256257] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868303755] [mono3d_detection]: target type: width, value: 0.236816
[mono3d_indoor_detection-1] [INFO] [1662612553.868358420] [mono3d_detection]: target type: height, value: 0.305664
[mono3d_indoor_detection-1] [INFO] [1662612553.868404002] [mono3d_detection]: target type: length, value: 0.224182
[mono3d_indoor_detection-1] [INFO] [1662612553.868448000] [mono3d_detection]: target type: rotation, value: -1.571989
[mono3d_indoor_detection-1] [INFO] [1662612553.868487790] [mono3d_detection]: target type: x, value: -0.191978
[mono3d_indoor_detection-1] [INFO] [1662612553.868530705] [mono3d_detection]: target type: y, value: -0.143963
[mono3d_indoor_detection-1] [INFO] [1662612553.868570870] [mono3d_detection]: target type: z, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868611119] [mono3d_detection]: target type: depth, value: 0.714024
[mono3d_indoor_detection-1] [INFO] [1662612553.868651409] [mono3d_detection]: target type: score, value: 0.973215
[mono3d_indoor_detection-1] [INFO] [1662612553.868760238] [mono3d_detection]: target type: trash_can
[mono3d_indoor_detection-1] [INFO] [1662612553.868799486] [mono3d_detection]: target type: width, value: 0.253052
[mono3d_indoor_detection-1] [INFO] [1662612553.868842610] [mono3d_detection]: target type: height, value: 0.282349
[mono3d_indoor_detection-1] [INFO] [1662612553.868885191] [mono3d_detection]: target type: length, value: 0.257935
[mono3d_indoor_detection-1] [INFO] [1662612553.868929273] [mono3d_detection]: target type: rotation, value: -1.542728
[mono3d_indoor_detection-1] [INFO] [1662612553.868968855] [mono3d_detection]: target type: x, value: 0.552460
[mono3d_indoor_detection-1] [INFO] [1662612553.869010645] [mono3d_detection]: target type: y, value: -0.164073
[mono3d_indoor_detection-1] [INFO] [1662612553.869050018] [mono3d_detection]: target type: z, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869088767] [mono3d_detection]: target type: depth, value: 1.088358
[mono3d_indoor_detection-1] [INFO] [1662612553.869126765] [mono3d_detection]: target type: score, value: 0.875521
```

log截取显示了一帧的处理结果，结果显示，订阅到的算法msg中的target type即分类结果为trash_can，同时也给出了trash_can的三维和距离以及旋转角度信息。

读取本地图片（可以通过修改mono3d_indoor_detection.launch.py中feed_image字段替换图片）渲染的结果保存成图片在程序运行的result目录下。对应图片推理结果以及渲染信息如下：

![](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/indoor_render.jpeg)
