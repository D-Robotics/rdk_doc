---
sidebar_position: 2
---
# 光流估计

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

光流估计算法是使用PwcNet在[FlyingChairs数据集](https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html)上训练出来的光流估计模型。

算法输入为两帧连续的图像数据，输出第一帧图像的光流图，显示第一帧图像中物体分别在水平和竖直方向上的运动矢量。

代码仓库： (https://github.com/D-Robotics/mono_pwcnet)

应用场景：光流估计是一种用于确定图像序列中物体表面上像素移动模式的技术，可以应用在自动驾驶、运动分析、目标追踪等领域中。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_pwcnet_feedback_0_0.jpeg)

## 支持平台

| 平台                             | 运行方式     | 示例功能                                                 |
| -------------------------------- | ------------ | -------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | 启动MIPI/USB摄像头/本地回灌，并通过Web展示推理渲染结果 |

## 算法信息

| 模型 | 平台 | 输入尺寸 | 推理帧率(fps) |
| ---- | ---- | ------------ | ---- |
| pwcnet | X5 | 1×6×384×512 | 23 |

## 准备工作

### RDK平台

1. RDK已烧录好RDK OS系统。

2. RDK已成功安装TogetheROS.Bot。

3. RDK已安装MIPI或者USB摄像头。

4. 确认PC机能够通过网络访问RDK。

## 使用介绍

光流估计(mono_pwcnet)package订阅sensor package发布的图片，经过推理后发布算法msg，通过websocket package实现在PC端浏览器上渲染显示sensor发布的图片和对应的算法结果。

### RDK平台

**使用MIPI摄像头发布图片**

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch mono_pwcnet pwcnet.launch.py
```

**使用USB摄像头发布图片**

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch mono_pwcnet pwcnet.launch.py
```


**使用本地回灌图片**

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# 配置本地回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch mono_pwcnet pwcnet.launch.py

```

## 结果分析

在运行终端输出如下信息：

```shell
[mono_pwcnet-3] [WARN] [0000000495.652908486] [mono_pwcnet]: Parameter:
[mono_pwcnet-3]  cache_img_limit: 11
[mono_pwcnet-3]  cache_task_limit: 8
[mono_pwcnet-3]  dump_render_img: 0
[mono_pwcnet-3]  feed_type(0:local, 1:sub): 1
[mono_pwcnet-3]  image_size: 2
[mono_pwcnet-3]  is_shared_mem_sub: 1
[mono_pwcnet-3]  is_sync_mode: 0
[mono_pwcnet-3]  ai_msg_pub_topic_name: /pwcnet_msg
[mono_pwcnet-3]  ros_img_sub_topic_name: /image
[mono_pwcnet-3] [WARN] [0000000495.653288277] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[mono_pwcnet-3] [WARN] [0000000495.653349777] [mono_pwcnet]: model_file_name_: config/model.hbm, task_num: 4
[mono_pwcnet-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono_pwcnet-3] [HBRT] set log level as 0. version = 3.15.49.0
[mono_pwcnet-3] [DNN] Runtime version = 1.23.8_(3.15.49 HBRT)
[mono_pwcnet-3] [WARN] [0000000495.864239611] [mono_pwcnet]: Get model name: pwcnet_pwcnetneck_flyingchairs from load model.
[mono_pwcnet-3] [WARN] [0000000495.890934569] [mono_pwcnet]: Create hbmem_subscription with topic_name: /hbmem_img
[mono_pwcnet-3] [WARN] [0000000495.920407361] [mono_pwcnet]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[mono_pwcnet-3] [WARN] [0000000497.404133403] [mono_pwcnet]: Sub img fps: 6.00, Smart fps: 5.84, pre process time ms: 19, infer time ms: 41, post process time ms: 2
[mono_pwcnet-3] [WARN] [0000000499.603858154] [mono_pwcnet]: Sub img fps: 5.04, Smart fps: 5.08, pre process time ms: 19, infer time ms: 41, post process time ms: 1
[mono_pwcnet-3] [WARN] [0000000500.623022321] [mono_pwcnet]: Sub img fps: 4.91, Smart fps: 4.91, pre process time ms: 38, infer time ms: 41, post process time ms: 2
[mono_pwcnet-3] [WARN] [0000000501.823021197] [mono_pwcnet]: Sub img fps: 5.00, Smart fps: 5.00, pre process time ms: 38, infer time ms: 41, post process time ms: 2
[mono_pwcnet-3] [WARN] [0000000503.023211572] [mono_pwcnet]: Sub img fps: 5.00, Smart fps: 5.00, pre process time ms: 38, infer time ms: 41, post process time ms: 2
[mono_pwcnet-3] [WARN] [0000000504.213473156] [mono_pwcnet]: Sub img fps: 5.00, Smart fps: 5.04, pre process time ms: 29, infer time ms: 41, post process time ms: 1
[mono_pwcnet-3] [WARN] [0000000505.404481615] [mono_pwcnet]: Sub img fps: 5.00, Smart fps: 5.04, pre process time ms: 39, infer time ms: 41, post process time ms: 1
[mono_pwcnet-3] [WARN] [0000000506.422719074] [mono_pwcnet]: Sub img fps: 5.00, Smart fps: 4.91, pre process time ms: 38, infer time ms: 41, post process time ms: 1
[mono_pwcnet-3] [WARN] [0000000507.422862825] [mono_pwcnet]: Sub img fps: 5.04, Smart fps: 5.00, pre process time ms: 38, infer time ms: 41, post process time ms: 1
```

在PC端的浏览器输入http://IP:8000 ,然后点击右侧的 'Full Image Segmentation' 查看渲染效果（IP为RDK设备的IP地址）

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/pwcnet.gif)
