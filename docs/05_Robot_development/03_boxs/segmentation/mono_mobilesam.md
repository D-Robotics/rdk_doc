---
sidebar_position: 4
---
# MobileSAM 分割一切

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

mono_mobilesam package 是基于 Mobile SAM 量化部署的使用示例。图像数据来源于本地图片回灌和订阅到的image msg。SAM 依赖检测框输入进行分割, 并分割检测框中的目标, 无需指定目标的类别信息, 仅需提供框。最终将算法信息通过话题发布, 同时在Web页面渲染可视化。

本示例中, 我们提供了两种部署展示方式:
- 固定框分割：固定了检测框（图片中央）用以分割。
- 订阅框分割：订阅上游检测网络输出的检测框信息, 对框中的信息进行分割。

代码仓库： (https://github.com/D-Robotics/mono_mobilesam.git)

应用场景：结合检测框进行障碍物分割、水渍区域分割等。

## 支持平台

| 平台                  | 运行方式     | 示例功能                                                     |
| --------------------- | ------------ | ------------------------------------------------------------ |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | · 启动MIPI/USB摄像头/本地回灌, 推理渲染结果在Web显示/保存在本地 |

## 算法信息

| 模型 | 平台 | 输入尺寸 | 推理帧率(fps) |
| ---- | ---- | ------------ | ---- |
| mobilesam | X5 | 1×3×384×384 | 6.6 |

## 准备工作

### RDK平台

1. RDK已烧录好RDK OS系统。

2. RDK已成功安装TogetheROS.Bot。

## 使用介绍

package对外发布包含语义分割和目标检测信息的算法msg, 用户可以订阅发布msg用于应用开发。

### RDK平台

**mipi摄像头发布图片**

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
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# 配置MIPI摄像头
export CAM_TYPE=mipi

# 启动launch文件
ros2 launch mono_mobilesam sam.launch.py 
```

**使用usb摄像头发布图片**

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
# 从tros的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# 配置USB摄像头
export CAM_TYPE=usb

# 启动launch文件
ros2 launch mono_mobilesam sam.launch.py 
```

**使用单张回灌图片**

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
# 从tros的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_mobilesam/config/ .

# 配置回灌图片
export CAM_TYPE=fb

# 启动launch文件
ros2 launch mono_mobilesam sam.launch.py 
```

## 结果分析

**使用mipi摄像头发布图片**

package初始化后, 在运行终端输出如下信息：

```
[INFO] [launch]: All log files can be found below .ros/log/1970-01-02-22-39-09-001251-buildroot-22955
[INFO] [hobot_codec_republish-2]: process started with pid [22973]
[INFO] [mono_mobilesam-3]: process started with pid [22975]
[INFO] [websocket-4]: process started with pid [22977]
[hobot_codec_republish-2] [WARN] [0000167949.975123376] [HobotCodec]: This is HobotCodecNode: hobot_codec_22973.
[hobot_codec_republish-2] [WARN] [0000167950.040208542] [HobotCodecNode]: Parameters:
[hobot_codec_republish-2] sub_topic: /image
[hobot_codec_republish-2] pub_topic: /hbmem_img
[hobot_codec_republish-2] channel: 1
[hobot_codec_republish-2] in_mode: ros
[hobot_codec_republish-2] out_mode: shared_mem
[hobot_codec_republish-2] in_format: jpeg
[hobot_codec_republish-2] out_format: nv12
[hobot_codec_republish-2] enc_qp: 10
[hobot_codec_republish-2] jpg_quality: 60
[hobot_codec_republish-2] input_framerate: 30
[hobot_codec_republish-2] output_framerate: -1
[hobot_codec_republish-2] dump_output: 0
[hobot_codec_republish-2] [WARN] [0000167950.050887417] [HobotCodecImpl]: platform x5
[websocket-4] [WARN] [0000167950.068235417] [websocket]:
[websocket-4] Parameter:
[websocket-4]  image_topic: /image
[websocket-4]  image_type: mjpeg
[websocket-4]  only_show_image: 0
[websocket-4]  smart_topic: hobot_sam
[websocket-4]  output_fps: 0
[mono_mobilesam-3] [WARN] [0000167950.510756918] [mono_mobilesam]: Parameter:
[mono_mobilesam-3]  cache_len_limit: 8
[mono_mobilesam-3]  dump_render_img: 0
[mono_mobilesam-3]  feed_type(0:local, 1:sub): 1
[mono_mobilesam-3]  image: config/00131.jpg
[mono_mobilesam-3]  is_regular_box: 1
[mono_mobilesam-3]  is_shared_mem_sub: 1
[mono_mobilesam-3]  is_sync_mode: 0
[mono_mobilesam-3]  ai_msg_pub_topic_name: /hobot_sam
[mono_mobilesam-3]  ai_msg_sub_topic_name: /hobot_dnn_detection
[mono_mobilesam-3]  ros_img_sub_topic_name: /image
[mono_mobilesam-3] [BPU_PLAT]BPU Platform Version(1.3.6)!
[mono_mobilesam-3] [HBRT] set log level as 0. version = 3.15.52.0
[mono_mobilesam-3] [DNN] Runtime version = 1.23.9_(3.15.52 HBRT)
[mono_mobilesam-3] [A][DNN][packed_model.cpp:247][Model](1970-01-02,22:39:10.889.592) [HorizonRT] The model builder version = 1.23.5
[mono_mobilesam-3] [W][DNN]bpu_model_info.cpp:491][Version](1970-01-02,22:39:11.25.90) Model: mobilesam_encoder_384_all_BPU. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[mono_mobilesam-3] [A][DNN][packed_model.cpp:247][Model](1970-01-02,22:39:11.239.603) [HorizonRT] The model builder version = 1.23.5
[mono_mobilesam-3] [WARN] [0000167951.353811293] [mono_mobilesam]: Create hbmem_subscription with topic_name: /hbmem_img
[mono_mobilesam-3] [W][DNN]bpu_model_info.cpp:491][Version](1970-01-02,22:39:11.318.569) Model: mobilesam_decoder_384. Inconsistency between the hbrt library version 3.15.52.0 and the model build version 3.15.47.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[mono_mobilesam-3] [WARN] [0000167951.606431085] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 43, infer time ms: 152, post process time ms: 24
[mono_mobilesam-3] [WARN] [0000167951.779821293] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 36, infer time ms: 149, post process time ms: 21
[mono_mobilesam-3] [WARN] [0000167951.952713293] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 36, infer time ms: 150, post process time ms: 22
[mono_mobilesam-3] [WARN] [0000167952.123928377] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 37, infer time ms: 149, post process time ms: 21
[mono_mobilesam-3] [WARN] [0000167952.295540585] [mono_mobilesam]: Smart fps: 5.00, pre process time ms: 35, infer time ms: 150, post process time ms: 21
```

示例中推理的结果会渲染到Web上, 在PC端的浏览器输入http://IP:8000 即可查看图像和算法渲染效果（IP为RDK的IP地址）, 打开界面右上角设置, 选中”全图分割“选项, 可以显示渲染效果。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_sam.png)

## 进阶使用

如需调整检测框大小, 可参考下面方法验证。更重要的是可以通过上游检测节点检测结果作为sam输入。

运行sam, 取消固定框模式 sam_is_regular_box:=0
```shell
ros2 launch mono_mobilesam sam.launch.py sam_is_regular_box:=0
```

在另一个终端发布ai话题。
```shell
ros2 topic pub /hobot_dnn_detection ai_msgs/msg/PerceptionTargets '{"targets": [{"rois": [{"rect": {"x_offset": 96, "y_offset": 96, "width": 192, "height": 96}, "type": "anything"}]}] }'
```

说明：这里发布的话题名为"/hobot_dnn_detection", 检测框坐标起点(96, 96), 宽度192, 高度96。这里检测框起止点, 应该不超过输入图像的大小, 在实际使用中需留意。