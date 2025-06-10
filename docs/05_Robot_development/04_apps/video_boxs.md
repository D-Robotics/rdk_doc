---
sidebar_position: 9
---

# 5.4.9 智能盒子

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 功能介绍

智能盒子App功能为实现IPC视频流输入后智能分析，App由RTSP视频流、视频解码、人体人脸检测、图像编码、Web展示端组成，流程如下图：

![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow.jpg)

对应实际的客户应用中显示部分由客户的业务系统完成，智能盒子的主要功能由RTSP视频流、视频解码、人体人脸检测，智能结果和图像发布给客户的业务系统，流程如下图：
![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow2.jpg)

代码仓库： (https://github.com/D-Robotics/hobot_rtsp_client.git)

## 支持平台

| 平台                    | 运行方式                  |
|-----------------------|-----------------------|
| RDK X3, RDK X3 Module | Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module, RDK S100 | Ubuntu 22.04 (Humble) |

## 准备工作

### RDK平台

1. RDK已烧录好Ubuntu 22.04系统镜像。

2. RDK已成功安装TogetheROS.Bot。

3. 准备支持传输H264/H265码流RTSP协议的IPC设备，并且配置同一网段的IP地址。

4. 和RDK在同一网段（有线或者连接同一无线网，IP地址前三段需保持一致）的PC.

5. 系统配置成性能模式

```bash
sudo bash -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
```

6.
启动多路时，ion_size配置成1G，请参考[srpi-config配置](https://developer.d-robotics.cc/rdk_doc/System_configuration/srpi-config)

## 使用介绍

### RDK平台

**启动具备显示pipeline**
<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# 配置tros.b环境
source /opt/tros/humble/setup.bash

# 设置不同的域
export ROS_DOMAIN_ID=101

# 从TogetheROS的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 启动launch文件
ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'  websocket_channel:=0
```

</TabItem>

</Tabs>


**启动component的pipeline**
<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# 配置tros.b环境
source /opt/tros/humble/setup.bash

# 设置不同的域
export ROS_DOMAIN_ID=103

# 从TogetheROS的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 启动launch文件
ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'  websocket_channel:=0
```

</TabItem>

</Tabs>


**启动非显示的pipeline**
<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# 配置tros.b环境
source /opt/tros/humble/setup.bash

# 设置不同的域
export ROS_DOMAIN_ID=105

# 从TogetheROS的安装路径中拷贝出运行示例需要的配置文件。
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# 启动launch文件
ros2 launch hobot_rtsp_client hobot_rtsp_client_ai.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'
```

</TabItem>

</Tabs>

**注意**

1. 启动一个pipeline对应一路RTSP流，局域网中启动多路码流时，需要配置ROS_DOMAIN_ID。
2. launch脚本带"_plugin"，将以component模式启动。
3. 同一个设备启动多个带"_websocket"的launch，web无法同时看多路的码流。

## 结果分析

在RDK板端运行终端输出如下信息：

```text
[hobot_codec_republish-2] [WARN] [1732169402.355433988] [hobot_codec_decoder]: Sub imgRaw fps = -1774563328
[hobot_codec_republish-2] [WARN] [1732169402.906547961] [hobot_codec_decoder]: sub h264 1920x1080, fps: 24.7706, pub nv12, fps: 9.17431, comm delay [-8.8148]ms, codec delay [171.2000]ms
[dnn_node_example-4] [WARN] [1732169402.906916796] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1080, w: 1920, step: 1920, index: 2508, stamp: 1732169402_735947000, data size: 3133440, comm delay [170.9541]ms
[hobot_codec_republish-3] [WARN] [1732169403.274412126] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 10.8055, pub jpeg, fps: 10.8055, comm delay [164.9091]ms, codec delay [7.6364]ms
[dnn_node_example-4] [WARN] [1732169403.321086039] [mono2d_body_det]: input fps: 10.81, out fps: 10.81, infer time ms: 92, post process time ms: 10
[hobot_codec_republish-2] [WARN] [1732169403.946849482] [hobot_codec_decoder]: sub h264 1920x1080, fps: 25, pub nv12, fps: 10.5769, comm delay [-7.0000]ms, codec delay [168.2727]ms

```

启另一个终端使用`ros2 topic list`命令可以查询到RDK的topic信息：

```shell
$ ros2 topic list
/hobot_dnn_detection
/image_decode
/image_mjpeg
/parameter_events
/rosout
/rtsp_image_ch_0

```

其中`/rtsp_image_ch_0`是RDK发布的通过RTSP获取IPC视频后发布的视频，`/hobot_dnn_detection`
是RDK发布的包含人体检测结果的算法msg，`//image_decode`是RDK接收H264后解码的NV12图片，`/image_mjpeg`是RDK编码后的JPEG图片。

在PC端的浏览器输入http://IP:8000 ，人体人脸检测框，关键点和姿态检测结果在web端展示渲染效果（IP为RDK的IP地址）：

![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_box_detection.jpg)
