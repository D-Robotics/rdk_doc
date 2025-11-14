---
sidebar_position: 9
---

# 5.4.9 Video Boxs

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The app is used to Intelligent Analysis of IPC video stream input . The app consists of RTSP video stream, body and face
detection, image coding and decoding, and a web display interface. The workflow is shown in the following diagram:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow.jpg)

In actual customer applications, the display part is completed by the customer's business system. The app consists of
RTSP video stream, body and face detection, image coding and decoding. The workflow is shown in the following diagram:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow2.jpg)

Code Repository:  (https://github.com/D-Robotics/hobot_rtsp_client.git)

## Supported Platforms

| Platform                      | System       | Function                                                                                                                                                     |
|-------------------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| RDK X3, RDK X3 Module, RDK X5, RDK S100 | Ubuntu 22.04 | Start RTSP Client for receive video stream , and from H264 deceoded to NV12,  perform body face keypoints detection, and display the detection effect in Web |

## Preparation

1. RDK has been flashed with the Ubuntu 22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on RDK, as[5.1.2 apt installation and upgrade](../01_quick_start/install_tros.md).


3. Get IPC devices that support RTSP stream of H264/H265，and configure IP addresses for the same network segment.

4. The PC used for RDK should be in the same network segment (either wired or connected to the same wireless network,
   with the first three parts of the IP address being consistent).

5. Setup system

```bash
sudo bash -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
``` 

6. When run more pipeline, set 1G of ion_size.refor
   to [srpi-config](https://developer.d-robotics.cc/rdk_doc/System_configuration/srpi-config)



## Usage

### multiple pipeline start

channel 1 (Terminal 1):

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID=101

cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'  websocket_channel:=0
```

</TabItem>

</Tabs>


channel 2 (Terminal 2):

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID=102

cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.58:554/0' hobot_transport_0:='udp'  websocket_channel:=1
```

</TabItem>

</Tabs>

**attention**

1. Set different ROS_DOMAIN_ID and websocket_channel for different channels.
2. The method of activating the two channels mentioned above can be used to activate other channels 3 to 8, etc., according to the relevant method.
3.  Launch scripts with "_plugin" will be launched in component mode; such as hobot_rtsp_client_ai_websocket_plugin.launch.py and hobot_rtsp_client_ai_plugin.launch.py
4.  The launch script with "_websocket" can enable web browsing; examples include hobot_rtsp_client_ai_websocketTo enhance the capacity of connection channels, it is necessary to reduce frames in multiple video streams and configure the frame rate from the IPC.

## Algorithm model switching
The default algorithm in the launch script is yolov8;


Please refer to the hobot_rtsp_client_ai_websocket_plugin.launch.py，
```shell
    ComposableNode(
        package='dnn_node_example',
        plugin='DnnExampleNode',
        name='dnn_example',
        parameters=[
            {"config_file": 'config/yolov8workconfig.json'},
            {"dump_render_img": 0},
            {"feed_type": 1},
            {"is_shared_mem_sub": 0},
            {"msg_pub_topic_name": "/hobot_dnn_detection"}
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    ) 
```
If referencing the YOLOv5 algorithm, please modify the config.FILE to "config/YOLOv5xworkconfig. json", refer to [YOLO] (../03-box/detection/YOLO. md),

## Result Analysis

The following information is outputted in the terminal when running on the RDK.

```text
[hobot_codec_republish-2] [WARN] [1732169402.355433988] [hobot_codec_decoder]: Sub imgRaw fps = -1774563328
[hobot_codec_republish-2] [WARN] [1732169402.906547961] [hobot_codec_decoder]: sub h264 1920x1080, fps: 24.7706, pub nv12, fps: 9.17431, comm delay [-8.8148]ms, codec delay [171.2000]ms
[dnn_node_example-4] [WARN] [1732169402.906916796] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1080, w: 1920, step: 1920, index: 2508, stamp: 1732169402_735947000, data size: 3133440, comm delay [170.9541]ms
[hobot_codec_republish-3] [WARN] [1732169403.274412126] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 10.8055, pub jpeg, fps: 10.8055, comm delay [164.9091]ms, codec delay [7.6364]ms
[dnn_node_example-4] [WARN] [1732169403.321086039] [mono2d_body_det]: input fps: 10.81, out fps: 10.81, infer time ms: 92, post process time ms: 10
[hobot_codec_republish-2] [WARN] [1732169403.946849482] [hobot_codec_decoder]: sub h264 1920x1080, fps: 25, pub nv12, fps: 10.5769, comm delay [-7.0000]ms, codec delay [168.2727]ms

```

Use the command `ros2 topic list`on other terminal,the topic is as below：

```shell
$ ros2 topic list
/hobot_dnn_detection
/image_decode
/image_mjpeg
/parameter_events
/rosout
/rtsp_image_ch_0

```

Among them, `/rtsp_image_ch_0` is h264/h265 stream from IPC through RTSP protocol, `/hobot_dnn_detection` is the
algorithm message published by the RDK which contains the human body and face detection results, and `/image_decode` is
from h264 decoded to NV12 image,`/image_mjpeg` is from nv12 coded to jpeg image.

In the PC's browser, enter `http://IP:8000`, and configure split screen:
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_websocket.jpg)

The body detection frame, keypoints, and pose detection results will be
displayed in the web interface (IP refers to the IP address of the RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_box_detection.jpg)