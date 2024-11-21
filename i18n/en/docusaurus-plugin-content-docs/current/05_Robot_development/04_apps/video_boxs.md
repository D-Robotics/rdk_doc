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

![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow.jpg)

In actual customer applications, the display part is completed by the customer's business system. The app consists of
RTSP video stream, body and face detection, image coding and decoding. The workflow is shown in the following diagram:

![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow2.jpg)

Code Repository:  (https://github.com/D-Robotics/hobot_rtsp_client.git)

## Supported Platforms

| Platform                      | System       | Function                                                                                                                                                     |
|-------------------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| RDK X3, RDK X3 Module, RDK X5 | Ubuntu 22.04 | Start RTSP Client for receive video stream , and from H264 deceoded to NV12,  perform body face keypoints detection, and display the detection effect in Web |

## Preparation

### RDK

1. RDK has been flashed with the Ubuntu 22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on RDK.


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

### RDK

**Start Pipeline with web**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID=101

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'
```

</TabItem>

</Tabs>

**Start Pipeline of component**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID=103

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'
```

</TabItem>

</Tabs>


**Start Pipeline without web**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

export ROS_DOMAIN_ID=105

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

ros2 launch hobot_rtsp_client hobot_rtsp_client_ai.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'
```

</TabItem>

</Tabs>


**attention**

1. ROS_SOMAIN-ID must be configured.when start multiple pipeline in a local area network.
2. The launch with "_plugin" is component mode.
3. When start multiple pipeline with "_websocket", Don't view multiple channel stream on web.

## Result Analysis

The following information is outputted in the terminal when running on the RDK.

```text
[hobot_codec_republish-2] [WARN] [1732169402.355433988] [hobot_codec_decoder]: Sub imgRaw fps = -1774563328
[hobot_codec_republish-2] [WARN] [1732169402.906547961] [hobot_codec_decoder]: sub h264 1920x1080, fps: 24.7706, pub nv12, fps: 9.17431, comm delay [-8.8148]ms, codec delay [171.2000]ms
[mono2d_body_detection-4] [WARN] [1732169402.906916796] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1080, w: 1920, step: 1920, index: 2508, stamp: 1732169402_735947000, data size: 3133440, comm delay [170.9541]ms
[hobot_codec_republish-3] [WARN] [1732169403.274412126] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 10.8055, pub jpeg, fps: 10.8055, comm delay [164.9091]ms, codec delay [7.6364]ms
[mono2d_body_detection-4] [WARN] [1732169403.321086039] [mono2d_body_det]: input fps: 10.81, out fps: 10.81, infer time ms: 92, post process time ms: 10
[hobot_codec_republish-2] [WARN] [1732169403.946849482] [hobot_codec_decoder]: sub h264 1920x1080, fps: 25, pub nv12, fps: 10.5769, comm delay [-7.0000]ms, codec delay [168.2727]ms

```

Use the command `ros2 topic list`on other terminal,the topic is as below：

```shell
$ ros2 topic list
/hobot_mono2d_body_detection
/image_decode
/image_mjpeg
/parameter_events
/rosout
/rtsp_image_ch_0

```

Among them, `/rtsp_image_ch_0` is h264/h265 stream from IPC through RTSP protocol, `/hobot_mono2d_body_detection` is the
algorithm message published by the RDK which contains the human body and face detection results, and `/image_decode` is
from h264 decoded to NV12 image,`/image_mjpeg` is from nv12 coded to jpeg image.

In the PC's browser, enter `http://IP:8000`, and the body detection frame, keypoints, and pose detection results will be
displayed in the web interface (IP refers to the IP address of the RDK):

![](/../static/img/05_Robot_development/04_apps/image/video_boxs/video_box_detection.jpg)