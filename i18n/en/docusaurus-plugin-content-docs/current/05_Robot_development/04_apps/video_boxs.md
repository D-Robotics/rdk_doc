---
sidebar_position: 9
---

# 5.4.9 Smart Box

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Overview

The Smart Box app implements intelligent analysis on incoming IPC video streams. The app consists of the following components: RTSP video stream input, video decoding, human body and face detection, image encoding, and a web-based display interface. The workflow is illustrated in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow.jpg)

In actual customer deployments, the display component is handled by the customer’s own business system. The core functionalities of the Smart Box—RTSP video stream reception, video decoding, human body and face detection—are performed locally, and the resulting AI inference data along with processed images are published to the customer’s business system. The workflow is shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_workflow2.jpg)

Code repository: (https://github.com/D-Robotics/hobot_rtsp_client.git)

## Supported Platforms

| Platform                  | Runtime Environment       |
|--------------------------|---------------------------|
| RDK X3, RDK X3 Module    | Ubuntu 22.04 (Humble)     |
| RDK X5, RDK X5 Module    | Ubuntu 22.04 (Humble)     |
| RDK S100, RDK S100P      | Ubuntu 22.04 (Humble)     |

## Prerequisites

1. The RDK has been flashed with the Ubuntu 22.04 system image.

2. TogetheROS.Bot has been installed on the RDK; refer to [5.1.2 Install and Upgrade via apt](../01_quick_start/install_tros.md).

3. Prepare an IPC device that supports RTSP protocol streaming with H.264/H.265 codecs, and configure it with an IP address on the same subnet as the RDK.

4. A PC connected to the same network subnet as the RDK (either wired or on the same Wi-Fi network; the first three segments of the IP address, e.g., `192.168.1.x`, must match).

5. Set the system to performance mode:

```bash
sudo bash -c "echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor"
```

6. When running multiple channels simultaneously, set `ion_size` to 1GB. Refer to [srpi-config Configuration](https://developer.d-robotics.cc/rdk_doc/en/System_configuration/srpi-config).

## Usage Guide

### Method for Launching Multiple Channels

Channel 1 (Terminal 1):

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# Configure the TogetheROS.Bot environment
source /opt/tros/humble/setup.bash

# Set a unique ROS domain ID
export ROS_DOMAIN_ID=101

# Copy required configuration files for the demo from the TogetheROS installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# Launch the launch file
ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.57:554/0' hobot_transport_0:='udp'  websocket_channel:=0
```

</TabItem>

</Tabs>

Channel 2 (Terminal 2):

<Tabs groupId="tros-distro">
<TabItem value="humble" label="Humble">

```shell
# Configure the TogetheROS.Bot environment
source /opt/tros/humble/setup.bash

# Set a unique ROS domain ID
export ROS_DOMAIN_ID=102

# Copy required configuration files for the demo from the TogetheROS installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .

# Launch the launch file
ros2 launch hobot_rtsp_client hobot_rtsp_client_ai_websocket_plugin.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://admin:admin123@10.112.148.58:554/0' hobot_transport_0:='udp'  websocket_channel:=1
```

</TabItem>

</Tabs>

...

**Notes**  
1. Each channel must use a distinct `ROS_DOMAIN_ID` and `websocket_channel`.  
2. The example above demonstrates launching two channels; additional channels (e.g., 3–8) can be started using the same method.  
3. Launch scripts containing "_plugin" (e.g., `hobot_rtsp_client_ai_websocket_plugin.launch.py` or `hobot_rtsp_client_ai_plugin.launch.py`) start nodes in component mode.  
4. Launch scripts containing "_websocket" (e.g., `hobot_rtsp_client_ai_websocket_plugin.launch.py` or `hobot_rtsp_client_ai_websocket.launch.py`) enable web-based viewing.  
5. To support more concurrent streams, reduce the frame rate of each video stream at the IPC side.

### Switching AI Models

By default, the launch script uses the YOLOv8 model.

Refer to `hobot_rtsp_client_ai_websocket_plugin.launch.py`:

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

To switch to YOLOv5, change `config_file` to `"config/yolov5xworkconfig.json"`. See [YOLO](../03_boxs/detection/yolo.md) for details.

## Result Analysis

When running on the RDK board, the terminal outputs messages like the following:

```text
[hobot_codec_republish-2] [WARN] [1732169402.355433988] [hobot_codec_decoder]: Sub imgRaw fps = -1774563328
[hobot_codec_republish-2] [WARN] [1732169402.906547961] [hobot_codec_decoder]: sub h264 1920x1080, fps: 24.7706, pub nv12, fps: 9.17431, comm delay [-8.8148]ms, codec delay [171.2000]ms
[dnn_node_example-4] [WARN] [1732169402.906916796] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1080, w: 1920, step: 1920, index: 2508, stamp: 1732169402_735947000, data size: 3133440, comm delay [170.9541]ms
[hobot_codec_republish-3] [WARN] [1732169403.274412126] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 10.8055, pub jpeg, fps: 10.8055, comm delay [164.9091]ms, codec delay [7.6364]ms
[dnn_node_example-4] [WARN] [1732169403.321086039] [mono2d_body_det]: input fps: 10.81, out fps: 10.81, infer time ms: 92, post process time ms: 10
[hobot_codec_republish-2] [WARN] [1732169403.946849482] [hobot_codec_decoder]: sub h264 1920x1080, fps: 25, pub nv12, fps: 10.5769, comm delay [-7.0000]ms, codec delay [168.2727]ms

```

Open another terminal and run `ros2 topic list` to view available topics on the RDK:

```shell
$ export ROS_DOMAIN_ID=101
$ ros2 topic list
/hobot_dnn_detection
/image_decode
/image_mjpeg
/parameter_events
/rosout
/rtsp_image_ch_0

```

- `/rtsp_image_ch_0`: Video stream received via RTSP from the IPC and republished by the RDK.  
- `/hobot_dnn_detection`: AI inference messages containing human detection results published by the RDK.  
- `/image_decode`: NV12 images decoded from H.264 streams by the RDK.  
- `/image_mjpeg`: JPEG images encoded by the RDK.

On a PC browser, navigate to `http://<RDK_IP>:8000` to access the split-screen view:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_boxs_websocket.jpg)

Human body and face bounding boxes, keypoints, and pose estimation results are rendered and displayed in the web interface (replace `<RDK_IP>` with the actual IP address of your RDK):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/video_boxs/video_box_detection.jpg)