---
sidebar_position: 2
---
# Optical Flow Estimation

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The optical flow estimation algorithm uses PWC-Net trained on the [FlyingChairs dataset](https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html) to produce an optical flow estimation model.

The algorithm takes two consecutive image frames as input and outputs an optical flow map for the first frame, illustrating motion vectors of objects in the horizontal and vertical directions within the first frame.

Code repository: (https://github.com/D-Robotics/mono_pwcnet)

Application scenarios: Optical flow estimation is a technique used to determine pixel movement patterns on object surfaces in image sequences. It can be applied in fields such as autonomous driving, motion analysis, and object tracking.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_pwcnet_feedback_0_0.jpeg)

## Supported Platforms

| Platform                         | Runtime Environment | Example Functionality                                           |
| -------------------------------- | ------------------- | --------------------------------------------------------------- |
| RDK X5, RDK X5 Module            | Ubuntu 22.04 (Humble) | Launch MIPI/USB camera or local image replay, and render inference results via web browser |

## Algorithm Information

| Model   | Platform | Input Size      | Inference FPS |
| ------- | -------- | --------------- | ------------- |
| pwcnet  | X5       | 1×6×384×512     | 23            |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with the Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on the RDK.
3. An MIPI or USB camera has been installed on the RDK.
4. Ensure your PC can access the RDK over the network.

## Usage Guide

The optical flow estimation (`mono_pwcnet`) package subscribes to images published by the sensor package, performs inference, publishes algorithm messages, and renders both the original sensor images and corresponding algorithm results in a web browser on the PC via the websocket package.

### RDK Platform

**Publishing Images Using an MIPI Camera**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Launch the launch file
ros2 launch mono_pwcnet pwcnet.launch.py
```
</TabItem>

</Tabs>

**Publishing Images Using a USB Camera**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configure USB camera
export CAM_TYPE=usb

# Launch the launch file
ros2 launch mono_pwcnet pwcnet.launch.py
```

</TabItem>

</Tabs>

**Replaying Local Images**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```shell
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy required configuration files for running the example from the tros.b installation path.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configure local image replay
export CAM_TYPE=fb

# Launch the launch file
ros2 launch mono_pwcnet pwcnet.launch.py

```
</TabItem>

</Tabs>

## Result Analysis

The following output appears in the terminal during execution:

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

On your PC's web browser, navigate to `http://IP:8000`, then click **'Full Image Segmentation'** on the right side to view the rendered results (replace `IP` with the actual IP address of your RDK device).

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/pwcnet.gif)