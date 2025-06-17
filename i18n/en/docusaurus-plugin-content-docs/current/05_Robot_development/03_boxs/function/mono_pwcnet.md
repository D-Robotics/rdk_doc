---
sidebar_position: 15
---
# Optical Flow Estimation

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

PwcNet is an optical flow estimation model trained on the [FlyingChairs](https://lmb.informatik.uni-freiburg.de/resources/datasets/FlyingChairs.en.html) dataset.

PwcNet requires two consecutive frames as input and calculates the optical flow of the first frame, displaying the motion vectors of objects in both horizontal and vertical directions.


Code Repository: (https://github.com/D-Robotics/mono_pwcnet)

Application Scenarios: Optical flow prediction is a technique used to determine the pattern of pixel movement on the surface of objects in image sequences, and it can be applied in fields such as autonomous driving, motion analysis, and object tracking.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/render_pwcnet_feedback_0_0.jpeg)

## Supported Platforms

| Platform                            | System | Function                                     |
| ----------------------------------- | -------------- | -------------------------------------------------------- |
| RDK X5 | Ubuntu 22.04 (Humble) | Start MIPI/USB camera/local video and display inference rendering results via web      |

## Preparation

### RDK

1. RDK has flashed the  Ubuntu 22.04 system image provided by D-Robotics.

2. RDK has successfully installed TogetheROS.Bot.

3. RDK has installed the MIPI or USB camera.

4. Confirm that the PC is able to access the RDK via the network.


## Usage

Optical flow estimation (mono_pwcnet) package subscribes to images published by the sensor package, performs inference, and publishes algorithm messages. The websocket package is used to render and display the images and corresponding algorithm results on a PC browser.


### RDK

**Use MIPI Camera to Publish Images**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configuring MIPI camera
export CAM_TYPE=mipi

# Start the launch file
ros2 launch mono_pwcnet pwcnet.launch.py
```

</TabItem>

</Tabs>

**Use USB Camera to Publish Images**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell

# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configuring USB camera
export CAM_TYPE=usb

# Start the launch file
ros2 launch mono_pwcnet pwcnet.launch.py
```

</TabItem>

</Tabs>

**Use Local Image Offline**

<Tabs groupId="tros-distro">

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

```shell
# Copy the configuration file required for running the example from the installation path of tros.b.
cp -r /opt/tros/${TROS_DISTRO}/lib/mono_pwcnet/config/ .

# Configure the local playback image.
export CAM_TYPE=fb

# Start the launch file
ros2 launch mono_pwcnet pwcnet.launch.py

```

</TabItem>

</Tabs>

## Result analysis

The following information is outputted in the terminal:

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

On the PC browser, enter http://IP:8000 and then click on the 'Full Image Segmentation' to view the rendering effect of the image and algorithm.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/pwcnet.gif)