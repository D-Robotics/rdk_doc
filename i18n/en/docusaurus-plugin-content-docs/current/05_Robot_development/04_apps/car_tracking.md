---
sidebar_position: 4
---

# 5.4.4 Robot Follows the Human Body

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The app is used to control the robot to follow the movement of the human body. The app consists of MIPI image acquisition, body detection and tracking, body tracking strategy, image coding and decoding, and a web display interface. The workflow is shown in the following diagram:

![](./image/car_tracking/body_tracking_workflow.jpg)

The app is demonstrated using a virtual car in the PC-side Gazebo simulation environment, but the control commands can also be directly used to control a physical robot.

Code Repository:  `https://github.com/HorizonRDK/body_tracking>

## Supported Platforms

| Platform | System | Function                  |
| ---------| ---------------- | ------------------------------------- |
| RDK X3, RDK X3 Module  | Ubuntu 20.04  | Start MIPI/USB camera to capture images, perform body keypoints detection and body tracking, and display the tracking effect in Gazebo |

## Preparation

### Horizon RDK

1. Horizon RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on Horizon RDK.

3. MIPI or USB camera has been installed on Horizon RDK.

4. The PC used for Horizon RDK should be in the same network segment (either wired or connected to the same wireless network, with the first three parts of the IP address being consistent). The PC should have the following environment installed:

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

   - Ubuntu 20.04 system and [ROS2 Foxy Desktop Full](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3 related packages. Installation commands:

   ```shell
   sudo apt-get install ros-foxy-gazebo-*
   sudo apt install ros-foxy-turtlebot3
   sudo apt install ros-foxy-turtlebot3-simulations
   ```

 </TabItem>
 <TabItem value="humble" label="Humble">

   - Ubuntu 22.04 system and [ROS2 Humble Desktop Full](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3 related packages. Installation commands:

    ```shell
    sudo apt-get install ros-humble-gazebo-*
    sudo apt install ros-humble-turtlebot3
    sudo apt install ros-humble-turtlebot3-simulations
    ```

 </TabItem>
 </Tabs>

## Usage

### Horizon RDK

After running the app, the car motion control package will select the human body closest to the front of the car (with the largest width of the body detection box) as the tracking target. When the human body is far from the car, the car starts to move forward to approach the body and keeps it in front of the car.After the app is launched, the sensor will publish images and corresponding algorithm results, which can be rendered and displayed on the PC browser. (Enter http://IP:8000 in the browser, where IP is the IP address of the Horizon RDK).

Launch the simulation environment on the PC side:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/ros/foxy/setup.bash
```

</TabItem>
<TabItem value="humble" label="Humble">

```shell
source /opt/ros/humble/setup.bash
```

</TabItem>
</Tabs>

```shell
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

After successful launch, the car effect in the simulation environment is as follows:

![](./image/car_gesture_control/gazebo.jpeg)


**Publish images using MIPI camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

export CAM_TYPE=mipi

ros2 launch body_tracking body_tracking_without_gesture.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

export CAM_TYPE=mipi

ros2 launch body_tracking body_tracking_without_gesture.launch.py
```

</TabItem>

</Tabs>

**Publish images using USB camera**

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```shell
source /opt/tros/setup.bash

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

export CAM_TYPE=usb

ros2 launch body_tracking body_tracking_without_gesture.launch.py
```

</TabItem>

<TabItem value="humble" label="Humble">

```shell
source /opt/tros/humble/setup.bash

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

export CAM_TYPE=usb

ros2 launch body_tracking body_tracking_without_gesture.launch.py
```

</TabItem>

</Tabs>

## Result Analysis


The following information is outputted in the terminal when running on the Horizon RDK.

```text
[body_tracking-7] [WARN] [1653430533.523069034] [ParametersClass]: TrackCfg param are
[body_tracking-7] activate_wakeup_gesture: 0
[body_tracking-7] track_serial_lost_num_thr: 100
[body_tracking-7] activate_robot_rotate_thr: 45
[body_tracking-7] activate_robot_move_thr: 5
[body_tracking-7] move_step: 0.3
[body_tracking-7] rotate_step: 0.5
[body_tracking-7] img_width: 960
[body_tracking-7] img_height: 544
[body_tracking-7] 
[body_tracking-7] [WARN] [1653430533.712812076] [TrackingManager]: update frame_ts 395787, 873
[body_tracking-7] [WARN] [1653430533.713105576] [TrackingManager]: Tracking body start!, track_id: 1, frame_ts: 395787, tracking_sta(0:INITING, 1:TRACKING, 2:LOST): 1, gesture: 0
[body_tracking-7] [WARN] [1653430535.018442618] [TrackingManager]: Do move! body_rect_width: 353, thr: 864, move_step_ratio: 1, body_rect_to_top: 20, img_height: 544, move_step: 0.3
[body_tracking-7] [WARN] [1653430535.220268535] [TrackingManager]: Do rotate move, ts sec: 3397, nanosec: 387800000
[body_tracking-7] [WARN] [1653430535.220408576] [RobotCmdVelNode]: RobotCtl, angular: 0 0 0, linear: 0.3 0 0, pub twist ts: 1653430535220394
```

The above log captures a section of the output after the app is launched. Print the relevant configuration (TrackCfg param) first after startup. After detecting the human body, the car starts to enter a following state (tracking_sta value is 1) and moves forward at a speed of 0.3m/s (RobotCtl, angular: 0 0 0, linear: 0.3 0 0) to approach the human body.

Use the command `ros2 topic list`on PC,the topic is as below：

```shell
$ ros2 topic list
/camera_info
/cmd_vel
/hbmem_img04054242060426080500012020112713
/hobot_mono2d_body_detection
/image
/parameter_events
/rosout
```
Among them, `/image` is the image captured by the Horizon RDK from the MIPI sensor and encoded in JPEG format, `/hobot_mono2d_body_detection` is the algorithm message published by the Horizon RDK which contains the human body detection results, and `/cmd_vel` is the motion control command published by the Horizon RDK.

On the PC, using the `ros2 topic echo /cmd_vel` command on the terminal can view the motion control commands issued by Horizon RDK:

```shell
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5
---
linear:x: 0.5
y: 0.0
z: 0.0
angular:
x: 0.0
y: 0.0
z: -0.5
---
```

In the PC simulation environment, the car follows the movement of the human body. The simulated car movement effect is as follows:

![](./image/car_tracking/tracking.gif)