---
sidebar_position: 4
---
# Visual Inertial Odometry Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Overview

Visual Inertial Odometry (VIO) is an algorithm that fuses data from cameras and Inertial Measurement Units (IMUs) to achieve robot localization. VIO offers advantages such as low cost and broad environmental applicability, effectively compensating for GNSS signal degradation scenarios like occlusion and multipath interference in outdoor environments. A robust and high-performance VIO algorithm is key to achieving high-precision outdoor navigation and localization.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/hobot_vio_rviz.jpeg)

Code repository: (https://github.com/D-Robotics/hobot_vio.git)

## Supported Platforms

| Platform                  | Runtime Environment                        | Example Functionality                                                                                                               |
| ------------------------- | ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- |
| RDK X3, RDK X3 Module     | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Uses RealSense camera images and IMU data as algorithm inputs; outputs the robot’s trajectory, which can be visualized in rviz2 on a PC |
| RDK X5, RDK X5 Module     | Ubuntu 22.04 (Humble)                      | Uses RealSense camera images and IMU data as algorithm inputs; outputs the robot’s trajectory, which can be visualized in rviz2 on a PC |
| RDK Ultra                 | Ubuntu 20.04 (Foxy)                        | Uses RealSense camera images and IMU data as algorithm inputs; outputs the robot’s trajectory, which can be visualized in rviz2 on a PC |

## Prerequisites

1. The RDK has been flashed with either Ubuntu 20.04 or Ubuntu 22.04 system image.
2. TogetheROS.Bot and the RealSense ROS2 package have been successfully installed on the RDK.
3. A RealSense camera is connected to the RDK's USB 3.0 port.
4. Ensure your PC can access the RDK over the network.

## Usage Instructions

The algorithm subscribes to image and IMU data from the RealSense camera as inputs, computes the camera trajectory, and publishes the resulting motion trajectory via ROS2 topics. The trajectory can be visualized using rviz2 on a PC. The input and output topics are listed below:

### Input Topics

| Parameter Name | Type          | Description                                                                                                                              | Required | Default Value                                                                                   |
| -------------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------- | -------- | ----------------------------------------------------------------------------------------------- |
| path_config    | std::string   | Path to the VIO algorithm configuration file                                                                                             | Yes      | `/opt/tros/${TROS_DISTRO}/lib/hobot_vio/config/realsenseD435i.yaml`                              |
| image_topic    | std::string   | Topic name for image data subscribed by the VIO algorithm                                                                                | Yes      | /camera/infra1/image_rect_raw                                                                   |
| imu_topic      | std::string   | Topic name for IMU data subscribed by the VIO algorithm                                                                                  | Yes      | /camera/imu                                                                                     |
| sample_gap     | std::string   | Processing frequency of the VIO algorithm: 1 means every frame is used for trajectory calculation, 2 means every second frame, etc.     | Yes      | 2                                                                                               |

### Output Topic

| Topic Name                    | Type                | Description                       |
| ----------------------------- | ------------------- | --------------------------------- |
| horizon_vio/horizon_vio_path  | nav_msgs::msg::Path | Robot motion trajectory output by the VIO algorithm |

Launch command:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure the tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure the tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```shell
ros2 launch hobot_vio hobot_vio.launch.py 
```

## Result Analysis

After launching the algorithm example on the RDK, the terminal displays the following logs. First, the RealSense node starts publishing image and IMU data. Then, the algorithm enters its initialization phase, during which the user must translate the camera to complete initialization. Once initialized, the algorithm begins outputting localization coordinates:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-07-19-48-31-464088-ubuntu-562910
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_vio-1]: process started with pid [563077]
[INFO] [ros2 launch realsense2_camera rs_launch.py  depth_module.profile:=640x480x30 enable_depth:=false enable_color:=false enable_gyro:=true enable_accel:=true enable_sync:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2 enable_infra1:=true-2]: process started with pid [563081]
[hobot_vio-1] T_CtoI:
[hobot_vio-1]    0.999934   0.0103587   0.0049969   0.0270761
[hobot_vio-1]  -0.0104067    0.999899  0.00967935 -0.00272628
[hobot_vio-1] -0.00489613 -0.00973072    0.999941  -0.0518149
[hobot_vio-1]           0           0           0           1
[hobot_vio-1] system use_rtk_: 0
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] not enough imu readings
[hobot_vio-1] [static initializer] IMU belows th 0.011508, 0.00274453 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.0105996, 0.00273085 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00964632, 0.00280866 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00892132, 0.00279346 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00816016, 0.00281761 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00776753, 0.00277049 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.00744219, 0.00274874 < 0.5, 0
[hobot_vio-1] [static initializer] IMU belows th 0.420251, 0.36058 < 0.5, 0
[hobot_vio-1] HorizonVIO Successfully initialized!
[hobot_vio-1] [WARN] [1688730518.534178615] [horizon_vio_node]: Localization position[x, y, z]: [0.0225533, -0.0504654, 0.00943574]
[hobot_vio-1] [WARN] [1688730518.534634139] [horizon_vio_node]: Image time 1688730518.314490318
[hobot_vio-1] [WARN] [1688730518.621440869] [horizon_vio_node]: Localization position[x, y, z]: [0.0231779, -0.0533648, 0.00787081]
[hobot_vio-1] [WARN] [1688730518.621558739] [horizon_vio_node]: Image time 1688730518.380982161
[hobot_vio-1] [WARN] [1688730518.743525086] [horizon_vio_node]: Localization position[x, y, z]: [0.0290396, -0.0610474, 0.0106718]
[hobot_vio-1] [WARN] [1688730518.743637249] [horizon_vio_node]: Image time 1688730518.447472572
[hobot_vio-1] [WARN] [1688730518.866076119] [horizon_vio_node]: Localization position[x, y, z]: [0.0381324, -0.0737757, 0.0164843]
[hobot_vio-1] [WARN] [1688730518.866186156] [horizon_vio_node]: Image time 1688730518.513962030
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 338
[hobot_vio-1] 132.853 ms all consumed
[hobot_vio-1] travel(m): 0.000
[hobot_vio-1] [WARN] [1688730519.002002975] [horizon_vio_node]: Localization position[x, y, z]: [0.05018, -0.088422, 0.0240244]
[hobot_vio-1] [WARN] [1688730519.002130095] [horizon_vio_node]: Image time 1688730518.580449104
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 31
[hobot_vio-1] 142.996 ms all consumed
[hobot_vio-1] travel(m): 0.014
[hobot_vio-1] [WARN] [1688730519.146149433] [horizon_vio_node]: Localization position[x, y, z]: [0.0167176, -0.0189649, 0.0588413]
[hobot_vio-1] [WARN] [1688730519.146279428] [horizon_vio_node]: Image time 1688730518.646935701
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 26
[hobot_vio-1] 96.911 ms all consumed
[hobot_vio-1] travel(m): 0.025
[hobot_vio-1] [WARN] [1688730519.244168068] [horizon_vio_node]: Localization position[x, y, z]: [0.000805884, 0.0134815, 0.0730707]
[hobot_vio-1] [WARN] [1688730519.244270439] [horizon_vio_node]: Image time 1688730518.713421583
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 23
[hobot_vio-1] 52.470 ms all consumed
[hobot_vio-1] travel(m): 0.034
[hobot_vio-1] [WARN] [1688730519.297642444] [horizon_vio_node]: Localization position[x, y, z]: [0.00226324, 0.0120054, 0.0796328]
[hobot_vio-1] [WARN] [1688730519.297738190] [horizon_vio_node]: Image time 1688730518.779906034
[hobot_vio-1] SLAM feats: 0
[hobot_vio-1] KF feats: 33
[hobot_vio-1] 47.407 ms all consumed
[hobot_vio-1] travel(m): 0.042
```