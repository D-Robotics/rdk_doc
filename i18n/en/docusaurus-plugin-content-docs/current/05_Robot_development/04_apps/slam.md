---
sidebar_position: 1
---

# 5.4.1 SLAM Mapping

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

SLAM stands for Simultaneous Localization and Mapping.  
This section uses ROS2's SLAM-Toolbox as the mapping algorithm. We control a robot car in Gazebo to build a map and observe the mapping results via Rviz2.  
SLAM-Toolbox runs on the RDK, while Gazebo and Rviz2 run on a PC connected to the same network segment as the RDK.

## Supported Platforms

| Platform                          | Runtime Environment                     |
| --------------------------------- | --------------------------------------- |
| RDK X3, RDK X3 Module             | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |
| RDK X5, RDK X5 Module             | Ubuntu 22.04 (Humble)                   |
| RDK S100, RDK S100P               | Ubuntu 22.04 (Humble)                   |
| RDK Ultra                         | Ubuntu 20.04 (Foxy)                     |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with either Ubuntu 20.04 or Ubuntu 22.04 system image.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. After successful installation of tros.b, install SLAM-Toolbox:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

    ```bash
    sudo apt-get install ros-foxy-slam-toolbox
    ```

</TabItem>
<TabItem value="humble" label="Humble">

    ```bash
    sudo apt-get install ros-humble-slam-toolbox
    ```

</TabItem>
</Tabs>

:::info
If the installation fails with an error like:

```bash
  The following packages have unmet dependencies:
   ros-foxy-slam-toolbox : Depends: ros-foxy-nav2-map-server but it is not going to be installed
  E: Unable to correct problems, you have held broken packages.
```

Please run the following commands before attempting installation again:

```bash
apt update
sudo apt install libwebp6=0.6.1-2ubuntu0.20.04.3
```
:::

:::caution **Note**
**If the `sudo apt update` command fails or reports errors, please refer to the FAQ section [Common Issues](/docs/08_FAQ/01_hardware_and_system.md), specifically `Q10: How to resolve failures or errors when running apt update?`**
:::

4. On a PC within the same network segment as the RDK, ensure that Ubuntu 20.04/Ubuntu 22.04, ROS2 Desktop, the Gazebo simulation environment, and the visualization tool Rviz2 are already installed.

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

   - Ubuntu 20.04 and [ROS2 Foxy Desktop](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - After successfully installing ROS2 on the PC, install Gazebo and Turtlebot3-related packages as follows:

    ```bash
    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-turtlebot3
    sudo apt install ros-foxy-turtlebot3-bringup
    sudo apt install ros-foxy-turtlebot3-simulations
    sudo apt install ros-foxy-teleop-twist-keyboard
    ```

</TabItem>
<TabItem value="humble" label="Humble">

   - Ubuntu 22.04 and [ROS2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - After successfully installing ROS2 on the PC, install Gazebo and Turtlebot3-related packages as follows:

    ```bash
    sudo apt-get install ros-humble-gazebo-*
    sudo apt install ros-humble-turtlebot3
    sudo apt install ros-humble-turtlebot3-bringup
    sudo apt install ros-humble-turtlebot3-simulations
    sudo apt install ros-humble-teleop-twist-keyboard
    ```

</TabItem>
</Tabs>

## Usage Instructions

### RDK Platform

This section describes how to run the SLAM algorithm on the RDK and observe the mapping results on the PC.

Start the simulation environment on the PC:

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

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

:::info
If startup fails with the error `[ERROR] [gzclient-2]: process has died`, please run `source /usr/share/gazebo/setup.sh` before launching again.
:::

The simulation environment appears as shown below:  
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/slam/gazebo.jpg)

Open another terminal on the PC and launch Rviz2 to visualize the mapping process:

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

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

After opening Rviz2, add the "Map" visualization display to show the generated map. Follow the steps illustrated below:  
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/slam/rvizsetting.jpg)

Run SLAM-Toolbox on the RDK:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
# Configure tros.b environment
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
# Configure tros.b environment
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```bash
# Launch SLAM launch file
ros2 launch slam_toolbox online_sync_launch.py
```

Open another terminal on the PC and launch the teleoperation tool to control the robot using your keyboard. Refer to the instructions printed in the terminal for control details—these will not be repeated here:

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

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Control the robot's movement. As the robot's LiDAR detects more environmental information, the SLAM algorithm simultaneously constructs a map of the environment, which can be observed in Rviz2.  
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/06_Application_case/amr/map.jpg)

## Result Analysis

The terminal output when running on the RDK board is as follows:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-10-06-40-34-204213-ubuntu-5390
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [sync_slam_toolbox_node-1]: process started with pid [5392]
[sync_slam_toolbox_node-1] [INFO] [1654843239.403931058] [slam_toolbox]: Node using stack size 40000000
[sync_slam_toolbox_node-1] [INFO] [1654843240.092340814] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[sync_slam_toolbox_node-1] [INFO] [1654843240.096554433] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[sync_slam_toolbox_node-1] Info: clipped range threshold to be within minimum and maximum range!
[sync_slam_toolbox_node-1] [WARN] [1654843589.431524393] [slam_toolbox]: maximum laser range setting (20.0 m) exceeds the capabilities of the used Lidar (3.5 m)
[sync_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]
```