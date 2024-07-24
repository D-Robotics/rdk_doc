---
sidebar_position: 1
---

# 5.4.1 SLAM

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

SLAM (Simultaneous Localization and Mapping) is a technique used to simultaneously estimate the location of a robot and create a map of its environment. In this chapter, we will use ROS2 SLAM-Toolbox to perform mapping on a simulated car in Gazebo, and observe the mapping results through Rviz2. The SLAM-Toolbox runs on the Horizon RDK, while Gazebo and Rviz2 run on a PC in the same network as the Horizon RDK.

## Supported Platforms

| Platform | System | Function |
| -------- | ---------------- | -------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start the simulation environment on the PC and perform SLAM mapping on the Horizon RDK, finally display the mapping results using Rviz2. |

## Preparation

### Horizon RDK

1. The Horizon RDK has been flashed with the  Ubuntu 20.04/22.04 image provided by Horizon.

2. TogetheROS.Bot has been successfully installed on the Horizon RDK.

3. After the successful installation of tros.b, install the SLAM-Toolbox:

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

:::caution
If the installation fails and the error is as follows:

 ```bash
   The following packages have unmet dependencies:
    ros-foxy-slam-toolbox : Depends: ros-foxy-nav2-map-server but it is not going to be installed
   E: Unable to correct problems, you have held broken packages.
 ```

Please execute the following command before installing:
 
   apt update

   sudo apt install libwebp6=0.6.1-2ubuntu0.20.04.3
:::

4. The PC, which is in the same network as the Horizon RDK, has been installed with Ubuntu 20.04, ROS2 Foxy Desktop version, Gazebo simulation environment, and the data visualization tool Rviz2.

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

   - Ubuntu 20.04 system and [ROS2 Foxy Desktop Full](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3 related packages. Installation commands:

   ```shell
    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-turtlebot3
    sudo apt install ros-foxy-turtlebot3-bringup
    sudo apt install ros-foxy-turtlebot3-simulations
    sudo apt install ros-foxy-teleop-twist-keyboard
   ```

 </TabItem>
 <TabItem value="humble" label="Humble">

   - Ubuntu 22.04 system and [ROS2 Humble Desktop Full](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3 related packages. Installation commands:

    ```shell
    sudo apt-get install ros-humble-gazebo-*
    sudo apt install ros-humble-turtlebot3
    sudo apt install ros-humble-turtlebot3-bringup
    sudo apt install ros-humble-turtlebot3-simulations
    sudo apt install ros-humble-teleop-twist-keyboard
    ```

 </TabItem>
 </Tabs>

## Usage

### Horizon RDK

This section introduces how to use Horizon RDK to run SLAM and observe mapping effect using PC.

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

The simulation environment is shown in the figure below:
![](./image/slam/gazebo.jpg)

Open another console on the PC and start Rviz2 to observe the mapping effect:

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

After opening Rviz2, the "map" visualization option needs to be added to display the built map. The steps are as follows:
![](./image/slam/rvizsetting.jpg)

Run SLAM-Toolbox on the Horizon RDK:

<Tabs groupId="tros-distro">
<TabItem value="foxy" label="Foxy">

```bash
source /opt/tros/setup.bash
```

</TabItem>

<TabItem value="humble" label="Humble">

```bash
source /opt/tros/humble/setup.bash
```

</TabItem>

</Tabs>

```bash
ros2 launch slam_toolbox online_sync_launch.py
```

Open another console on the PC and start the control tool to control the movement of the robot car with the keyboard. The control method can be found in the log printed on the console:

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

Control the robot car to move. As the robot car detects more environmental information with the radar, the SLAM algorithm also builds the environmental map, which can be observed in Rviz2.
![](./image/slam/map.jpg)

## Result Analysis

The terminal output of running on the Horizon RDK board is as follows:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-10-06-40-34-204213-ubuntu-5390
[INFO] [launch]: Default logging verbosity is set to INFO
```[INFO] [sync_slam_toolbox_node-1]: process started with pid [5392]
[sync_slam_toolbox_node-1] [INFO] [1654843239.403931058] [slam_toolbox]: Node using stack size 40000000
[sync_slam_toolbox_node-1] [INFO] [1654843240.092340814] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[sync_slam_toolbox_node-1] [INFO] [1654843240.096554433] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[sync_slam_toolbox_node-1] Info: clipped range threshold to be within minimum and maximum range!
[sync_slam_toolbox_node-1] [WARN] [1654843589.431524393] [slam_toolbox]: maximum laser range setting (20.0 m) exceeds the capabilities of the used Lidar (3.5 m)
[sync_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]