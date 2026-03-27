---
sidebar_position: 2
---

# 5.4.2 Navigation2

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Overview

Nav2 (Navigation2) is the built-in navigation framework in ROS 2, designed to find a safe way for mobile robots to move from point A to point B. Nav2 can also be applied to other robotic navigation applications, such as dynamic waypoint tracking, which involves dynamic path planning, calculating motor velocities, obstacle avoidance, and more.

The [SLAM Mapping](./slam) section explains how to run SLAM algorithms to create a map. This section describes how to use Nav2 for navigation based on an existing map. Similarly, we use Gazebo to create a virtual environment and robot car on a PC, set the navigation goal using Rviz2, and run the Nav2 program on the RDK for navigation.

## Supported Platforms

| Platform                | Execution Method                                                                 | Example Functionality                                                                                                                                              |
| ----------------------- | -------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| RDK X3, RDK X3 Module   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)                                       | Launch the simulation environment on the PC, start navigation on the RDK, and visualize the navigation results using Rviz2                                          |
| RDK X5, RDK X5 Module   | Ubuntu 22.04 (Humble)                                                            | Launch the simulation environment on the PC, start navigation on the RDK, and visualize the navigation results using Rviz2                                          |
| RDK S100, RDK S100P     | Ubuntu 22.04 (Humble)                                                            | Launch the simulation environment on the PC, start navigation on the RDK, and visualize the navigation results using Rviz2                                          |

## Prerequisites

### RDK Platform

1. The RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 system image.

2. tros.b has been successfully installed on the RDK.

3. After successfully installing tros.b, install Nav2.

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

   ```shell
   sudo apt update 
   sudo apt install ros-foxy-navigation2
   sudo apt install ros-foxy-nav2-bringup
   ```

 </TabItem>
 <TabItem value="humble" label="Humble">

   ```shell
   sudo apt update 
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   ```

 </TabItem>
 </Tabs>

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, refer to the FAQ section [Common Issues](../../08_FAQ/01_hardware_and_system.md), specifically `Q10: How to handle failures or errors when running apt update?` for solutions.**
:::

4. A PC on the same network segment as the RDK, with Ubuntu 20.04/Ubuntu 22.04, ROS 2 Desktop, the Gazebo simulation environment, and the visualization tool Rviz2 already installed.

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

   - Ubuntu 20.04 and [ROS 2 Foxy Desktop](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - After successfully installing ROS 2 on the PC, install Gazebo and Turtlebot3-related packages as follows:

    ```bash
    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-turtlebot3*
    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup
    ```

 </TabItem>
 <TabItem value="humble" label="Humble">

   - Ubuntu 22.04 and [ROS 2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - After successfully installing ROS 2 on the PC, install Gazebo and Turtlebot3-related packages as follows:

    ```bash
    sudo apt-get install ros-humble-gazebo-*
    sudo apt install ros-humble-turtlebot3*
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    ```

 </TabItem>
 </Tabs>

## Usage Instructions

### RDK Platform

This section describes how to set up a simulation environment on the PC, configure the navigation goal, run navigation functionality on the RDK, and observe the navigation results.

1. **On the PC**, launch the Gazebo simulation environment:

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
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

   :::info
   If startup fails with the error `[ERROR] [gzclient-2]: process has died`, run the command `source /usr/share/gazebo/setup.sh` before relaunching.
   :::

   The simulation environment appears as shown below:

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/gazebo.png)

2. **On the RDK**, launch the navigation functionality:

   <Tabs groupId="tros-distro">
   <TabItem value="foxy" label="Foxy">

   ```bash
   # Configure tros.b environment
   source /opt/tros/setup.bash
   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/foxy/share/nav2_bringup/maps/turtlebot3_world.yaml
   ```

   </TabItem>

   <TabItem value="humble" label="Humble">

   ```bash
   # Configure tros.b environment
   source /opt/tros/humble/setup.bash
   ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
   ```

   </TabItem>

   </Tabs>

3. **On the PC**, launch the Rviz2 tool:

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
   ros2 launch nav2_bringup rviz_launch.py
   ```

   The Rviz2 interface appears as shown below:

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/rviz.png)

4. Set the robot's initial pose and orientation in Rviz2:

   After launching Rviz2, the robot initially does not know its location. By default, Nav2 waits for the user to provide an approximate starting position. Observe the robot’s position in Gazebo and locate it on the map. Click the "2D Pose Estimate" button in Rviz2, then click on the estimated robot position on the map to set its initial pose. Drag forward from the clicked point to define the robot’s initial heading direction, as illustrated below:

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/rviz_init.png)

   Once the initial pose is set, the transform tree is completed, and Nav2 becomes fully active and ready. At this point, you should see the robot and its point cloud data.

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/rviz_start.png)

5. Set a navigation goal in Rviz2:

   Click the "Navigation2 Goal" button and select a destination.

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/rviz_goal.png)

   You will now see the robot moving toward the goal.
## Results Analysis

The navigation performance is shown in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/nav2/rviz_nav2.gif)