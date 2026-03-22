---
sidebar_position: 6
---

# 5.4.6 Voice-Controlled Car Movement

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Feature Introduction

The voice-controlled car movement feature enables controlling the robot to move forward, backward, left, or right through voice commands. This functionality requires integration with the D-Robotics RDK robotic operating system's intelligent voice module. The workflow is illustrated in the figure below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/car_audio_tracking/audio_control.jpg)

This application uses a simulated virtual car in the Gazebo environment on a PC as an example. The published control commands can also be directly applied to physical robots.

Code repository: (https://github.com/D-Robotics/audio_control.git)

## Supported Platforms

| Platform | Runtime Environment | Example Functionality |
| -------- | ------------------- | --------------------- |
| RDK X3 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Launch the intelligent voice module to parse voice commands and perform voice control; demonstrate control effects via Gazebo |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Launch the intelligent voice module to parse voice commands and perform voice control; demonstrate control effects via Gazebo |

**Note: Only RDK X3 is supported; RDK X3 Module is currently not supported.**

## Prerequisites

### RDK Platform

1. RDK has been flashed with Ubuntu 20.04 or Ubuntu 22.04 image.

2. TogetheROS.Bot has been successfully installed on RDK.

3. The intelligent voice algorithm package has been successfully installed on RDK. Installation commands:

   <Tabs groupId="tros-distro">
   <TabItem value="foxy" label="Foxy">

   ```bash
   sudo apt update
   sudo apt install tros-hobot-audio
   ```

   </TabItem>
   <TabItem value="humble" label="Humble">

   ```bash
   sudo apt update
   sudo apt install tros-humble-hobot-audio
   ```

   </TabItem>
   </Tabs>

:::caution **Note**
**If the `sudo apt update` command fails or returns an error, refer to the FAQ section [Common Issues](../../08_FAQ/01_hardware_and_system.md), specifically `Q10: How to resolve issues when apt update fails or reports errors?`**
:::

4. A compatible audio board has been properly connected to the RDK (refer to the [Intelligent Voice chapter](../03_boxs/audio/hobot_audio.md)).

5. A PC on the same network segment as the RDK (either wired or connected to the same Wi-Fi network; the first three segments of the IP addresses must match). The following environments must be installed on the PC:

 <Tabs groupId="tros-distro">
 <TabItem value="foxy" label="Foxy">

   - Ubuntu 20.04 system and [ROS2 Foxy Desktop](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3-related packages. Installation commands:

    ```shell
    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-turtlebot3
    sudo apt install ros-foxy-turtlebot3-simulations
    ```

 </TabItem>
 <TabItem value="humble" label="Humble">

   - Ubuntu 22.04 system and [ROS2 Humble Desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   - Gazebo and Turtlebot3-related packages. Installation commands:

    ```shell
    sudo apt-get install ros-humble-gazebo-*
    sudo apt install ros-humble-turtlebot3
    sudo apt install ros-humble-turtlebot3-simulations
    ```

 </TabItem>
 </Tabs>

## Usage Instructions

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

```shell
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

After successful launch, the simulated car appears as follows in the Gazebo environment:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/car_tracking/gazebo.jpeg)

Launch the program on the RDK platform:

1. Copy the audio configuration files

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
    # Copy required configuration files for the example from the tros.b installation path.
    cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_audio/config/ .
    ```

2. Verify the microphone device

    The microphone device ID is configured via the `micphone_name` field in the configuration file *config/audio_config.json*. The default value is "hw:0,0", indicating audio device Card 0, Device 0. You can check available devices using the command `ls /dev/snd`, e.g., "pcmC0D1c": the trailing "c" denotes a capture device, "C0" indicates Card 0, and "D1" indicates Device 1. In this case, set the parameter to "hw:0,1".

3. Launch the program

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
    # Launch the launch file
    ros2 launch audio_control audio_control.launch.py
    ```

    After successful startup, you can control the car’s movement using voice commands such as “Move forward,” “Move backward,” “Turn left,” “Turn right,” and “Stop moving.”

## Result Analysis

The RDK terminal outputs the following information upon execution:

```shell
        This is audio control package.

============================================
        audio control usage

Wake up device is "D-Robotics Hello".
Audio control command word definitions are:
        "Move forward": move front.
        "Move backward": move back.
"Turn right": rotate robot to right.  
        "Turn left": rotate robot to left.  
============================================

```

The above log captures a segment of output after launching the audio control package. The log shows that the wake-up word configured for this voice control module is "D-Robotics hello," and the command words for controlling the robot car's movement include: "Move forward," "Move backward," "Turn left," and "Turn right."

On the PC side, you can use the terminal command `ros2 topic list` to query the RDK's topic information:

```shell
$ ros2 topic list
/audio_smart
/cmd_vel
```

Here, `/audio_smart` is the algorithm message published by X3 containing smart voice recognition results, and `/cmd_vel` is the motion control command published by the RDK.

On the PC side, you can use the terminal command `ros2 topic echo /cmd_vel` to view the motion control commands published by the RDK:

```shell
linear:
  x: 0.30000001192092896
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 0.0
  y: -0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.5
---
linear:
  x: 0.0
  y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5
---
```

In the PC-side simulation environment, the robot car moves according to the voice control commands. The simulated robot car's motion is shown below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/04_apps/image/car_audio_control/move.gif)