---
sidebar_position: 6
---

# 5.4.6 Voice Control The Car

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The function of voice-controlled car movement allows users to control the robot's movement forward, backward, left, and right using chinese voice commands. The process is as shown in the following diagram:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_control/audio_control.jpg)

The app uses a virtual car in the PC Gazebo simulation environment as an example, but the control commands can also be directly used to control a physical robot.

Code repository:  (https://github.com/D-Robotics/audio_control.git)

## Supported Platforms

| Platform | System | Function                    |
| -------- | -------------- | ----------------------------------- |
| RDK X3   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)   | Start smart voice module, parse voice information, and control the car in Gazebo |

**Note: Only RDK X3 is supported, RDK X3 Module is not supported.**

## Preparation

### RDK

1. The RDK is flashed with the  Ubuntu 20.04/22.04 image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. The smart voice algorithm package has been successfully installed on the RDK. Installation command: 
   
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

4. The compatible audio board has been successfully connected to the RDK (refer to the [Smart Voice section](/i18n/en/docusaurus-plugin-content-docs/current/05_Robot_development/03_boxs/function/hobot_audio.md) for more details).

5. The PC is on the same network (either wired or connected to the same Wi-Fi network) as the RDK. The PC-side environment package requirements include:

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

After successful launch, the simulation environment shows the following effect of the car:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/gazebo.jpeg)

RDK startup program:

1. Copy the audio configuration file

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

    ```shell
    cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_audio/config/ .
    ```


2. Check the microphone device

    The microphone device number is set in the configuration file *config/audio_config.json* with the `micphone_name` field. The default is "hw:0,0", which represents audio device Card0 Device0. The device number can be checked by the command `ls /dev/snd`, for example: "pcmC0D1c". The letter 'c' represents the capture device, 'C0' represents Card0, and 'D1' represents Device1. Modify the parameter to "hw:0,1".

3. Start the program

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

    ```shell
    ros2 launch audio_control audio_control.launch.py
    ```

    After the program is successfully started, you can control the car's movement using chinese commands such as “向前走”、“向后退”、“向左转”，“向右转” and “停止运动”.

## Result Analysis

The following information is output in the RDK running terminal:

```shell
        This is audio control package.

============================================
        audio control usage

Wake up device is "Hello D-Robotics".
Audio control command word definitions are:
        "go forward": move front.
        "go backward": move back.
```

The above log snippet captures the output from the audio control package after its launch. The log content indicates that the wake-up word configured for this voice control module is "Hello D-Robotics", and the chinese command words for controlling the movement of the robot are: “向前走”、“向后退”、“向左转”，“向右转”.

On the PC, you can use the `ros2 topic list` command in the terminal to query the topic information of the RDK:

```shell
$ ros2 topic list
/audio_smart
/cmd_vel
```

Among them, `/audio_smart` is the topic published by X3 that contains the algorithm message for intelligent voice results, and `/cmd_vel` is the topic published by RDK for motion control commands.

On the PC side, you can use the `ros2 topic echo /cmd_vel` command in the terminal to view the motion control commands published by RDK:

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

The simulation car on the PC follows the instructions of voice control commands to move. The motion effect of the simulated car is as follows:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_control/move.gif)