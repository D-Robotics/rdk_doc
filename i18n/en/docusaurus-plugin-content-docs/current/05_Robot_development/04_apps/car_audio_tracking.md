---
sidebar_position: 7
---

# 5.4.7 Robot Follows the Voice's DOA And Command

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Introduction

The voice tracking control car movement function controls the robot to rotate towards the direction of the sound source based on the DOA angle information of the sound source localization, and controls the robot to move forward. This function needs to be used together with the intelligent voice module of the D-Robotics Robot Operating System. When the user speaks the wake-up word configured by the intelligent voice recognition module to wake up the device, the voice tracking control car function will be activated. After that, when the user speaks the wake-up word or the configured command word, the intelligent voice recognition module will output the DOA angle information of the sound source. After receiving the DOA angle information, this module will control the robot to turn towards the direction of the sound source and move forward a certain distance.

The process is as follows:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/audio_control.jpg)

This app uses a virtual car in Gazebo simulation environment on the PC as an example, and the published control instructions can also be directly used to control a physical robot.

The DOA angle information of the sound source localization output by the intelligent voice function is in units of degrees, and supports two types of microphone arrays: linear and circular. The angle range of the linear microphone array is 0 degrees to 180 degrees, and the angle range of the circular microphone array is 0 degrees to 360 degrees. The relative positional relationship between the microphone angles is strongly related to the installation position of the microphones. The actual angle diagram is as follows:

Linear microphone array:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/doa_line.jpg)

Circular microphone array:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/doa_circle.jpg)

Code repository:  (https://github.com/D-Robotics/audio_tracking.git)

## Supported Platforms

| Platform | System | Function          |
| -------- | ---------------- | ------------------------- |
| RDK X3   | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)     | Start intelligent voice module, parse voice information, perform voice tracking, and display tracking results in Gazebo |

**Note: Only supports RDK X3, RDK X3 Module is not supported for now.**

## Preparation

### RDK

1. The RDK has been burned with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. The intelligent voice algorithm package has been successfully installed on the RDK. Install command:

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

4. The compatible audio board has been successfully connected to the RDK (refer to the [Intelligent Voice section](../03_boxs/function/hobot_audio.md) for details).

5. The PC that is in the same network segment as the RDK (either wired or connected to the same wireless network) needs to have the following environment packages installed:

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

### RDK

After running the voice tracking function, the voice tracking control module will receive the voice message results published by the intelligent voice function module, and parse the message. Based on the wake-up event and DOA angle information in the message, the control module will publish commands to the car to turn to a specific angle. After the car turns to the specific angle, the control module will continue to control the car to move forward a certain distance (by default, the module controls the car to move forward by 0.2 meters).

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

After successful startup, the car in the simulation environment will appear as follows:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/gazebo.jpeg)

Startup program for the RDK:

1. Copy the audio configuration file and load the audio driver

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

2. Confirm the microphone device

    The microphone device number is set through the `microphone_name` field in the configuration file *config/audio_config.json*, and the default value is "hw:0,0", which indicates audio device Card0 Device0. The device number can be checked with the command `ls /dev/snd` such as: "pcmC0D1c"; the letter `c` indicates a capture device, `C0` indicates Card0, and `D1` indicates Device1. Modify the parameter to "hw:0,1".

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
    # Start the launch file and specify the voice DOA angle corresponding to the front of the car, taking 90 as an example
    ros2 launch audio_tracking audio_tracking.launch.py car_front_audio_angle:=90
    ```
    
## Result Analysis

The following information is the output of the RDK running on the terminal:

```text

        This is audio tracking package.

============================================
        audio tracking usage

Wake up device is "D-Robotics Hello".
Audio control command word definitions are:
        "Go forward"
        "Go backward"
        "Turn right"
        "Turn left" 
When you say the wake word, the car turns toward you 
Let's start the experience
============================================

[INFO] [1663149803.248119421] [audio_tracking]: AudioTrackingEngine construct
[INFO] [1663149803.313949108] [rclcpp]: ParametersClass node construct
[WARN] [1663149803.337782049] [AudioTrackingNode]: Parameter:
 ai_msg_sub_topic_name: /audio_smart
 twist_pub_topic_name: /cmd_vel
[WARN] [1663149804.316577383] [audio_control_parameter_node]: Robot Move param are
move_step: 0.3
rotate_step: 0.348

[INFO] [1663149814.967019845] [audio_tracking]: process audio frame type:2
[INFO] [1663149814.967377380] [audio_tracking]: process audio event type:1
[INFO] [1663149815.012831677] [audio_tracking]: process audio frame type:5
[WARN] [1663149815.013112088] [audio_tracking]: process audio doa theta:80.000000
[INFO] [1663149815.168426039] [audio_tracking]: process audio doa move to front distance:0.200000, speed:0.300000, duration:0.666667, ticks:6
[WARN] [1663149815.769833806] [audio_tracking]: cancel move
[INFO] [1663149822.128098383] [audio_tracking]: process audio frame type:2
[INFO] [1663149822.128389794] [audio_tracking]: process audio event type:1
[INFO] [1663149822.145186562] [audio_tracking]: process audio frame type:5
[WARN] [1663149822.145491473] [audio_tracking]: process audio doa theta:55.000000
[INFO] [1663149822.174037772] [audio_tracking]: process audio doa move theta:35.000000, angle:0.610865, direction:1, ticks:6
[WARN] [1663149822.775398926] [audio_tracking]: cancel move
[INFO] [1663149822.775698796] [audio_tracking]: process audio doa move to front distance:0.200000, speed:0.300000, duration:0.666667, ticks:6
[WARN] [1663149823.377099758] [audio_tracking]: cancel move
```

The above logs capture a segment of the output after the audio control package is launched. The log shows that the wake-up word configured for the intelligent voice recognition module is "D-Robotics Hello". After receiving the wake-up event, the audio tracking control module receives DOA angle information. As shown in the log above, the DOA is 80 degrees. At this time, the audio tracking control module publishes a command to make the car turn left by 20 degrees. After the rotation, the car moves forward, and later the car stops moving.

On the PC, you can use the `ros2 topic list` command in the terminal to query the topic information of the RDK.

```shell
ros2 topic list
/audio_smart
/cmd_vel
```

The topic "/audio_smart" is the algorithm perception message published, which contains intelligent voice results. The topic "/cmd_vel" is the motion control command published by RDK.

On the PC, the command "ros2 topic echo /cmd_vel" can be used in the terminal to view the motion control command published by RDK:

```text
linear:
  x: 0.0
  y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.1136000156402588
---
linear:
  x: 0.0
  y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.1136000156402588
---
linear:
  x: 0.0
  y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.1136000156402588
---
linear:
  x: 0.0
  y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.1136000156402588
---
linear:
  x: 0.0
```y: 0.30000001192092896
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.1136000156402588
---
```

The PC simulation environment controls the movement of the car based on voice tracking. The effect is shown as follows:

![](/../static/img/05_Robot_development/04_apps/image/car_audio_tracking/audio_tracking.gif)

In the image above, the simulated car on the left rotates according to the angle of the sound source, and the log outputted by the program is on the right. The log contains the DOA angle information.