---
sidebar_position: 2
---

# 5.2.2 Display

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Web

### Introduction

Web is used to preview camera images (JPEG format) and algorithm results. The images and algorithm results are transmitted to the PC browser through the network and rendered for display. The display interface also supports displaying only the video without rendering the intelligent results.

Code Repository:  (https://github.com/D-Robotics/hobot_websocket)

### Supported Platforms

| Platform             | Operating System              | Example Functionality                  |
| -------------------- | ----------------------------- | --------------------------------------- |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start MIPI cameras and display images via Web |
| RDK X5               | Ubuntu 22.04 (Humble)         | Start MIPI cameras and display images via Web |
| RDK Ultra            | Ubuntu 20.04 (Foxy)           | Start MIPI cameras and display images via Web |
| X86                  | Ubuntu 20.04 (Foxy)           | Start USB cameras and display images via Web |
| RDK S100               | Ubuntu 22.04 (Humble)         | Start MIPI cameras and display images via Web |


### Preparation

#### RDK

1. Confirm that the camera F37 is correctly connected to the RDK.

2. Confirm that the PC can access the RDK through the network.

3. Confirm that TogetheROS.Bot has been successfully installed.

### Usage

#### RDK

1. Log in to the RDK through SSH and start the programs on the board.

    a. Launch mipi_cam

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
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
    ```

    b. Launch encoding

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
    ros2 launch hobot_codec hobot_codec_encode.launch.py
    ```

    c. Launch WebSocket

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
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
    ```

2. Open a PC browser (Chrome/Firefox/Edge) and enter  `http://IP:8000` to view the image and algorithm effects. IP refers to the RDK IP address.

   ![websocket](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/websocket.png)

### Notes

1. WebSocket uses port 8000. If the port is already in use, the launch will fail. Here are some solutions:

   - Use the command `lsof -i:8000` to check the processes that are occupying port 8000, and use `kill <PID>` to close the process, and then relaunch WebSocket.

   - If the user does not want to stop the service that is currently using port 8000, you can modify the `listen` port number in the configuration file `/opt/tros/lib/websocket/webservice/conf/nginx.conf` to a port number that is greater than 1024 and not being used. After modifying the port number, the URL used in the browser also needs to be modified accordingly.
   
## HDMI

### Introduction

This chapter introduces the use of displaying camera nv12 images through HDMI. RDK can display real-time image effects by connecting to a monitor via HDMI, corresponding to the hobot_hdmi package.

Code Repository:  (https://github.com/D-Robotics/hobot_hdmi)

### Supported Platforms

| Platform | System | Function                    |
| -------- | ------------ | ----------------------------------- |
| RDK X3, RDK X3 Module, RDK X5 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start MIPI camera and display image through HDMI |

:::caution
HDMI `EOL` Description:
- The `RDK X3` and `RDK X3 Module` platforms are supported up to version `2.1.0`, corresponding to TROS version `2.2.0 (2024-04-11)`.
- The `RDK X5` and `RDK X5 Module` platforms are supported up to version `2.4.2`, corresponding to TROS version `2.3.1 (2024-11-20)`.
:::

### Preparation

#### RDK

1. RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. TogetheROS.Bot has been successfully installed on the RDK.

3. RDK is connected to a monitor via HDMI.

### Instructions

#### RDK

Log in to the development board via SSH and start the relevant programs on the board:

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

If use RDK X5, run commod:
```bash
# 关闭桌面显示
sudo systemctl stop lightdm
# 复制运行依赖
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_hdmi/config/ .
```
</TabItem>

</Tabs>

```shell
ros2 launch hobot_hdmi hobot_hdmi.launch.py device:=F37
```

### Result Analysis

The following information is displayed in the running terminal:

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-07-27-15-27-26-362299-ubuntu-13432
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mipi_cam-1]: process started with pid [13434]
[INFO] [hobot_hdmi-2]: process started with pid [13436]
```

The monitor displays the image as follows:
![hdmi](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/hdmi.png)

## RViz2

### Introduction

TogetheROS.Bot is compatible with ROS2 Foxy version. To conveniently preview image effects, you can use RViz2 to get images.

### Supported Platforms

| Platform | System | Sample Function                                        |
| -------- | -------------- | ------------------------------------------------------ |
| RDK X3, RDK X3 Module, RDK X5  | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start the MIPI camera to capture images and use RViz2 to preview on PC |

### Preparation

#### RDK

1. RDK has flashed with the  Ubuntu 20.04/22.04 system image provided by D-Robotics.

2. RDK has successfully installed tros.b.

3. The PC has installed Ubuntu 20.04, ROS2 Foxy Desktop version, and the data visualization tool RViz2. The PC and RDK are on the same network segment (the first three segments of the IP address are the same).

   Reference for ROS2 Foxy installation:  (https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

   On the PC, install RViz2 with the command: `sudo apt install ros-foxy-rviz-common ros-foxy-rviz-default-plugins ros-foxy-rviz2`

### Usage

#### RDK

1. SSH into the development board and start the corresponding program on the board

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
   # Start the F37 camera to publish images in BGR8 format
   ros2 launch mipi_cam mipi_cam.launch.py mipi_out_format:=bgr8 mipi_image_width:=480 mipi_image_height:=272 mipi_io_method:=ros mipi_video_device:=F37
   ```

   Note: Do not change the `mipi_out_format` arbitrarily. RViz2 only supports image formats like RGB8, RGBA8, BGR8, BGRA8, etc.

2. If the following information is output, it means that the node has been successfully started

   ```
   [INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-03-53-54-778203-ubuntu-2881662
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [mipi_cam-1]: process started with pid [2881781]
   ```
3. A new window is created in the RDK to execute the topic query command and the results are as follows:

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
   # Query topics
   ros2 topic list
   ```

   Output:

   ```shell
   /camera_info
   /image_raw
   /parameter_events
   /rosout
   ```

4. On the PC, the current topics are queried using the following command and the results are as follows:

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
   ros2 topic list
   ```

   Output:

   ```shell
   /camera_info
   /image_raw
   /parameter_events
   /rosout
   ```

5. Subscribing to a topic and previewing camera data on the PC:

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
   ros2 run rviz2 rviz2
   ```

   On the RViz2 interface, first click the "add" button, then select the published image based on the topic, which in this example is named /image_raw. Then click "image":

   ![rviz2-config](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rviz2-config.png)

   The image result is as follows:

   ![rviz2-result](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rviz2-result.png)   
### Attention

1. If the PC terminal's `ros2 topic list` does not recognize the camera topic, please check the following:

   - Check if RDK is publishing images properly:

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
      ros2 topic list
      ```

      Output:

      ```shell
      /camera_info
      /image_raw
      /parameter_events
      /rosout
      ```

   - Check if the PC and RDK networks can ping each other;
   - Check if the IP addresses of the PC and RDK have the same first three digits;

## RQt

### Introduction

TogetheROS.Bot is compatible with ROS2 Foxy and supports previewing compressed format images through RQt, greatly reducing network bandwidth consumption.

### Supported Platforms

| Platform       | System | Function           |
| -------------- | ------------ | ------------------------------- |
| RDK X3, RDK X3 Module, RDK X5 | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) | Start MIPI camera to capture images and use RQt to preview on PC |

### Preparation

#### RDK

1. RDK has been flashed with the provided  Ubuntu 20.04/22.04 system image.

2. RDK has successfully installed tros.b.

3. PC has been installed with Ubuntu 20.04 system, ROS2 Foxy desktop version, and the visualization tool RQt. Both PC and RDK are on the same network segment (with the same first three digits of the IP addresses).

   ROS2 Foxy installation reference:  (https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

   Installation method for `rqt-image-view` on PC terminal: `sudo apt install ros-foxy-rqt-image-view ros-foxy-rqt`
   
### Usage

#### RDK

1. SSH into the development board and start relevant programs on the board:

   a. Start F37 camera

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
   ros2 launch mipi_cam mipi_cam.launch.py mipi_image_width:=640 mipi_image_height:=480 mipi_video_device:=F37
   ```

   b. Start hobot_codec and publish compressed format images

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
   ros2 launch hobot_codec hobot_codec_encode.launch.py codec_out_format:=jpeg codec_pub_topic:=/image_raw/compressed
   ```

2. If the program output shows the following information, it means the nodes have been successfully launched

   ```shell
   [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-15-17-08-02-144621-ubuntu-4755
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [mipi_cam-1]: process started with pid [4757]
   [mipi_cam-1] This is version for optimizing camera timestamp 
   ```

   ```shell
   [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-15-17-08-17-960398-ubuntu-4842
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [hobot_codec_republish-1]: process started with pid [4844]
   ```

3. Subscribe to the topic on the PC and preview the camera data;

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
   ros2 run rqt_image_view rqt_image_view
   ```

   Select the topic `/image_raw/compressed`, and the image is as follows:

   ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/rqt-result.png)

### Notes

1. If ros2 topic list does not recognize the camera topic on the PC, perform the following troubleshooting steps:

   - Check if the RDK is publishing images correctly

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
      ros2 topic list
      ```

      Output:

      ```text
      /camera_info
      /hbmem_img000b0c26001301040202012020122406
      /image_raw
      /image_raw/compressed
      /parameter_events
      /rosout
      ```

   - Check if the PC and RDK can ping each other;
   - Check if the PC and RDK have the same first three segments of IP address;

## Foxglove

### Introduction

Foxglove is an open-source toolkit that includes both online and offline versions. It aims to simplify the development and debugging of robotic systems. It provides a range of features for building robot applications.

In this section, we will primarily use the data recording and playback feature of Foxglove: Foxglove allows recording the data of ROS2 topics into files for subsequent playback and analysis. This is very useful for system troubleshooting, performance optimization, and algorithm debugging.

In the demonstration, we will use the hobot_visualization package developed by TogetheROS.Bot to convert intelligent inference results into ROS2 rendered topic information.

Code repository:  (https://github.com/D-Robotics/hobot_visualization)

### Supported Platforms

| Platform | System | Function                                     |
| -------- | -------------- | -------------------------------------------------------- |
| RDK X3, RDK X3 Module, RDK X5      |  Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)   | Offline object detection, and display images and algorithm effects using Foxglove |

### Preparation

#### RDK

1. Confirm that TogetheROS.Bot has been successfully installed.

2. Confirm that the PC can access the RDK via the network. 

### Usage

#### RDK

1. Log in to the RDK via SSH and start the relevant programs on the board side:

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
export CAM_TYPE=fb

ros2 launch hobot_visualization hobot_vis_render.launch.py
```

At the same time, log in to another terminal using SSH and record topic information on the board side:

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
# Record rosbag data, which will be generated in the current working directory
ros2 bag record -a
```

2. Play rosbag data on the Foxglove online page

3) In a PC browser (chrome/firefox/edge), enter  (https://foxglove.dev/studio) to access the Foxglove website.

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_1.png)

   PS: Registration is required for first-time use. You can register using a Google account or a third-party email.

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_11.png)

4) Enter the visualization function interface.

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_2.png)

5) Click to select the local rosbag file.

   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_3.png)

6) Open the layout interface. In the top right corner of the layout interface, click on the settings, select the icon, and open the play marker rendering message function.
   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_4.png)
   
7) Click Play
   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_5.png)

8) View Data
   ![foxglove](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/02_quick_demo/image/demo_render/foxglove_guide_6.png)

### Note

1. Foxglove visualizes image data using the official ROS2 message format and supports image encoding formats. For more details, please refer to  (https://foxglove.dev/docs/studio/panels/image).

2. When recording messages with `rosbag`, it may record topic information from other devices. To ensure clean `rosbag` data, you can set `export ROS_DOMAIN_ID=xxx`, such as `export ROS_DOMAIN_ID=1`.