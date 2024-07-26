---
sidebar_position: 1
---
# 5.2.1 Image Capture

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## USB camera

### Introduction

In order to achieve environmental perception capability, robot products usually carry cameras to obtain image information. USB cameras are easy to obtain, easy to use, and have good versatility. TogetheROS.Bot supports USB cameras and supports ROS2 standard image messages.

Code repository:  (https://github.com/D-Robotics/hobot_usb_cam.git)

### Supported Platforms

| Platform    | Operating Mode     |
| ------- | ------------ |
| RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble) |

### Preparation

#### Horizon RDK

1. Confirm that the USB camera is working properly and connect it to the USB slot of the Horizon RDK.

2. Horizon RDK has burned the  Ubuntu 20.04/22.04 system image provided by Horizon.

3. Horizon RDK has successfully installed tros.b.

4. Confirm that the PC can access the Horizon RDK via the network.

### How to Use (default usb_pixel_format is mjpeg)

Taking Horizon RDK as an example:

1. Log in to the Horizon RDK via SSH and confirm the device name of the USB camera. Here, let's take `/dev/video8` as an example.

2. Start the USB camera using the following command:

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

    ```bash
    # Start the launch file
    ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8
    ```

3. If the program outputs the following information, it means the node has been successfully launched.

    ```text
    [INFO] [launch]: All log files can be found below /root/.ros/log/2024-01-18-19-44-39-419588-ubuntu-3951
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [hobot_usb_cam-1]: process started with pid [3953]
    [hobot_usb_cam-1] [WARN] [1705578280.808870437] [hobot_usb_cam]: framerate: 30
    [hobot_usb_cam-1] [WARN] [1705578280.809851560] [hobot_usb_cam]: pixel_format_name: mjpeg
    [hobot_usb_cam-1] [WARN] [1705578280.936383062] [hobot_usb_cam]: Camera calibration file: [/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml] does not exist!
    [hobot_usb_cam-1] If you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!
    [hobot_usb_cam-1] [WARN] [1705578280.936697507] [hobot_usb_cam]: This devices supproted formats:
    [hobot_usb_cam-1] [WARN] [1705578280.936858791] [hobot_usb_cam]:        Motion-JPEG: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.936912830] [hobot_usb_cam]:        Motion-JPEG: 1920 x 1080 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.936960328] [hobot_usb_cam]:        Motion-JPEG: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937007285] [hobot_usb_cam]:        Motion-JPEG: 800 x 600 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937053241] [hobot_usb_cam]:        Motion-JPEG: 1280 x 720 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937098906] [hobot_usb_cam]:        Motion-JPEG: 1024 x 576 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937144528] [hobot_usb_cam]:        YUYV 4:2:2: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937190068] [hobot_usb_cam]:        YUYV 4:2:2: 1920 x 1080 (5 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937235858] [hobot_usb_cam]:        YUYV 4:2:2: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937282064] [hobot_usb_cam]:        YUYV 4:2:2: 800 x 600 (20 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937328020] [hobot_usb_cam]:        YUYV 4:2:2: 1280 x 720 (10 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937373518] [hobot_usb_cam]:        YUYV 4:2:2: 1024 x 576 (15 Hz)
    ```

4. Open another terminal to view the USB camera image on the web page:

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

    ```bash
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image websocket_only_show_image:=true
    ```

5. Open a web browser (Chrome/Firefox/Edge) on your PC and enter  `http://IP:8000` (where IP is the Horizon RDK IP address). Click on the upper left corner to view the real-time image from the USB camera.
![image-usb-camera](./image/demo_sensor/usb_cam_pic.png)

### Usage Method 2 (usb_pixel_format is yuyv2rgb)

Here is an example using the Horizon RDK platform:

1. SSH into the Horizon RDK and confirm the USB camera device name, for example `/dev/video8`.

2. Start the USB camera using the following command:

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

    ```bash
    ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8 usb_pixel_format:=yuyv2rgb usb_image_width:=640 usb_image_height:=480
    ```

3. If the following information is outputted by the program, it indicates that the node has been successfully launched

    ```text
    [INFO] [launch]: All log files can be found below /root/.ros/log/2024-01-18-19-44-39-419588-ubuntu-3951
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [hobot_usb_cam-1]: process started with pid [3953]
    [hobot_usb_cam-1] [WARN] [1705578280.808870437] [hobot_usb_cam]: framerate: 30
    [hobot_usb_cam-1] [WARN] [1705578280.809851560] [hobot_usb_cam]: pixel_format_name: yuyv2rgb
    [hobot_usb_cam-1] [WARN] [1705578280.936383062] [hobot_usb_cam]: Camera calibration file: [/opt/tros/lib/hobot_usb_cam/config/usb_camera_calibration.yaml] does not exist!
    [hobot_usb_cam-1] If you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!
    [hobot_usb_cam-1] [WARN] [1705578280.936697507] [hobot_usb_cam]: This devices supproted formats:
    [hobot_usb_cam-1] [WARN] [1705578280.936858791] [hobot_usb_cam]:        Motion-JPEG: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.936912830] [hobot_usb_cam]:        Motion-JPEG: 1920 x 1080 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.936960328] [hobot_usb_cam]:        Motion-JPEG: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937007285] [hobot_usb_cam]:        Motion-JPEG: 800 x 600 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937053241] [hobot_usb_cam]:        Motion-JPEG: 1280 x 720 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.9379806] [hobot_usb_cam]:        Motion-JPEG: 1024 x 576 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937144528] [hobot_usb_cam]:        YUYV 4:2:2: 640 x 480 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937190068] [hobot_usb_cam]:        YUYV 4:2:2: 1920 x 1080 (5 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937235858] [hobot_usb_cam]:        YUYV 4:2:2: 320 x 240 (30 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937282064] [hobot_usb_cam]:        YUYV 4:2:2: 800 x 600 (20 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937328020] [hobot_usb_cam]:        YUYV 4:2:2: 1280 x 720 (10 Hz)
    [hobot_usb_cam-1] [WARN] [1705578280.937373518] [hobot_usb_cam]:        YUYV 4:2:2: 1024 x 576 (15 Hz)
    ```

4. Encode to mjpeg with hobot codec

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

    ```bash
    # Start the launch file
    ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=ros codec_in_format:=rgb8 codec_out_mode:=ros codec_sub_topic:=/image codec_pub_topic:=/image_mjpeg
    ```

5. View the USB camera image on the web side, open a new terminal:

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

    ```bash
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_mjpeg websocket_only_show_image:=true
    ```

6. Open a browser on your PC (chrome/firefox/edge) and enter  `http://IP:8000` (IP is the Horizon RDK IP address), click on the top left to display the web side to view the real-time image from the USB camera
    ![image-usb-camera](./image/demo_sensor/usb_cam_pic.png)
    
### Notes

1. USB cameras need to be calibrated and the camera calibration file path needs to be set in order to publish camera parameters. However, this does not affect other functionalities.
2. To set the camera calibration file path, follow the steps below:

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

   ```bash
   # Start with launch command
   ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_camera_calibration_file_path:=（actual calibration file absolute path）
   ```

3. Changes to the pixel_format configuration:

   hobot_usb_cam supports the following configurations:
   "mjpeg", "mjpeg-compressed", "mjpeg2rgb", "rgb8", "yuyv", "yuyv2rgb", "uyvy", "uyvy2rgb", "m4202rgb", "mono8", "mono16", "y102mono8"
   
   Start usb camera with the default parameters of the first type to query the formats supported by the device hardware, as shown in the log below:

   ```text
   [hobot_usb_cam-1] [WARN] [1705548544.174669672] [hobot_usb_cam]: This devices supproted formats:
   [hobot_usb_cam-1] [WARN] [1705548544.174844917] [hobot_usb_cam]:        Motion-JPEG: 640 x 480 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.174903166] [hobot_usb_cam]:        Motion-JPEG: 1920 x 1080 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.174950581] [hobot_usb_cam]:        Motion-JPEG: 320 x 240 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.174996788] [hobot_usb_cam]:        Motion-JPEG: 800 x 600 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175043412] [hobot_usb_cam]:        Motion-JPEG: 1280 x 720 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175089161] [hobot_usb_cam]:        Motion-JPEG: 1024 x 576 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175135035] [hobot_usb_cam]:        YUYV 4:2:2: 640 x 480 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175180325] [hobot_usb_cam]:        YUYV 4:2:2: 1920 x 1080 (5 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175226449] [hobot_usb_cam]:        YUYV 4:2:2: 320 x 240 (30 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175272365] [hobot_usb_cam]:        YUYV 4:2:2: 800 x 600 (20 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175318697] [hobot_usb_cam]:        YUYV 4:2:2: 1280 x 720 (10 Hz)
   [hobot_usb_cam-1] [WARN] [1705548544.175365195] [hobot_usb_cam]:        YUYV 4:2:2: 1024 x 576 (15 Hz)
   ```
   a. Query the image formats supported by the usb camera, as shown in the log above. The log shows support for mjpeg and YUYV.

   b. Only "mjpeg", "mjpeg-compressed", "mjpeg2rgb", "yuyv", and "yuyv2rgb" can be set; otherwise, the hobot_usb_cam program will exit.

## MIPI camera

### Introduction

To achieve environmental perception capabilities, robots often carry cameras, ToF, and other types of sensors. In order to reduce the cost of sensor adaptation and usage for users, TogetheROS.Bot wraps multiple commonly used sensors into the hobot_sensor module and abstracts them into ROS standard image messages. When the configured sensor parameters do not match the connected camera, the program will automatically adapt to the correct sensor type. The currently supported MIPI sensor types are as follows:

| Type | Model | Specifications | Supported Platforms |
| ------ | ------ | ------ | ------ |
| Camera | F37 | 200W | RDK X3, RDK X3 Module |
| Camera | GC4663 | 400W | RDK X3, RDK X3 Module |
| Camera | IMX219 | 200W | RDK X3, RDK X3 Module  |
| Camera | IMX477 | 200W | RDK X3, RDK X3 Module |
| Camera | OV5647 | 200W | RDK X3, RDK X3 Module |

Code repository:  (https://github.com/D-Robotics/hobot_mipi_cam.git)

### Supported Platforms

| Platform   | System      | Function                          |
| ------ | ------------- | --------------------------------- |
|RDK X3, RDK X3 Module | Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)  | Start MIPI camera and display images through Web |

### Preparation

#### Horizon RDK

1. Confirm that the camera is correctly connected to Horizon RDK. For example, the connection between the F37 camera and RDK X3 is shown in the following figure:

    ![image-X3-PI-Camera](./image/demo_sensor/image-X3-PI-Camera.png)

2. Horizon RDK is flashed with the  Ubuntu 20.04/22.04 system image provided by Horizon

3. Horizon RDK has successfully installed tros.b

4. Confirm that the PC can access Horizon RDK through the network

### Usage

#### Horizon RDK Platform

Take the F37 as an example to introduce the method of acquiring and previewing images:

1. SSH into Horizon RDK and determine the camera model, take `F37` as an example, and determine the path to read the camera calibration file, take `/opt/tros/lib/mipi_cam/config/F37_calibration.yaml` as an example.

2. Start the `hobot_sensor` node with the following command:

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
    # Start the launch file
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37 mipi_camera_calibration_file_path:=/opt/tros/${TROS_DISTRO}/lib/mipi_cam/config/F37_calibration.yaml
    ```

3. If the following information is outputted, it means that the node has been successfully started:

    ```text
    [INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-11-15-16-13-641715-ubuntu-8852
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [mipi_cam-1]: process started with pid [8854]
    ...
    ```
4. To view the F37 camera image on the web, as raw data needs to be encoded into JPEG images, two terminals need to be launched separately: one for subscribing to MIPI data and encoding it into JPEG, and one for publishing with a webservice.

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
    # Start encoding
    ros2 launch hobot_codec hobot_codec_encode.launch.py

    # Launch another terminal
    # Start websocket
    ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
    ```

5. Open a web browser on the PC (Chrome/Firefox/Edge) and enter  `http://IP:8000` (IP address of the Horizon RDK) to see the real-time display of the F37 camera's output.
    ![web-f37-codec](./image/demo_sensor/web-f37-codec.png "Real-time image")

6. To query the camera's intrinsic parameters on the PC (the specific data may vary depending on the calibrated camera file), use the following command and view the results:

   <Tabs groupId="tros-distro">
   <TabItem value="foxy" label="Foxy">

      ```shell
      root@ubuntu:~# source /opt/ros/foxy/setup.bash
      ```

   </TabItem>
   <TabItem value="humble" label="Humble">

      ```shell
      root@ubuntu:~# source /opt/ros/humble/setup.bash
      ```

   </TabItem>
   </Tabs>

   ```shell
    root@ubuntu:~# ros2 topic echo /camera_info
        header:
    stamp:
        sec: 1662013622
        nanosec: 672922214
    frame_id: default_cam
    height: 1080
    width: 1920
    distortion_model: plumb_bob
    d:
    - 0.169978
    - -0.697303
    - -0.002944
    - -0.004961
    - 0.0
    k:
    - 1726.597634
    - 0.0
    - 904.979671
    - 0.0
    - 1737.359551
    - 529.123375
    - 0.0
    - 0.0
    - 1.0
    r:
    - 1.0
    - 0.0- 0.0
   - 0.0
   - 1.0
   - 0.0
   - 0.0
   - 0.0
   - 1.0
   p:
   - 1685.497559
   - 0.0
   - 881.6396
   - 0.0
   - 0.0
   - 1756.460205
   - 526.781147
   - 0.0
   - 0.0
   - 0.0
   - 1.0
   - 0.0
   binning_x: 0
   binning_y: 0
   roi:
   x_offset: 0
   y_offset: 0
   height: 0
   width: 0
   do_rectify: false

   ```

### Caution

1. mipi_cam provides the calibration files for two types of cameras, F37 and GC4663. By default, it reads the calibration file for F37, `F37_calibration.yaml`. If you want to use GC4663, please change the path to the camera calibration file accordingly, as below:

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
    # Start the launch file
    ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=GC4663 mipi_camera_calibration_file_path:=/opt/tros/${TROS_DISTRO}/lib/mipi_cam/config/GC4663_calibration.yaml
    ```

2. Caution when plugging/unplugging the camera:

   **NEVER plug or unplug the camera module without powering off the development board first. Otherwise, it may result in damaging the camera module.**

3. If you encounter any issues with the startup of the hobot_sensor node, you can troubleshoot the problems by following these steps:
	- Check the hardware connections.
	- Make sure you have set up the tros.b environment.
	- Verify the parameters are correct, for more details refer to the Hobot_Sensors README.md file.

## RGBD camera

### Introduction

In order to achieve environmental perception capability, robot products usually carry cameras, ToF and other types of sensors. To reduce the cost of sensor adaptation and usage for users, TogetheROS.Bot encapsulates and abstracts multiple commonly used sensors into the hobot_sensor module, which supports ROS standard image messages, custom image message output, and camera calibration data publishing. Currently supported types of RGBD sensors are as follows:

| Type | Model | Specification | Supported Platforms |
| ------ | ------ | ------ | ---- |
| Camera | CP3AM | 200W | RDK X3 |

Code Repository:  (https://github.com/D-Robotics/hobot_rgbd_cam.git)

### Supported Platforms

| Platform   | System      | Function                                           |
| ------ | ------------- | -------------------------------------------------- |
|RDK X3| Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)  | Start RGBD camera and preview RGB and depth images on PC using rviz2 |

**Note: Only supports RDK X3, RDK X3 Module is not supported yet.**

### Preparations

#### Horizon RDK Platform

1. Make sure the camera is correctly connected to the Horizon RDK. The connection for RGBD module to RDK X3 is shown as below:

    ![hobot_rgbd](./image/demo_sensor/hobot_rgbd.png)

    **Note: The RGBD module needs an additional adapter board to connect to Horizon RDK X3**.
2. Horizon RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by Horizon.

3. Horizon RDK has successfully installed tros.b.

4. Make sure the PC can access the Horizon RDK through the network.

5. Install ros2 foxy version and rviz2 on the PC, using the following command:

```shell
  sudo apt install ros-foxy-rviz-common ros-foxy-rviz-default-plugins ros-foxy-rviz2
```

### Usage

#### Horizon RDK

Taking CP3AM as an example, the method of acquiring and previewing camera data is introduced below:

1. SSH into the Horizon RDK and start the hobot_sensor node with the following command:

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
   cp -r /opt/tros/lib/rgbd_sensor/parameter .
   # Launch the node
   ros2 launch rgbd_sensor rgbd_sensor.launch.py
   ```

2. If the program outputs the following information, it indicates that the node has been successfully launched:

```text
[WARN] [1654573498.706920307] [example]: [wuwl]->This is rgbd!
sh: 1: echo: echo: I/O error
pipeId[1], mipiIdx[1], vin_vps_mode[3]
[ERROR]["LOG"][irs2381c_utility.c:192] 2381 enter sensor_init_setting
[ERROR]["LOG"][irs2381c_utility.c:200] start write 2381c reg
camera read reg: 0xa001 val:0x7
...
[ERROR]["LOG"][irs2381c_utility.c:207] end write 2381c reg
HB_MIPI_InitSensor end
HB_MIPI_SetDevAttr end
pstHbVideoDev->vin_fd = 29
sensorID: 634-2362-2676-68d0 
find local calib_file

find local calib_file

SDK Version: V4.4.35 build 20220525 09:27:53.
read file(./calib-0634-2362-2676-68d0.bin), ok, file_len=132096, read_len=132096.......
module config file(user custom) is: ./parameter/T00P11A-17.ini.
parse calib data, data len:132096...
sunny_degzip2 decode_len=155575.
calib data with crc.
parse calib data, ok.
max roi (firstly): (0, 224, 0, 128).
cur roi (firstly): (0, 224, 0, 128).
HB_MIPI_InitSensor end
HB_MIPI_SetDevAttr end
pstHbVideoDev->vin_fd = 55
vencChnAttr.stRcAttr.enRcMode=11
mmzAlloc paddr = 0x1a6e6000, vaddr = 0x917e1000
camera read reg: 0x9400 val:0x1
...

[wuwl-StartCamera]->camT=3, ret=0.
camera read reg: 0x3e val:0x40
[ERROR]["vio_devop"][utils/dev_ioctl.c:121] [499334.399304]dev_node_dqbuf_ispoll[121]: failed to ioctl: dq (14 - Bad address)
[ERROR]["vio_devop"][utils/dev_ioctl.c:189] [499334.399355]entity_node_dqbuf_ispoll[189]: dev type(9) dq failed

[ERROR]["vio_core"][commom_grp/binding_main.c:1034] [499334.399371]comm_dq_no_data[1034]: G1 MIPI_SIF_MODULE module chn0 dq failed! maybe framedrop error_detail -14
```[wuwl-StartCamera]->camT=1, ret=0.
[INFO] [1654573500.226606117] [rclcpp]: [childStart]-> ret=0 !

[INFO] [1654573500.226831567] [rclcpp]: [StartStream]->pthread create sucess

[INFO] [1654573500.226963854] [rclcpp]: <========>[doCapStreamLoop]->Start.

[WARN] [1654573500.226998103] [rgbd_node]: [RgbdNode]->mipinode init sucess.

[WARN] [1654573500.227352507] [example]: [wuwl]->rgbd init!
[WARN] [1654573500.228502174] [example]: [wuwl]->rgbd add_node!

[INFO] [1662723985.860666547] [rgbd_node]: publish camera info.
[INFO] [1662723985.866077156] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662723985.876428980] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.

[INFO] [1662723985.946767230] [rgbd_node]: publish camera info.
[INFO] [1662723985.951415418] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662723985.960161280] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
...

```
3. Query the current topic on the PC, and the command are as follows:

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
ros2 topic list
```
   The result are as follows:

```text
/rgbd_CP3AM/depth/image_rect_raw

/rgbd_CP3AM/depth/color/points

/rgbd_CP3AM/color/camera_info

/rgbd_CP3AM/aligned_depth_to_color/color/points

/rgbd_CP3AM/infra/image_rect_raw

/rgbd_CP3AM/color/image_rect_raw

/parameter_events

/rosout
```
4. Subscribe to topics and preview camera data on a PC.

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
    ros2 run rviz2 rviz2
    ```

    Click the "add" button in the rviz2 interface to add topics published by rgbd_sensor (refer to the rgbd_CP3AM related topics indicated in section 3). To subscribe to point cloud data, modify the "Fixed Frame" option in the Global Options of rviz2 configuration to "depth". Then you can view real-time point cloud information. In the point topic configuration, select "points" as the point type.

    ![hobot_rgbd_sensor](./image/demo_sensor/hobot_rgbd_sensor.png "Real-time Image")

5. Query camera intrinsics on a PC.

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
    ros2 topic echo /rgbd_CP3AM/color/camera_info
    ```

    The output result is as follows:

   ```text
    header:
    stamp:
        sec: 119811
        nanosec: 831645108
    frame_id: color
    height: 1080
    width: 1920
    distortion_model: plumb_bob
    d:
    - -0.32267
    - 0.083221
    - 0.000164
    - -0.002134
    - 0.0
    k:
    - 1066.158339
    - 0.0
    - 981.393777
    - 0.0
    - 1068.659998
    - 545.569587
    - 0.0
    - 0.0
    - 1.0
    r:
    - 1.0
    - 0.0
    - 0.0```
   - 0.0
   - 1.0
   - 0.0
   - 0.0
   - 0.0
   - 1.0
   p:
   - 741.315308
   - 0.0
   - 968.865379
   - 0.0
   - 0.0
   - 969.43042
   - 546.524343
   - 0.0
   - 0.0
   - 0.0
   - 1.0
   - 0.0
   binning_x: 0
   binning_y: 0
   roi:
   x_offset: 0
   y_offset: 0
   height: 0
   width: 0
   do_rectify: false
   ```

### Instructions

If there is an abnormality in the startup of the hobot_sensor node, you can troubleshoot the problem using the following steps:

1. Check hardware connections.
2. Check if the tros.b environment is set.
3. Check if the parameters are correct. For specific details, please refer to the Hobot_Sensors README.md file.