---
sidebar_position: 5
---

# Stereo Depth Algorithm

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Function Introduction

The D-Robotics stereo depth estimation algorithm takes stereo image data as input and outputs a disparity map and a depth map corresponding to the left view. The algorithm draws inspiration from the IGEV network and employs a GRU architecture, offering good data generalization and high inference efficiency.

Stereo algorithm repository: https://github.com/D-Robotics/hobot_stereonet

MIPI camera repository: https://github.com/D-Robotics/hobot_mipi_cam

ZED camera repository: https://github.com/D-Robotics/hobot_zed_cam

Stereo algorithm explanation: [Live Recording | AI Stereo Algorithm Deployment on RDK X5](https://www.bilibili.com/video/BV1KdEjzREMz/?share_source=copy_web&vd_source=deb3551e36cc4b1c1020033ad17c564b)

## Supported Platforms

| Platform              | OS Support            | Example Function                                                 |
| --------------------- | --------------------- | ---------------------------------------------------------------- |
| RDK X5, RDK X5 Module | Ubuntu 22.04 (Humble) | Start stereo camera, infer depth results, and display on the web |
| RDK S100, RDK S100P   | Ubuntu 22.04 (Humble) | Start stereo camera, infer depth results, and display on the web |

## Algorithm Version Information

The following versions of the stereo algorithm are currently available:

| Algorithm Version | Features                                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------------------------------- |
| X5 V2.0           | int16 quantization, high precision, low frame rate, max 15FPS depth map at 640*352 resolution                 |
| X5 V2.1           | int16 quantization, with confidence output, max 15FPS depth map at 640*352 resolution                         |
| X5 V2.2           | int8 quantization, lower precision, higher frame rate, max 23FPS depth map at 640*352 resolution              |
| X5 V2.3           | int8 quantization, further improved frame rate, max 27FPS depth map at 640*352 resolution                     |
| X5 V2.4 int16     | int16 quantization, trained with more data, max 15FPS depth map at 640*352 resolution                         |
| X5 V2.4 int8      | V2.4 int8 quantization version, max 23FPS depth map at 640*352 resolution                                     |
| S100 V2.1         | int16 quantization, with confidence output, max 53FPS depth map at 640*352 resolution                         |
| S100 V2.4         | int16 quantization, with confidence output, trained with more data, max 53FPS depth map at 640*352 resolution |

## Preparation

### RDK Platform

1. RDK has been flashed with Ubuntu 22.04 system image.
2. TogetheROS.Bot has been successfully installed on RDK.
3. If real-time online inference is required, please prepare a stereo camera. Currently, the official MIPI camera and ZED mini/2i USB camera are supported.
4. If offline inference is required, please prepare <strong style={{ color: 'red' }}>rectified</strong> stereo image data.
5. Ensure that the PC can access RDK over the network.

### System and Package Versions

|                                             | Version              | Query Method                                    |
| ------------------------------------------- | -------------------- | ----------------------------------------------- |
| RDK X5 System Image Version                 | 3.3.1 or higher      | `cat /etc/version`                              |
| RDK S100 System Image Version               | 4.0.2-Beta or higher | `cat /etc/version`                              |
| tros-humble-hobot-stereonet Package Version | 2.4.4 or higher      | `apt list \| grep tros-humble-hobot-stereonet/` |
| tros-humble-mipi-cam Package Version        | 2.3.11 or higher     | `apt list \| grep tros-humble-mipi-cam/`        |
| tros-humble-hobot-zed-cam Package Version   | 2.3.3 or higher      | `apt list \| grep tros-humble-hobot-zed-cam/`   |

- If the system version does not meet the requirements, please refer to the corresponding documentation section for image flashing.
- If the package version does not meet the requirements, please execute the following commands to update:

```bash
sudo apt update
sudo apt upgrade
```

:::caution **Note**
**If the `sudo apt update` command fails or reports errors, please refer to [FAQ](/docs/08_FAQ/01_hardware_and_system.md) section `Q10: How to handle failures or errors with the apt update command?` for solutions.**
:::

## Usage Instructions

:::caution **Note**
**Please execute the commands in this document as the `root` user. Other users may encounter permission issues, leading to unnecessary errors.**
:::

![os_user](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/os_user.png)

The stereo depth algorithm supports multiple cameras, and the startup commands vary. The specific startup commands are as follows:

### (1) Startup with RDK Stereo Camera Module

- The RDK official MIPI stereo camera is shown below:

![RDK_Stereo_Cam_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_230ai.png)

<p style={{ color: 'red' }}> Note: Please check the silkscreen on the back of the camera for CDPxxx-V3 to confirm that the camera is version V3. </p>

- The installation method is shown below. Ensure the wiring is correct to avoid swapping left and right images, which can cause errors in the stereo algorithm:

![RDK_X5_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_230ai.png)

![RDK_S100_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_S100_230ai.png)

- Confirm that the camera is connected properly by connecting to RDK via SSH and executing the following command. If addresses such as 0x30, 0x32, and 0x50 are output, the camera is connected properly:

```bash
# RDK X5
i2cdetect -r -y 4
i2cdetect -r -y 6

# RDK S100
i2cdetect -r -y 1
i2cdetect -r -y 2
```

![i2cdetect_230ai](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_230ai.png)

- Start the corresponding version of the stereo algorithm using different launch files. Connect to RDK X5 via SSH and execute the following command:

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```bash
# Only supports RDK X5

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```bash
# Supports RDK X5 and RDK X100

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_v2.1.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```bash
# Only supports RDK X5

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.2.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.3" label="V2.3">

```bash
# Only supports RDK X5

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.3.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```bash
# Run the following command on RDK X5:

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# int16 version, start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# int8 version, start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# Run the following command on RDK S100:

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes

ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
</Tabs>

Parameter descriptions:

| Name                 | Value        | Description                                                                                                                                                          |
| -------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| mipi_image_width     | Set to 640   | MIPI camera output resolution is 640*352                                                                                                                             |
| mipi_image_height    | Set to 352   | MIPI camera output resolution is 640*352                                                                                                                             |
| mipi_lpwm_enable     | Set to True  | MIPI camera enables hardware synchronization                                                                                                                         |
| mipi_image_framerate | Set to 30.0  | MIPI camera output frame rate is 30.0FPS                                                                                                                             |
| need_rectify         | Set to False | Since the official camera comes with factory calibration parameters and automatically rectifies, there is no need to load custom calibration files for rectification |
| height_min           | Set to -10.0 | Minimum point cloud height is -10.0m                                                                                                                                 |
| height_max           | Set to 10.0  | Maximum point cloud height is 10.0m                                                                                                                                  |
| pc_max_depth         | Set to 5.0   | Maximum point cloud distance is 5.0m                                                                                                                                 |
| uncertainty_th       | Set to 0.1   | Confidence threshold, used to filter noise points. It is recommended to set it to 0.1. A lower confidence threshold results in stricter filtering.                   |

- The following log indicates that the stereo algorithm has started successfully. `fx/fy/cx/cy/base_line` are the camera intrinsic parameters. If the depth map is normal but the estimated distance is incorrect, there may be an issue with the camera intrinsic parameters:

![stereonet_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success_log.png)

- View the depth map via the web interface by entering http://ip:8000 in the browser (the RDK IP in the image is 192.168.1.100):

![web_depth_visual](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

- View the point cloud via rviz2. RDK can directly install rviz2 for viewing. Note the following configurations in rviz2:

```bash
# Install rviz2
sudo apt install ros-humble-rviz2
# Start rviz2
source /opt/tros/humble/setup.bash
rviz2
```

![stereonet_rviz](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rviz.png)

- If the user wants to save the depth estimation results, the following parameters can be added to enable saving. `save_image_all` enables saving, `save_freq` controls the saving frequency, `save_dir` specifies the save directory (automatically created if it does not exist), and `save_total` controls the total number of saves. The program will save **camera intrinsic parameters, left and right images, disparity map, depth map, and visualization map**:

```bash
# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Here, the V2.0 version of the algorithm is used as an example. Similar parameters can be added to other versions of the algorithm.
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
save_image_all:=True save_freq:=4 save_dir:=./online_result save_total:=10
```

Parameter descriptions:

| Name           | Value                           | Description                                                  |
| -------------- | ------------------------------- | ------------------------------------------------------------ |
| save_image_all | Set to True                     | Enable image saving                                          |
| save_freq      | Set to 4                        | Save every 4 frames. Can be modified to any positive number. |
| save_dir       | Set to the image save directory | Can be set to the desired save location.                     |
| save_total     | Set to 10                       | Save a total of 10 images. Set to -1 to save continuously.   |

![stereonet_save_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_log.png)

![stereonet_save_files](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)

### (2) Startup with RDK Stereo Camera GS130W

- The official RDK MIPI stereo camera is shown in the figure below:

![RDK_Stereo_Cam_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_Stereo_Cam_132gs.png)

- The installation method is shown in the figure below. **Ensure the connections are correct**; reversing them will cause the left and right images to swap, leading to errors in stereo algorithm operation:

![RDK_X5_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/RDK_X5_132gs.png)

- **Verify camera connection**: Connect to the RDK via SSH and execute the following commands. If addresses like `0x32`, `0x33`, or `0x50` appear in the output, the camera connection is normal.

    ```bash
    # For RDK X5
    i2cdetect -r -y 4
    i2cdetect -r -y 6

    # For RDK S100
    i2cdetect -r -y 1
    i2cdetect -r -y 2
    ```

![i2cdetect_132gs](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/i2cdetect_132gs.png)

- Launch the corresponding version of the stereo algorithm using the appropriate launch file. Connect to the RDK X5 via SSH and execute the following command:

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```bash
# Only supports RDK X5

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```bash
# Supports RDK X5 and RDK X100

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.1.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```bash
# Only supports RDK X5

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.2.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.3" label="V2.3">

```bash
# Only supports RDK X5

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.3.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```bash
# For RDK X5, run the following commands:

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# int16 version: Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# int8 version: Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0

# For RDK S100, run the following command:

# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which starts both the algorithm and the stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4.launch.py \
mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0 mipi_rotation:=90.0 \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
uncertainty_th:=0.1
```

</TabItem>
</Tabs>

The parameter meanings are as follows:

| Parameter Name        | Value        | Description                                                                                                                               |
| --------------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------- |
| mipi_image_width      | Set to 640   | The MIPI camera output resolution is 640x352.                                                                                             |
| mipi_image_height     | Set to 352   | The MIPI camera output resolution is 640x352.                                                                                             |
| mipi_lpwm_enable      | Set to True  | Enables hardware synchronization for the MIPI camera.                                                                                     |
| mipi_image_framerate  | Set to 30.0  | The MIPI camera output frame rate is 30.0 FPS.                                                                                            |
| need_rectify          | Set to False | The official camera comes pre-calibrated and performs automatic correction; there's no need to load custom calibration files for rectification. |
| height_min            | Set to -10.0 | Minimum point cloud height is -10.0m.                                                                                                     |
| height_max            | Set to 10.0  | Maximum point cloud height is 10.0m.                                                                                                      |
| pc_max_depth          | Set to 5.0   | Maximum point cloud distance is 5.0m.                                                                                                     |
| uncertainty_th        | Set to 0.1   | Confidence threshold for filtering noise points. It is recommended to set it to 0.1. A smaller value results in stricter filtering.       |

- The following log indicates the stereo algorithm has started successfully. `fx/fy/cx/cy/base_line` are the camera intrinsic parameters. If the depth map appears normal but the estimated distances are inaccurate, the camera intrinsics might be incorrect.

![stereonet_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_run_success_log.png)

- View the depth map via the web interface by entering `http://<RDK_IP>:8000` in your browser (e.g., `http://192.168.1.100:8000` in the figure):

![web_depth_visual](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

- View the point cloud via rviz2. rviz2 can be installed directly on the RDK. **Note the following configuration is required in rviz2**:

    ```bash
    # Install rviz2
    sudo apt install ros-humble-rviz2
    # Launch rviz2
    source /opt/tros/humble/setup.bash
    rviz2
    ```

![stereonet_rviz](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rviz.png)

- To save the depth estimation results, add the following parameters: `save_image_all` enables the save function, `save_freq` controls the save frequency, `save_dir` specifies the save directory (created automatically if it doesn't exist), and `save_total` sets the total number of saves. The program will save the **camera intrinsics, left/right images, disparity map, depth map, and visualization images**.

    ```bash
    # Configure the tros.b humble environment
    source /opt/tros/humble/setup.bash

    # Using V2.0 algorithm as an example; other versions are similar.
    ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
    mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True mipi_image_framerate:=30.0  mipi_rotation:=90.0 \
    need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
    save_image_all:=True save_freq:=4 save_dir:=./online_result save_total:=10
    ```

The parameter meanings are as follows:

| Parameter Name | Value                  | Description                                                                        |
| -------------- | ---------------------- | ---------------------------------------------------------------------------------- |
| save_image_all | Set to True            | Enables the image saving function.                                                 |
| save_freq      | Set to 4               | Saves an image every 4 frames. Can be modified to any positive integer.            |
| save_dir       | Set to the save directory | Specifies the save location as needed.                                             |
| save_total     | Set to 10              | Saves a total of 10 images. Setting it to -1 means saving indefinitely.            |

![stereonet_save_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_log.png)

![stereonet_save_files](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_save_files.png)

### (3) Offline Image Replay on Local Machine

- If you want to evaluate the algorithm's performance using local images, you can use the following command to specify the algorithm's operating mode, image data path, and camera intrinsic parameters. Ensure that the image data has been **rectified and epipolar aligned**. The image format is shown below. The first left-eye image is named left000000.png, the second left-eye image is named left000001.png, and so on. The corresponding first right-eye image is named right000000.png, the second right-eye image is named right000001.png, and so on. The algorithm traverses the images in sequence until all images have been processed:

![stereonet_rdk](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

- The offline operation mode of the algorithm is as follows. **Refer to the online startup command above**, remove the `mipi`-related parameters from the online startup command, add `use_local_image=True` to enable offline inference, set `local_image_path` to the offline image directory, set `camera_fx/camera_fy/camera_cx/camera_cy/base_line` to the rectified camera parameters, set `save_image_all:=True` to enable saving results, and set `save_dir` to control the save directory (automatically created if it does not exist). <span style={{ color: 'red' }}> Note: The replayed images must be epipolar rectified, and correct camera parameters must be set; otherwise, the saved results may be incorrect. </span>

```shell
# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Here, the V2.0 version of the algorithm is used as an example. Similar parameters can be added to other versions of the algorithm.
ros2 launch hobot_stereonet stereonet_model_web_visual_v2.0.launch.py \
need_rectify:=False height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 \
use_local_image:=True local_image_path:=./online_result \
camera_fx:=216.696533 camera_fy:=216.696533 camera_cx:=335.313477 camera_cy:=182.961578 base_line:=0.070943 \
save_image_all:=True save_dir:=./offline_result
```

![stereonet_offline_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_offline_log.png)

Parameter descriptions:

| Name             | Value                                              | Description                                                                                                                                 |
| ---------------- | -------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| use_local_image  | Set to True                                        | Enable image replay mode                                                                                                                    |
| local_image_path | Set to the offline data directory                  | Directory of replayed images                                                                                                                |
| need_rectify     | Set to False                                       | Replayed images must be epipolar rectified, so this switch does not need to be enabled, but rectified parameters must be manually passed in |
| camera_fx        | Set to the rectified camera intrinsic parameter fx | Camera intrinsic parameters                                                                                                                 |
| camera_fy        | Set to the rectified camera intrinsic parameter fy | Camera intrinsic parameters                                                                                                                 |
| camera_cx        | Set to the rectified camera intrinsic parameter cx | Camera intrinsic parameters                                                                                                                 |
| camera_cy        | Set to the rectified camera intrinsic parameter cy | Camera intrinsic parameters                                                                                                                 |
| base_line        | Set to the rectified camera baseline               | Baseline distance in meters                                                                                                                 |
| height_min       | Set to -10.0                                       | Minimum point cloud height is -10.0m                                                                                                        |
| height_max       | Set to 10.0                                        | Maximum point cloud height is 10.0m                                                                                                         |
| pc_max_depth     | Set to 5.0                                         | Maximum point cloud distance is 5.0m                                                                                                        |
| save_image_all   | Set to True                                        | Save replayed results                                                                                                                       |
| save_dir         | Set to the image save directory                    | Can be set to the desired save location.                                                                                                    |

- After the algorithm runs successfully, the result data can also be displayed via the web interface and rviz2. Refer to the corresponding settings above.

### (4) Startup with ZED Stereo Camera

- The ZED stereo camera is shown below:

![zed_cam](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/zed_cam.png)

- Connect the ZED camera to RDK via USB, then start the stereo algorithm. Connect to RDK via SSH and execute the following command:

<p style={{ color: 'red' }}> Note: RDK must be connected to the internet when running the ZED camera because ZED needs to download calibration files online. </p>

<Tabs groupId="stereo-version">
<TabItem value="V2.0" label="V2.0">

```shell
# Only supports RDK X5

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.0.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.1" label="V2.1">

```shell
# Only supports RDK S100

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.1.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.2" label="V2.2">

```shell
# Only supports RDK X5

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.2.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
<TabItem value="V2.4" label="V2.4">

```shell
# Only supports RDK S100

# Configure tros.b humble environment
source /opt/tros/humble/setup.bash

# Start the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual_zed_v2.4.launch.py use_mipi_cam:=False \
height_min:=-10.0 height_max:=10.0 pc_max_depth:=5.0 uncertainty_th:=0.10
```

</TabItem>
</Tabs>

![stereonet_zed_run_success_log](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_zed_run_success_log.png)

When connected to the internet, the program will automatically download the calibration file. If RDK is not connected to the internet, you can manually download the calibration file and upload it to RDK: According to the log information, open a browser on your PC and enter (https://calib.stereolabs.com/?SN=38085162) to download the calibration file SN38085162.conf. Note that each ZED camera has a different SN code. Use the calibration file corresponding to the error message提示. Upload the calibration file to the `/root/zed/settings/` directory on RDK. If the directory does not exist, create one.

- View the depth map via the web interface by entering http://ip:8000 in the browser (ip is the corresponding IP address of RDK). To view the **point cloud** and **save images**, refer to the corresponding settings above.

## hobot_stereonet Package Description

### Subscribed Topics

| Name               | Message Type                 | Description                                                                                                                    |
| ------------------ | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| /image_combine_raw | sensor_msgs::msg::Image      | Topic for the combined left and right images published by the stereo camera node, used for model inference to generate depth   |
| /camera_info_topic | sensor_msgs::msg::CameraInfo | Topic for the camera parameters published by the stereo camera node, used for converting between disparity maps and depth maps |

### Published Topics

| Name                                 | Message Type                  | Description                                                                |
| ------------------------------------ | ----------------------------- | -------------------------------------------------------------------------- |
| /StereoNetNode/stereonet_depth       | sensor_msgs::msg::Image       | Published depth image, with pixel values representing depth in millimeters |
| /StereoNetNode/stereonet_visual      | sensor_msgs::msg::Image       | Published intuitive visualization rendering image                          |
| /StereoNetNode/stereonet_pointcloud2 | sensor_msgs::msg::PointCloud2 | Published point cloud depth topic                                          |

### Other Important Parameters

| Name                   | Value                                | Description                                                                                                                                                                                            |
| ---------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| stereo_image_topic     | Default /image_combine_raw           | Topic name for subscribing to stereo image messages                                                                                                                                                    |
| camera_info_topic      | Default /image_right_raw/camera_info | Topic name for subscribing to camera calibration parameter messages                                                                                                                                    |
| need_rectify           | Default True                         | Whether to specify a custom calibration file for image rectification                                                                                                                                   |
| stereo_calib_file_path | Default stereo.yaml                  | When need_rectify=True, load the calibration file from this path for calibration                                                                                                                       |
| stereo_combine_mode    | Default 1                            | Left and right images are often combined into one image and then published. 1 indicates vertical combination, 0 indicates horizontal combination, indicating how the stereo algorithm splits the image |
| KMean                  | Default 10                           | Number of neighboring points for each point when filtering sparse outliers. The distances to the nearest 10 points are statistically analyzed.                                                         |
| stdv                   | Default 0.01                         | Threshold for determining outliers when filtering sparse outliers. The standard deviation multiplier is set to 0.01.                                                                                   |
| leaf_size              | Default 0.05                         | Sets the unit density of the point cloud, indicating that there is only one point within a 3D sphere with a radius of 0.05 meters.                                                                     |
