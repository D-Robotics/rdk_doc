---
sidebar_position: 13
---
# Stereo Depth Algorithm

## Function Introduction

The stereo depth estimation algorithm uses the `StereoNet` model trained on the [SceneFlow](https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html) dataset with the Horizon [OpenExplorer](https://developer.horizon.ai/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/stereonet.html). 

The algorithm input consists of stereo image data, specifically the left and right views. The algorithm output is the disparity map for the left view.

This example uses a MIPI stereo camera as the image data input source. The algorithm inference is performed using the BPU, and the stereo image left view along with the perception results are published as topic messages. The results are rendered in `rviz2` on the PC side.

## Supported Platforms

| Platform  | Operating Mode   | Example Functionality                                       |
| --------- | ---------------- | ----------------------------------------------------------- |
| RDK X5    | Ubuntu 22.04 (Humble) | · Start stereo cameras, infer depth results, and display on the web |

## Materials List

- Stereo Camera

## Usage Instructions

### Function Installation

Run the following command in the terminal on the RDK system to quickly install the necessary software:

tros humble version

```bash
sudo apt update

# Uninstall the existing package:
sudo apt-get remove tros-humble-stereonet-model

# If uninstallation fails, use the force removal command:
# sudo dpkg --remove --force-all tros-humble-stereonet-model

# Install the new package:
sudo apt install -y tros-humble-hobot-stereonet

```

### Start Stereo Image Publishing, Algorithm Inference, and Image Visualization

Run the following command in the terminal on the RDK system to start:

tros humble version

```shell
# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Launch the stereo model launch file, which includes the startup of the algorithm and stereo camera nodes
ros2 launch hobot_stereonet stereonet_model_web_visual.launch.py \
stereo_image_topic:=/image_combine_raw stereo_combine_mode:=1 need_rectify:="True" \
height_min:=0.1 height_max:=1.0 KMean:=10 stdv:=0.01 leaf_size:=0.05

```

Additionally, you can start the nodes using the component method.
```shell 
# Configure the tros.b humble environment
source /opt/tros/humble/setup.bash

# Terminal 1: Launch the stereo model launch file
ros2 launch hobot_stereonet stereonet_model_component.launch.py \
stereo_image_topic:=/image_combine_raw stereo_combine_mode:=1 need_rectify:="True" \
height_min:=0.1 height_max:=1.0 KMean:=10 stdv:=0.01 leaf_size:=0.05

# Terminal 2: Launch the MIPI stereo camera launch file
ros2 launch mipi_cam mipi_cam_dual_channel.launch.py \
mipi_image_width:=1280 mipi_image_height:=640
```
If you want to evaluate the algorithm using local images, you can use the following command to specify the algorithm mode, image data path, and camera parameters. Also, make sure the image data has been undistorted and baseline aligned.
```shell
# Configure tros.b Humble Environment

source /opt/tros/humble/setup.bash

# Enter the algorithm data directory

cd /opt/tros/humble/share/hobot_stereonet/

# Start the stereo model launch file
ros2 launch hobot_stereonet stereonet_model_web_visual.launch.py \
need_rectify:="False" use_local_image:="True" local_image_path:=`pwd`/data/ \
camera_fx:=505.044342 camera_fy:=505.044342 camera_cx:=605.167053 camera_cy:=378.247009 base_line:=0.069046
```
The parameters are defined as follows:

| Name                         | Value                  | Description                               |
| ---------------------------- | ---------------------- | ----------------------------------------- |
| use_local_image              | Default: False         | Whether to enable image injection mode    |
| local_image_path             | -                      | The directory path for image injection    |
| camera_fx                    | -                      | Camera intrinsic parameter                |
| camera_fy                    | -                      | Camera intrinsic parameter                |
| camera_cx                    | -                      | Camera intrinsic parameter                |
| camera_cy                    | -                      | Camera intrinsic parameter                |
| base_line                    | -                      | Baseline distance                         |

The image format is as shown below. The first left image is named `left000000.png`, the second left image is named `left000001.png`, and so on. The corresponding right images are named `right000000.png`, `right000001.png`, and so on. The algorithm processes the images in sequence until all images are processed.

![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/image_format.png)

After a successful start, open RViz2 on the same network computer, subscribe to the relevant topics published by the stereo model node, and you can observe the real-time visual effects of the algorithm:

![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/stereonet_rdk.png)

You can also observe the algorithm's running results on the PC through a browser by entering the address of X5's 8000 port. For example, if the X5 IP address is `192.168.31.111`, enter `192.168.31.111:8000` in the browser:

![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/web_depth_visual.png)

## Interface Description

### Subscribed Topics

| Name                      | Message Type                        | Description                                              |
| ------------------------- | ------------------------------------ | -------------------------------------------------------- |
| /image_combine_raw         | sensor_msgs::msg::Image             | Stereo camera node publishes the combined left and right images topic for model depth inference |

### Published Topics

| Name                                | Message Type                        | Description                                              |
| ----------------------------------- | ------------------------------------ | -------------------------------------------------------- |
| /StereoNetNode/stereonet_pointcloud2 | sensor_msgs::msg::PointCloud2        | Published point cloud depth topic                        |
| /StereoNetNode/stereonet_depth      | sensor_msgs::msg::Image             | Published depth image with pixel values in millimeters    |
| /StereoNetNode/stereonet_visual     | sensor_msgs::msg::Image             | Published intuitive visualized render image               |

### Parameters

| Name                         | Value                  | Description                               |
| ---------------------------- | ---------------------- | ----------------------------------------- |
| stereo_image_topic           | Default: /image_combine_raw | Topic name to subscribe to stereo image messages |
| need_rectify                 | Default: True          | Whether to rectify and undistort stereo data; camera parameters specified in `config/stereo.yaml` |
| stereo_combine_mode          | Default: 1             | Defines how stereo images are combined: 1 for vertical stitching, 0 for horizontal stitching |
| height_min                   | Default: -0.2          | Filter out points with height less than `height_min` in the camera's vertical direction (in meters) |
| height_max                   | Default: 999.9         | Filter out points with height greater than `height_max` in the camera's vertical direction (in meters) |
| KMean                        | Default: 10            | Number of neighboring points considered for each point when filtering sparse outliers; calculates distance to the 10 nearest points |
| stdv                         | Default: 0.01          | Threshold for determining outliers when filtering sparse points; set as a multiple of the standard deviation |
| leaf_size                    | Default: 0.05          | Sets the unit density of the point cloud; indicates only one point in a 3D sphere with a radius of 0.05 meters |

## Algorithm Execution Time
When the log level is set to debug, the program will print the execution time for each stage of the algorithm to help users debug performance bottlenecks.
```shell
ros2 launch hobot_stereonet stereonet_model.launch.py \
stereo_image_topic:=/image_combine_raw stereo_combine_mode:=1 need_rectify:="True" log_level:=debug
```
![stereonet_rdk](/../static/img/05_Robot_development/03_boxs/function/image/box_adv/consume.png)
## Notes
1. The input size for the model is width: 1280, height: 640. The resolution of the images published by the camera should be 1280x640.
2. If the stereo camera publishes images in NV12 format, the stereo image stitching must be done vertically (top-down stitching).

