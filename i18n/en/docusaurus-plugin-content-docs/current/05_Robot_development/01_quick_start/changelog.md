---
sidebar_position: 6
---

# 5.1.6 Version Release Notes

## tros-humble

### Version: 2.2.0

Functionality changes:
- Based on TROS Foxy 2.1.3 version, adapted to Ubuntu 22.04 system and ROS2 Humble.
- The installation path of TROS has been changed from **`/opt/tros`** to **`/opt/tros/humble`**, which is consistent with the installation path level and naming of ROS2.
- The `tros-ros-base` installation package (including ROS2 basic function packages such as rclcpp, rclpy, ros2cli, etc.) is no longer provided. The standard ROS2 distribution package is used. The dependent ROS2 Humble is automatically installed when installing TROS Humble.
- Use the zero-copy communication function of ROS2 fastdds, which involves data collection, image encoding and decoding, algorithm examples and other modules that use image data.
- The Reliability of QoS used in zero-copy communication has been changed from `RMW_QOS_POLICY_RELIABILITY_RELIABLE` (rclcpp::QoS()) to `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT` (rclcpp::SensorDataQoS()) to avoid potential stability risks when using zero-copy.
- Refactor `hobot_dnn` to use the lower-level board-side inference framework `libdnn` instead of `easydnn`.
- `hobot_audio` upgrades the speech algorithm SDK and uses the lower-level board-side inference framework `libdnn` instead of `easydnn`.
- `hobot_trigger` adapts to ROS2 Humble version rosbag2.

New features:
- `robot_dev_config` adds bloom compilation and packaging scripts, which are used to compile and package TROS for the ARM platform.
- `hobot_mipi_cam` node adds frame_ts_type configuration item, which supports realtime (used to calculate communication delay) and sensor (default, used for sensor timestamp synchronization) configuration parameters.
- Added `hobot_shm` node for configuring ROS2 zero-copy environment.

Bug fixes:
- Fixed compatibility issues introduced by compiler upgrade.
- Fixed the path dependency problem in the ROS2 pkg compiled on the board end.

## tros-foxy

### Version: 2.1.3

Functionality changes:

- The data type used for jpeg compressed images has been changed from `sensor_msgs::msg::Image` to the standard `sensor_msgs::msg::CompressedImage`. It supports using tools such as foxglove and ros2 rqt to view jpeg format images released by TROS. Involving the hobot_websocket, hobot_codec, hobot_image_publisher, hobot_usb_cam modules.
- Unify the use of jpeg/mjpeg configuration items to specify publishing/subscribing of jpeg compressed format images, and delete the jpeg-compressed/mjpeg-compressed configuration items, which involve the hobot_codec and hobot_usb_cam modules.
- The environment variable TROS_DISTRO representing the TROS distribution is introduced. After executing the `source /opt/tros/setup.bash`/`source /opt/tros/local_setup.bash` command, the value of the environment variable `TROS_DISTRO` is empty. The configuration file path used by hobot_codec, hobot_audio, hobot_mipi_cam, hobot_usb_cam and other modules has been changed from `/opt/tros/lib` to `/opt/tros/${TROS_DISTRO}/lib`.
  
### Version: 2.1.2

New features:

- Refactored hobot_usb_cam to support more format configurations and transcoding.
- Updated hobot_audio to support both 2-mic and 4-mic microphone boards; added microphone_name configuration for device ID.

Bug fixes:

- Fixed an issue in the hobot_rgbd_cam node where the step field of the data message was set incorrectly.
- Updated the audio playback function call in hobot_tts to fix playback failure on the new version of the system.
- Deleted the config device tree file in hobot_llm, and updated README; the new version of the system can now set the size of ION memory through command tools.

### Version: 2.1.1

New features:

- Added hobot_chatbot node, which calls intelligent speech, large language model, and text-to-speech modules to implement on-board voice chatting.

Bug fixes:

- Fixed an issue in the text-to-speech hobot_tts node where certain characters caused the application to exit.

### Version: 2.1.0

Functionality changes:

- Updated tros-ros-base to the latest ROS2 foxy source code, making it compatible with the latest ROS2 foxy packages.
- To use the ROS2 foxy packages, simply run `source /opt/tros/setup.bash`. There is no longer a need to use the script to create symbolic links.

New features:

- Added a parameter to the text-to-speech hobot_tts node to specify the audio playback device.
- Added a new large language model hobot_llm node for on-device LLM experience.
- Added the "jpeg-compressed" configuration option to the in_format parameter of the hobot_codec node for image encoding and decoding. The data type of the subscribed topics is now selected based on the configuration option.

Bug fixes:

- Fixed an issue in the MIPI image acquisition hobot_mipi_cam node where the step field of the RGB format data message was set incorrectly.

### Version: 2.0.2

Functionality changes:- Configured ROS2 source (`/etc/apt/sources.list.d/ros2.list`) during the installation of tros.b has been changed to the Tsinghua mirror source, resolving the issue of slow and failed installation of ROS2 packages.

New features:

- Added a permission check function during the configuration of the environment in the tros.b script (`source /opt/tros/setup.bash` and `source /opt/tros/local_setup.bash`). If the current account does not have root privileges, it will automatically switch to the root account, resolving the issue of tros.b failing to be used due to insufficient privileges.
- Added a parameter configuration function for the audio device number in the intelligent speech algorithm `hobot_audio` node, facilitating secondary development.
- Added the functionality to send tasks to the Trigger module through std_msg topics in the event-triggered `hobot_trigger` node, standardizing the Trigger configuration method.

Bug fixes:

- Fixed the issue where image processing failed when performing crop&resize processing of images in the accelerated image processing `hobot_cv` node.
- Fixed the issue where error logs were output during the startup of the MIPI image acquisition `hobot_mipi_cam` node.
- Fixed the issue where the launch file configuration of the data visualization message conversion `hobot_visualization` node was invalid.

### Version: 2.0-Release (2.0.1)

Function changes:

- Upgraded the speech algorithm to optimize the ASR (Automatic Speech Recognition) performance.
- Optimized the `model_name` configuration item in the algorithm example, automatically parsing the `model_name` configuration from the model file to resolve the issue of loading model failure caused by parameter configuration errors, improving the usability for algorithm development.
- The tros.b package no longer includes the nav2 package. Users can directly install the latest version of the nav2 package in ROS2 using the apt command, resolving the stability issues in the older versions of nav2.

New features:

- Added support for the `RDK Ultra` platform.
- Added the `hobot_trigger` and `hobot_visualization` nodes for triggering events, capturing and visualizing rosbag data. These nodes help users locate, reproduce, and visualize perception and control issues in robot scenes. Users can also develop custom features for data triggering, recording, and real-time feedback.
- The USB image acquisition node now adapts to the device number of the USB camera, reducing the threshold for using USB cameras.
- Added the Visual Inertial Odometry (VIO) algorithm node, a low-cost and robust robot high-precision localization algorithm based on vision.
- Added the `hobot_tts` node for text-to-speech conversion, enabling speech synthesis from text.
- Added the laser-based object detection algorithm `hobot_centerpoint` node.
- Added the Bird's Eye View (BEV) perception algorithm `hobot_bev` node.
- Added the stereo depth estimation algorithm `hobot_stereonet` node.

Bug fixes:

- Upgraded easydnn (version 1.6.1) and dnn (version 1.18.4) for RDK X3, fixing operator crash issues and supporting more operators.
- Fixed the issue where the depth data published by the RGBD image acquisition node was incorrect.

Other updates:

- Optimized the human detection and tracking algorithm nodes to support adaptive output of algorithm perception results based on the input image resolution.
- Fixed the compilation failure issue caused by incorrect path in the orb_slam3 algorithm compilation script.

### Version: 2.0-Beta (2.0.0)

2.0-Beta (2.0.0) is the first 2.x version of tros.b, and it is recommended for users of the 1.x version of tros.b to upgrade to the 2.x version.Changes in Functionality:

- The code hosting platform has been changed from Gitlab to GitHub, making it easier for more developers to contribute to the project.
- Integration of a more efficient package management mechanism to accelerate version upgrades and make installation of robot applications more convenient.

New Features:

- Support for the new RDK X3 Module development kit for core boards.
- Added voice ASR recognition result output to hobot_audio, facilitating the development of voice applications.

Bug Fixes:

- Fixed a crash issue with the MobileNet_SSD model post-processing in dnn_node when running in multi-threaded scenarios.
- Fixed a model inference failure issue in dnn_node on X86 platforms when using DDR inputs.
- Fixed compilation failures of hobot_codec and hobot_image_publisher on X86 platforms.

Other Updates:

- Updated the launch script for examples to reference dependency modules' launch scripts and configure parameters.
- Updated the horizon logo on the display end of the websocket.