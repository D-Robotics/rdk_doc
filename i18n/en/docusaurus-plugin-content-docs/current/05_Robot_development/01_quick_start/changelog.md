---
sidebar_position: 6
---
# 5.1.6 Version Release Notes

## tros-humble

### Version: 2.4.5 (2025-10-28)
Bug Fixes (**RDK X5** Platform):
- Fixed the bug of image resize acceleration with vse in the [**hobot_cv**](/docs/05_Robot_development/02_quick_demo/demo_cv.md) image processing acceleration module.
- Fixed the incorrect inference latency statistics in the on-board model inference framework [**dnn_node**](https://github.com/D-Robotics/hobot_dnn.git).
- Fixed the runtime loading failure caused by incorrect installation paths of `ros component so` in modules such as MIPI image capture and algorithm packages.
- Optimized the configuration of [Image Codec](/docs/05_Robot_development/02_quick_demo/hobot_codec.md), removed invalid configuration parameters, and added debug-specific configuration parameters.

### Version: 2.4.4 (2025-10-24)
New Features (**RDK S100** Platform):
- Added **ROI** inference mode to the on-board algorithm inference framework [**dnn_node**](https://github.com/D-Robotics/hobot_dnn.git); the number of output `output tensor` is `output_size x roi_size`.
- Added the algorithm example of [Human Detection and Tracking (Ultralytics YOLO Pose)](/docs/05_Robot_development/03_boxs/body/mono2d_yolo_pose.md).
- Added the algorithm example of [Human Instance Tracking](/docs/05_Robot_development/03_boxs/body/reid.md): extracts human features based on the `reid` model, and stores, manages and queries features via the `SQlite` database. Disabled the ROI-based human tracking mode of the human detection and tracking node `mono2d_body_detection` in the launch file; restricted the input ROI size to be less than 3.5 times the actual model input size.
- Added the algorithm example of [Hand Keypoint and Gesture Recognition (mediapipe)](/docs/05_Robot_development/03_boxs/body/hand_lmk_gesture_mediapipe.md), which implements basic palm detection, pre/post-processing, and `ai msg` publishing. Supports MIPI cameras, USB cameras, and local image playback; supports zero-copy and non-zero-copy image acquisition.
- Added the algorithm example of [DeepSeek Large Language Model](/docs/05_Robot_development/03_boxs/generate/hobot_xlm.md), supporting human-machine dialogue. **RDK S100** and **RDK S100P** added support for the `DeepSeek_R1_Distill_Qwen_1.5B` and `DeepSeek_R1_Distill_Qwen_7B` models.

### Version: 2.4.3 (2025-09-15)
New Features (**RDK X5** Platform):
- [MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) supports multi-channel image capture startup.
- [Stereo MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) supports the `sc132gs` stereo camera.

### Version: 2.4.2 (2025-08-29)
New Features (**RDK S100** Platform):
- [MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) supports the `230ai` stereo module.
- [Object Detection YOLO](/docs/05_Robot_development/03_boxs/detection/yolo.md) supports the `yolo11` and `yolov12` algorithms; added scripts for startup in `component` mode, supporting 4K image capture for inference via `mipi cam`.
- [EdgeSAM Segment Anything](/docs/05_Robot_development/03_boxs/segmentation/mono_edgesam.md) added `edgesam` for model inference, `nv12` format data input, and padding for segmentation results (for co-visualization in stereo depth cases).
- Added the [Text-Image Feature Retrieval Algorithm](/docs/05_Robot_development/03_boxs/function/hobot_clip.md) for text-image feature extraction and retrieval.
- Added the [DOSOD Algorithm](/docs/05_Robot_development/03_boxs/detection/hobot_dosod.md): added the self-developed open-vocabulary detection DOSOD edge deployment package, and a re-parameterization quantization method for custom modification of detection categories in the DOSOD model.
- Added [Stereo OCC](/docs/05_Robot_development/03_boxs/spatial/dstereo_occupancy.md), integrating the self-developed stereo OCC network.
- [Vision-Language Model](/docs/05_Robot_development/04_apps/hobot_llamacpp.md) added adaptation for the `smolvlm2` model (supporting image playback and subscription mode), and the ability to output complete topics after LLM model inference.
- Added the [Stereo Depth Estimation Algorithm](/docs/05_Robot_development/03_boxs/spatial/hobot_stereonet.md).

### Version: 2.4.1 (2025-07-30)
New Features (**RDK X5** Platform):
- [MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) supports the `imx415` module.
- [EdgeSAM Segment Anything](/docs/05_Robot_development/03_boxs/segmentation/mono_edgesam.md) added `edgesam` for model inference, `nv12` format data input, and padding for segmentation results (for co-visualization in stereo depth cases).
- Added the [Human Instance Tracking Algorithm](/docs/05_Robot_development/03_boxs/body/reid.md): extracts human features based on the `reid` model, and stores, manages and queries features via the `SQlite` database.
- Added the [Stereo OCC Algorithm](/docs/05_Robot_development/03_boxs/spatial/dstereo_occupancy.md), integrating the self-developed stereo OCC network.
- [Vision-Language Model](/docs/05_Robot_development/04_apps/hobot_llamacpp.md) added adaptation for the `smolvlm2` model (supporting image playback and subscription mode), and the ability to output complete topics after LLM model inference.

### Version: 2.4.0 (2025-05-12)
New Features:
- Supported the **RDK S100** platform.

### Version: 2.3.3 (2025-04-30)
New Features:
- Supported the **RDK X5 Module** platform.
- Added an open-source [ASR Solution](/docs/05_Robot_development/03_boxs/audio/sensevoice_ros2.md) based on `sensevoice_cpp`, supporting push of command words and ASR data.
- Optimized the post-processing latency of the [Stereo Depth Estimation Algorithm](/docs/05_Robot_development/03_boxs/spatial/hobot_stereonet.md) and added the V2.3 model version.
- Added an edge-side [Vision-Language Model](/docs/05_Robot_development/04_apps/hobot_llamacpp.md) algorithm example based on `llama.cpp`.

### Version: 2.3.2 (2025-01-15)
Feature Changes:
- Updated the stereo model of the [Stereo Depth Estimation Algorithm](/docs/05_Robot_development/03_boxs/spatial/hobot_stereonet.md) to optimize depth estimation performance.
- Optimized the processing flow and web-side visualization effect of the [Multi-Channel Video Analysis](/docs/05_Robot_development/04_apps/video_boxs.md) algorithm application example.
- Removed non-startable launch files from the [Stereo Auxiliary Package](https://github.com/D-Robotics/hobot_stereonet_utils).

New Features:
- Added [ZED Camera Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) to start the ZED camera for stereo image acquisition (as input for the stereo depth estimation algorithm).
- Added the [DOSOD Algorithm](/docs/05_Robot_development/03_boxs/detection/hobot_dosod.md) and the self-developed open-vocabulary detection DOSOD edge deployment package.

Bug Fixes:
- Fixed the crash caused by box out-of-bounds in the post-processing of the [yolov8-seg Image Segmentation](/docs/05_Robot_development/03_boxs/segmentation/yolov8_seg.md) algorithm.
- Fixed incorrect frame rate statistics in the [Image Codec](/docs/05_Robot_development/02_quick_demo/hobot_codec.md) module.
- Fixed the i2c detection issue in [Stereo MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) and added lpwm switch configuration.

### Version: 2.3.1 (2024-11-20)
Feature Changes:
- Upgraded the dependent `opencv` version from 3.4.5 to 4.5.4 (the latest release for Ubuntu 22.04).

New Features:
- [Image Publishing Tool](/docs/05_Robot_development/02_quick_demo/demo_tool.md) supports publishing `bgr/rgb` format message data and configuring the frame_id of published messages.
- [Human Detection and Tracking Algorithm](/docs/05_Robot_development/03_boxs/body/mono2d_body_detection.md) supports configuring subscribed message topics and running in component mode; the algorithm preprocessing supports inference after scaling input images; the launch script supports compressed image playback and configuring image paths.
- Fixed incorrect inference latency calculation in multi-threaded inference of the [On-Board Algorithm Model Inference and Deployment Framework](https://github.com/D-Robotics/hobot_dnn.git); added support for configuring the number of tasks in the configuration file.
- [Image Codec Node](/docs/05_Robot_development/02_quick_demo/hobot_codec.md) uses the frame_id of subscribed image messages as the frame_id of output image messages and supports publish frame drop control.
- [Gesture Recognition Algorithm](/docs/05_Robot_development/03_boxs/body/hand_gesture_detection.md) supports configuring post-processing thresholds at startup and dynamic gesture recognition.
- Added the [Human Face Age Detection Algorithm](/docs/05_Robot_development/03_boxs/body/mono_face_age_detection.md) for human age detection.
- Added the [Human Face 106 Keypoint Detection Algorithm](/docs/05_Robot_development/03_boxs/body/mono_face_landmarks_detection.md) for detecting 106 facial keypoint information.
- Added the [Perception Message Fusion Node](https://github.com/D-Robotics/tros_perception_fusion): subscribes to multiple topics of type [PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg), performs time alignment and data deduplication, and fuses them into a single topic for publication. See [Multi-Algorithm Inference](/docs/05_Robot_development/02_quick_demo/ai_predict.md) for application.
- Added the [Perception Message Filter Node](https://github.com/D-Robotics/tros_lowpass_filter): adopts the OneEuroFilter strategy to smooth points and bounding boxes, used for position correction of detection boxes and keypoint data (human body, face, hand, etc.) in perception results to fix jitter issues. See [Multi-Algorithm Inference](/docs/05_Robot_development/02_quick_demo/ai_predict.md) for application.
- Added the [Stereo Auxiliary Package](https://github.com/D-Robotics/hobot_stereonet_utils) for acquisition of stereo images and depth images.
- Added the [Multi-Channel Video Analysis](/docs/05_Robot_development/04_apps/video_boxs.md) algorithm application example: pulls multi-channel H264 and H265 streams via the RTSP protocol for inference and visualizes perception results on the web side.

Bug Fixes:
- Fixed the startup failure of the `imx219` module in [MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md).
- Added hand box expansion in the preprocessing of the [Hand Keypoint Detection Algorithm](/docs/05_Robot_development/03_boxs/body/hand_lmk_detection.md) to resolve incorrect keypoint output by the algorithm.

### Version: 2.3.0 (2024-09-19)
New Features:
- Supported the **RDK X5** platform.
- Added [Stereo MIPI Image Capture](/docs/05_Robot_development/02_quick_demo/demo_sensor.md) for data acquisition.
- Added reference algorithms for `yolov8` and `yolov10` [Object Detection](/docs/05_Robot_development/03_boxs/detection/yolo.md), and `yolov8-seg` [Image Segmentation](/docs/05_Robot_development/03_boxs/segmentation/yolov8_seg.md) in the algorithm repository.
- Added the [YOLO-World Algorithm](/docs/05_Robot_development/03_boxs/detection/hobot_yolo_world.md) to the algorithm repository for open-vocabulary input detection.
- Added the [Optical Flow Estimation Algorithm](/docs/05_Robot_development/03_boxs/function/mono_pwcnet.md) to the algorithm repository for optical flow detection.
- Added the [Segment Anything Algorithm](/docs/05_Robot_development/03_boxs/segmentation/mono_mobilesam.md) to the algorithm repository for universal segmentation.
- Added the [Text-Image Feature Retrieval Algorithm](/docs/05_Robot_development/03_boxs/function/hobot_clip.md) to the algorithm repository for text-image feature extraction and retrieval.
- Added the [Stereo Depth Estimation Algorithm](/docs/05_Robot_development/03_boxs/spatial/hobot_stereonet.md) to the algorithm repository, implementing vision-based depth estimation.

### Version: 2.2.0 (2024-04-11)
Feature Changes:
- Adapted to Ubuntu 22.04 and ROS2 Humble based on TROS Foxy 2.1.3.
- Changed the TROS installation path from `/opt/tros` to `/opt/tros/humble`, aligning with the directory hierarchy and naming of ROS2 installation paths.
- Discontinued the `tros-ros-base` installation package (including ROS2 core packages such as rclcpp, rclpy, ros2cli); uses standard ROS2 distribution packages, and dependent ROS2 Humble packages are installed automatically when installing TROS Humble.
- Adopted ROS2 FastDDS zero-copy communication for modules involving image data (data acquisition, image codec, algorithm examples, etc.).
- Changed the QoS Reliability for zero-copy communication from `RMW_QOS_POLICY_RELIABILITY_RELIABLE` (rclcpp::QoS()) to `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT` (rclcpp::SensorDataQoS()) to avoid potential stability risks in zero-copy mode.
- Refactored `hobot_dnn` to use the underlying on-board inference framework `libdnn` instead of `easydnn`.
- Upgraded the speech algorithm SDK in `hobot_audio` to use the underlying on-board inference framework `libdnn` instead of `easydnn`.
- Adapted `hobot_trigger` to ROS2 Humble rosbag2.

New Features:
- Added bloom compilation and packaging scripts to `robot_dev_config` for compiling and packaging TROS on the ARM platform.
- Added the `frame_ts_type` configuration item to the `hobot_mipi_cam` node, supporting `realtime` (for calculating communication latency) and `sensor` (default, for sensor timestamp synchronization) parameters.
- Added the `hobot_shm` node for configuring the ROS2 zero-copy environment.

Bug Fixes:
- Fixed compatibility issues introduced by compiler upgrades.
- Fixed path dependency issues in compiling some ROS2 packages on the board.

## tros-foxy

### Version: 2.1.3 (2024-03-11)
Feature Changes:
- Changed the data type of JPEG-compressed images from `sensor_msgs::msg::Image` to the standard `sensor_msgs::msg::CompressedImage`, supporting viewing TROS-published JPEG images with tools such as Foxglove and ROS2 RQT (involving hobot_websocket, hobot_codec, hobot_image_publisher, hobot_usb_cam modules).
- Unified the use of the `jpeg/mjpeg` configuration item to specify publishing/subscribing JPEG-compressed images; removed the `jpeg-compressed/mjpeg-compressed` configuration items (involving hobot_codec and hobot_usb_cam modules).
- Introduced the `TROS_DISTRO` environment variable for indicating the TROS distribution; the value of `TROS_DISTRO` is empty after executing `source /opt/tros/setup.bash`/`source /opt/tros/local_setup.bash`. Changed the configuration file path for modules such as hobot_codec, hobot_audio, hobot_mipi_cam, and hobot_usb_cam from `/opt/tros/lib` to `/opt/tros/${TROS_DISTRO}/lib`.

### Version: 2.1.2 (2024-01-19)
New Features:
- Refactored `hobot_usb_cam` to support more format configurations and transcoding.
- Updated the speech SDK in `hobot_audio` to support both 2-mic and 4-mic microphone boards; added the `micphone_name` configuration for device ID.

Bug Fixes:
- Fixed the incorrect step field setting in data messages sent by the `hobot_rgbd_cam` node.
- Updated the audio playback function call in `hobot_tts` to resolve playback failures on new system versions.
- Removed the config device tree file from `hobot_llm` and updated the README; the ION memory size can be set via command-line tools on new system versions.

### Version: 2.1.1 (2023-11-03)
New Features:
- Added the `hobot_chatbot` node: calls the intelligent speech, large language model, and text-to-speech modules to implement on-board voice chat functionality.

Bug Fixes:
- Fixed the application exit caused by specific characters in the text-to-speech `hobot_tts` node.

### Version: 2.1.0 (2023-09-14)
Feature Changes:
- Updated `tros-ros-base` to the latest ROS2 Foxy source code, compatible with the latest ROS2 Foxy packages.
- Only `source /opt/tros/setup.bash` is required to use ROS2 Foxy packages; no more script-based soft link creation.

New Features:
- Added a parameter to the text-to-speech `hobot_tts` node for specifying the audio playback device.
- Added the large language model `hobot_llm` node for edge-side LLM experience.
- Added the `jpeg-compressed` configuration item to the `in_format` parameter of the [Image Codec](/docs/05_Robot_development/02_quick_demo/hobot_codec.md) `hobot_codec` node, and the subscribed topic data type is selected based on the configuration item.

Bug Fixes:
- Fixed the incorrect step field setting in RGB format data messages sent by the MIPI image capture `hobot_mipi_cam` node.

### Version: 2.0.2 (2023-08-28)
Feature Changes:
- Changed the ROS2 source configured during tros.b installation (`/etc/apt/sources.list.d/ros2.list`) to the Tsinghua mirror source, resolving slow installation and failure of ROS2 packages.

New Features:
- Added permission check when configuring the environment via tros.b startup scripts (`source /opt/tros/setup.bash` and `source /opt/tros/local_setup.bash`). Automatically switches to the root account if the current account does not have root privileges, resolving tros.b usage failures due to insufficient permissions.
- Added audio device ID parameter configuration to the intelligent speech algorithm `hobot_audio` node for easy secondary development.
- Added the ability to send tasks to the Trigger module via std_msg topics in the event trigger `hobot_trigger` node, standardizing the Trigger configuration method.

Bug Fixes:
- Fixed the processing failure of simultaneous crop&resize in the image acceleration processing `hobot_cv` node.
- Fixed the error log output during startup of the MIPI image capture `hobot_mipi_cam` node.
- Fixed the invalid configuration of the launch file for the data visualization message conversion `hobot_visualization` node.

### Version: 2.0-Release (2.0.1) (2023-06-10)
Feature Changes:
- Upgraded the speech algorithm to optimize ASR (Automatic Speech Recognition) performance.
- Optimized the `model_name` configuration item of algorithm examples: automatically parses the `model_name` configuration from the model file, resolving model loading failures caused by incorrect parameter configuration and improving the ease of secondary development for algorithms.
- Removed the nav2 package from the tros.b installation package; users can install the latest ROS2 nav2 package directly on the RDK via the `apt` command, resolving stability issues with the old nav2 version.

New Features:
- Added support for the **RDK Ultra** platform.
- Added nodes such as `hobot_trigger` and `hobot_visalization` for Trigger event triggering and rosbag data acquisition/visualization, helping users locate, reproduce and visualize perception, planning and control issues in robot scenarios. Users can also perform secondary development to implement data triggering, recording and real-time transmission.
- The USB image capture node automatically adapts to the device ID of USB cameras, lowering the threshold for using USB cameras.
- Added the Visual Inertial Odometry (VIO) algorithm node: implements a low-cost, high-robustness high-precision robot positioning algorithm based on vision.
- Added the text-to-speech `hobot_tts` node for converting text to speech for broadcast.
- Added the LiDAR object detection algorithm `hobot_centerpoint` node.
- Added the BEV perception algorithm `hobot_bev` node.
- Added the stereo depth estimation algorithm `hobot_stereonet` node.

Bug Fixes:
- Upgraded easydnn (v1.6.1) and dnn (v1.18.4) for **RDK X3**, fixing operator crash issues and adding support for more operators.
- Fixed incorrect depth data published by the RGBD image capture node.

Other Updates:
- Optimized the human detection and tracking algorithm node to support adaptive output of algorithm perception result coordinates based on input image resolution.
- Fixed the compilation failure caused by incorrect path in the orb_slam3 algorithm compilation script.

### Version: 2.0-Beta (2.0.0) (2023-05-29)
2.0-Beta (2.0.0) is the first 2.x version of tros.b. Users of the [1.x version of tros.b](https://developer.d-robotics.cc/api/v1/fileData/TogetherROS/index.html) are recommended to upgrade to the 2.x version.

Feature Changes:
- Changed the code hosting platform from GitLab to GitHub for easier secondary development by more developers.
- Integrated a more efficient package management mechanism to speed up version upgrades and simplify robot application installation.

New Features:
- Supported the new core board development kit **RDK X3 Module**.
- Added speech ASR recognition result output to `hobot_audio` for easy development of speech applications.

Bug Fixes:
- Fixed the crash of the MobileNet_SSD model post-processing built into `dnn_node` in multi-threaded scenarios.
- Fixed the model inference failure with DDR input in `dnn_node` on the X86 platform.
- Fixed the compilation failure of `hobot_codec` and `hobot_image_publisher` on the X86 platform.

Other Updates:
- Updated the launch scripts of examples: applications reference the launch scripts of dependent modules and configure parameters.
- Updated the D-Robotics logo on the display side of websocket.
