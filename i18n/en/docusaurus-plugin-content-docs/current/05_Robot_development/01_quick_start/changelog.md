---
sidebar_position: 6
---
# 5.1.6 Version Release Notes

## tros-humble

### Version: 2.5.2 (2026-03-17)

New Features (`RDK X5` Platform):
- Added epipolar alignment detection for binocular depth estimation algorithm; adjusted subscribed messages to "image_combine_raw/left/camera_info" and "image_combine_raw/right/camera_info" according to hobot_mipi_cam.
- Binocular OCC algorithm now supports MIPI cameras.
- Object detection algorithm supports `yolo26`.
- Added configuration options for `sensevoice_ros2` algorithm, supporting Chinese-English mode configuration.
- Added human hand keypoint and gesture recognition algorithms based on `palm_detection_mediapipe` and `hand_landmarks_mediapipe`.

Feature Changes (`RDK X5` Platform):
- Adapted system version: RDK 3.5.0 (Linux SDK V1.1.2)
- Refactored 'mipi_cam' videobuff management and stitching threads from bitstream acquisition to message publishing. Modified the calibration info topics for stitched images to "image_combine_raw/left/camera_info" and "image_combine_raw/right/camera_info". Refactored EEPROM read processing for X5. Added subdirectory publishing for X5. Added IMU data publishing in conjunction with 132gs.


### Version: 2.4.6 (2026-02-02)

Feature Changes (`RDK S100` platform):
- Adapted to system version `V4.0.5` and `OE` version `V3.7.0`.


### Version: 2.4.5 (2025-10-28)

Bug Fixes (`RDK X5` platform):

- Fixed a bug in the image processing acceleration module [`hobot_cv`](../02_quick_demo/demo_cv.md) where VSE-accelerated image resizing failed.
- Fixed incorrect inference latency statistics in the on-device model inference framework [`dnn_node`](https://github.com/D-Robotics/hobot_dnn.git).
- Fixed runtime loading failures caused by incorrect installation paths of `ros component so` files for MIPI image capture and algorithm modules.
- Optimized [image codec](../02_quick_demo/hobot_codec.md) configurations: removed invalid parameters and added debug-specific configuration parameters.


### Version: 2.4.4 (2025-10-24)

New Features (`RDK S100` platform):

- The on-device algorithm inference framework [`dnn_node`](https://github.com/D-Robotics/hobot_dnn.git) now supports `ROI` inference mode, where the number of output tensors equals `output_size × roi_size`.
- Added a new algorithm example: [Human Detection and Tracking (Ultralytics YOLO Pose)](../03_boxs/body/mono2d_yolo_pose.md).
- Added a new algorithm example: [Human Instance Tracking](../03_boxs/body/reid.md), which extracts human features using a `reid` model and stores, manages, and queries these features via an `SQLite` database. The launch file disables the `ROI`-based human tracking mode in the `mono2d_body_detection` node and restricts input `ROI` dimensions to less than 3.5× the actual model input size.
- Added a new algorithm example: [Hand Keypoint and Gesture Recognition (MediaPipe)](../03_boxs/body/hand_lmk_gesture_mediapipe.md). This implements basic palm detection, pre/post-processing, and `ai msg` publishing; supports MIPI cameras, USB cameras, and local image replay; and supports both zero-copy and non-zero-copy image acquisition methods.
- Added a new algorithm example: [DeepSeek Large Language Model](../03_boxs/generate/hobot_xlm.md), supporting human-robot dialogue. The `RDK S100` and `RDK S100P` platforms now support the `DeepSeek_R1_Distill_Qwen_1.5B` and `DeepSeek_R1_Distill_Qwen_7B` models.


### Version: 2.4.3 (2025-09-15)

New Features (`RDK X5` platform):

- [MIPI Image Capture](../02_quick_demo/demo_sensor.md) now supports multi-channel image acquisition.
- [Stereo MIPI Image Capture](../02_quick_demo/demo_sensor.md) now supports the `sc132gs` stereo camera.


### Version: 2.4.2 (2025-08-29)

New Features (`RDK S100` platform):

- [MIPI Image Capture](../02_quick_demo/demo_sensor.md) now supports the `230ai` stereo module.
- [YOLO Object Detection](../03_boxs/detection/yolo.md) now supports `yolo11` and `yolov12` algorithms; added scripts to launch via `component` mode, supporting 4K image inference from `mipi cam`.
- The [EdgeSAM Everything Segmentation](../03_boxs/segmentation/mono_edgesam.md) algorithm now uses `edgesam` for model inference, supports `nv12` format input, and adds padding to segmentation results for joint visualization in stereo depth examples.
- Added a new algorithm: [Text-to-Image Feature Retrieval](../03_boxs/function/hobot_clip.md) for extracting and retrieving text-image features.
- Added a new algorithm: [DOSOD](../03_boxs/detection/hobot_dosod.md), including Diguas self-developed open-vocabulary detection DOSOD edge deployment package and a reparameterized quantization method for customizing detection categories in DOSOD models.
- Added a new algorithm: [Stereo OCC](../03_boxs/spatial/dstereo_occupancy.md), integrating Diguas stereo `OCC` network.
- The [Vision-Language Model](../04_apps/hobot_llamacpp.md) algorithm now supports the `smolvlm2` model, with image replay and subscription modes; added capability to publish complete topics after `llm` model inference completes.
- Added a new algorithm: [Stereo Depth Estimation](../03_boxs/spatial/hobot_stereonet.md).


### Version: 2.4.1 (2025-07-30)

New Features (`RDK X5` platform):

- [MIPI Image Capture](../02_quick_demo/demo_sensor.md) now supports the `imx415` module.
- The [EdgeSAM Everything Segmentation](../03_boxs/segmentation/mono_edgesam.md) algorithm now uses `edgesam` for model inference, supports `nv12` format input, and adds padding to segmentation results for joint visualization in stereo depth examples.
- Added a new algorithm: [Human Instance Tracking](../03_boxs/body/reid.md), which extracts human features using a `reid` model and stores, manages, and queries these features via an `SQLite` database.
- Added a new algorithm: [Stereo OCC](../03_boxs/spatial/dstereo_occupancy.md), integrating Diguas stereo `OCC` network.
- The [Vision-Language Model](../04_apps/hobot_llamacpp.md) now supports the `smolvlm2` model, with image replay and subscription modes; added capability to publish complete topics after `llm` model inference completes.


### Version: 2.4.0 (2025-05-12)

New Features:

- Added support for the `RDK S100` platform.

### Version: 2.3.3 (2025-04-30)

New Features:

- Added support for the `RDK X5 Module` platform.
- Added an open-source [ASR solution based on `sensevoice_cpp`](../03_boxs/audio/sensevoice_ros2.md), supporting keyword and ASR data streaming.
- Optimized post-processing time for the [Stereo Depth Estimation](../03_boxs/spatial/hobot_stereonet.md) algorithm and added the V2.3 model version.
- Added a new edge-side [Vision-Language Model](../04_apps/hobot_llamacpp.md) algorithm example based on `llama.cpp`.


### Version: 2.3.2 (2025-01-15)

Feature Changes:

- Updated the stereo model in [Stereo Depth Estimation](../03_boxs/spatial/hobot_stereonet.md) to improve depth estimation accuracy.
- Optimized the workflow and web-based visualization effects in the [Multi-Stream Video Analysis](../04_apps/video_boxs.md) algorithm example.
- Removed some non-functional launch files from the [Stereo Utility Package](https://github.com/D-Robotics/hobot_stereonet_utils).

New Features:

- Added [ZED Camera Image Capture](../02_quick_demo/demo_sensor.md) to acquire stereo images as input for stereo depth estimation algorithms.
- Added the [DOSOD algorithm](../03_boxs/detection/hobot_dosod.md), including Diguas self-developed open-vocabulary detection DOSOD edge deployment package.

Bug Fixes:
- Fixed a crash in the post-processing of the [yolov8-seg Image Segmentation](../03_boxs/segmentation/yolov8_seg.md) algorithm caused by bounding boxes exceeding image boundaries.
- Fixed incorrect frame rate statistics in [Image Codec](../02_quick_demo/hobot_codec.md).
- Fixed I2C detection issues in [Stereo MIPI Image Capture](../02_quick_demo/demo_sensor.md) and added LPWM switch configuration.


### Version: 2.3.1 (2024-11-20)

Feature Changes:

- Upgraded the dependent `opencv` version from 3.4.5 to 4.5.4 (the latest release version used in Ubuntu 22.04).

New Features:

- The [Image Publisher Tool](../02_quick_demo/demo_tool.md) now supports publishing `bgr/rgb` format message data and configuring the `frame_id` of published messages.
- The [Human Detection and Tracking Algorithm](../03_boxs/body/mono2d_body_detection.md) now supports configurable subscription topics, `component` mode execution, input image scaling during preprocessing, and launch scripts that support compressed image replay with configurable image paths.
- The [On-Device Algorithm Model Inference and Deployment Framework](https://github.com/D-Robotics/hobot_dnn.git) fixed incorrect inference time calculation in multi-threaded inference and added support for configuring task counts in config files.
- The [Image Codec Node](../02_quick_demo/hobot_codec.md) now uses the `frame_id` from subscribed image messages as the output message `frame_id` and supports frame-dropping control.
- The [Gesture Recognition Algorithm](../03_boxs/body/hand_gesture_detection.md) now supports configurable post-processing thresholds at startup and dynamic gesture recognition.
- Added a new algorithm: [Face Age Detection](../03_boxs/body/mono_face_age_detection.md) for estimating human age.
- Added a new algorithm: [106-Face Landmark Detection](../03_boxs/body/mono_face_landmarks_detection.md) for detecting 106 facial landmarks.
- Added a new [Perception Message Fusion Node](https://github.com/D-Robotics/tros_perception_fusion) to subscribe to multiple topics of type [PerceptionTargets](https://github.com/D-Robotics/hobot_msgs/blob/develop/ai_msgs/msg/PerceptionTargets.msg), perform time alignment and deduplication, and publish a fused topic. See [Multi-Algorithm Inference](../02_quick_demo/ai_predict.md) for usage reference.
- Added a new [Perception Message Filtering Node](https://github.com/D-Robotics/tros_lowpass_filter) using the OneEuroFilter strategy to smooth points and bounding boxes, correcting jitter in detection results for humans, faces, hands, etc. See [Multi-Algorithm Inference](../02_quick_demo/ai_predict.md) for usage reference.
- Added a new [Stereo Utility Package](https://github.com/D-Robotics/hobot_stereonet_utils) for capturing stereo and depth images.
- Added a new [Multi-Stream Video Analysis](../04_apps/video_boxs.md) algorithm example that pulls multiple H.264/H.265 streams via RTSP, performs inference, and visualizes perception results on the web.

Bug Fixes:

- Fixed startup failure of the `imx219` module in [MIPI Image Capture](../02_quick_demo/demo_sensor.md).
- Added hand bounding box expansion in the preprocessing of the [Hand Keypoint Detection Algorithm](../03_boxs/body/hand_lmk_detection.md) to resolve incorrect keypoint outputs.


### Version: 2.3.0 (2024-09-19)

New Features:

- Added support for the `RDK X5` platform.
- Added [Stereo MIPI Image Capture](../02_quick_demo/demo_sensor.md) to data acquisition.
- Added reference algorithms to the algorithm repository: `yolov8` and `yolov10` for [Object Detection](../03_boxs/detection/yolo.md), and `yolov8-seg` for [Image Segmentation](../03_boxs/segmentation/yolov8_seg.md).
- Added the [YOLO-World Algorithm](../03_boxs/detection/hobot_yolo_world.md) for open-vocabulary object detection.
- Added the [Optical Flow Estimation Algorithm](../03_boxs/function/mono_pwcnet.md) for optical flow detection.
- Added the [Everything Segmentation Algorithm](../03_boxs/segmentation/mono_mobilesam.md) for universal segmentation.
- Added the [Text-to-Image Feature Retrieval Algorithm](../03_boxs/function/hobot_clip.md) for text-image feature extraction and retrieval.
- Added the [Stereo Depth Estimation Algorithm](../03_boxs/spatial/hobot_stereonet.md) for vision-based depth estimation.


### Version: 2.2.0 (2024-04-11)

Feature Changes:

- Based on TROS Foxy 2.1.3, adapted to Ubuntu 22.04 and ROS2 Humble.
- Changed the TROS installation path from `/opt/tros` to `/opt/tros/humble` to align with ROS2’s directory structure and naming convention.
- Discontinued the `tros-ros-base` installation package (which included core ROS2 packages like rclcpp, rclpy, ros2cli); standard ROS2 distribution packages are now used, and ROS2 Humble dependencies are automatically installed when installing TROS Humble.
- Enabled ROS2 FastDDS zero-copy communication for modules handling image data, including data acquisition, image codec, and algorithm examples.
- Changed the QoS Reliability policy for zero-copy communication from `RMW_QOS_POLICY_RELIABILITY_RELIABLE` (`rclcpp::QoS()`) to `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT` (`rclcpp::SensorDataQoS()`) to mitigate potential stability risks with zero-copy.
- Refactored `hobot_dnn` to use the lower-level on-device inference framework `libdnn`, replacing `easydnn`.
- Upgraded the speech algorithm SDK in `hobot_audio` to use the lower-level on-device inference framework `libdnn`, replacing `easydnn`.
- Adapted `hobot_trigger` to ROS2 Humble’s rosbag2.

New Features:
- Added Bloom compilation and packaging scripts in `robot_dev_config` for compiling and packaging TROS on ARM platforms.
- Added a `frame_ts_type` configuration option to the `hobot_mipi_cam` node, supporting `realtime` (for communication latency measurement) and `sensor` (default, for sensor timestamp synchronization).
- Added a new `hobot_shm` node for configuring the ROS2 zero-copy environment.

Bug Fixes:
- Fixed compatibility issues introduced by compiler upgrades.
- Fixed path dependency issues in on-device compilation of certain ROS2 packages.

## tros-foxy

### Version: 2.1.3 (2024-03-11)

Feature Changes:

- Changed the data type for JPEG-compressed images from `sensor_msgs::msg::Image` to the standard `sensor_msgs::msg::CompressedImage`, enabling tools like Foxglove and ROS2 rqt to view JPEG images published by TROS. Affects modules: hobot_websocket, hobot_codec, hobot_image_publisher, hobot_usb_cam.
- Unified JPEG/MJPEG configuration options for publishing/subscribing JPEG-compressed images; removed `jpeg-compressed`/`mjpeg-compressed` options. Affects modules: hobot_codec and hobot_usb_cam.
- Introduced the environment variable `TROS_DISTRO` to indicate the TROS distribution. After running `source /opt/tros/setup.bash` or `source /opt/tros/local_setup.bash`, `TROS_DISTRO` is empty. Configuration file paths for modules like hobot_codec, hobot_audio, hobot_mipi_cam, and hobot_usb_cam changed from `/opt/tros/lib` to `/opt/tros/${TROS_DISTRO}/lib`.


### Version: 2.1.2 (2024-01-19)

New Features:

- Refactored `hobot_usb_cam` to support more format configurations and transcoding.
- Updated the voice SDK in `hobot_audio` to support both 2-mic and 4-mic microphone boards; added `micphone_name` configuration for device ID.

Bug Fixes:

- Fixed incorrect `step` field settings in data messages sent by the `hobot_rgbd_cam` node.
- Updated audio playback function calls in `hobot_tts` to resolve playback failures on newer system versions.
- Removed the config device tree file from `hobot_llm` and updated the README; ION memory size can now be set via command-line tools on newer system versions.

### Version: 2.1.1 (2023-11-03)

New Features:

- Added the `hobot_chatbot` node, integrating smart voice, large language models, and text-to-speech modules to enable on-device voice chat functionality.

Bug Fixes:

- Fixed an issue in the text-to-speech `hobot_tts` node where certain characters caused the application to exit unexpectedly.

### Version: 2.1.0 (2023-09-14)

Feature Changes:
- Updated `tros-ros-base` to the latest ROS2 Foxy source code, ensuring compatibility with the newest ROS2 Foxy packages.  
- When using ROS2 Foxy packages, simply run `source /opt/tros/setup.bash`; creating symbolic links via scripts is no longer required.

New Features:

- The text-to-speech node `hobot_tts` now supports a new parameter to specify the audio playback device.
- Added a large language model (LLM) node `hobot_llm`, enabling on-device LLM experiences.
- The image codec node `hobot_codec` now includes a new `in_format` configuration option: `jpeg-compressed`. The subscribed topic data type is automatically selected based on this configuration.

Bug Fixes:

- Fixed an incorrect `step` field setting in RGB format data messages sent by the MIPI camera node `hobot_mipi_cam`.

### Version: 2.0.2 (2023-08-28)

Feature Changes:

- During tros.b installation, the configured ROS2 apt source (`/etc/apt/sources.list.d/ros2.list`) has been switched to Tsinghua University’s mirror, resolving slow download speeds and installation failures for ROS2 packages.

New Features:

- Added permission checks when sourcing tros.b environment setup scripts (`source /opt/tros/setup.bash` and `source /opt/tros/local_setup.bash`). If the current user lacks root privileges, the system will automatically prompt and switch to a root account, preventing tros.b usage failures due to insufficient permissions.
- The intelligent audio algorithm node `hobot_audio` now supports configuring an audio device ID parameter, facilitating secondary development.
- The event-trigger node `hobot_trigger` now supports task assignment to the Trigger module via standard `std_msgs` topics, standardizing Trigger configuration methods.

Bug Fixes:

- Fixed an issue where the accelerated image processing node `hobot_cv` failed during simultaneous crop & resize operations.
- Resolved erroneous error logs output by the MIPI camera node `hobot_mipi_cam` upon startup.
- Fixed invalid launch file configurations for the data visualization message conversion node `hobot_visualization`.

### Version: 2.0-Release (2.0.1) (2023-06-10)

Feature Changes:

- Upgraded voice algorithms to improve ASR (Automatic Speech Recognition) performance.
- Optimized the `model_name` configuration in algorithm examples: `model_name` is now automatically parsed from the model file, preventing model loading failures caused by incorrect manual configuration and improving ease of secondary development.
- Removed the Nav2 package from the tros.b installer. Users should now directly install the latest ROS2 Nav2 package on RDK using `apt`, resolving stability issues present in older Nav2 versions.

New Features:

- Added support for the `RDK Ultra` platform.
- Introduced new nodes including `hobot_trigger` and `hobot_visualization` for triggering events, capturing, and visualizing rosbag data—helping users diagnose, reproduce, and visualize perception and planning/control issues in robotic scenarios. Users can also extend these nodes for custom data-triggering, recording, and real-time transmission features.
- The USB camera capture node now auto-detects USB camera device IDs, lowering the barrier to using USB cameras.
- Added a Visual Inertial Odometry (VIO) algorithm node, enabling low-cost, robust, high-precision robot localization based on visual inputs.
- Added a text-to-speech node `hobot_tts` to convert text into spoken audio.
- Added a LiDAR-based object detection algorithm node `hobot_centerpoint`.
- Added a Bird’s-Eye View (BEV) perception algorithm node `hobot_bev`.
- Added a stereo depth estimation algorithm node `hobot_stereonet`.

Bug Fixes:

- Upgraded `easydnn` (v1.6.1) and `dnn` (v1.18.4) on `RDK X3`, fixing operator crashes and adding support for more operators.
- Fixed incorrect depth data published by the RGBD image capture node.

Other Updates:

- Optimized the human detection and tracking algorithm node to adaptively scale output coordinates based on input image resolution.
- Fixed a compilation failure in the ORB-SLAM3 algorithm caused by an incorrect script path.

### Version: 2.0-Beta (2.0.0) (2023-05-29)

Version 2.0-Beta (2.0.0) is the first release in the 2.x series of tros.b. Users of [tros.b 1.x](https://developer.d-robotics.cc/api/v1/fileData/TogetherROS/index.html) are recommended to upgrade to the 2.x series.

Feature Changes:

- Migrated the code hosting platform from GitLab to GitHub to facilitate broader community participation in secondary development.
- Integrated a more efficient package management mechanism to accelerate version upgrades and simplify robot application deployment.

New Features:

- Added support for the new core board development kit: RDK X3 Module.
- Enhanced `hobot_audio` to output ASR recognition results, simplifying voice application development.

Bug Fixes:

- Fixed a crash in the post-processing stage of the built-in MobileNet_SSD model within `dnn_node` under multi-threaded conditions.
- Resolved model inference failures in `dnn_node` on x86 platforms when using DDR as input.
- Fixed compilation failures of `hobot_codec` and `hobot_image_publisher` on x86 platforms.

Other Updates:

- Updated example launch scripts to reference dependent modules’ launch files and configure parameters accordingly.
- Updated the D-Robotics logo displayed on the WebSocket frontend.