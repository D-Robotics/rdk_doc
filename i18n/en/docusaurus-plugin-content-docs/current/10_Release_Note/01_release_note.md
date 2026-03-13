---
sidebar_position: 1
---

# RDK X Series Release History

## RDK X5

> **Notes:**  
> - After updating the system, please use the `rdk-miniboot-update` command to update the NAND firmware to the latest version.  
> - Official new images are released regularly, containing the latest feature optimizations and bug fixes. Users can either download and install the latest image or update the system online.

### Version: 3.3.3

#### Version Information

- **System Version**: RDKOS V3.3.3
- **Release Date**: October 2025
- **Platform**: RDK X5

#### Version Update Overview

**(1) System and Driver Updates**

- **Added YuGuang-SC132GS stereo camera module driver**
- **srpi-config functionality enhancement**: Supports automatic interface pin multiplexing, added MIPI screen selection
- **GPU desktop stability improvements**
- **Wi-Fi driver upgraded to 2025_0410**, improving wireless connection stability
- **Flash connection interface enhancements**:
  - Supports usage on macOS systems
  - Added RDK Studio USB connection functionality
- **Multiple bug fixes and performance optimizations**

**(2) Documentation Optimizations**

- **V4L2 usage documentation optimized**, added examples and descriptions
- **Display screen usage documentation optimized**, improved typical screen configuration descriptions
- **Other documentation detail optimizations and structural adjustments**

**(3) Ecosystem Support**

The following ecosystem libraries have been released to PyPI:

| Library Name | Version | Update Content |
|--------------|---------|----------------|
| hobot-dnn-rdkx5 | 3.0.6 | Added dependency numpy >= 1.26.4 |
| hobot-vio-rdkx5 | 3.0.6 | Fixed cropping not taking effect; sp_open_camera interface default output size is 1920×1080 |

#### TROS Update Notes (V2.4.3)

- **Monocular MIPI image acquisition**: Supports starting multiple image acquisition channels
- **Stereo MIPI image acquisition**: Added support for SC132GS stereo camera

#### How to Obtain

RDK X5 V3.3.3 version has been synchronized to:

- D-Robotics Developer Community Download Center, download and upgrade
- `sudo apt update && sudo apt upgrade`; direct local upgrade

---

### TROS Algorithm Upgrade & Version: 3.2.3

#### Image Updates

- **20250610**  
  - Updated ROS repository GPG signing key  
  - Supports [real-time kernel switching](../Advanced_development/linux_development/realtime_kernel#x5系列板卡)  
  - Integrated the latest miniboot firmware; after burning NAND with `rdk-miniboot-update`, more memory is released to the system
- **20250604**  
  - Fixed system boot failure issue on Huaner carrier board

#### System Layer Updates

- **Desktop Display Optimization**: Ubuntu desktop now supports 3D GPU accelerated rendering for a smoother visual experience
- **Audio Sub-board Adaptation Added**: Supports Waveshare WM8960 Audio HAT and Huaner carrier board, accelerating voice solution integration
- **WIFI Driver Upgrade**: Enhanced connection stability in weak signal environments, optimized auto-reconnection during sleep/wake
- **Sensor Acquisition Framework Expansion**: Integrated V4L2 framework, adapted for imx477, ov5647, imx219; more sensor support coming soon
- **Network & Remote Optimization**: Default switched to iptables legacy mode, improved VNC display smoothness, remote desktop is lag-free
- **Interface Enhancement**: CAN interface stability optimized, solved high-speed data packet loss; `srpi-config` tool adds Uart7 support for better serial expansion
- **Storage Compatibility Optimization**: Improved SD card compatibility, supports more storage card models

#### Application Layer Updates

- **Voice Capability Enhancement**: Added ASR voice recognition solution for more efficient voice algorithm development
- **Stereo Depth Algorithm Upgrade**: Optimized depth estimation algorithm, significantly improved detection speed and accuracy
- **Multimodal Demo Integration**: Built-in edge large model multimodal demos, enabling custom application solutions in 3 minutes

---

### Stereo Algorithm Upgrade & Version: 3.1.1

#### Core Features

- **System Backup**: Brand new `rdk-backup` tool for one-click system backup and easy image generation. See [rdk-backup introduction](../Appendix/rdk-command-manual/cmd_rdk-backup)
- **Configuration Management**: Supports using `config.txt` to configure 40pin pin initialization state during U-Boot, improving system boot stability
- **Touchscreen Enhancement**: Added double-tap and long-press actions; long-press simulates right-click for more flexible screen control
- **Device Tree Overlay Support**: Added 1_wire device tree overlay (dtoverlay) example for more hardware customization options
- **Stereo Algorithm Upgrade**: StereoNet depth algorithm upgraded for significantly improved depth results; added ZED camera support for stereo image acquisition, easily build intelligent vision systems with StereoNet
- **New Application: Smart Video Box**: `hobot_rtsp_client` supports RTSP streaming, decoding, intelligent inference, and web-based result display for quick edge AI integration
- **Open Vocabulary Detection: DOSOD**: `hobot_dosod` provides an open-vocabulary detection algorithm developed by D-Robotics, enabling flexible voice interaction on edge devices

#### Bug Fixes & Optimizations

- Fixed efficientnasnet_m_300x300_nv12.bin model import issue
- Optimized SD card protocol support for better compatibility
- Fixed black screen issue caused by portrait display
- `dnn_node` fixed YOLOv8-seg post-processing box out-of-bounds crash
- `hobot_codec` fixed frame rate calculation error
- `hobot_stereonet_utils` removed non-startable launch files
- Resolved multi-channel I2C detection issues, added LPWM switch configuration

---

### Version: 3.1.0

#### New Features

- Added key sleep and wake-up function
- Enabled 40Pin secondary functions

#### Optimizations

- Bug fixes
- Corrected CAN frequency anomaly
- Supports more sensors and resolutions

> **Notes:**  
> When upgrading from an older version to this version using `apt update && apt upgrade`, you must first uninstall `tros-humble-stereonet-model` and then install the `tros-humble-hobot-stereonet` package.
>
> ```shell
> sudo apt-get remove tros-humble-stereonet-model
> sudo dpkg --remove --force-all tros-humble-stereonet-model
> sudo apt install -y tros-humble-hobot-stereonet
> ```

---

### Version: 3.0.1

#### New Features

- Provided Server version firmware
- Supported 7 Waveshare MIPI DSI LCD screens for desktop display and touch
- Supported running sample programs as non-root users

#### Optimizations

- Bug fixes
- Supports more sensors and resolutions
- Improved high-resolution display stability

---

### Version: 3.0.0

First release of RDK X5 firmware, based on Ubuntu 22.04, providing rich multimedia and algorithm samples, supporting robot application development for various scenarios.

---

## RDK X3

### Version: 3.0.0

#### New Features

- Supports Ubuntu 22.04

---

### Version: 2.1.0

#### New Features

- Improved `srpi-config` system configuration tool: supports Wi-Fi connection, SSH/VNC toggle, enabling/disabling peripheral buses on 40pin, localization, CPU overclocking, ION memory size, etc.
- Supports `/boot/config.txt` system config file: dtoverlay, CPU overclock, IO boot state, etc.
- Added yolov5s v6/v7 model samples

#### Optimizations

- HDMI display now outputs boot logs and user shell for easier use
- Supports more HDMI resolutions for better compatibility
- Optimized pre-installed software for Desktop/Server, removed redundancy, added essentials like VLC
- Simplified Desktop menu bar
- Bluetooth enabled by default
- Added C++ interface for post-processing to improve efficiency
- Used udisk2 for auto-mounting USB drives, fixed NTFS access issues
- Users can retain VNC password file
- VNC service is off by default to save resources; can be enabled via `srpi-config`
- RDK X3 v2.1 and RDK Module boards run up to 1.5GHz (normal), 1.8GHz (overclocked)

#### Bug Fixes

- Removed redundant Wi-Fi driver kernel logs
- Changed apt source domain to archive.d-robotics.cc

#### Other Updates

- Supports Chromium browser (`sudo apt install chromium`)

---

### Version: 2.0.0

This release brings many anticipated features and improvements for a better development experience and broader application support.

#### Open Source

- Fully open source OS, including core and functional modules. Developers can freely view and modify for customization and optimization
- Detailed code documentation and comments to help developers understand and use
- Community contributions and discussions welcome; source code maintained at [D-Robotics](https://github.com/D-Robotics)

#### RDK X3 Module Support

- Introduced new core board developer kit: RDK X3 Module
- Smaller size, compatible with Raspberry Pi CM4 interface
- Developers can choose third-party carrier boards as needed to expand functions and applications

#### Other Updates

- Optimized existing features, fixed known issues and vulnerabilities, improved OS stability and performance
- Revised documentation for more comprehensive and accurate technical materials and guides
- Provided lower-level APIs for secondary development and integration

---

### Version: 1.0.0

1. `cat /etc/version` shows 1.x.x
2. `rdkos_info` command not available
3. Old system, no open source code, uses old manual, minor bugs fixed in new images
4. D-Robotics source domain and key changed

## TogetheROS.Bot

### tros-humble

#### Version: 2.4.0 (2025-05-12)

**New Features:**

- Supports `RDK S100` platform.

#### Version: 2.3.3 (2025-04-30)

**New Features:**

- Supports `RDK X5 Module` platform.
- Added [ASR open-source solution](../05_Robot_development/03_boxs/function/sensevoice_ros2.md) based on `sensevoice_cpp`, supporting command words and ASR data push.
- [Stereo depth estimation algorithm](../05_Robot_development/03_boxs/function/hobot_stereonet.md) optimized post-processing time, added V2.3 model.
- Added [vision-language model](../05_Robot_development/02_quick_demo/hobot_llamacpp.md) demo based on `llama.cpp`.

#### Version: 2.3.2 (2025-01-15)

**Feature Changes:**

- [Stereo depth estimation algorithm](../05_Robot_development/03_boxs/function/hobot_stereonet.md) updated stereo model, improved depth estimation.
- [Multi-channel video analysis](../05_Robot_development/04_apps/video_boxs.md) demo optimized processing and web visualization.
- [StereoNet utility package](https://github.com/D-Robotics/hobot_stereonet_utils) removed some non-startable launch files.

**New Features:**

- Added [ZED camera image acquisition](../05_Robot_development/02_quick_demo/demo_sensor.md) for stereo image input.
- Added [DOSOD algorithm](../05_Robot_development/03_boxs/function/hobot_dosod.md), providing open-vocabulary detection on edge devices.

**Bug Fixes:**

- Fixed [yolov8-seg image segmentation](../05_Robot_development/03_boxs/segmentation/yolov8_seg.md) post-processing box out-of-bounds crash.
- [Image codec](../05_Robot_development/02_quick_demo/hobot_codec.md) fixed frame rate statistics error.
- [Stereo MIPI image acquisition](../05_Robot_development/02_quick_demo/demo_sensor.md) fixed i2c detection, added lpwm switch config.

#### Version: 2.3.1 (2024-11-20)

**Feature Changes:**

- Upgraded `opencv` dependency from 3.4.5 to 4.5.4 (latest Ubuntu 22.04 release).

**New Features:**

- [Image publisher tool](../05_Robot_development/02_quick_demo/demo_tool.md) supports `bgr/rgb` message publishing and frame_id config.
- [Human detection and tracking](../05_Robot_development/03_boxs/function/mono2d_body_detection.md) supports topic config, component mode, image scaling, compressed image playback, etc.
- [Onboard model inference framework](https://github.com/D-Robotics/hobot_dnn.git) fixed multi-thread inference timing, supports task count config.
- [Image codec Node](../05_Robot_development/02_quick_demo/hobot_codec.md) supports frame_id passing and frame drop control.
- [Gesture recognition](../05_Robot_development/03_boxs/function/hand_gesture_detection.md) supports post-processing threshold config and dynamic gesture recognition.
- Added [face age detection](../05_Robot_development/03_boxs/function/mono_face_age_detection.md).
- Added [face 106 landmark detection](../05_Robot_development/03_boxs/function/mono_face_landmarks_detection.md).
- Added [perception message fusion Node](https://github.com/D-Robotics/tros_perception_fusion) for multi-topic result fusion.
- Added [perception message filter Node](https://github.com/D-Robotics/tros_lowpass_filter) using OneEuroFilter for smoothing.
- Added [StereoNet utility package](https://github.com/D-Robotics/hobot_stereonet_utils).
- Added [multi-channel video analysis](../05_Robot_development/04_apps/video_boxs.md) demo.

**Bug Fixes:**

- [MIPI image acquisition](../05_Robot_development/02_quick_demo/demo_sensor.md) fixed `imx219` module startup failure.
- [Hand landmark detection](../05_Robot_development/03_boxs/function/hand_lmk_detection.md) improved hand box expansion and keypoint output.

#### Version: 2.3.0 (2024-09-19)

**New Features:**

- Supports `RDK X5` platform.
- Data acquisition adds [Stereo MIPI image acquisition](../05_Robot_development/02_quick_demo/demo_sensor.md).
- Algorithm repo adds `yolov8`, `yolov10` [object detection](../05_Robot_development/03_boxs/detection/yolo.md), `yolov8-seg` [image segmentation](../05_Robot_development/03_boxs/segmentation/yolov8_seg.md).
- Added [YOLO-World](../05_Robot_development/03_boxs/function/hobot_yolo_world.md), [optical flow estimation](../05_Robot_development/03_boxs/function/mono_pwcnet.md), [segment anything](../05_Robot_development/03_boxs/function/mono_mobilesam.md), [text-image feature retrieval](../05_Robot_development/03_boxs/function/hobot_clip.md), [stereo depth estimation](../05_Robot_development/03_boxs/function/hobot_stereonet.md).

#### Version: 2.2.0 (2024-04-11)

**Feature Changes:**

- Based on TROS Foxy 2.1.3, adapted for Ubuntu 22.04 and ROS2 Humble.
- TROS install path changed to `/opt/tros/humble`.
- No longer provides `tros-ros-base` package, directly depends on standard ROS2 release.
- Uses ROS2 fastdds zero-copy communication, QoS Reliability set to `BEST_EFFORT`.
- `hobot_dnn`, `hobot_audio` refactored based on `libdnn`.
- `hobot_trigger` adapted for ROS2 Humble rosbag2.

**New Features:**

- `robot_dev_config` adds bloom build script.
- `hobot_mipi_cam` node adds frame_ts_type config.
- Added `hobot_shm` node for ROS2 zero-copy environment config.

**Bug Fixes:**

- Fixed compiler upgrade compatibility issues.
- Fixed board-side build ROS2 pkg path dependencies.

---

### tros-foxy

#### Version: 2.1.3 (2024-03-11)

**Feature Changes:**

- JPEG compressed image type changed from `sensor_msgs::msg::Image` to `sensor_msgs::msg::CompressedImage`, supports foxglove/ros2 rqt tools.
- Unified jpeg/mjpeg config, removed jpeg-compressed/mjpeg-compressed configs.
- Added `TROS_DISTRO` env variable, config path changed to `/opt/tros/${TROS_DISTRO}/lib`.

#### Version: 2.1.2 (2024-01-19)

**New Features:**

- Refactored `hobot_usb_cam`, supports more format configs and transcoding.
- `hobot_audio` updated voice SDK, supports 2mic/4mic boards, adds micphone_name config.

**Bug Fixes:**

- `hobot_rgbd_cam` node fixed step field error.
- `hobot_tts` fixed audio playback failure.
- `hobot_llm` removed config device tree file, updated README, supports ION memory config via command tool.

#### Version: 2.1.1 (2023-11-03)

**New Features:**

- Added `hobot_chatbot` node for on-board voice chat.

**Bug Fixes:**

- `hobot_tts` node fixed exit issue caused by special characters.

#### Version: 2.1.0 (2023-09-14)

**Feature Changes:**

- `tros-ros-base` upgraded to latest ROS2 foxy source.
- Using ROS2 foxy only requires `source /opt/tros/setup.bash`.

**New Features:**

- `hobot_tts` node adds audio device parameter.
- Added `hobot_llm` node for edge LLM.
- `hobot_codec` node adds `jpeg-compressed` config.

**Bug Fixes:**

- `hobot_mipi_cam` node fixed RGB format step field error.

#### Version: 2.0.2 (2023-08-28)

**Feature Changes:**

- ROS2 source switched to Tsinghua mirror.

**New Features:**

- tros.b script adds permission check, auto switches to root.
- `hobot_audio` node adds audio device number parameter.
- `hobot_trigger` node adds std_msg topic trigger.

**Bug Fixes:**

- `hobot_cv` node fixed crop&resize simultaneous processing failure.
- `hobot_mipi_cam` node startup error log issue.
- `hobot_visualization` node launch config invalid issue.

#### Version: 2.0-Release (2.0.1) (2023-06-10)

**Feature Changes:**

- Upgraded voice algorithm, improved ASR.
- Algorithm sample `model_name` config auto parsing.
- tros.b package no longer includes nav2, use apt to install latest nav2.

**New Features:**

- Supports `RDK Ultra` platform.
- Added Trigger event, rosbag visualization node.
- USB image acquisition node supports device number auto-adaptation.
- Added VIO algorithm node.
- Added `hobot_tts` node.
- Added `hobot_centerpoint`, `hobot_bev`, `hobot_stereonet` node.

**Bug Fixes:**

- Upgraded `RDK X3` easydnn/dnn, fixed operator crash.
- Fixed RGBD image acquisition node depth data error.

**Other Updates:**

- Optimized human detection/tracking node for resolution adaptation.
- Fixed orb_slam3 build script path error.

#### Version: 2.0-Beta (2.0.0) (2023-05-29)

2.0-Beta (2.0.0) is the first 2.x version, 1.x users are recommended to upgrade.

**Feature Changes:**

- Code hosting migrated to GitHub.
- Integrated efficient package management.

**New Features:**

- Supports RDK X3 Module.
- `hobot_audio` adds ASR result output.

**Bug Fixes:**

- Fixed dnn_node MobileNet_SSD multi-thread crash.
- Fixed X86 dnn_node, hobot_codec, hobot_image_publisher build failures.

**Other Updates:**

- Updated sample launch scripts.
- Websocket display updated D-Robotics logo.

---

### TogetheROS-V1.x

#### V1.1.6

- Added X86 platform support, including Hobot_DNN, Hobot_codec, etc.
- Fixed audio codec driver auto-load I2C conflict.
- Fixed USB camera video device node change issue.

#### V1.1.5

- Added hobot_audio support for linear/ring mic config.
- Optimized hobot_sensors MIPI Camera timestamp accuracy.
- Added hobot_image_publisher h265 encoded test video.
- Fixed TROS package path dependency.
- Fixed user file overwrite after system update.

#### V1.1.4

- Optimized hobot_sensors for more modules.
- Added VIO raw/yuv image interface.
- Added hobot_image_publisher MP4/H264/H265 video publishing.
- Fixed hobot_dnn post-processing no return value.
- Added hobot_cv benchmark tool.
- Optimized toolchain module, added bug fix guidance.
- Fixed USB3.0 and HDMI capture card compatibility.

#### V1.1.3

- Optimized toolchain, one-click model quantization.
- Added C RTSP streaming decode sample.
- Added hobot_cv image padding and nv12 processing.
- Optimized hobot_codec log output.
- Optimized hobot_sensor, added more camera support.

#### V1.1.2

- Added on-board docker support.
- Added 2D garbage detection sample.
- Added C/C++ multimedia and algorithm dev interfaces.
- Added dnn_node_sample package usage sample.
- Added adaptive multi-core model inference.
- Added perception result confidence info.

#### V1.1.1

- Optimized vision/image module error prompts.
- Optimized smart voice algorithm samples.
- Added DOA sound source localization sample.
- Integrated Navigation2 source.
- User manual adds AI toolchain guide.

#### V1.1.0

- Opened kernel driver build environment.
- Optimized TF card compatibility.
- Added car line-following, auto-parking samples.
- Added more camera adaptations.
- Added Sunyu RGBD module adaptation.
- hobot_codec fixed image crash.
- Supports output frame rate control.
- Added EDID detection, adaptive resolution.
- Added joystick remote control support.
- Added xfs/ntfs filesystem support.

#### V1.0.5

- Added parking scenario algorithm sample.
- Added local image publishing tool.
- Added hobot_cv neon accelerated filtering.
- hobot_dnn fixed inference output frame rate error.
- Supports camera type selection for human detection, etc.
- hobot_codec fixed image step parameter error.
- System software adds restart_network command.

#### V1.0.4

- Added ORB-SLAM3 and optimization.
- hobot_dnn fixed inference errors, upgraded prediction library.
- hobot_cv supports pyramid output.
- System software adds RTSP video hardware decoding, Python API.
- Fixed AI model I2C error, python pwm period setting, spi write/read issues.
- Fixed Ethernet and wifi simultaneous startup error.
- Added 4-mic audio sub-board instructions.

#### V1.0.3

- Added ROS2 vision_opencv adaptation.
- hobot_audio optimized voice recognition, changed default wake word.
- hobot_cv added crop/resize/rotate interfaces.
- hobot_sensor optimized NV12 to RGB efficiency.
- hobot_msgs optimized keypoint info, added audio_msg.

#### V1.0.2

- Added GC4663 mipi camera, RealSense camera adaptation.
- Optimized AI samples, supports usb camera multi-resolution.
- Optimized first boot time.
- Fixed Desktop black screen issue.
- mono2d_body_detection added keypoint confidence.
- audio_control added voice control for robot movement.
- hobot_audio added voice recognition.
- hobot_websocket optimized web display.
- hobot_msgs optimized keypoint info, added audio_msg.

#### V1.0.1

- Updated hobot-arm64-modules, fixed Desktop mouse display.
- HobotDNN optimized async inference.
- HobotCV optimized log output.
- HobotSensor fixed rgbd sensor message publishing.
- HobotWebsocket fixed memory leak.
- HobotCodec optimized codec CPU usage.
- Added HobotHDMI module.
- mono2d_body_detection optimized performance stats and log.
- hand_lmk_detection fixed memory leak and inference performance.
- hand_gesture_detection fixed multi-hand detection and optimized performance.

