---
sidebar_position: 1
---

# D-Robotics RDK Suite

This document is the user manual for the D-Robotics RDK Suite. It provides developers with usage instructions and development guidance for products such as RDK X3 (Sunrise X3 Pi), RDK X3 Module (Sunrise X3 Module), RDK X5, and RDK Ultra. The content covers hardware design, system customization, application development, algorithm toolchains, and more.

:::tip 💾 Download Resources Summary

For related download resources, please refer to: [Download Resources Summary](./01_Quick_start/download)

Includes system images, hardware materials, toolchains, and all download resources.

:::

For details on how to use the suite, please refer to the [System Burning](/install_os) section.

:::info RDK X3 Note

- In this document, **RDK X3** refers to the 2.0 and 3.0 system versions. For users who still need the **Sunrise X3 Pi 1.0 system**, please refer to the following links:<br/>
    [Sunrise X3 Pi User Manual](https://developer.d-robotics.cc/api/v1/fileData/documents_pi/index.html)<br/>
    [Sunrise X3 Pi Ubuntu Images](https://archive.d-robotics.cc/downloads/os_images/), select images under the 1.x.x directory<br/>
    [Sunrise X3 Pi Resource Package](https://developer.d-robotics.cc/api/v1/static/fileData/X3%E6%B4%BE%E8%B5%84%E6%96%99%E5%8C%85_20220711175326.zip)<br/>

- **RDK X3 Module** comes with a pre-flashed test system image. To ensure you are using the latest system version, it is recommended to refer to this document to complete [the burning of the latest version of the system image](/install_os).

To check your system version, use the command `cat /etc/version`. For version 2.1.0 and above, you can use the `rdkos_info` command for detailed version information.

:::

## Overview of D-Robotics RDK Suite

**D-Robotics Developer Kits** (RDK Suite) are developer kits based on D-Robotics intelligent chips, including RDK X3 (Sunrise X3 Pi), RDK X3 Module (Sunrise X3 Module), RDK X5, and RDK Ultra.

With the TogetheROS.Bot robot middleware, the RDK Suite helps developers quickly build robot prototypes for evaluation and validation.

This document details how to use the RDK Suite, including setting up the development environment, running sample tutorials, developing applications, and customizing system images. Regardless of the hardware you choose, the content here will provide a consistent user experience.

## Product Introduction

**RDK X3 (Sunrise X3 Pi)** is a full-featured development board with 5Tops edge inference performance. It offers flexible hardware expansion and connectivity options with a variety of sensors and extension components.

**RDK X3 Module (Sunrise X3 Module)** is a compact core module with the same specifications as RDK X3. Its size and interfaces are compatible with the Raspberry Pi CM4 module. With an expansion board, it provides efficient computing and communication capabilities for various applications.

**RDK X5** is a full-featured development board with 10Tops edge inference performance and an 8-core ARM A55 processor. It supports 2 MIPI Camera inputs and 4 USB3.0 ports, offering flexible hardware expansion and connectivity options.


![image-rdk-serials-en](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/image-rdk-serials-en.jpg)

### Operating Temperature and Cooling Recommendations

#### Operating Temperature Range

- Recommended ambient temperature: **-20°C to 60°C**  
    Ensure the device operates within this range for stability and reliability.

#### Chip Thermal Characteristics

- Maximum chip junction temperature: **105°C**
- When the chip junction temperature exceeds **95°C**, the system will automatically throttle to reduce temperature, which may affect performance.

#### Cooling Design Recommendations

- **Development and Testing:** It is recommended to use heatsinks, RDK Case, and cooling fans for additional heat dissipation.
- **Productization:** Perform thorough thermal design evaluation and optimization based on the overall system environment to ensure the chip operates within a reasonable temperature range and avoid performance degradation or system issues due to overheating.

## Document Usage Guide

Below is an overview of the manual's structure to help users quickly understand and utilize the documentation for development and learning.

**1. Quick Start**  
Introduction to system installation and hardware interface usage to help users get started with the development board.

**2. System Configuration**  
Covers configuration steps and tips to ensure the system works properly and meets specific needs, including system upgrades, network, and Bluetooth configuration.

**3. Basic Application Development**  
Introduces pre-installed functional examples such as IO pin control, audio/video capture, and basic multimedia usage.

**4. Algorithm Application Development**  
Describes how to use simple algorithm interfaces in Python and C++, providing easy-to-use APIs and basic usage examples.

**5. Robot Application Development**  
Introduces the robot operating system for robot manufacturers and ecosystem developers, enabling efficient and convenient robot development for competitive intelligent robot products.

**6. Application Development Guide**  
Includes guides for deep learning line-following cars, AMR development, large model applications, and more.

**7. Advanced Development**  
Covers hardware development, system development, multimedia application development, and algorithm development, including hardware design, system configuration and compilation, multimedia usage and debugging, algorithm training, and quantization.

**8. FAQ**  
Answers common questions and issues users may encounter when using the developer kits, providing solutions and tips.

**9. Appendix**  
Provides commonly used commands in RDK OS, including RDK-specific commands for querying key system information.

## Release Notes

### RDK X5

<font color="red">Note:</font>

- **‌Factory Settings** 
The device is pre-installed with the latest matching miniboot firmware. To ensure hardware compatibility, downgrading to older firmware versions is strictly prohibited, as it may cause the device to fail to start.
- **‌Firmware Upgrade Guide‌** 
After a system update, execute the rdk-miniboot-update command to burn the latest miniboot firmware from the system image to the NAND.
- **‌Upgrade Verification Mechanism‌‌** 
Starting from hobot-miniboot_3.0.3, the rdk-miniboot-update tool automatically displays both the current NAND firmware version and the target upgrade version, reducing upgrade risks and ensuring operational safety.

#### TROS Algorithm Upgrade & Version: 3.2.3

System updates:

- **Desktop Display Optimization:** Ubuntu desktop now supports 3D GPU acceleration for smoother visuals.
- **Audio Sub-board Support:** Added support for Waveshare WM8960 Audio HAT and Huaner carrier board for faster voice solution integration.
- **WiFi Driver Upgrade:** Improved connection stability in weak signal environments and optimized auto-reconnect during sleep/wake.
- **Sensor Acquisition Framework Expansion:** Integrated V4L2 framework, supporting imx477, ov5647, imx219 sensors, with more to come.
- **Network & Remote Optimization:** Default to iptables legacy mode, improved VNC display smoothness, and lag-free remote desktop.
- **Interface Enhancement:** Improved CAN interface stability, resolved high-speed data loss, srpi-config tool adds Uart7 support.
- **Storage Compatibility:** Enhanced SD card compatibility, supporting more card models.

Application updates:

- **Voice Capability Enhancement:** Added ASR voice recognition for more efficient voice algorithm development.
- **Stereo Depth Algorithm Upgrade:** Improved depth estimation algorithm for better speed and accuracy.
- **Multimodal Example Integration:** Built-in large model multimodal examples for quick custom application development.

#### Stereo Algorithm Upgrade & Version: 3.1.1

**Key Features:**

- **System Backup:** New rdk-backup tool for one-click system backup and image generation. [rdk-backup introduction](./Appendix/rdk-command-manual/cmd_rdk-backup).
- **Configuration Management:** Use config.txt to configure 40-pin pins during U-Boot initialization for improved system stability.
- **Touchscreen Enhancement:** Added double-tap and long-press (right-click) support for more flexible screen control.
- **Device Tree Overlay Support:** Added 1_wire device tree overlay (dtoverlay) example for custom hardware connections.
- **Stereo Algorithm Upgrade:** StereoNet depth algorithm improved; added ZED camera support for stereo image capture and smart vision systems.
- **New Application: Smart Video Box:** hobot_rtsp_client supports RTSP streaming, decoding, AI inference, and web-based result display for quick edge AI integration.
- **Open Vocabulary Detection: DOSOD:** hobot_dosod provides an on-device open vocabulary detection algorithm for flexible voice interaction.

**Bug Fixes and Optimizations:**

- Model import fix: Resolved efficientnasnet_m_300x300_nv12.bin import issue.
- Compatibility: Improved SD card protocol support.
- Display fix: Fixed black screen issue on portrait displays.
- Post-processing: dnn_node fixed YOLOv8-seg box overflow crash.
- Frame rate statistics: hobot_codec fixed frame rate calculation error.
- File cleanup: hobot_stereonet_utils removed unusable launch files.
- MIPI camera fix: Resolved multi-channel I2C detection and added LPWM switch configuration.

#### Version: 3.1.0

New features:
- Added button sleep and wake-up functionality.
- Enabled 40Pin secondary functions.

Improvements:
- Bug fixes.
- Corrected CAN frequency issues.
- Support for more sensors and resolutions.

<font color="red">Note:</font>

- When upgrading from an older version using `apt update && apt upgrade`, uninstall `tros-humble-stereonet-model` first, then install `tros-humble-hobot-stereonet`.
```shell
sudo apt-get remove tros-humble-stereonet-model
sudo dpkg --remove --force-all tros-humble-stereonet-model
sudo apt install -y tros-humble-hobot-stereonet
```

#### Version: 3.0.1

New features:
- Provided Server version firmware.
- Supported 7 Waveshare MIPI DSI LCD screens for desktop display and touch.
- Allowed non-root users to run sample programs.

Improvements:
- Bug fixes.
- Support for more sensors and resolutions.
- Improved high-resolution display stability.

#### Version: 3.0.0

First release of RDK X5 firmware, based on Ubuntu 22.04, with rich multimedia and algorithm samples, supporting various robot application scenarios.

### RDK X3

#### Version: 3.0.0

New features:

- Ubuntu 22.04 support.

#### Version: 2.1.0

New features:

- Improved srpi-config tool for Wi-Fi, SSH/VNC toggling, 40pin peripheral bus, localization, CPU overclocking, and ION memory size.
- Supported /boot/config.txt for dtoverlay, CPU overclocking, and IO boot state configuration.
- Added yolov5s v6/v7 model samples.

Improvements:

- HDMI display for boot logs and user command line.
- More HDMI resolutions for better compatibility.
- Optimized pre-installed software for Desktop and Server versions, removed redundancies, added essentials like VLC.
- Simplified Desktop menu layout.
- Bluetooth enabled by default.
- Added C++ interface for post-processing to improve efficiency.
- Used udisk2 for auto-mounting USB drives, fixed NTFS access issues.
- Allowed users to retain VNC password files.
- VNC service disabled by default to save resources; can be enabled via srpi-config.
- RDK X3 v2.1 and RDK Module boards run at up to 1.5GHz (normal) and 1.8GHz (overclocked).

Bug fixes:

- Removed redundant Wi-Fi kernel logs.
- Changed apt source domain to archive.d-robotics.cc.

Other updates:

- Chromium browser support, install with `sudo apt install chromium`.

#### Version: 2.0.0

This release brings many anticipated features and improvements for a better development experience and broader application support. Highlights:

Open Source:

- Full OS source code released, including core and functional modules. Developers can freely view and modify the code for customization and optimization.
- Detailed code documentation and comments provided.
- Community contributions and discussions are welcome; source code is maintained on [D-Robotics](https://github.com/D-Robotics).

RDK X3 Module Support:

- Introduced the new core board developer kit, RDK X3 Module.
- RDK X3 Module is smaller and compatible with Raspberry Pi CM4 interface.
- Developers can choose third-party carrier boards to expand functionality and application scenarios.

Other updates:

- Optimized existing features, fixed known issues and vulnerabilities, improved OS stability and performance.
- Revised documentation for more comprehensive and accurate technical information.
- Provided lower-level APIs for secondary development and integration, enabling flexible software customization.
