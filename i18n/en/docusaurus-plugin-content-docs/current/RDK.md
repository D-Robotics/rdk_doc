---
slug: /
sidebar_position: 1
---

# D-Robotics  RDK Suite

This document is the user manual for the D-Robotics RDK Suite, providing developers with instructions and development guidance for products such as RDK X3, RDK X3 Module, and RDK Ultra. The content covers various aspects such as hardware design, system customization, application development, and algorithm toolchains. Users are welcome to update and experience, please refer to the [Quick Start](/category/installation) section for specific instructions.

:::info Note

- All **RDK X3** mentioned in this document use the Ubuntu 2.0 and 3.0 version operating system.

- The **RDK X3 Module** comes with a pre-burned test version system image, to ensure the use of the latest version of the system, it is recommended to refer to this document to complete [the burning of the latest version of the system image](/installation/install_os).

To check the system version number, you can use the following command `cat /etc/version`.
:::



## Overview of D-Robotics RDK Suite

**D-Robotics Developer Kits**, abbreviated as D-Robotics RDK Suite, is a robot development kit built on the D-Robotics intelligent chip, including RDK X3, RDK X3 Module, and RDK Ultra. In combination with the TogetheROS.Bot robot middleware, the D-Robotics RDK Suite can help developers quickly build robot prototypes and carry out evaluation and verification work.

This document will provide detailed instructions on how to use the D-Robotics RDK Suite, including setting up the development environment, running example tutorials, developing applications, and customizing system images. Regardless of which hardware you choose to use, the content described in this document will provide you with a consistent user experience.

## Product Introduction

**RDK X3 (Sunrise X3)** is a full-featured development board with 5Tops edge inference power. It provides developers with flexible hardware expansion and connection options by supporting a variety of sensors and expansion components.

**RDK X3 Module (Sunrise X3 Module)** is a compact core module that maintains the same specifications as RDK X3 and is compatible with the Raspberry Pi CM4 module in terms of size and interface. By combining with the expansion board, it can provide efficient computing and communication capabilities for various application scenarios.

![image-20230522171439846](../../../../static/img/image-rdk-serials_en.jpg)


## Document Usage Guide

The following will introduce the overall content division of the user manual, helping users quickly understand the structure and content of the document in order to better utilize the document for development and learning work.

**[System Installation and Login](/category/installation)**  
Introducing system installation and beginner's guide to hardware interface usage, helping users quickly get started with the development board.

**[System Configuration](/category/configuration)**  
Introducing a series of configuration steps and tips to ensure that the system works properly and meets specific requirements. It guides users to configure the system, including system upgrades, network, and Bluetooth configurations.**[First Application](/category/first_application)**  
Introduces pre-installed functional examples in the system, such as IO pin control, video capture, and algorithm inference.

**[Python Development Guide](/category/python_development)**  
Introduces the usage of Python language version's simplified interfaces for video, image, and algorithm. This interface is simple and easy to use, allowing users to quickly get started. It is based on encapsulating lower-level multimedia interfaces.

**[C/C++ Development Guide](/category/clang_development)**  
Introduces the usage of C/C++ language version's simplified interfaces for video, image, and algorithm, as well as the libdnn algorithm interface library. This chapter also provides application examples of C/C++ on the RDK X3 development board to help users develop more rapidly.

**[Linux Development Guide](/category/linux_development)**  
Introduces relevant content of operating system software development, including installation and configuration of development environment, compilation and building methods for Ubuntu platform, driver development, system debugging and optimization, and more.

**[Multimedia Development Guide](/category/multimedia_development)**  
Introduces the usage of video, image, and multimedia low-level interfaces, covering technologies and examples in image processing, audio processing, video processing, video codec/decoding, etc. The interface functions are rich and can implement complex and flexible functional requirements.

**[Hardware Development Guide](/category/hardware_development)**  
Introduces the hardware specifications, interfaces, design files, and design guidelines for RDK X3 (Sunrise X3), RDK X3 Module (Sunrise X3 Module), and RDK Ultra. It provides design materials such as specifications, schematics, dimension drawings, etc.

**[Algorithm Toolchain Development Guide](/category/toolchain_development)**  
Introduces the usage of D-Robotics's algorithm quantization toolchain, covering commonly used algorithm models, usage of development tools, optimization techniques, and more.

**[Common Questions](/category/common_questions)**  
This chapter answers common questions and doubts that users may encounter when using the developer kit. It provides solutions and tips to help users solve common problems and carry out development work smoothly.

## Version Release History

### Version: 2.1.0

New Features:

- Improved srpi-config system configuration tool, supporting Wi-Fi connections, enabling/disabling SSH and VNC, enabling/disabling peripheral buses on the 40-pin connector, language localization configuration, CPU overclocking, ION memory size configuration, and more.
- Support for /boot/config.txt system configuration file, supporting options such as dtoverlay, CPU overclocking, and IO boot state configuration.
- Added yolov5s v6/v7 model examples.

Enhancements:

- Support for outputting boot logs and entering the user command-line interface on HDMI displays to facilitate user use.
- Support for more HDMI display resolutions, greatly enhancing compatibility.
- Optimized pre-installed software lists for Desktop and server versions, removing redundant items and adding necessary software, such as VLC.
- Optimized layout of the Desktop menu bar, simplifying options.
- Bluetooth functionality is enabled by default.
- Added C++ interface for post-processing, improving post-processing efficiency.
- Automatically mount USB flash drives using udisk2, solving the problem of not being able to access NTFS file systems after automatic mounting.
- Support for retaining VNC password file.
- VNC service is not automatically started by default to reduce system resource consumption. Users can enable it through the srpi-config tool.
- RDK X3 v2.1 and RDK Module development board's CPU can run at a maximum frequency of 1.5GHz in normal mode and 1.8GHz after overclocking.

Bug Fixes: 
- Remove redundant kernel logs for Wi-Fi drivers.
- Modify apt source domain to sunrise.D-Robotics.cc.

Other updates:

- Support for the Chromium browser, users can install and use it with `sudo apt install chromium`.

### Version: 2.0.0

This release brings many anticipated features and improvements, aiming to provide a better development experience and broader application support. Here are the main highlights of this version release:

Open-source:

- We have fully open-sourced the source code of the operating system, including the source code of system core modules and functional modules. Developers can freely view and modify the source code, providing greater flexibility for customization and optimization.
- Detailed code documentation and comments will be provided to developers to help them better understand and use the source code.
- We welcome developers to participate in code contribution and discussions through the open-source community, together driving the improvement and optimization of the operating system. The source code is maintained on [D-Robotics](https://github.com/D-Robotics).

Support for RDK X3 Module:

- We introduce a brand new core board development kit, the RDK X3 Module.
- The RDK X3 Module has a smaller size and is compatible with the Raspberry Pi CM4 interface.
- Developers can choose compatible third-party carrier boards according to their needs to expand the functionality and application scenarios of the core board.

Other updates:

- We have optimized existing functions, fixed known issues and vulnerabilities, improving the stability and performance of the operating system.
- Revised the documentation and help documents, providing more comprehensive and accurate technical information and guidelines.
- We provide lower-level APIs to facilitate developers for secondary development and integration, enabling them to customize software more flexibly.