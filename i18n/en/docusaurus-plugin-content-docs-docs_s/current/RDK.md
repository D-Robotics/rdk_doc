---
sidebar_position: 0
---

# D-Robotics RDK Kit

This document serves as the user manual for the D-Robotics RDK Kit Super series, providing developers with usage instructions and development guidance for products such as the RDK S100 and RDK S100P. The content covers multiple aspects including hardware design, system customization, application development, and algorithm toolchains. Users are welcome to refer to this manual; for specific usage instructions, please see the **System Flashing** section.

:::info Note
To check the system version number, users can run the command `cat /etc/version`. For versions 2.1.0 and above, detailed version information can be viewed using the `rdkos_info` command.
:::

## Overview of D-Robotics RDK Kit

**D-Robotics Developer Kits**, abbreviated as RDK Kits, are robot development kits built upon D-Robotics' intelligent chips. The kits include the following hardware products:

- RDK X3 (Sunrise X3 Pi)
- RDK X3 Module (Sunrise X3 Module)
- RDK X5
- RDK Ultra
- RDK S100 Series

Combined with the TogetherROS.Bot robot middleware, the RDK Kits enable developers to rapidly build robot prototypes and carry out evaluation and validation tasks.

This document provides detailed instructions on using the RDK S100 and RDK S100P kits, covering development environment setup, running example tutorials, application development, and system image customization. Regardless of the hardware you choose, the content described here ensures a consistent user experience.

## Product Introduction

The **RDK S100 Series** is a high-performance development kit featuring 80/128 TOPS of on-device AI inference capability and a 6-core ARM Cortex-A78AE processor. It supports dual MIPI camera inputs, four USB 3.0 ports, and two PCIe 3.0 interfaces, fully meeting the requirements of various application scenarios.

![image-rdks100-serials](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/image-rdks100-serials.png)

## Documentation Usage Guide

The following sections outline the overall structure of this user manual to help users quickly understand its organization and content, enabling more effective development and learning.

**1. Quick Start**  
Introduces system installation and basic hardware interface usage to help users get started quickly with the development board.

**2. System Configuration**  
Describes a series of configuration steps and tips to ensure the system operates correctly and meets specific requirements, including system upgrades, network setup, and Bluetooth configuration.

**3. Basic Application Development**  
Covers pre-installed functional examples in the system, such as GPIO pin control, audio/video capture, and introductory multimedia usage.

**4. Algorithm Application Development**  
Explains how to use simplified algorithm interfaces in both Python and C++. These interfaces are easy to use, built on top of lower-level inference APIs, and accompanied by basic usage examples to help users get started quickly.

**5. Robotics Application Development**  
Introduces a robot operating system designed for robot manufacturers and ecosystem developers, aiming to unlock the intelligent potential of robotic applications and empower ecosystem partners and commercial customers to develop competitive intelligent robots efficiently and conveniently.

**6. Application Development Guides**  
Includes comprehensive guides on various applications such as deep learning line-following robots, AMR (Autonomous Mobile Robot) development, and large model applications.

**7. Advanced Development**  
Provides a complete guide covering hardware development, system development, multimedia application development, and algorithm development. It details the entire workflow—from hardware design and system configuration/compilation to multimedia usage/debugging and algorithm training/quantization deployment—offering rich APIs to implement complex and flexible functionalities.

**8. FAQs**  
Addresses common questions and issues users may encounter while using the developer kit, offering solutions and practical tips to help users resolve problems and proceed smoothly with development.

**9. Appendix**  
Lists commonly used commands in RDK OS, including RDK-specific commands that help users retrieve and query key system information.

**10. Release Notes**  
Documents the release history of RDK OS, including version numbers, release dates, and update details, helping users stay informed about system updates and changes.