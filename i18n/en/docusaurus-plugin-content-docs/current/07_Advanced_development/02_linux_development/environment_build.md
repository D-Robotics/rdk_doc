---
sidebar_position: 1
---
# 7.2.1 Development Environment Setup and Compilation Instructions

This chapter introduces the requirements and setup of the cross-compilation development environment, as well as instructions for downloading the source code and compiling the system image.

## Cross-compilation Development Environment

Cross-compilation is a method of developing and building software on a host machine, with the resulting software deployed to the development board for execution.

- **Host Characteristics**: The host machine typically has higher performance and larger memory capacity, which significantly accelerates the build process.
- **Tool Support**: The host machine can support more development tools, enhancing development efficiency.

Setting up a cross-compilation environment is an essential step in the development process. The following diagram illustrates the overall framework of the cross-compilation environment:

![image-20220329140159441](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/environment_build/image-20220329140159441.png)

## RDK OS Development Guide

### Developing RDK X3

The source code for RDK X3 is hosted in the [rdk-gen repository](https://github.com/D-Robotics/rdk-gen) on GitHub.

- **Setting Up the Development Environment**: The repository's [README](https://github.com/D-Robotics/rdk-gen/blob/main/README_EN.md) provides detailed instructions for configuring the development environment.
- **System Compilation Methods**: This includes setting up the development environment, building system images, configuring the cross-compilation toolchain, software development methods, and solutions to common issues.

### Developing RDK X5

The source code for RDK X5 is hosted in the [x5-rdk-gen repository](https://github.com/D-Robotics/x5-rdk-gen) on GitHub.

- **Setting Up the Development Environment**: The repository's [README](https://github.com/D-Robotics/x5-rdk-gen/blob/main/README_EN.md) provides a comprehensive guide from environment setup to source code compilation.
- **System Compilation Methods**: This includes setting up the development environment, building system images, configuring the cross-compilation toolchain, software development methods, and solutions to common issues.

By following these steps, you can complete the development and compilation of the RDK OS system. For more details, refer to the respective GitHub repository documentation.