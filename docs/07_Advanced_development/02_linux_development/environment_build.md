---
sidebar_position: 1
---

# 7.2.1 开发环境搭建及编译说明

本章节介绍交叉编译开发环境的要求及搭建，源码下载和系统镜像的编译方法说明。

## 交叉编译开发环境

交叉编译是一种在主机上开发和构建软件的方法，构建的软件随后被部署到开发板上运行。

- **主机特点**：主机通常具备更高的性能和更大的内存容量，可显著加速代码构建。
- **工具支持**：主机可以安装更多的开发工具，从而提升开发效率。

交叉编译环境的搭建是开发过程中必不可少的一步。下图展示了交叉编译环境的整体框架：

![image-20220329140159441](../../../static/img/07_Advanced_development/02_linux_development/image/environment_build/image-20220329140159441.png)

## RDK OS 开发指南

### 开发 RDK X3

RDK X3 的相关源码托管在 GitHub 的 [rdk-gen 仓库](https://github.com/D-Robotics/rdk-gen)。

- **开发环境搭建**：仓库中的 [README](https://github.com/D-Robotics/rdk-gen/blob/main/README.md) 提供了详细的开发环境配置指南。
- **系统编译方法**：包括搭建开发环境、构建系统镜像、交叉编译工具链配置、软件开发方法以及常见问题的解决方法。

### 开发 RDK X5

RDK X5 的相关源码托管在 GitHub 的 [x5-rdk-gen 仓库](https://github.com/D-Robotics/x5-rdk-gen)。

- **开发环境搭建**：仓库中的 [README](https://github.com/D-Robotics/x5-rdk-gen/blob/main/README.md) 提供了从环境安装到源码编译的完整说明。
- **系统编译方法**：包括搭建开发环境、构建系统镜像、交叉编译工具链配置、软件开发方法以及常见问题的解决方法。

通过以上步骤，您可以完成 RDK OS 系统的开发和编译。详细信息请参考对应的 GitHub 仓库说明文件。
