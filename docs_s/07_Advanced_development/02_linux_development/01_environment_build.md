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

![build_host_target](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/environment_build/build_host_target.png)

## RDK OS 开发指南

### 开发 RDK S100

:::tip
产品未上市前，请联系 FAE 获取。
:::
