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

:::info BSP 源码包

BSP 源码包下载地址参见： [系统软件](../../01_Quick_start/download.md#系统软件)（需要注册登录）

:::


:::tip 商业支持

商业版提供更完整的功能支持、更深入的硬件能力开放和专属的定制内容。为确保内容合规、安全交付，我们将通过以下方式开放商业版访问权限。

商业版本获取流程：
1. 填写问卷：提交您的机构信息、使用场景等基本情况
2. 签署保密协议（NDA）：我们将根据提交信息与您联系，双方确认后签署保密协议
3. 内容释放：完成协议签署后，我们将通过私有渠道为您开放商业版本资料
  
如您希望获取商业版内容，请点击下方链接填写问卷，我们将在 3 ～ 5 个工作日内与您联系：
https://horizonrobotics.feishu.cn/share/base/form/shrcnpBby71Y8LlixYF2N3ENbre
:::
