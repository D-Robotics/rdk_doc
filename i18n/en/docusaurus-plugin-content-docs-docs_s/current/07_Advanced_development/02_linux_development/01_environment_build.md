---
sidebar_position: 1
---

# 7.2.1 Development Environment Setup and Build Instructions

This section describes the requirements and setup of a cross-compilation development environment, as well as instructions for downloading source code and building system images.

## Cross-Compilation Development Environment

Cross-compilation is a method of developing and building software on a host machine, where the compiled software is subsequently deployed and executed on a development board.

- **Host Advantages**: The host typically offers higher performance and larger memory capacity, significantly accelerating code builds.
- **Tool Support**: The host can accommodate a wider range of development tools, thereby enhancing development efficiency.

Setting up a cross-compilation environment is an essential step in the development process. The following diagram illustrates the overall architecture of a cross-compilation environment:

![build_host_target](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/environment_build/build_host_target.png)

## RDK OS Development Guide

### Developing for RDK S100

:::tip Commercial Support
The commercial edition provides more comprehensive feature support, deeper hardware capability exposure, and exclusive customization options. To ensure compliance and secure delivery, access to the commercial edition will be granted through the following process:

**Commercial Edition Access Procedure:**
1. **Complete a Questionnaire**: Submit basic information about your organization and intended use case.
2. **Sign a Non-Disclosure Agreement (NDA)**: We will contact you based on your submission, and both parties will sign an NDA upon mutual confirmation.
3. **Content Release**: After the NDA is signed, we will provide access to commercial edition materials via a private channel.

If you wish to obtain the commercial edition, please complete the questionnaire below. We will contact you within 3–5 business days:

Questionnaire Link: https://horizonrobotics.feishu.cn/share/base/form/shrcnJQBMIkRm6K79rjXR0hr0Fg
:::