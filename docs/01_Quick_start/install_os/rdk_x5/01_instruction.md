---
sidebar_position: 1
---

# 1.2.3.1 烧录说明


:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备
- RDK X5 的 Type-C USB 接口仅用作供电
- 选用正规品牌的 USB Type-C 口供电线，否则会出现供电异常，导致系统异常断电的问题
- 请不要使用电脑 USB 接口为开发板供电，否则会因供电不足造成开发板**异常断电、反复重启**等异常情况。
- 更多供电方式参见[PoE 供电使用](../../../07_Advanced_development/01_hardware_development/rdk_x5/POE.md)。

:::


## 系统烧录

RDK X5 支持 SD 卡烧录和 SD 卡在板烧录，可借助 PC 端工具 RDK Studio 和 Rufus 来完成 Ubuntu 系统的烧录工作。

### RDK Studio 工具

- 支持使用本地镜像和在线下载
- 支持 Windows、Linux、Mac 系统

### Rufus 工具

- 支持使用本地镜像
- 支持 Windows 系统
- 支持 SD 卡烧录和 SD 卡在板烧录

## 固件烧录

:::warning 固件烧录说明

- RDK 最小系统存储于 `NAND Flash` 中，包含 `Bootloader（Miniboot、U-Boot）` 等关键启动组件。
- 设备出厂时已预装与硬件匹配的最新 NAND 固件。
- 为确保兼容性与设备稳定性，严禁降级刷入旧版本固件，否则可能导致设备无法正常启动。
- 若您已遇到设备无法启动的情况，请重新烧录 NAND 固件。

:::

### XBurn 工具

- 用于 NAND 固件烧录
- 支持 Windows、Linux、Mac 系统



