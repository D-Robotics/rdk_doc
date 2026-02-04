# 1.2.2.1 烧录说明


:::warning 注意事项

- 禁止带电时拔插除 USB、HDMI、网线之外的任何设备
- RDK X3 Module通过载板上的电源接口供电，[官方载板](../../accessory.md#rdk-x3-module-配件清单)通过 DC 接口供电，推荐使用认证配件清单中推荐的 12V/2A 适配器。
- 请不要使用电脑 USB 接口为开发板供电，否则会因供电不足造成开发板**异常断电、反复重启**等异常情况。

:::


## 启动模式

RDK X3 Module 支持从 eMMC 和 SD 卡两种模式启动系统。


### 从 SD 卡启动

使用 RDK Studio 工具和 Rufus 工具进行烧录。

#### RDK Studio 工具
  - 支持使用本地镜像和在线下载镜像
  - 支持 Windows、Linux、Mac 系统
  
#### Rufus 工具
  - 支持使用本地镜像
  - 支持 Windows 系统

### 从 eMMC 启动

使用 Rufus 工具 UMS 方式烧录系统镜像。

#### Rufus 工具
  - 支持使用本地镜像
  - 支持 Windows 系统