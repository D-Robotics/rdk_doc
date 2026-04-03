---
sidebar_position: 1
---

# 1.9.7.1 安装并配置 OpenClaw

本章节介绍如何使用 RDK Studio 在 RDK X5/RDK S100 中快速安装和配置 OpenClaw 。

OpenClaw 是一款开源、可自托管的 AI 执行引擎，能够理解自然语言指令并自动执行实际操作，通过配置如飞书、QQ 等通讯工具就能成为你的 24/7 私人数字助理。

其技术架构如下：

![技术架构图](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/Technical_Structure.PNG)

## 使用 RDK Studio 安装 OpenClaw

### 安装 RDK Studio 并添加设备


安装 OpenClaw 之前请先确保已安装 RDK Studio 并已连接设备。
- RDK Studio 下载安装步骤参见 [RDK Studio 下载安装](../02_download.md)。
- 使用 RDK Studio 添加设备参见 [RDK Studio 添加设备](../05_Device_management/01_hardware_resource.md)。



### 安装 OpenClaw

1. 进入 RDK Studio 页面点击左侧的 `OpenClaw` 图标进入配置页面。

    ![进入配置页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/home.png)

2. 需确保当前设备已经联网，如果没有联网点击 `连接 WiFi` 按钮进入 WiFi 连接页面。

    ![连接WiFi](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/wifi.png)

3. WiFi 连接后点击 `确认进入` 按钮，进入首次安装向导页面进行模型配置，在模型厂商平台注册并获取模型 API 基地址和 API Key，本文以阿里的 coding plan 为例，配置如下图所示。

    ![配置模型图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/config_model.png)

4. 点击 `执行安装` 按钮后等待安装完成，也可以点击 `查看安装日志` ，安装状态信息会显示在终端中。安装过程中出现问题会给出修复指令，复制后在终端执行该指令即可继续安装流程。

    ![安装信息终端](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/log_error.png)

5. 验证安装：

   - 安装完成后，快捷操作区提示首次部署已完成。

        ![首次部署已完成](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/install_complete.png)

   - 网关状态显示 `运行中`，若网关状态未出现可点击网关右侧的 `刷新状态` 按钮，或点击右上角工具箱中 `重启网关` 选项的  `执行重启`。

        ![安装完成](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/toolbox.png)

### 验证 OpenClaw 安装

1. 安装完成后点击快捷操作区的 `进入 Web 对话`，即可进入图形对话界面

    ![进入Web对话按钮位置](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/web_conversation.png)

    ![进入Web对话](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/web_conversation_page.png)

2. 如果需要多窗口对话，可以点击快捷操作区的 `进入终端对话`，即可新建终端窗口，终端窗口下可选择 `终端` 或 `对话`。

    ![多窗口对话](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/multi_windows.png)

### 切换模型

1. 点击快捷操作区的 `模型配置`，点击 `+ 添加 Provider` 配置模型厂商和模型信息。

    ![添加Provider](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/provider_info.png)

2. 配置完成后，在模型列表中选择想切换的目标模型，点击 `设为主模型`即可完成切换。

    ![设为主模型图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/main_model.png)

### 更新 OpenClaw 版本

点击右上角的 `工具箱`，点击 `升级 OpenClaw` 选项下的 `升级` 按钮，即可将 OpenClaw 升级到最新版本。

    ![升级OpenClaw](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/update_version.png)

### 卸载 OpenClaw

点击右上角的 `工具箱`，点击 `卸载 OpenClaw` 选项下的 `卸载` 按钮，即可卸载 OpenClaw。

    ![卸载OpenClaw](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/uninstall.png)

### 工具箱

工具箱用于快速执行高频运维操作，减少跨页面切换。

- 网关诊断：即时检测网关状态并写入终端日志。
- 重启网关：用于网关异常后的快速恢复。
- 升级 OpenClaw：将已安装的 OpenClaw 升级到最新版。
- 导出配置快照：导出当前主要配置用于备份。
- 自动修复配置：修复版本升级导致的 schema 不兼容等问题。
- 卸载 OpenClaw：移除 OpenClaw 及其配置，此操作不可恢复。

        ![工具箱](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/tool_box.png)













