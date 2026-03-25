---
sidebar_position: 1
---

# 1.9.7.1 Install and Configure OpenClaw

This section describes how to use RDK Studio to quickly install and configure OpenClaw on RDK X5/RDK S100.

OpenClaw is an open-source, self-hostable AI execution engine that understands natural language instructions and automatically performs real-world actions. By configuring communication tools such as Feishu or QQ, it can become your 24/7 personal digital assistant.

Its technical architecture is as follows:

![Technical Architecture Diagram](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/Technical_Structure.PNG)

## Install OpenClaw Using RDK Studio

### Install RDK Studio and Add Device

Before installing OpenClaw, ensure that RDK Studio is installed and the device is connected.
- For RDK Studio download and installation steps, refer to [RDK Studio Download and Installation](../02_download.md).
- For adding a device using RDK Studio, refer to [RDK Studio Add Device](../05_Device_management/01_hardware_resource.md).

### Install OpenClaw

1. Enter the RDK Studio page and click the `OpenClaw` icon on the left to access the configuration page.

    ![Access Configuration Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/home.png)

2. Ensure the current device is connected to the internet. If not, click the `Connect WiFi` button to go to the WiFi connection page.

    ![Connect WiFi](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/wifi.png)

3. After connecting to WiFi, click the `Confirm` button to enter the first-time installation wizard page for model configuration. Register on the model provider platform and obtain the model API base URL and API Key. This document uses Alibaba's coding plan as an example, with the configuration shown in the figure below.

    ![Model Configuration Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/config_model.png)

4. Click the `Execute Install` button and wait for the installation to complete. You can also click `View Install Log`; the installation status will be displayed in the terminal. If issues occur during installation, repair instructions will be provided. Copy and execute the instructions in the terminal to continue the installation process.

    ![Installation Information Terminal](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/log_error.png)

5. Verify the installation:

   - After installation, the quick actions area will indicate that the first-time deployment is complete.

        ![First-time Deployment Complete](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/install_complete.png)

   - The gateway status will show `Running`. If the gateway status does not appear, click the `Refresh Status` button to the right of the gateway, or click `Restart` under the `Restart Gateway` option in the toolbox at the top right corner.

        ![Installation Complete](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/toolbox.png)

### Verify OpenClaw Installation

1. After installation, click `Open Web Chat` in the quick actions area to enter the graphical conversation interface.

    ![Enter Web Conversation Button Location](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/web_conversation.png)

    ![Enter Web Conversation](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/web_conversation_page.png)

2. If you need multi-window conversations, click `Open Terminal Chat` in the quick actions area to open a new terminal window. In the terminal window, you can select `Terminal` or `Chat`.

    ![Multi-window Conversation](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/multi_windows.png)

### Switch Models

1. Click `Model Config` in the quick actions area, then click `+ Add Provider` to configure the model provider and model information.

    ![Add Provider](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/provider_info.png)

2. After configuration, select the target model you wish to switch to from the model list and click `Set Primary` to complete the switch.

    ![Set as Main Model Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/main_model.png)

### Update OpenClaw Version

Click the `Toolbox` in the top right corner, then click the `Upgrade` button under the `Upgrade OpenClaw` option to update OpenClaw to the latest version.

    ![Upgrade OpenClaw](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/update_version.png)

### Uninstall OpenClaw

Click the `Toolbox` in the top right corner, then click the `Uninstall` button under the `Uninstall OpenClaw` option to uninstall OpenClaw.

    ![Uninstall OpenClaw](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/uninstall.png)

### Toolbox

Quick access to high-frequency ops, reducing page switching.

- Gateway Diagnosis: Check gateway status and write to terminal log.
- Restart Gateway: Quick recovery after gateway abnormality.
- Upgrade OpenClaw: Upgrade installed OpenClaw to latest version.
- Uninstall OpenClaw: Exports the current main configuration for backup.
- Export Configuration Snapshot: Export current main config for backup.
- Auto Fix Configuration: Fix schema incompatibility after version upgrade.
- Uninstall OpenClaw: Remove OpenClaw and its config. This cannot be undone.


<img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/tool_box.png" 
      style={{ width: '100%', height: 'auto' }}
    />
