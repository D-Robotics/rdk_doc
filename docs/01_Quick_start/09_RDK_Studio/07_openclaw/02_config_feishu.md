---
sidebar_position: 2
---

# 1.9.7.2 配置飞书机器人

本章介绍如何将 OpenClaw 接入飞书机器人，实现通过飞书与 AI 助手进行交互。

## 创建飞书应用


### 创建应用

1. 使用飞书账号登录 [飞书开放平台](https://open.feishu.cn/app)。

2. 点击 `创建企业自建应用`。

    ![创建应用](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/create_app.PNG)

2. 填写应用名称和描述，选择应用图标，点击 `创建` 按钮。

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/create_app_info.PNG" 
      style={{ width: '100%', height: 'auto' }}
    />

3. 点击 `版本管理与发布` → `创建版本`。

    ![创建版本](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/create_version.PNG)


4. 填写创建版本信息，点击 `保存` 按钮。

    ![创建版本](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/create_version_info.PNG)





### 配置应用权限

1. 在 `权限管理` 页面，点击 `批量导入` 按钮。

    ![批量导入](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/import.png)

2. 粘贴以下 JSON 配置一键导入所需权限：

    ```json
    {
      "scopes": {
        "tenant": [
          "aily:file:read",
          "aily:file:write",
          "application:application.app_message_stats.overview:readonly",
          "application:application:self_manage",
          "application:bot.menu:write",
          "cardkit:card:write",
          "contact:user.employee_id:readonly",
          "corehr:file:download",
          "docs:document.content:read",
          "event:ip_list",
          "im:chat",
          "im:chat.access_event.bot_p2p_chat:read",
          "im:chat.members:bot_access",
          "im:message",
          "im:message.group_at_msg:readonly",
          "im:message.group_msg",
          "im:message.p2p_msg:readonly",
          "im:message:readonly",
          "im:message:send_as_bot",
          "im:resource",
          "sheets:spreadsheet",
          "wiki:wiki:readonly"
        ],
        "user": ["aily:file:read", "aily:file:write", "im:chat.access_event.bot_p2p_chat:read"]
      }
    }
    ```
3. 导入权限后点击 `下一步，确认新增权限`，点击 `申请开通` 权限。

### 启用机器人能力

1. 进入 `添加应用能力 > 机器人` 页面。

    ![添加应用能力](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/add_skill.png)



2. 开启机器人能力，填写 “如何开始使用” 信息，点击 `保存` 按钮。

    ![配置机器人](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/bot_name.png)

## 配置 OpenClaw

### 获取应用凭证

    :::warning 注意
    请妥善保管 App Secret，不要分享给他人。
    :::
    
在应用的 `凭证与基础信息` 页面，复制：
    - App ID
    - App Secret

    ![凭证与基础信息](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/base_info.png)
    


### 配置应用凭证

1. 点击 RDK Studio 中的 `飞书配置`。

    ![飞书配置](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/config_feishu.png)



2. 填入 App ID 和 App Secret，点击 `保存配置` 按钮。

    ![配置](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/id_secret.png)



### 配置事件订阅

:::warning 注意
如果网关未启动或渠道未添加，长连接设置将保存失败。

![注意](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/gateway_warning.png)


:::

1. 在 `事件与回调` 页面，事件配置选项卡下面选择 `使用长连接接收事件`，点击 `保存` 按钮。

    ![事件与回调](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/connect.png)


2. 点击 `添加事件` 按钮，搜索框中输入 `im.message.receive_v1` 搜索接收消息事件，勾选后点击 `添加` 按钮。

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/massage_receive.png" 
      style={{ width: '100%', height: 'auto' }}
    />

3. 添加的事件会显示在 `已添加事件` 列表中。

    ![已添加事件](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/added_matter.png)
    


### 发布应用
1. 在 `版本管理与发布` 页面，打开之前创建的版本，点击 `确认发布`。
2. 等待管理员审批，企业自建应用通常自动通过。

    ![确认发布](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/release.png)

### 配对授权

1. 在飞书中向机器人发送任意消息，默认情况下，机器人会回复一个配对码。
2. 点击 RDK Studio 中的 `飞书配置`，填入配对码，点击 `提交授权` 按钮。

    ![提交授权](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/zh/feishu/commit_permission.jpg)

3. 等待提示飞书配对已完成，完成配对，重启后即可通过飞书交流。