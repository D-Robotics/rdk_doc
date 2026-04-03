---
sidebar_position: 2
---

# 1.9.7.2 Configuring Feishu Bot

This chapter describes how to integrate OpenClaw with the Feishu bot, enabling interaction with the AI assistant through Feishu.

## Creating a Feishu Application

### Creating the Application

1. Log in to the [Feishu Open Platform](https://open.feishu.cn/app?lang=en-US) using your Feishu account.

2. Click `Create Custom App`.

    ![Create Application](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/create_app.PNG)

2. Fill in the application name and description, select an application icon, and click the `Create` button.

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/create_app_info.PNG" 
      style={{ width: '100%', height: 'auto' }}
    />

3. Click `Version Management & Release` → `Create a Version`.

    ![Create Version](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/create_version.PNG)

4. Fill in the version information and click the `Save` button.

    ![Create Version](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/create_version_info.PNG)

### Configuring Application Permissions

1. On the `Permissions & Scopes` page, click the `Batch Import/export scopes` button.

    ![Batch Import](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/import.png)

2. Paste the following JSON configuration to import the required permissions in one click:

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
3. After importing the permissions, click `Next, Review New Scopes`, then click `Add` → `Confirm` to enable the permissions.

### Enabling Bot Capabilities

1. Go to the `Add Features > Bot` page.

    ![Add Application Capabilities](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/add_skill.png)

2. Enable bot capabilities, fill in the "Bot guide" information, and click the `Save` button.

    ![Configure Bot](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/bot_name.png)

## Configuring OpenClaw

### Obtaining Application Credentials

    :::warning Note
    Please keep the App Secret secure and do not share it with others.
    :::
    
On the application's `Credentials & Basic Info` page, copy:
    - App ID
    - App Secret

    ![Credentials & Basic Information](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/base_info.png)

### Configuring Application Credentials

1. Click `Feishu Config` in RDK Studio.

    ![Feishu Configuration](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/config_feishu.png)

2. Enter the App ID and App Secret, and click the `Save Config` button.

    ![Configuration](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/id_secret.png)

### Configuring Event Subscriptions

:::warning Note
If the gateway is not started or the channel is not added, the long connection settings will fail to save.

![Note](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/gateway_warning.png)

:::

1. On the `Events & Callbacks` page, under the Event Configuration tab, select `Receive events through persistent connection` and click the `Save` button.

    ![Events & Callbacks](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/connect.png)

2. Click the `Add Events` button, enter `im.message.receive_v1` in the search box to search for the message reception event, check it, and click the `Add` button.

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/massage_receive.png" 
      style={{ width: '100%', height: 'auto' }}
    />

3. The added event will appear in the `Added Events` list.

    ![Added Events](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/added_matter.png)

### Publishing the Application
1. On the `Version Management & Release` page, open the previously created version and click `Publish`.
2. Wait for administrator approval. Enterprise self-built applications are usually approved automatically.

    ![Confirm Release](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/release.png)

### Pairing Authorization

1. Send any message to the bot in Feishu. By default, the bot will reply with a pairing code.
2. Click `Feishu Config` in RDK Studio, enter the pairing code, and click the `Approve` button.

    ![Submit Authorization](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/openclaw/en/feishu/commit_permission.jpg)

3. Wait for the prompt indicating that Feishu pairing is complete. Once paired, restart and you can communicate via Feishu.