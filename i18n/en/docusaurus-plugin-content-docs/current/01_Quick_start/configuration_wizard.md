---
sidebar_position: 3
---
# 1.3 Getting Started Configuration

:::tip

The getting started configuration methods described in this chapter are only supported on RDK X3 and RDK X3 Module models.

The system version should be no lower than `2.1.0`.

:::

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## Connecting to Wi-Fi

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

Use the Wi-Fi management tool in the top-right corner of the menu bar to connect to Wi-Fi. As shown in the following figure, click on the Wi-Fi name you need to connect to, and then enter the Wi-Fi password in the pop-up dialog box.

![image-20231127111045649](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160653.jpg)

![image-20231127111253803](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160732.jpg)

</TabItem>

<TabItem value="server" label="Server">

Use the srpi-config tool to connect to Wi-Fi.

Execute the command `sudo srpi-config`, select System Options -> Wireless LAN, and enter the Wi-Fi name (`SSID`) and password (`passwd`) as prompted.

![image-20231127112139204](../../../../../static/img/01_Quick_start/image/configuration_wizard/image-20231127112139204.png)

</TabItem>
</Tabs>

## Enabling SSH Service

The SSH login service is enabled by default in the current system version. Users can use this method to toggle the service.

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">
Find the `RDK Configuration` option through the menu bar and click to open it.

![image-20231127112029088](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160737.jpg)

Select Interface Options -> SSH, and follow the prompts to enable or disable the `SSH` service.

![image-20231127115151834](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160740.jpg)

</TabItem>

<TabItem value="server" label="Server">

Execute the `sudo srpi-config` command to enter the configuration menu. Select Interface Options -> SSH, and follow the prompts to enable or disable the `SSH` service.

![image-20231127115009424](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160737.jpg)

</TabItem>

</Tabs>

Please refer to [Remote Login - SSH Login](./remote_login#ssh) for how to use SSH.

## Enable VNC Service

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

Find the `RDK Configuration` option through the menu bar and click to open it.

![image-20231127112029088](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160737.jpg)

Select Interface Options -> VNC, and follow the prompts to enable or disable the `VNC` service. When enabling `VNC`, you need to set a login password, which must be an 8-character string composed of numbers and characters.

![image-20231127112202713](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160744.jpg)

</TabItem>
</Tabs>

Please refer to [Remote Login - VNC Login](./remote_login#vnc-login) for how to use VNC.

## Set Login Mode

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

For the desktop graphical system, there are four login modes available:

1. Start the graphical interface and automatically log in.
2. Start the graphical interface and require manual login by the user.
3. Character terminal, automatically log in.
4. Character Terminal, User Manual Login

Open `RDK Configuration` through the menu bar. Select System Options -> Boot / Auto Login to enter the following configuration options. Select the corresponding options according to your needs.

![image-20231127112703844](../../../../../static/img/01_Quick_start/image/configuration_wizard/image-20231127112703844.png)

It will take effect after restarting.

</TabItem>

<TabItem value="server" label="Server">

Server system, supporting four login modes:

1. Character Terminal, Automatic Login
2. Character Terminal, User Manual Login

Execute the command `sudo srpi-config` to enter the configuration menu. Select System Options -> Boot / Auto Login to enter the following configuration options. Select the corresponding options according to your needs.

It will take effect after restarting.

</TabItem>
</Tabs>

## Set up Chinese Environment

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

Open `RDK Configuration` through the menu bar. Select Localisation Options -> Locale to enter the following configuration.

Step 1: Select the language environment(s) you need (multiple choices), generally choose `en_US.UTF-8 UTF-8` and `zh_CN.UTF-8 UTF-8`. Press Enter to confirm and proceed to the next step.

![image-20231127113356503](../../../../../static/img/01_Quick_start/image/configuration_wizard/image-20231127113356503.png)

Step 2: Select the default language environment, choose `zh_CN.UTF-8 UTF-8` for Chinese environment. Press Enter and wait for a while to complete the configuration.

Step 3: Restart the machine to apply the latest configuration. `sudo reboot`

:::tip

After booting, you will be prompted whether to update the names of several commonly used folders in the home directory.
It is recommended to choose `Don't ask me again` `Keep Old Name`, so that the directory names such as `Desktop Documents Downloads` under the user's working directory will not change with the language environment.

:::

</TabItem>

<TabItem value="server" label="Server">

To enter the configuration menu, execute the command `sudo srpi-config`. Select the option "Localisation Options -> Locale" to enter the following configuration.

Step 1: Select the desired language environment(s) (multiple selection). Usually, selecting both `en_US.UTF-8 UTF-8` and `zh_CN.UTF-8 UTF-8` is sufficient. Press Enter to confirm and proceed to the next step.

![image-20231127113356503](../../../../../static/img/01_Quick_start/image/configuration_wizard/image-20231127113356503.png)

Step 2: Select the default language environment. For the Chinese language environment, select `zh_CN.UTF-8 UTF-8`. Press Enter to confirm and wait for the configuration to complete.

Step 3: Restart the machine to apply the latest configuration. Execute `sudo reboot`.

</TabItem>
</Tabs>

## Set up  Chinese input method

<Tabs groupId="rdk-type">
<TabItem value="desktop" label="Desktop">

First step: Locate the EN input method icon on the desktop, right-click and select Preferences.

![QQ_1IGglEVRSO.png](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160754.jpg)

Step two: Click on 'Input Sources' -> 'Add on the right' -> Select 'Chinese'.

![QQ_MxWDZrZ7Wk.png](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160757.jpg)

Step three: Select "Intelligent Pinyin" and then you can right-click on the EN in the top right corner to choose "Intelligent Pinyin".

![QQ_rICn3iU1Vc.png](../../../../../static/img/01_Quick_start/image/configuration_wizard/20250507-160805.jpg)

</TabItem>
</Tabs>
