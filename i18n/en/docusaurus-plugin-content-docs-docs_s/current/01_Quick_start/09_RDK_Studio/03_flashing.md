---
sidebar_position: 3
---

# 1.9.3 System Flashing

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

<Tabs groupId="rdk-type">
<TabItem value="windows" label="Windows">

:::info Note
If your storage device has already completed system flashing, you can skip this chapter and proceed directly to [Add Device](../09_RDK_Studio/04_Device_management/01_hardware_resource.md).
:::

## Flashing Preparation

1. Click `Imager` to pop up the RDK System Installation Tool. It will prompt that the system installation function requires opening RDK Studio with administrator privileges.
   ![Permission Prompt Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_permission.png)

2. Click `OK` to close the pop-up. If RDK Studio is not currently opened with administrator privileges, first close RDK Studio. Return to the desktop, right-click the RDK Studio application icon, select `Run as administrator`. After opening RDK Studio, enter the flashing function interface again.
   ![Open as Administrator Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_usertype.png)

## Image Flashing Guide

1. On the Select RDK Device interface, choose the type of device you want to install the system on. Here, select "RDK S100".
   ![Installation Type Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/en/flashing_select_device.png)

2. Click `Learn More` to jump to a webpage to learn more about RDK device information; Click `Next` to enter the interface for selecting the RDK operating system image.
   ![Select System Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/en/flashing_select_os_image.png)

3. Select the desired image version to install, click `Next`, and enter the tool download and image download interface.
   ![Download Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/en/flashing_download.png)

4. Click `Tool Download` to automatically open the browser and start downloading the flashing tool D-Navigation.
   ![Tool Download Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/flashing_download_tool.png)

5. Click `Image Download` to automatically open the browser and start downloading the system image.
   ![Installation Process Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/en/flashing_download_image.png)

6. The download process requires a certain waiting time; after the image download is complete, please refer to [System Flashing Method](../02_install_os/rdk_s100.md) for flashing.

</TabItem>

<TabItem value="linux" label="Linux">

:::tip
Currently, the RDK Studio Windows system has been officially released. Friends using Linux and Mac, please wait a little while as the developers are typing at full speed.
:::

</TabItem>

<TabItem value="mac" label="Mac">

:::tip
Currently, the RDK Studio Windows system has been officially released. Friends using Linux and Mac, please wait a little while as the developers are typing at full speed.
:::

</TabItem>

</Tabs>