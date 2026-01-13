---
sidebar_position: 4
---

# 1.9.4 Flashing the System

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```


<Tabs groupId="rdk-type">
<TabItem value="windows" label="Windows">

:::info Note

If your storage device has already completed the system flashing process, you can skip this chapter and proceed directly to [Adding an RDK Device](../09_RDK_Studio/04_Device_management/01_hardware_resource.md).
:::

## Flashing Preparation

1. Prepare a Micro SD card with at least 16GB capacity and an SD card reader. Connect the Micro SD card to your computer using the card reader.
2. Click `Imager` to open the RDK System Installation Tool. A prompt will indicate that the system installation function requires opening RDK Studio with administrator privileges.
   
   ![Permission Prompt Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_permission.png)

3. Click `OK` to close the pop-up. If RDK Studio is not currently opened with administrator privileges, close RDK Studio first. Return to the desktop, right-click on the RDK Studio application icon, select `Run as administrator`, open RDK Studio, and then enter the flashing function interface again.
   
   ![Run as Administrator Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_usertype.png)



## Selecting a Local Existing Image for Flashing

:::info Note

There are two methods for flashing the system: "Using Local Image" and "Downloading Image Online". If you have not downloaded the target image file to your local machine, please skip this section and go directly to [Selecting and Flashing via RDK Studio Downloaded Image](#selecting-and-flashing-via-rdk-studio-downloaded-image).

:::

1. On the Select RDK Device interface, choose the device type for which you want to install the system. This chapter uses installing the RDK X5 system as an example.
   
   ![Installation Type Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_device.png)
   
2. The system installation methods are divided into "Using TF Card Reader" and "Using Flash Connect (Type-C)". Click the corresponding red buttons to view the device connection tutorials; click `Learn More` to jump to a webpage for more information about RDK devices.
   
   ![Guide Information Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_device_method.png)

3. Click `Finish` to close the tutorial window. Click `Next` to enter the interface for selecting the RDK operating system image.
   
   ![Select System Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_os_image.png)
   
4. Click `Choose custom image`. The file explorer will open automatically. Navigate to the storage path of the image file and double-click to confirm the selection.
   
   ![Image Storage Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_os_image_local.png)


5. Click `Next` to enter the Select Storage Device interface. Check the correct storage device and click `Flash`.
   
   :::warning
   - You can identify the storage device here using the following method: Unplug the storage device and click the refresh button to see which device option disappears from the list; then reconnect the storage device, click the refresh button, and select the storage device that changes (appears/disappears) with the plug/unplug operation.
   - Flashing the system will <font color="red">erase all data on the storage device</font>. Please ensure you select the correct storage device!
   :::

    ![Select Storage Device Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_storage_refresh.png)

6. The system flashing begins.
 
   ![Flashing Process Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_success_local.png)


7. Upon successful flashing, a completion prompt is shown.
  
    
     ![Flashing Complete Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_success.png)

   
## Selecting and Flashing via RDK Studio Downloaded Image

:::warning Caution

If you already have the target image locally, please go directly to [Selecting a Local Existing Image for Flashing](#selecting-a-local-existing-image-for-flashing). Downloading an image file with the same name again via RDK Studio will result in an error!

:::

1. On the Select RDK Device interface, choose the device type for which you want to install the system. This chapter uses installing the RDK X5 system as an example.
   
   ![Installation Type Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_device.png)
   
2. The system installation methods are divided into "Using TF Card Reader" and "Using Flash Connect (Type-C)". Click the corresponding red buttons to view the device connection tutorials; click `Learn More` to jump to a webpage for more information about RDK devices.
   
   ![Guide Information Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_device_method.png)

3. Click `Finish` to close the tutorial window. Click `Next` to enter the interface for selecting the RDK operating system image.
   
   ![Select System Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_os_image1.png)
   
4. Select the image version you want to install, click `Next`, enter the Select Storage Device interface, check the correct storage device, and click `Flash`.
   
   :::warning
   - You can identify the storage device here using the following method: Unplug the storage device and click the refresh button to see which device option disappears from the list; then reconnect the storage device, click the refresh button, and select the storage device that changes (appears/disappears) with the plug/unplug operation.
   - Flashing the system will <font color="red">erase all data on the storage device</font>. Please ensure you select the correct storage device!
   :::

    ![Select Storage Device Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_select_storage_refresh.png)

5. The system image download begins.
 
   ![Installation Process Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_download_osimage.png)

6. The download process requires some waiting time. Once the image download is complete, the system flashing will start automatically. Upon successful completion, an installation finished prompt is shown.
  
    
     ![Flashing Complete Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/flashing_install_success.png)

</TabItem>

<TabItem value="linux" label="Linux">

:::tip

Currently, the RDK Studio Windows system has been officially released. For friends using Linux and Mac, please wait a moment—the development team is rapidly coding!

:::

</TabItem>



<TabItem value="mac" label="Mac">

:::tip

Currently, the RDK Studio Windows system has been officially released. For friends using Linux and Mac, please wait a moment—the development team is rapidly coding!

:::


</TabItem>

</Tabs>