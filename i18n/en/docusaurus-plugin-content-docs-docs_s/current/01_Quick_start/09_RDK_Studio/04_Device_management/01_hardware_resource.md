---
sidebar_position: 2
---

# Adding an RDK Device

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

<Tabs groupId="rdk-type">
<TabItem value="Windows" label="Windows">

1. Click `+Hardware` in the upper right corner to enter the connection type selection interface.
   
   ![+ RDK Device Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device.png)


2. Click the question mark icon next to "Select Connection Type." A connection guide window will pop up, instructing the user on the connection method corresponding to the selected type. Click `Next` to view subsequent content, or click `Previous` to review previous prompts again. After viewing all guide content, click `Finish` to close the pop-up. You can also click the `×` in the upper right corner at any time to close the pop-up directly.
  
   ![Guide Window Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_help.png)

3. Taking "Create Ethernet Connection" as an example, click to select the connection type, then click `Next` to enter the network selection interface.
    
    :::warning
    - Be sure to select the correct <font color="red">Ethernet</font> here. Selecting the wrong one may cause the computer's own network to become unavailable.
    - You can confirm the correct network using the following method: Unplug and then reconnect the Ethernet cable, and the correct Ethernet will be matched automatically.
  
    :::

   ![Select Network Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_choosenet.png)

    :::tip Tip
    
      Only the Type-C interface of RDK X5 supports adding devices via Flash Connect. If you choose the Flash Connect method here, please perform the following steps in advance:
      1.  Confirm the development board's network IP  
        Taking the RDK X5 version 3.0 image as an example (do not use the Beta version), the IP segment corresponding to the Type-C network card is 192.168.128.10.For other version mirrors, you can first try other connection methods and use the `ifconfig` command to check the IP.
      2.  Personal PC network settings  
        Open Control Panel -> Network and Internet -> Network and Sharing Center -> Change adapter settings on the left -> Find the board's Ethernet (Note: Unplugging and replugging the board's connection cable multiple times will help identify which Ethernet belongs to the development board) -> Right-click, select Properties, and fill in as shown below.
    
            ![config_ethernet.png](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_config_ethernet.png)

 :::

1. Click `Next` to enter the user type selection interface. Set the user type for logging into the RDK device here. You can choose "sunrise (normal user privileges)" or "root (superuser privileges)".
    
    
   ![Select User Type Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_usertype.png)


5. Click `Next` to enter the WIFI wireless network connection interface to connect the RDK device to a network. Expand the option list, select the desired network, and enter the password.
   
   
   :::warning
    If you choose "Create WIFI Connection" to add the device, ensure that the laptop and the RDK device are connected to the same local area network.
   :::
   
   
   ![WIFI Wireless Network Connection Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_net.png)


6. Click `Next` to enter the Create RDK Device Entry interface. Fill in the device name and description.

    :::tip Tip

    The device name and description filled in here will appear on the device card after successful addition. In the case of adding multiple devices, they can be distinguished by name.

    :::

   ![Create RDK Device Entry Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_name.png)


7. Click `Confirm` to successfully add the RDK device. The device card and information will be displayed on the list page.

    
   ![Device Card List Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_device_manage_hr_add_device_success.png)

    :::tip Tip
    
    Because connecting to WIFI takes time, "WIFI not found" may be briefly displayed upon completion of device addition. Wait a moment and refresh the card.
    
    :::

</TabItem>

<TabItem value="linux" label="Linux">

:::tip

The RDK Studio Windows system is now officially released. Linux and Mac users, please hold tight—our developers are typing away as fast as they can!

:::

</TabItem>



<TabItem value="mac" label="Mac">

:::tip

The RDK Studio Windows system is now officially released. Linux and Mac users, please hold tight—our developers are typing away as fast as they can!

:::

</TabItem>

</Tabs>