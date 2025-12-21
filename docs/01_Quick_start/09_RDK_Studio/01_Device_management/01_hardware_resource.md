---
sidebar_position: 2
---

# 添加 RDK 设备


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

<Tabs groupId="rdk-type">
<TabItem value="Windows" label="Windows">

1. 点击右上角 `+ RDK 设备`，进入选择连接类型界面。
   
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device.png)


2. 点击“选择连接类型”后面的问号图标，弹出连接方式导览窗口，指导用户进行所选连接类型对应的设备连接方法，点击 `下一步` 可查看后续内容，也可通过点击 `上一步` 再次查看之前的提示，查看全部导览内容后点击 `结束导览` 关闭窗口，也可随时点击右上角 ` × ` 直接关闭弹窗。
  
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_help.png)

3. 此处以 “创建网线连接” 为例，点击选中连接类型，点击 `下一步` ，进入选择网络界面。
    
    :::warning
    - 此处请务必谨慎选择网络，确认后再进行下一步，可通过以下方法确认网络选项：拔出网线再重新连接，会自动匹配正确的以太网。
    - <font color="red">此处务必选择正确的**以太网**，错选可能导致电脑本身网络无法连接。</font>
    :::

   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/device_manage/rdk_studio_left_menu_device_manage_hr_add_device_choosenet.png)


4. 点击 `下一步` ，进入选择用户类型界面，此处设置登录到 RDK 设备的用户类型，可选择 “普通用户” 或 “root 用户”。
    
    
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/device_manage/rdk_studio_left_menu_device_manage_hr_add_device_usertype.png)


5. 点击 `下一步` ，进入WIFI无线网络连接界面，为 RDK 设备连接网络，展开选项列表选择想要连接的网络。
   
   
   :::warning
    如选择“创建WIFI连接方式” 添加设备，需确保笔记本与 RDK 设备连接至同一局域网。
   :::
   
   
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/device_manage/rdk_studio_left_menu_device_manage_hr_add_device_net.png)


1. 点击 `下一步` ，进入创建 RDK 设备条目界面，填写设备名称及描述。

    
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/device_manage/rdk_studio_left_menu_device_manage_hr_add_device_name.png)


7. 点击 `确认`，成功添加 RDK 设备，在设备卡列表页面显示设备信息。

    
   ![界面展示](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/device_manage/rdk_studio_left_menu_device_manage_hr_add_device_success.png)



</TabItem>

<TabItem value="linux" label="Linux">

:::tip

目前RDK Studio Windows系统已正式出炉，使用Linux和Mac的小伙伴们，稍微等等开发小哥正在火速敲键盘

:::

</TabItem>



<TabItem value="mac" label="Mac">

:::tip

目前RDK Studio Windows系统已正式出炉，使用Linux和Mac的小伙伴们，稍微等等开发小哥正在火速敲键盘

:::

</TabItem>

</Tabs>