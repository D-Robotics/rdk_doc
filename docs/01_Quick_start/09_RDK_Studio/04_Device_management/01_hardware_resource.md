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

1. 点击右上角 `+ RDK设备`，进入选择连接类型界面。
   
   ![+ RDK 设备界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device.png)


2. 点击“选择连接类型”后面的问号图标，弹出连接方式导览窗口，指导用户进行所选连接类型对应的设备连接方法，点击 `下一步` 可查看后续内容，也可通过点击 `上一步` 再次查看之前的提示，查看全部导览内容后点击 `结束导览` 关闭弹窗，也可随时点击右上角 ` × ` 直接关闭弹窗。
  
   ![导览窗口界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_help.png)

3. 此处以 “创建网线连接” 为例，点击选中连接类型，点击 `下一步` ，进入选择网络界面。
    
    :::warning
    - 此处务必选择正确的<font color="red">以太网</font>，错选可能导致电脑本身网络无法连接。
    - 可通过以下方法确认正确的网络：拔出网线再重新连接，会自动匹配正确的以太网。
  
    :::

   ![选择网络界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_choosenet.png)

    :::tip 提示
    
      仅RDK X5 的 Type-C 接口支持通过闪连方式添加设备，此处选择闪连方式添加设备请提前执行以下步骤：
      1. 开发板网络 IP 确认  
       以 RDK X5 的 3.0 版本镜像为例（切勿使用Beta版本镜像），Type-C 网卡所对应的IP网段为192.168.128.10，其他版本镜像可以先选用其他连接方式，使用 `ifconfig` 命令进行查看 IP。
      2. 个人PC网络的设置  
       打开控制面板——>网络和Internet——>网络和共享中心——>左侧更改适配器设置——>找到板卡的以太网（PS：将板卡与电脑的连接线拔插多次即可知道哪个是开发板的以太网）——>右键选择属性，按照下图方式填写。
    
            ![r03XYBKpQH.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/configuration_wizard/r03XYBKpQH.png)

 :::

1. 点击 `下一步` ，进入选择用户类型界面，此处设置登录到 RDK 设备的用户类型，可选择 “sunrise（普通用户权限）” 或 “root（超级用户权限）”。
    
    
   ![选择用户类型界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_usertype.png)


5. 点击 `下一步` ，进入WIFI无线网络连接界面为 RDK 设备连接网络，展开选项列表选择想要连接的网络，填写密码。
   
   
   :::warning
    如选择 “创建WIFI连接方式” 添加设备，需确保笔记本与 RDK 设备连接至同一局域网。
   :::
   
   
   ![WIFI无线网络连接界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_net.png)


6. 点击 `下一步` ，进入创建 RDK 设备条目界面，填写设备名称及描述。

    :::tip 提示

    此处填写的设备名称和描述会出现在添加成功后的设备卡片上，添加多个设备情况下可通过名称进行区分。

    :::

   ![创建 RDK 设备条目界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_name.png)


7. 点击 `确认`，成功添加 RDK 设备，在列表页面显示设备卡片及信息。

    
   ![设备卡列表界面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_success.png)

:::tip 提示

由于连接 WIFI 需要时间，可能在设备添加完成时，短暂显示未发现 WIFI，稍等片刻刷新卡片即可。

:::

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