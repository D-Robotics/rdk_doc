---
sidebar_position: 3
---

# 1.9.3 烧录系统

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```


<Tabs groupId="rdk-type">
<TabItem value="windows" label="Windows">

:::info 提示

如果您的存储设备已完成过系统烧录，可跳过此章节直接进行[添加设备](../09_RDK_Studio/04_Device_management/01_hardware_resource.md)。
:::

## 烧录准备

1. 点击 `烧写` 弹出 RDK 系统安装工具，提示系统安装功能需通过管理员权限打开 RDK Studio。
   
   ![权限提示页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_permission.png)

2. 点击 `知道了` 关闭弹窗。如果当前不是通过管理员权限打开的RDK Studio，先关闭RDK Studio；返回到桌面右键点击 RDK Studio 应用图标，选择 `以管理员身份运行`，打开 RDK Studio 后再次进入烧写功能界面。
   
   ![管理员打开页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_usertype.png)



## 镜像烧写引导


1. 在选择 RDK 设备界面，选择想要安装系统的设备类型，此处选择 “RDK S100”。
   
   ![安装类型页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/flashing_select_device.png)
   
2. 点击 `了解更多` 可跳转至网页了解更多 RDK 设备信息；点击 `下一步` 进入选择 RDK 操作系统镜像界面。 
   
   ![选择系统页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/flashing_select_os_image.png)
   
3. 选择想要安装的镜像版本，点击 `下一步` ，进入工具下载和镜像下载界面。

    ![下载页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/flashing_download.png)

4. 点击 `工具下载`，自动打开浏览器开始下载烧录工具 D-Navigation。
   
    ![工具下载页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/flashing_download_tool.png)

5. 点击 `镜像下载`，自动打开浏览器开始下载系统镜像。
 
   ![安装过程页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/s100/en/flashing_download_image.png)

6. 下载过程需要等待一定的时间；镜像下载完成后请参考[系统烧录方法](../02_install_os/rdk_s100.md)进行烧录。
  

</TabItem>

<TabItem value="linux" label="Linux">

:::tip

目前RDK Studio Windows系统已正式出炉，使用 Linux 和 Mac 的小伙伴们，稍微等等开发小哥正在火速敲键盘

:::

</TabItem>



<TabItem value="mac" label="Mac">

:::tip

目前 RDK Studio Windows 系统已正式出炉，使用 Linux 和 Mac 的小伙伴们，稍微等等开发小哥正在火速敲键盘

:::


</TabItem>

</Tabs>