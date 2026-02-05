---
sidebar_position: 4
---

# 1.9.4 烧录系统

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```


<Tabs groupId="rdk-type">
<TabItem value="windows" label="Windows">

:::info 提示

如果您的存储设备已完成过系统烧录，可跳过此章节直接进行[添加 RDK 设备](../09_RDK_Studio/04_Device_management/01_hardware_resource.md)。
:::

## 烧录准备

1. 准备至少 16GB 容量的 Micro SD 卡和 SD 读卡器，将 Micro SD 卡通过读卡器连接到电脑。
2. 点击 `烧写` 弹出 RDK 系统安装工具，提示系统安装功能需通过管理员权限打开 RDK Studio。
   
   ![权限提示页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_permission.png)

3. 点击 `知道了` 关闭弹窗。如果当前不是通过管理员权限打开的 RDK Studio，先关闭 RDK Studio；返回到桌面右键点击 RDK Studio 应用图标，选择 `以管理员身份运行`，打开 RDK Studio 后再次进入烧写功能界面。
   
   ![管理员打开页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_usertype.png)



## 选择本地已有镜像进行烧录

:::info 提示

烧录系统有 “使用本地镜像” 和 “在线下载镜像” 两种方式，如果没有下载过目标镜像文件到本地，请跳过此节直接[选择通过 RDK Studio 下载镜像进行烧录](#选择通过-rdk-studio-下载镜像进行烧录)。

:::

1. 在选择 RDK 设备界面，选择想要安装系统的设备类型，本章节以安装 RDK X5 系统为例。
   
   ![安装类型页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_device.png)
   
2. 安装系统方式分为 “使用 TF 读卡器” 和 “使用闪连（Type-C）” 两种，点击对应的红色按钮可查看设备连接教程；点击 `了解更多` 可跳转至网页了解更多 RDK 设备信息。
   
   ![导览信息页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_device_method.png)

3. 点击 `结束导览` 关闭教程窗口，点击 `下一步` 进入选择 RDK 操作系统镜像界面。 
   
   ![选择系统页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_os_image.png)
   
4. 点击选择 `选择本地镜像文件`，自动打开文件资源管理器，进入镜像文件的存储路径，双击打开镜像文件。
   
   ![镜像存储页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_os_image_local.png)


5. 点击 `下一步`，进入选择存储设备界面，勾选正确的存储设备，点击 `安装`。
   
   :::warning
   - 此处可通过以下方法确定存储设备：拔出存储设备点击刷新按钮，查看设备列表中消失的设备选项；再连接存储设备，点击刷新按钮，选择随拔插操作发生减增变化的存储设备。
   - RDK X3 Module 烧录 eMMC 系统镜像时，可以通过[将 eMMC 映射成 U盘](../install_os/rdk_x3_module/04_rufus.md#emmc-烧录) 进行烧录，烧录步骤中 “选择存储设备” 步骤务必选择 eMMC 映射成的 U 盘。
   - RDK X5 Module 烧录 eMMC 系统镜像时，可以通过[将 eMMC 映射成 U盘](../install_os/rdk_x5_module/04_rufus.md#在板烧录) 进行烧录，烧录步骤中 “选择存储设备” 步骤务必选择 eMMC 映射成的 U 盘。
   - RDK X5 烧录在板 SD 卡系统镜像时，可以通过[将 SD 卡映射成 U盘](../install_os/rdk_x5/04_rufus.md#sd-卡在板烧录) 进行烧录，烧录步骤中 “选择存储设备” 步骤务必选择 SD 卡映射成的 U 盘。
   - 烧录系统会<font color="red">清空存储设备中的所有数据</font>，请务必选择正确的存储设备！
   :::

    ![选择存储设备页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_storage_refresh.png)

6. 开始进行系统烧录。
 
   ![烧录过程页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_success_local.png)


7. 烧录成功后提示安装完成。
  
    
     ![烧录完成页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_success.png)

   
## 选择通过 RDK Studio 下载镜像进行烧录

:::warning 注意

如果本地已有目标镜像，请直接[选择本地已有镜像进行烧录](#选择本地已有镜像进行烧录)，如果再次通过 RDK Studio 下载同名镜像文件会提示错误！

:::

1. 在选择 RDK 设备界面，选择想要安装系统的设备类型，本章节以安装 RDK X5 系统为例。
   
   ![安装类型页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_device.png)
   
2. 安装系统方式分为 “使用 TF 读卡器” 和 “使用闪连（Type-C）” 两种，点击对应的红色按钮可查看设备连接教程；点击 `了解更多` 可跳转至网页了解更多 RDK 设备信息。
   
   ![导览信息页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_device_method.png)

3. 点击 `结束导览` 关闭教程窗口，点击 `下一步` 进入选择 RDK 操作系统镜像界面。 
   
   ![选择系统页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_os_image1.png)
   
4. 选择想要安装的镜像版本，点击 `下一步` ，进入选择存储设备界面，勾选正确的存储设备，点击 `安装`。
   
   :::warning
   - 此处可通过以下方法确定存储设备：拔出存储设备点击刷新按钮，查看设备列表中消失的设备选项；再连接存储设备，点击刷新按钮，选择随拔插操作发生减增变化的存储设备。
   - 烧录系统会<font color="red">清空存储设备中的所有数据</font>，请务必选择正确的存储设备！
   :::

    ![选择存储设备页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_select_storage_refresh.png)

5. 开始下载系统镜像。
 
   ![安装过程页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_download_osimage.png)

6. 下载过程需要等待一定的时间；镜像下载完成后自动进行系统烧录，成功后提示安装完成。
  
    
     ![烧录完成页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/flashing_install_success.png)

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