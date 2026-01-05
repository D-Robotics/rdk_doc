---
sidebar_position: 2
---

# 人体关键点检测

    
## 应用场景

RDK Studio 帮助零基础开发者快速上手，开启高效人体关键点检测工作流：人体关键点检测算法主要用于捕捉人的骨骼关键点，适用于实时应用场景中的多人体姿态识别任务。

## 准备工作

支持连接 USB 和 MIPI 摄像头，本章节以 USB 摄像头为例进行说明，USB 摄像头连接方法如下：
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/connect_camera.png)

    
## 运行过程

:::tip 提示

点击 RDK Studio 右上角  ![链接图标](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) 图标可以快速在浏览器中打开示例！
:::

1. 点击 `人体关键点检测` 示例下的 Node-RED。
   
    ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_location.png)

2. 进入示例应用流程界面。
        
   ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_home.png)
    
3. 选择连接摄像头的类型，点击对应的 `启动` 指令，等待大约 10s 后自动打开可视化窗口。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_run_camera.png)

4. `性能信息输出`：点击右侧调试图标将右侧边栏定位至调试窗口，可在调试窗口查看性能信息输出结果。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_debug.png)
   
5.  `输出统计`：可输出检测到的人体关部位的统计结果。
 
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_count.png)


6. 点击执行 `关闭` 指令，关闭摄像头。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_close_camera.png)


    :::warning 注意
    
    如果对节点、流程等进行了修改，需点击右上角 ![部署图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) 按钮后才能生效！

    :::

7. 点击右上角 ` × ` 图标，选择 “关闭程序” 退出 Nude-RED 应用。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)
    
## 更多功能

### 可视化页面

1. 点击执行 `可视化界面` 指令，自动打开 TogetherROS Web Display。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
2.  点击 `Web Display`， 进入可视化页面实时检测目标。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_browser2.png)

3.  点击可视化页面右上角的 ` × ` 退出可视化页面。


### 了解更多

点击执行 `了解更多` 指令，可打开网页查阅有关示例的更多信息。

![示例说明图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_more.png)


