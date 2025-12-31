---
sidebar_position: 2
---

# 人体关键点检测

    
## 应用场景

RDK Studio 帮助零基础开发者快速上手，开启高效人体关键点检测工作流：人体检测和跟踪算法是人体运动视觉分析的重要组成部分，可实现人体姿态分析以及人流量统计等功能，主要应用于人机交互、游戏娱乐等领域。

## 准备工作

<font color = "red">支持连接 USB 和 MIPI 摄像头，本章节以 USB 摄像头为例进行说明，USB 摄像头连接方法如下：</font>
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_run_camera.png)

    
## 运行过程

:::tip 提示

点击 RDK Studio 右上角  ![链接图标](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) 图标可以快速在浏览器中打开示例！
:::

1. 点击 `人体检测和跟踪` 示例下的 Node-RED。
   
    ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_location.png)

2. 进入示例应用流程界面。
        
   ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_home.png)
    
3. 选择连接摄像头的类型，点击对应的 `启动` 指令，等待大约 10s 后自动打开可视化窗口。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_run_camera.png)

4. `性能信息输出`：可在调试窗口查看输出结果。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_debug.png)
   
5.  `输出统计`：可输出采集到的检测类型的统计结果。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_count.png)


6. 点击执行 `关闭` 指令，关闭摄像头。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_body_close_camera.png)


    :::warning 注意
    
    如果对节点、流程等进行了修改，需点击右上角 ![部署图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) 按钮后才能生效！

    :::

7. 点击右上角 ` × ` 图标退出 Nude-RED 应用。

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


