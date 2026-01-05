---
sidebar_position: 2
---

# 手势识别

## 应用场景

RDK Studio 帮助零基础开发者快速上手，开启高效手势识别工作流：手势识别算法集成了人手关键点检测，手势分析等技术，使得计算机能够将人的手势解读为对应指令，可实现手势控制以及手语翻译等功能，主要应用于智能家居，智能座舱、智能穿戴设备等领域。

## 准备工作

支持连接 USB 和 MIPI 摄像头，本章节以 USB 摄像头为例进行说明，USB 摄像头连接方法如下：
    ![接入摄像头图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/connect_camera.png)

    
## 运行过程

:::tip 提示

点击 RDK Studio 右上角  ![链接图标](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) 图标可以快速在浏览器中打开示例！
:::

1. 点击 `手势识别` 示例下的 Node-RED。
   
    ![示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_location.png)

2. 进入示例应用流程界面。
        
   ![手势识别示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_example_nodered_gesture_detection.png)

3. 选择连接摄像头的类型，点击对应的 `启动` 指令，等待大约 10s 后自动打开可视化窗口进行识别，并将识别到的结果通过语音播报。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_run_camera.png)

4.  `性能信息输出`：点击右侧调试图标将右侧边栏定位至调试窗口，可在调试窗口查看性能信息输出结果。
      
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_debug.png)
   
5.  `输出统计`: 输出采集到的手势次数统计结果。
   
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_count.png)


6. 点击执行 `关闭` 指令，关闭摄像头。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_close_camera.png)


    :::warning 注意
    
    如果对节点、流程等进行了修改，需点击右上角 ![部署图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) 按钮后才能生效！
    
       
    :::

7. 点击右上角 ` × ` 图标退出 Nude-RED 应用。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)



## 更多功能

### 可视化页面

1. 点击执行 `可视化界面` 指令，自动打开 TogetherROS Web Display。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
7. 点击 `Web Display`， 进入可视化页面实时识别手势。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser2.png)

8. 点击可视化页面右上角的 ` × ` 退出可视化页面。


### 了解更多

点击执行 `了解更多` 指令，可打开查阅有关示例的更多信息。

![示例说明图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_more.png)


