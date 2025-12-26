---
sidebar_position: 2
---

# YOLO 目标检测

## 应用场景

YOLO 系列作为单阶段目标检测中的代表算法，具有速度快，泛化性好的优点，可实现垃圾识别、车辆检测等功能，主要应用于自动驾驶、智能家居等领域。

## 准备工作

RDK 设备已连接 MIPI 或者 USB 摄像头。

    
## 运行过程

:::tip 提示

点击 RDK Studio 右上角  ![链接图标](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) 图标可以快速打开浏览器查看示例！
:::

1. 选择连接摄像头的类型，本章节以 MIPI 相机为例进行说明，点击对应的 `启动` 指令，等待节点运行，完成后自动打开可视化窗口。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_run_camera.png)

2. 激活 `性能信息输出`，可在调试窗口查看输出结果。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_debug.png)
   
3. 激活 `输出统计`，可输出采集到的检测类型的统计结果。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_count.png)

4. 点击执行 `可视化界面` 指令，自动打开 TogetherROS Web Display。

    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
5. 点击 `Web Display`， 进入可视化页面实时检测目标。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_browser2.png)

6. 点击可视化页面右上角的 ` × ` 退出可视化页面，点击执行 `关闭` 指令，关闭摄像头。
   
   ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_close_camera.png)


    :::warning 注意
    
    如果对节点、流程等进行了修改，需点击右上角 `部署` 后才能生效！
    
    ![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png)

    :::

## 更新软件包

执行 `更新软件包` 会将算法包或驱动更新到云端最新的版本，更新完成后，提示 “已完成更新”，同时通过语音播报 “智能更新已完成”。

![示例图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_update_package.png)


### 了解更多(<font color = "red">图片待替换</font>)

点击执行 `了解更多` 指令，可打开网页查阅有关示例的更多信息。

![示例说明图片](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_more.png)


