---
sidebar_position: 2
---

# Gesture Recognition

## Application Scenarios

RDK Studio helps developers with zero background get started quickly and initiate an efficient gesture recognition workflow: The gesture recognition algorithm integrates technologies such as hand keypoint detection and gesture analysis, enabling computers to interpret human gestures as corresponding commands. This can achieve functions like gesture control and sign language translation. It is primarily applied in fields such as smart home systems, smart cockpits, and wearable smart devices.

## Preparations

The RDK device is already connected to an MIPI or USB camera.

## Execution Process

:::tip Tip

Click the ![Link Icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) icon in the top right corner of RDK Studio to quickly open the example in a browser!
:::

1. Select the type of camera connected. This section uses an MIPI camera as an example. Click the corresponding `Start` command and wait for the node to run. Once completed, the visualization window will open automatically for recognition, and the recognized results will be announced via voice broadcast.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_run_camera.png)

2. Activate `Performance Information Output` to view the output results in the debug window.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_debug.png)
   
3. Activate `Output Statistics` to output the statistical results of detected gesture counts.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_count.png)

4. Click to execute the `Visualization Interface` command, which automatically opens TogetherROS Web Display.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
5. Click `Web Display` to enter the visualization page for real-time gesture recognition.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser2.png)

6. Click the ` × ` in the top right corner of the visualization page to exit. Click to execute the `Close` command to shut down the camera.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_close_camera.png)


    :::warning Note
    
    If modifications are made to nodes, workflows, etc., you must click the ![Deploy Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) button in the top right corner for them to take effect!
    
    
    
    :::

## Update Software Package

Executing `Update Software Package` will update the algorithm package or driver to the latest version available in the cloud. After the update is complete, the message "Update completed" is displayed, and a voice announcement "Smart update completed" is played.

![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_update_package.png)


### Learn More

Click to execute the `Learn More` command to open and view more information about the example.

![Example Explanation Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_more.png)