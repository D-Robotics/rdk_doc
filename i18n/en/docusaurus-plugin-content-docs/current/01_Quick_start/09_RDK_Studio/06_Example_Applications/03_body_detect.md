---
sidebar_position: 2
---

# TROS Body Detection

## Application Scenarios

RDK Studio helps beginners get started quickly and enables an efficient workflow for human body keypoint detection: The human body keypoint detection algorithm is primarily used to capture skeletal keypoints of the human body, suitable for multi-person pose recognition tasks in real-time application scenarios.

## Preparations

Supports connecting both USB and MIPI cameras. This section uses a USB camera as an example. The connection method for a USB camera is as follows:


![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/connect_camera.png)

## Running Process


1. Click Node-RED under the `TROS Body Detection` example.
   
    ![Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_location.png)

2. Enter the example application flow interface.

    :::tip Tip

    Click the  ![Link Icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) icon in the top right corner of RDK Studio to quickly open the example in a browser!
    
    :::
        
   ![Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_home.png)
    
3. Select the type of camera connected, click the corresponding `Start(USB Cam)` command, and wait for about 10 seconds for the visualization window to open automatically.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_run_camera.png)

4. `Performance Result`: Click the debug icon on the right to position the sidebar to the debug window. You can view the performance information output results in the debug window.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_debug.png)
   
5.  `Output Statistics`: Can output statistical results of detected human body keypoints.
 
    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_count.png)

6. Click to execute the `Stop` command to turn off the camera.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_close_camera.png)

    :::warning Note
    
    If you have modified nodes, flows, etc., you need to click the ![Deploy Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) button in the upper right corner for the changes to take effect!

    :::

7. Click the `×` icon in the upper right corner，Select "Close APP" to exit the Node-RED application.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)
    
## More Features

### Visualization Page

1. Click to execute the `Visualization Interface` command to automatically open TogetherROS Web Display.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
2.   Click `Web Display` to enter the visualization page for real-time target detection.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_browser2.png)

3.   Click the `×` in the upper right corner of the visualization page to exit.

### Learn More

Click to execute the `Learn More` command to open the online documentation to find more information about the examples.

![Example Description Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_body_more.png)