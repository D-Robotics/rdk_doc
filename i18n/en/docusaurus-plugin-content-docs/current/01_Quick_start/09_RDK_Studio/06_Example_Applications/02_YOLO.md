---
sidebar_position: 2
---

# TROS YOLO

## Application Scenarios

RDK Studio helps developers with zero experience quickly get started and enables an efficient object detection workflow: The YOLO series, as representative algorithms in single-stage object detection, offers the advantages of fast speed and good generalization. They can achieve functions such as garbage recognition and vehicle detection, primarily applied in fields like autonomous driving and smart homes.

## Preparations

Supports connection of USB and MIPI cameras. This section uses a USB camera as an example for explanation. The USB camera connection method is as follows:


    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/connect_camera.png)

    
## Running Process


1. Click Node-RED under the `TROS YOLO` example.
   
    ![Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_location.png)

2. Enter the example application flow interface.

    :::tip Tip

    Click the  ![Link Icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) icon in the top right corner of RDK Studio to quickly open the example in a browser!
    
    :::
        
   ![Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_home.png)


3. Select the type of connected camera, click the corresponding `Start(USB Cam)` command, and wait about 10s for the visualization window to open automatically.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_run_camera.png)

4. `Performance Result`: Click the debug icon on the right to position the right sidebar to the debug window. You can view the performance information output results in the debug window.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_debug.png)
   
5. `Output Statistics`: Can output statistical results of the detected types collected.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_count.png)

6. Click to execute the `Stop` command to turn off the camera.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_close_camera.png)

    :::warning Note
    
    If modifications are made to nodes, flows, etc., you need to click the ![Deploy Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) button in the upper right corner for them to take effect!
    
       
    :::

7. Click the `×` icon in the upper right corner，Select "Close APP" to exit the Node-RED application.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)


## More Features

### Visualization Page

1. Click to execute the `Visualization Interface` command to automatically open TogetherROS Web Display.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
2. Click `Web Display` to enter the visualization page for real-time object detection.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_browser2.png)

3. Click the `×` in the upper right corner of the visualization page to exit it.


### Learn More

Click to execute the `Learn More` command to open the online documentation to find more information about the examples.

![Example Description Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_object_more.png)