---
sidebar_position: 2
---

# YOLO

## Application Scenarios

As a representative algorithm in single-stage object detection, the YOLO series has the advantages of fast speed and good generalization, and can be used for garbage recognition, vehicle detection, and other functions, mainly applied in autonomous driving, smart home, and other fields.

## Preparations

The RDK device is already connected to an MIPI or USB camera.

    
## Execution Process

:::tip Tip

Click the ![Link Icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) icon in the top right corner of RDK Studio to quickly open the example in a browser!
:::

1. Select the type of camera connected. This section uses an MIPI camera as an example. Click the corresponding `Start` command and wait for the node to run. Once completed, the visualization window will open automatically for recognition.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_run_camera.png)

2. Activate `Performance Information Output` to view the output results in the debug window.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_debug.png)
   
3. Activate `Output Statistics` to output the statistical results of detected gesture counts.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_count.png)

4. Click to execute the `Visualization Interface` command, which automatically opens TogetherROS Web Display.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
5. Click `Web Display` to enter the visualization page for real-time gesture recognition.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_browser2.png)

6. Click the ` × ` in the top right corner of the visualization page to exit. Click to execute the `Close` command to shut down the camera.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_close_camera.png)


    :::warning Note
    
    If modifications are made to nodes, workflows, etc., you must click the ![Deploy Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) button in the top right corner for them to take effect!
    
    
    
    :::

## Update Software Package

Executing `Update Software Package` will update the algorithm package or driver to the latest version available in the cloud. After the update is complete, the message "Update completed" is displayed, and a voice announcement "Smart update completed" is played.

![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_update_package.png)


### Learn More

Click to execute the `Learn More` command to open and view more information about the example.

![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_object_more.png)


