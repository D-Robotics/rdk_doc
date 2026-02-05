---
sidebar_position: 2
---

# TROS Gesture Detection

## Application Scenarios

RDK Studio helps beginners get started quickly, enabling an efficient gesture recognition workflow: Gesture recognition algorithms integrate technologies such as hand keypoint detection and gesture analysis, allowing computers to interpret human gestures as corresponding commands. This enables functions like gesture control and sign language translation, mainly applied in smart home, smart cockpit, smart wearable devices, and other fields.

## Preparation

Supports connection of both USB and MIPI cameras. This section uses a USB camera as an example. The USB camera connection method is as follows:


    ![Camera Connection Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/connect_camera.png)

    
## Running Process


1. Click Node-RED under the `TROS Gesture Detection` example.
   
    ![Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_location.png)

2. Enter the example application flow interface.

    :::tip Tip

    Click the  ![Link Icon](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_link_browser.png) icon in the top right corner of RDK Studio to quickly open the example in a browser!

    :::
        
   ![Gesture Recognition Example Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/rdk_studio_left_menu_example_nodered_gesture_detection.png)

3. Select the type of connected camera, click the corresponding `Start(USB Cam)` command. After waiting about 10 seconds, the visualization window will automatically open for recognition, and the recognized results will be broadcast via voice.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_run_camera.png)

4.  `Performance Result`: Click the debug icon on the right to position the right sidebar to the debug window, where you can view performance information output results.
      
    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_debug.png)
   
5.  `Output Statistics`: Outputs statistical results of collected gesture counts.
   
    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_count.png)


6. Click to execute the `Stop` command to turn off the camera.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_close_camera.png)


    :::warning Note
    
    If you modify nodes, flows, etc., you need to click the ![Deploy Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_deploy.png) button in the top right corner for the changes to take effect!
    
       
    :::

7. Click the ` × ` icon in the top right corner ，Select "Close APP" to exit the Node-RED application.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_nodered_close.png)



## More Features

### Visualization Page

1. Click to execute the `Visualization Interface` command, which automatically opens TogetherROS Web Display.

    ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser1.png)
    
7. Click `Web Display` to enter the visualization page for real-time gesture recognition.
   
   ![Example Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/left_menu_example_application_gesture_display_browser2.png)

8. Click the ` × ` in the top right corner of the visualization page to exit it.


### Learn More

Click to execute the `Learn More` command to open the online documentation to find more information about the examples.

![Example Description Image](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/en/left_menu_example_application_gesture_more.png)