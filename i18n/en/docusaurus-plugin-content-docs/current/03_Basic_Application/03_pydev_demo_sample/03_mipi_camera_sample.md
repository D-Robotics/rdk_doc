---
sidebar_position: 9
---

# 3.3.9 MIPI Camera Sample Introduction

## Sample Overview
The MIPI camera sample is a **Python interface** development code example located in `/app/pydev_demo/03_mipi_camera_sample`, demonstrating how to use the onboard MIPI camera for real-time object detection. This sample uses the FCOS object detection model to perform real-time inference on the video stream captured by the MIPI camera, displays the detection results via HDMI, and outputs bounding box information and FPS performance data.

## Demo Effect

![output-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_runing.png)

## Hardware Preparation

### Hardware Connection
1. Prepare an RDK development board
2. Connect the officially compatible MIPI camera
3. Connect the display to the development board via HDMI cable
4. Connect power cable and network cable

![connect-img](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_hw_connect.png)

## Quick Start

### Code and Board Location
The MIPI camera sample file is located at `/app/pydev_demo/03_mipi_camera_sample/mipi_camera.py`

### Compilation and Execution
The Python sample does not require compilation and can be run directly:

### Execution Effect

After running, the program will initialize the MIPI camera and HDMI display, and begin real-time object detection. Detection results will be displayed via HDMI, and the console will print detected object information and FPS.

## Detailed Introduction

### Sample Program Parameter Options
The MIPI camera sample does not require command line parameters and can be run directly. The program will automatically detect and use the onboard MIPI camera.

### Software Architecture Description
The software architecture of the MIPI camera sample includes the following core components:

1. Model Loading: Load the FCOS object detection model

2. Camera Initialization: Initialize MIPI camera using srcampy.Camera()

3. Display Initialization: Initialize HDMI display

4. Camera and Display Binding: Directly bind camera output to display

5. Real-time Inference Loop:

- Acquire images from camera

- Model inference

- Post-processing (parse detection results)

- Draw bounding boxes and display

- Calculate and print FPS

<center>
![software_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_software_arch.png)
</center>

### API Flow Description

<center>
![API_Flow](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/03_pydev_demo_sample/image/pydev_03_mipi_camera_sample_api_flow.png)
</center>

### FAQ

**Q:** What should I do if camera initialization fails when running the sample?  
**A:** Please check if the MIPI camera is properly connected and ensure the camera driver is correctly loaded. Try restarting the device.

**Q:** What should I do if HDMI display is abnormal or has no output?  
**A:** Please check the HDMI connection and ensure the display service is stopped (such as using systemctl stop lightdm).

**Q:** How to adjust the detection threshold?  
**A:** Modify the value of fcos_postprocess_info.score_threshold in the code, for example, changing it to 0.5 can increase detection sensitivity.

**Q:** How to modify the display resolution?  
**A:** Modify the sensor_width and sensor_height variables in the code, but note whether the display device supports that resolution.

**Q:** What should I do if the frame rate is very low when running the sample?  
**A:** Try using a lighter model or adjust the camera capture resolution.

**Q:** How to save the detection result images?  
**A:** You can add image saving logic in the code, for example using cv2.imwrite() to save processed images.

**Q:** How to add new detection categories?  
**A:** You need to modify the get_classes() function, add new category names, and retrain the model or use a model that supports the new categories.


