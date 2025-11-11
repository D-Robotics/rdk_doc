---
sidebar_position: 1
---

# 3.1.1 Using MIPI Camera

The `mipi_camera_streamer.py` program is installed on the development board to test the MIPI camera data pipeline. This example captures image data from the MIPI camera in real time and outputs the image data via the HDMI interface.

## Environment Setup

  - Connect the MIPI camera module to the MIPI CSI interface on the development board. For specific connection instructions, please refer to - [Hardware Introduction - MIPI Interface](../../Quick_start/hardware_introduction/rdk_s100_camera_expansion_board)
  - Currently, this sample only supports MIPI sensors: IMX219, SC230AI
  - Connect the development board to a display using an HDMI cable

## How to Run
Execute the program with the following commands:

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/10_mipi_camera_sample
  sunrise@ubuntu:/app/pydev_demo/10_mipi_camera_sample$ python 05_mipi_camera_streamer.py -w 1920 -h 1080
  ```

## Expected Result
After running the program, the display will show the live camera feed in real time, as shown below:
![mipi_camera_streamer_2025-06-25_12-12-31](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/mipi_camera_streamer_2025-06-25_12-12-31.png)

<!--
Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=19

The `mipi_camera.py` program is installed on the development board to test the MIPI camera data pipeline. This example captures image data from the MIPI camera in real time, runs an object detection algorithm, and then overlays the detection results onto the image before outputting it via the HDMI interface.

## Environment Setup

  - Connect the MIPI camera module to the MIPI CSI interface on the development board. For specific connection instructions, please refer to - [Hardware Introduction - MIPI Interface](../../Quick_start/hardware_introduction/rdk_s100_camera_expansion_board)
  - Connect the development board to a display using an HDMI cable

## How to Run
Execute the program with the following commands:

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ python3 mipi_camera.py
  ```

<details>
  <summary>When using this demo on RDK X5, you will be prompted to select a camera configuration. Click to view details.</summary>

  After running the program in the terminal, you will see a prompt such as “please choose sensor config, xxxx”.

  ![screenshot-20241217-115245](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/mipi_camera/screenshot-20241217-115245.png)

  Select a configuration supported by RDK X5 during execution. In the image above, either option 0 or 1 is acceptable.

  You can refer to the following video for the startup process:
  ![gif-20241217-115536](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/mipi_camera/20241217-115536.gif)

</details>

## Expected Result
After running the program, the display will show the live camera feed along with the object detection results (object class and confidence score) in real time, as shown below:
![image-20220503221020331](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/image-20220511181747071.png)
-->