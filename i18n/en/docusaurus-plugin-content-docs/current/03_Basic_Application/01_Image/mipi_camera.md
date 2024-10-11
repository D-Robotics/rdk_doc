---
sidebar_position: 1
---
# 3.1.1 Using MIPI Camera

Video: https://www.youtube.com/watch?v=nabpS2CUkjY&list=PLSxjn4YS2IuFUWcLGj2_uuCfLYnNYw6Ld&index=18

The development board is installed with the "mipi_camera.py" program to test the data path of the MIPI camera. This example will capture the image data of the MIPI camera in real time, then run the object detection algorithm, and finally output the image data and detection results through the HDMI interface.

## Environment Preparation

  - Connect the MIPI camera module to the development board's MIPI CSI interface. For specific connection methods, please refer to the [MIPI Camera Connection Tutorial](../../01_Quick_start/hardware_introduction.md) chapter.
  - Connect the development board to the monitor via an HDMI cable.

## Running the Program
Execute the program with the following command:

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py 
  ```

## Expected Result
After the program is executed, the monitor will display the camera image and the results of the object detection algorithm (object type, confidence), as shown below:
 ![image-20220503221020331](../../../../../../static/img/03_Basic_Application/01_Image/image/mipi_camera/image-20220511181747071.png)

:::tip

For detailed code implementation explanation, please refer to the [MIPI Camera Inference](../../04_Algorithm_Application/01_pydev_dnn_demo/mipi_camera.md) chapter.

:::