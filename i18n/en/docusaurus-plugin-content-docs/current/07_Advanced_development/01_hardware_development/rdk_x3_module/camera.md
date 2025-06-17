---
sidebar_position: 4
---
# Camera Usage

The RDK X3 Module carrier board provides two 15pin MIPI CSI interfaces, `CAM0` and `CAM2`, which can support the connection of Raspberry Pi cameras such as OV5647, IMX219, and IMX477. When connecting the camera ribbon cable, make sure the blue side is facing up. In addition, the sample program has implemented automatic detection of the camera, so users do not need to worry about the corresponding relationship between the CAM interface and the camera.

The development board comes with the `mipi_camera.py` program for testing the data pathway of the MIPI camera. This example program will capture real-time image data from the MIPI camera, then run the target detection algorithm, and finally output the fused image data and detection results through the HDMI interface.

- Execution method: Execute the program according to the following command

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py 
  ```

- Expected result: After executing the program, the monitor will display the camera image and the results of the target detection algorithm in real-time (target type, confidence level).