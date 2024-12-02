---
sidebar_position: 4
---

# Camera Usage

The RDK Ultra development kit provides two 15-pin MIPI CSI interfaces, `CAM0` and `CAM2`, which support connecting the IMX219 camera that comes with the kit. When connecting the camera ribbon cable, make sure the blue side is facing up. The sample program already implements automatic camera detection, so users do not need to worry about the specific mapping between the CAM interfaces and the camera.

A program called `mipi_camera.py` is installed on the development board for testing the MIPI camera data path. This sample program collects image data from the MIPI camera in real-time and runs a target detection algorithm. Finally, the program combines the image data with the detection results and outputs them via the HDMI interface.

### Execution:

Run the program with the following command:


```bash
sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py 

  ```
- **Expected Result**: After running the program, the display will show the camera feed in real-time along with the results of the object detection algorithm (including object type and confidence).
