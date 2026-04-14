---  
sidebar_position: 1  
---  

# MIPI Camera Usage  


```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=19  

The `mipi_camera.py` program is installed on the development board to test the data path of the MIPI camera. This example captures real-time image data from the MIPI camera, runs an object detection algorithm, and finally outputs the fused image data and detection results through the HDMI interface.  

## Environment Preparation  

- Connect the MIPI camera module to the MIPI CSI interface of the development board. For specific connection methods, refer to - [Hardware Introduction - MIPI Interface](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/hardware_introduction/rdk_x3#mipi_port)  
- Connect the development board and the monitor using an HDMI cable  

## Execution Method  

<Tabs groupId="rdk-type">  
<TabItem value="desktop" label="Desktop">  

Run the following command to disable the desktop service  
  ```bash  
  sudo systemctl stop lightdm  
  ```  

Log in to the development board via SSH and execute the program using the following commands  
  ```bash  
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/  
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py  
  ```  

</TabItem>  

<TabItem value="server" label="Server">  

:::info Note  

The Server version outputs results via the terminal. To view the camera feed and the object detection algorithm results (object type, confidence), connect the development board and the monitor using an HDMI cable.  

:::  

Execute the program using the following commands  

  ```bash  
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/  
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py  
  ```  
</TabItem>  
</Tabs>  

## Expected Outcome  

After the program is executed, the monitor will display the real-time camera feed and the object detection algorithm results (object type, confidence), as shown below:  
![image-20220503221020331](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/08_FAQ/image/hardware_and_system/image-20220511181747071.png)