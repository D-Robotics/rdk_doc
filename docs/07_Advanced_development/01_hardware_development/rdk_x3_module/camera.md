---
sidebar_position: 4
---

# 摄像头使用

RDK X3 Module载板提供`CAM0`、`CAM2`两路15pin MIPI CSI接口，可以支持OV5647、IMX219、IMX477等树莓派摄像头的接入。摄像头排线接入时，需保持蓝面朝上。此外示例程序中已实现摄像头自动探测，因此用户无需关心CAM接口跟摄像头的对应关系。

开发板上安装了`mipi_camera.py`程序用于测试MIPI摄像头的数据通路，该示例会实时采集MIPI摄像头的图像数据，然后运行目标检测算法，最后把图像数据和检测结果融合后通过HDMI接口输出。

- 运行方式：按照以下命令执行程序

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ sudo python3 ./mipi_camera.py 
  ```

- 预期效果：程序执行后，显示器会实时显示摄像头画面及目标检测算法的结果(目标类型、置信度)。
