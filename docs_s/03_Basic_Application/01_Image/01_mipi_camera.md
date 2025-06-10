---
sidebar_position: 1
---

# 3.1.1 MIPI摄像头使用

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=19

开发板上安装了`mipi_camera.py`程序用于测试MIPI摄像头的数据通路，该示例会实时采集MIPI摄像头的图像数据，然后运行目标检测算法，最后把图像数据和检测结果融合后通过HDMI接口输出。

## 环境准备

  - 将MIPI摄像头模组连接到开发板MIPI CSI接口，具体连接方法可以参考-[硬件简介-MIPI接口](https://developer.d-robotics.cc/rdk_doc/Quick_start/hardware_introduction/rdk_x3#mipi_port)
  - 通过HDMI线缆连接开发板和显示器

## 运行方式
按照以下命令执行程序

  ```bash
  sunrise@ubuntu:~$ cd /app/pydev_demo/03_mipi_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/03_mipi_camera_sample$ python3 mipi_camera.py
  ```

<details>
  <summary>RDK X5在使用该demo的时候会要求选择摄像头的配置，具体可以点击查看</summary>

  在终端中运行之后，会出现“please choose sensor config,xxxx”的要求。

  ![screenshot-20241217-115245](../../../static/img/03_Basic_Application/01_Image/image/mipi_camera/screenshot-20241217-115245.png)
  
  在运行的时候选择RKD X5支持的配置，上图中选择0或者1都可以。

  启动过程可以参考如下视频：
  ![gif-20241217-115536](../../../static/img/03_Basic_Application/01_Image/image/mipi_camera/20241217-115536.gif)

</details>

## 预期效果
程序执行后，显示器会实时显示摄像头画面及目标检测算法的结果(目标类型、置信度)，如下所示：  
![image-20220503221020331](../../../static/img/03_Basic_Application/01_Image/image/mipi_camera/image-20220511181747071.png)

:::tip

详细代码实现说明请查阅[基于MIPI摄像头推理](../../04_Algorithm_Application/01_pydev_dnn_demo/mipi_camera.md)章节。

:::
