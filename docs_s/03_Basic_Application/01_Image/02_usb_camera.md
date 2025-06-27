---
sidebar_position: 2
---

# 3.1.2 USB摄像头使用

<!--
Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=18

开发板上安装了 `usb_camera_fcos.py` 程序用于测试USB摄像头的数据通路，该示例会实时采集USB摄像头的图像数据，然后运行目标检测算法，最后把图像数据和检测结果融合后通过HDMI接口输出。

## 环境准备

  - USB 摄像头接入到开发板上，确认生成 `/dev/videoX` 设备节点，`X` 代表数字，例如 `/dev/video0`
  - 通过 HDMI 线缆连接开发板和显示器

## 运行方式
按照以下命令执行程序

  ```shell
  sunrise@ubuntu:~$ cd /app/pydev_demo/02_usb_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/02_usb_camera_sample$ python3 usb_camera_fcos.py
  ```

## 预期效果
程序执行后，显示器会实时显示摄像头画面及目标检测算法的结果(目标类型、置信度)，如下所示：
  ![image-20220612110739490](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/usb_camera/image-20220612110739490.png)

:::tip

详细代码实现说明请查阅[基于USB摄像头推理](../rdk_s/04_Algorithm_Application/01_pydev_dnn_demo/usb_camera)章节。
对接两个USB摄像头前，需要通过 rmmod uvcvideo;modprobe uvcvideo quirks=128 限制 uvcvideo 带宽占用

:::

-->

开发板上安装了 `usb_camera_snap.py` 程序用于测试USB摄像头的数据通路，该示例会实时采集USB摄像头的图像数据, 并保存到本地img.jpg文件中。

## 环境准备

  - USB 摄像头接入到开发板上，确认生成 `/dev/videoX` 设备节点，`X` 代表数字，例如 `/dev/video0`
  - 通过 HDMI 线缆连接开发板和显示器

## 运行方式
按照以下命令执行程序

  ```shell
  sunrise@ubuntu:~$ cd /app/pydev_demo/02_usb_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/02_usb_camera_sample$ python3 usb_camera_snap.py
  ```

## 预期效果
程序执行后，显示器会抓取当前摄像头画面，如下所示：
  ![usbsnap_2025-06-24_15-50-26](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/usb_camera/usbsnap_2025-06-24_15-50-26.png)

:::tip

对接两个USB摄像头前，需要通过 rmmod uvcvideo;modprobe uvcvideo quirks=128 限制 uvcvideo 带宽占用

:::
