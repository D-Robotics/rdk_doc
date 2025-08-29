---
sidebar_position: 2
---

# 3.1.2 USB摄像头使用

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=18

开发板上安装了 `usb_camera_fcos.py` 程序用于测试USB摄像头的数据通路，该示例会实时采集USB摄像头的图像数据，然后运行目标检测算法，最后把图像数据和检测结果融合后通过HDMI接口输出。

## 环境准备

  - USB 摄像头接入到开发板上，确认生成 `/dev/videoX` 设备节点，`X` 代表数字，例如 `/dev/video0`
  - 通过 HDMI 线缆连接开发板和显示器

## 运行方式
按照以下命令执行程序

  ```shell
  sunrise@ubuntu:~$ cd /app/pydev_demo/02_usb_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/02_usb_camera_sample$ sudo python3 ./usb_camera_fcos.py
  ```

## 预期效果
程序执行后，显示器会实时显示摄像头画面及目标检测算法的结果(目标类型、置信度)，如下所示：
  ![image-20220612110739490](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/usb_camera/image-20220612110739490.png)

:::tip

详细代码实现说明请查阅[基于USB摄像头推理](../../03_Basic_Application/01_Image/usb_camera.md)章节。
对接两个USB摄像头前，需要通过 rmmod uvcvideo;modprobe uvcvideo quirks=128 限制 uvcvideo 带宽占用

:::

## 多摄像头支持

RDK系列板卡都只有 一路 USB 控制器，多路 USB 设备都是通过 USB HUB 扩展出来的。但本质上，所有外设共享的依然是这一路 USB 的总带宽。

- 对于 USB 2.0，所有设备加起来只能共享 480 Mbps（约 500Mbit/s） 的总带宽；
- 并不存在 “USB 2.0 外设共享 USB 3.0 总带宽” 这种情况。

因此，在默认情况下，如果接入一台 USB 2.0 Camera，它通常会“贪心”地申请几乎全部的带宽，导致无法再接第二台相机。

RDK 特殊的UVC 驱动（uvcvideo）可以通过`limitsize`参数在一定范围内限制相机的带宽占用，从而让两个相机同时取流。

  ```shell
rmmod uvcvideo.ko
modprobe uvcvideo limitsize=2
  ```

但是，USB 带宽预算的核心参数是`wMaxPacketSize` ，它在设备端 固件 的 端点描述符中写死。

如果固件提供了多套 `Video Streaming` 配置，`wMaxPacketSize` 可以从 `1 x 192 bytes`、`1 x 384 bytes` 一直到 `3 x 1020 bytes` 逐步选择。

在实际启用 MJPEG 视频流时，设备不会直接启用最大值，而是根据分辨率、帧率选择较合适的 `wMaxPacketSize`。

因此，它在带宽分配上更灵活，搭配`uvcvideo limitsize=2`能够允许两台相机同时取流，例如：

  ```shell
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        1
      bAlternateSetting       1
      bNumEndpoints           1
      bInterfaceClass        14 Video
      bInterfaceSubClass      2 Video Streaming
      bInterfaceProtocol      0 
      iInterface              0 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            5
          Transfer Type            Isochronous
          Synch Type               Asynchronous
          Usage Type               Data
        wMaxPacketSize     0x00c0  1x 192 bytes
        bInterval               1
   
   ......

    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        1
      bAlternateSetting      11
      bNumEndpoints           1
      bInterfaceClass        14 Video
      bInterfaceSubClass      2 Video Streaming
      bInterfaceProtocol      0 
      iInterface              0 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            5
          Transfer Type            Isochronous
          Synch Type               Asynchronous
          Usage Type               Data
        wMaxPacketSize     0x13fc  3x 1020 bytes
        bInterval 
  ```

 如果固件只提供了一套 `Video Streaming` 配置，`wMaxPacketSize = 3 x 1024 bytes`。

 这意味着它总是按最大带宽申请，无论驱动如何限制，都无法让两台相机同时稳定取流。

  ```shell
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        1
      bAlternateSetting       1
      bNumEndpoints           1
      bInterfaceClass        14 Video
      bInterfaceSubClass      2 Video Streaming
      bInterfaceProtocol      0 
      iInterface              0 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x88  EP 8 IN
        bmAttributes            5
          Transfer Type            Isochronous
          Synch Type               Asynchronous
          Usage Type               Data
        wMaxPacketSize     0x1400  3x 1024 bytes
        bInterval  
  ```

  查询wMaxPacketSize方法

  ```shell
  lsusb -v -d <vid>:<pid>
  ```

 