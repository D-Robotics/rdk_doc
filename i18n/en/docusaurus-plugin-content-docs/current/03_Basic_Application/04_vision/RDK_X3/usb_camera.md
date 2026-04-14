---
sidebar_position: 2
---

# USB Camera Usage

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=18

The `usb_camera_fcos.py` program is installed on the development board to test the data path of the USB camera. This example captures image data from the USB camera in real-time, runs the object detection algorithm, and then outputs the fused image data and detection results through the HDMI interface.

## Environment Preparation

  - Connect the USB camera to the development board and ensure that the `/dev/videoX` device node is generated, where `X` represents a number, e.g., `/dev/video0`
  - Connect the development board and the monitor using an HDMI cable

## How to Run
Execute the program using the following commands:

  ```shell
  sunrise@ubuntu:~$ cd /app/pydev_demo/02_usb_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/02_usb_camera_sample$ sudo python3 ./usb_camera_fcos.py
  ```

## Expected Outcome
After the program is executed, the monitor will display the camera feed and the results of the object detection algorithm (object type, confidence level) in real-time, as shown below:
  ![image-20220612110739490](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/usb_camera/image-20220612110739490.png)

:::tip

For detailed code implementation instructions, please refer to the [Inference Based on USB Camera](../../03_pydev_demo_sample/RDK_X3/02_usb_camera_sample.md) section.
Before connecting two USB cameras, you need to limit the bandwidth usage of uvcvideo by running `rmmod uvcvideo; modprobe uvcvideo quirks=128`

:::

## Multi-Camera Support

RDK series boards have only one USB controller. Multiple USB devices are expanded through a USB HUB. However, all peripherals essentially share the total bandwidth of this single USB port.

- For USB 2.0, all devices combined can only share a total bandwidth of 480 Mbps (approximately 500 Mbit/s);
- There is no scenario where "USB 2.0 peripherals share USB 3.0 total bandwidth."

Therefore, by default, when a USB 2.0 camera is connected, it tends to "greedily" request nearly all the bandwidth, making it impossible to connect a second camera.

The special UVC driver (uvcvideo) on the RDK can limit the camera's bandwidth usage within a certain range using the `limitsize` parameter, allowing two cameras to stream simultaneously.

  ```shell
rmmod uvcvideo.ko
modprobe uvcvideo limitsize=2
  ```

However, the core parameter for USB bandwidth budgeting is `wMaxPacketSize`, which is hardcoded in the endpoint descriptor of the device firmware.

If the firmware provides multiple `Video Streaming` configurations, `wMaxPacketSize` can be selected step by step from `1 x 192 bytes`, `1 x 384 bytes`, up to `3 x 1020 bytes`.

When an MJPEG video stream is actually enabled, the device does not directly use the maximum value but selects an appropriate `wMaxPacketSize` based on resolution and frame rate.

Therefore, it is more flexible in bandwidth allocation, and when combined with `uvcvideo limitsize=2`, it allows two cameras to stream simultaneously. For example:

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

If the firmware provides only one set of `Video Streaming` configurations, `wMaxPacketSize = 3 x 1024 bytes`.

This means it always requests the maximum bandwidth, and no matter how the driver limits it, two cameras cannot stream simultaneously and stably.

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

  How to query `wMaxPacketSize`

  ```shell
  lsusb -v -d <vid>:<pid>
  ```