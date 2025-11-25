---
sidebar_position: 2
---
# 3.1.2 Using USB Camera

Video: https://www.youtube.com/watch?v=7xNgU1i2xsk&list=PLSxjn4YS2IuFUWcLGj2_uuCfLYnNYw6Ld&index=2

The development board is equipped with the `usb_camera_fcos.py` program to test the data path of the USB camera. This example will capture the image data from the USB camera in real time, then run the object detection algorithm, and finally output the merged image data and detection results through the HDMI interface.

## Environment Preparation

  - Connect the USB camera to the development board and make sure that the `/dev/video8` device node is created.
  - Connect the development board to the monitor using an HDMI cable.

## Execution Method
Execute the program with the following command.

  ```shell
  sunrise@ubuntu:~$ cd /app/pydev_demo/02_usb_camera_sample/
  sunrise@ubuntu:/app/pydev_demo/02_usb_camera_sample$ sudo python3 ./usb_camera_fcos.py
  ```

## Expected Result
After running the program, the monitor will display the camera image and the results of the object detection algorithm (object type, confidence), as shown below:  
  ![image-20220612110739490](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_Image/image/usb_camera/image-20220612110739490.png)

:::tip

For detailed code implementation instructions, please refer to the [USB Camera Inference](../../04_Algorithm_Application/01_pydev_dnn_demo/usb_camera.md) chapter.

:::

## Multi-Camera Support

RDK series boards only have one USB controller, and multiple USB devices are expanded via a USB hub. Essentially, all peripherals still share the total bandwidth of this single USB controller.

For USB 2.0, all devices together can only share 480 Mbps (approximately 500 Mbit/s) of total bandwidth.

There is no situation where USB 2.0 devices share the bandwidth of USB 3.0.

Therefore, by default, if you connect a USB 2.0 camera, it will usually “greedily” request almost the entire bandwidth, making it impossible to use a second camera simultaneously.

RDK’s special UVC driver (uvcvideo) can use the limitsize parameter to limit the camera’s bandwidth usage within a certain range, allowing two cameras to stream at the same time.

  ```shell
rmmod uvcvideo.ko
modprobe uvcvideo limitsize=2
  ```

However, the core parameter for USB bandwidth budgeting is `wMaxPacketSize`, which is defined in the endpoint descriptor of the device firmware.

If the firmware provides multiple Video Streaming configurations, `wMaxPacketSize` can range from `1 x 192 bytes`, `1 x 384 bytes`, up to `3 x 1020 bytes`.

In practice, when enabling MJPEG video streaming, the device does not necessarily use the maximum packet size. Instead, it selects a suitable `wMaxPacketSize` based on resolution and frame rate.

Therefore, bandwidth allocation is more flexible, and with uvcvideo limitsize=2, two cameras can stream simultaneously.

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


If the firmware provides only one Video Streaming configuration with `wMaxPacketSize = 3 x 1024 bytes`,

this means the device always requests the maximum bandwidth.

No matter how the driver tries to limit it, it will be impossible for two cameras to stream simultaneously and stably.

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

  get wMaxPacketSize

  ```shell
  lsusb -v -d <vid>:<pid>
  ```

 