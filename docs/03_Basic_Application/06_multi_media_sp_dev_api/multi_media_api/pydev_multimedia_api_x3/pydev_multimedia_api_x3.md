# 3.4.3 RDK X3/X5多媒体接口说明

开发板Ubuntu系统预装了Python版本的`libsrcampy`图像多媒体模块，可以创建`Camera`，`Encode`，`Decode`，`Display`等几种对象，用于完成摄像头图像采集、图像处理、视频编码、视频解码和显示输出等功能。

模块基础使用方式如下：

## RDK X3:

```python
from hobot_vio import libsrcampy

#create camera object
camera = libsrcampy.Camera()

#create encode object
encode = libsrcampy.Encode()

#create decode object
decode = libsrcampy.Decode()

#create display object
display = libsrcampy.Display()
```

## RDK X5:

```python
from hobot_vio import libsrcampy

#create camera object
camera = libsrcampy.Camera()

#create encode object
encode = libsrcampy.Encoder()

#create decode object
decode = libsrcampy.Decoder()

#create display object
display = libsrcampy.Display()
```