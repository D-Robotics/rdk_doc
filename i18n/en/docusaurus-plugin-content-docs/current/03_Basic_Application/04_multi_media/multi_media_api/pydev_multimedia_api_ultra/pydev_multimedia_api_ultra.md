# 4.6 RDK Ultra 多媒体接口说明

开发板Ubuntu系统预装了Python版本的`hobot_spdev`图像多媒体模块，可以创建`Camera`，`Encode`，`Decode`，`Display`等几种对象，用于完成摄像头图像采集、图像处理、视频编码、视频解码和显示输出等功能。

模块基础使用方式如下：

```python
from hobot_spdev import libsppydev as srcampy

#create camera object
camera = srcampy.Camera()

#create encode object
encode = srcampy.Encode()

#create decode object
decode = srcampy.Decode()

#create display object
display = srcampy.Display()
```
