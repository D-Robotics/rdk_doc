# 3.4.3 RDK X3/X5 Multimedia Interface User Guide

The development board comes pre-installed with the Python version of the `libsrcampy` multimedia module on the Ubuntu system. This module allows for the creation of various objects such as `Camera`, `Encode`, `Decode`, and `Display`, which can be used for tasks like image capture, image processing, video encoding, video decoding, and display output.

### Basic Usage of the Module:

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