# RDK S100 Multimedia Interface Description

The Ubuntu system on the development board comes pre-installed with the Python version of the `libsrcampy` image and multimedia module, which allows you to create several types of objects such as `Camera`, `Encode`, `Decode`, and `Display` to perform functions including camera image capture, image processing, video encoding, video decoding, and display output.

Basic usage of the module is as follows:

## RDK S100:

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