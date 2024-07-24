# 4.5 RDK X3 Multimedia Interface Instructions

The Ubuntu system pre-installed on the development board comes with the Python version of the `libsrcampy` image multimedia module. It can create several objects such as `Camera`, `Encode`, `Decode`, and `Display`, which are used to complete functions such as camera image capture, image processing, video encoding, video decoding, and display output.

The basic usage of the module is as follows:

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