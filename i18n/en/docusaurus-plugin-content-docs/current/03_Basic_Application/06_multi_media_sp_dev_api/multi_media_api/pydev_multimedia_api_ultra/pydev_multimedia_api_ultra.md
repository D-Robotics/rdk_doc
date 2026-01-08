# RDK Ultra 多媒体接口说明

The development board's Ubuntu system comes pre-installed with the Python version of the `hobot_spdev` image multimedia module, which can create several objects such as `Camera`, `Encode`, `Decode`, and `Display`. These objects are used to perform functions such as camera image acquisition, image processing, video encoding, video decoding, and display output.

The basic usage of the module is as follows:

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
