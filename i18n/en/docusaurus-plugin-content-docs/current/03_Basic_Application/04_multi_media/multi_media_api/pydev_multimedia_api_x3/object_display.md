---
sidebar_position: 4
---
# Display Object

The Display object implements video display functionality, which can output image data to a monitor via the HDMI interface. This object includes methods such as `display`, `set_img`, `set_graph_rect`, `set_graph_word`, `close`, etc. The detailed description is as follows:

## display
<font color='Blue'>[Function Description]</font>

Initialize the display module and configure the display parameters.

<font color='Blue'>[Function Declaration]</font>  

```python
Display.display(chn, width, height, vot_intf, vot_out_mode)
```

<font color='Blue'>[Parameter Description]</font>  

| Parameter Name | Definition | Value Range |
| -------------- | ---------- | ----------- |
| chn | Display output layer | 0: Video Layer, 2: Graphic Layer |
| width | Width of the input image | Up to 1920 |
| height | Height of the input image | Up to 1080 |
| vot_intf | Video interface output resolution | Default: 0, 1080p |
| vot_out_mode | Video output interface | Default: 1, HDMI output |

<font color='Blue'>[Usage]</font> 

```python
#create display object
disp = libsrcampy.Display()

#enable display function, solution: 1080p, interface: HDMI
ret = disp.display(0, 1920, 1080, 0, 1)
```

<font color='Blue'>[Return Value]</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0 | Success |
| -1 | Failed |

<font color='Blue'>[Note]</font> 

The HDMI interface resolution on the development board is based on the EDID (Extended Display Identification Data) of the monitor. Currently, only a few resolutions are supported, including `1920x1080`, `1280x720`, `1024x600`, and `800x480`. When enabling the display module, it is necessary to configure the resolution to match the actual resolution of the monitor.

## set_img

<font color='Blue'>【Description】</font>

Send display data to the display module, the format needs to be `NV12`

<font color='Blue'>【Function Declaration】</font> 

```python
Display.set_img(img)
```

<font color='Blue'>【Parameter Description】</font> 

| Parameter | Definition          | Value Range |
| --------- | ------------------ | ----------- |
| img       | Image data to show | NV12 format |

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font> 

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font> 

This interface needs to be used after enabling the display function with the `display` interface. The input data needs to be in the `NV12` format.

<font color='Blue'>【Reference Code】</font> 

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_display():
    #create display object
    disp = libsrcampy.Display()

    # Enable display function
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    fo = open("output.img", "rb")
    img = fo.read()
    fo.close()

    # Send image data to display
    ret = disp.set_img(img)
    print ("Display set_img return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

test_display()
```

## set_graph_rect

<font color='Blue'>【Function Description】</font>

Draw a rectangular box on the graphic layer of the display module.

<font color='Blue'>【Function Declaration】</font>

```python
Display.set_graph_rect(x0, y0, x1, y1, chn, flush, color, line_width)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter   | Definition             |  Value Range      |
| ---------- | ----------------------- | --------- |
| x0         | x-coordinate of the upper left corner of the rectangle | Within the size of the video frame   |
| y0         | y-coordinate of the upper left corner of the rectangle | Within the size of the video frame   |
| x1         | x-coordinate of the lower right corner of the rectangle   | Within the size of the video frame   |
| y1         | y-coordinate of the lower right corner of the rectangle   | Within the size of the video frame   |
| chn        | Channel number of the graphic layer |  2 to 3 by default     |
| flush      | Whether to clear the graphic layer buffer   | 0: No, 1: Yes      |
| color      | Color settings of the rectangle | ARGB8888 format |
| line_width | Width of the rectangle border        | 1 to 16 by default      |

<font color='Blue'>【Usage】</font>

```python
# Enable graph layer 2
ret = disp.display(2)```python
print ("Display display 2 return:%d" % ret)

#set osd rectangle
ret = disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
```

<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Note】</font>

This interface needs to be used after enabling the display function using the `display` interface.

<font color='Blue'>【Reference Code】</font>

None

## set_graph_word

<font color='Blue'>【Description】</font>

Draws characters on the graphic layer of the display module.

<font color='Blue'>【Function Declaration】</font>

```python
Display.set_graph_word(x, y, str, chn, flush, color, line_width)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name   | Description   | Range |
| ---------- | ---------------------- | ------------- |
| x          | x-coordinate of the starting position for drawing characters   | Not exceeding the size of the video screen |
| y          | y-coordinate of the starting position for drawing characters   | Not exceeding the size of the video screen |
| str        | Character data to be drawn   | GB2312 encoding |
| chn        | Graphic layer channel number |  Range: 2~3, default: 2   |
| flush      | Whether to clear the graphic layer buffer   | 0: No, 1: Yes   |
| color      | Character color setting   | ARGB8888 format |
| line_width | Width of the character line   | Range: 1~16, default: 1   |

<font color='Blue'>【Usage】</font>

```python
#enable graph layer 2
ret = disp.display(2)
print("Display display 2 return:%d" % ret)

# set osd string
string = "D-Robotics"
ret = disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
print("Display set_graph_word return:%d" % ret)
```

<font color='Blue'>[Return Value]</font>

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failed |

<font color='Blue'>[Note]</font> 

This interface needs to be used after enabling the display function with the `display` interface.

<font color='Blue'>[Reference Code]</font>  

None

## close

<font color='Blue'>[Function Description]</font>

Close the display module.

<font color='Blue'>[Function Declaration]</font>  

```python
Display.close()
```

<font color='Blue'>[Parameter Description]</font>  

None

<font color='Blue'>[Usage]</font> 

None

<font color='Blue'>[Return Value]</font>  

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failed |<font color='Blue'>【Notice】</font>

This interface needs to be used after enabling the display function using the `display` interface.

<font color='Blue'>【Reference Code】</font> 

None

## bind interface

<font color='Blue'>【Function Description】</font>

This interface can bind the output and input data streams of the `Camera`, `Encoder`, `Decoder`, and `Display` modules. After binding, there is no need for user operation, and the data can automatically flow between the binding modules. For example, after binding the `Camera` and `Display`, the camera data will be automatically displayed on the screen through the display module, without the need to call additional interfaces.

<font color='Blue'>【Function Declaration】</font>

```python
    libsrcampy.bind(src, dst)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description | Value Range |
| -------- | ------------ | --- |
| src      | Source data module   | `Camera`, `Encoder`, `Decoder` modules |
| dst      | Destination data module | `Camera`, `Encoder`, `Decoder`, `Display` modules |

<font color='Blue'>【Usage】</font>

```python
#create camera object
cam = libsrcampy.Camera()
ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = libsrcampy.Encoder()
ret = enc.encode(0, 1, 1920, 1080)
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = libsrcampy.bind(cam, enc)
print("libsrcampy bind return:%d" % ret)
```

<font color='Blue'>【Return Value】</font>

| Return value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Reference Code】</font>

None

## unbind Interface

<font color='Blue'>【Description】</font>

Unbind two bound modules.

<font color='Blue'>【Function Declaration】</font>

```python
libsrcampy.unbind(src, dst)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description         | Value Range |
| -------- | ------------ | --- |
| src      | Source data module   | `Camera`, `Encoder`, `Decoder` modules |
| dst      | Destination data module | `Camera`, `Encoder`, `Decoder`, `Display` modules |

<font color='Blue'>【Usage】</font>

```python
#create camera object
cam = libsrcampy.Camera()
ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = libsrcampy.Encoder()
ret = enc.encode(0, 1, 1920, 1080)
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = libsrcampy.bind(cam, enc)
print("libsrcampy bind return:%d" % ret)

#unbind, input: cam, output: enc
ret = libsrcampy.unbind(cam, enc)
print("libsrcampy unbind return:%d" % ret)
```

<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------ | ---- |
| 0      | Successful |
| -1    | Failed |

<font color='Blue'>【Note】</font>

None

<font color='Blue'>【Reference Code】</font>

None