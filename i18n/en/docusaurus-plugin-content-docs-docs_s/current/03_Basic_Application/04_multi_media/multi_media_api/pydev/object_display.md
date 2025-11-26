---
sidebar_position: 4
---

# Display Object

The Display object implements video display functionality, capable of outputting image data to a monitor via the `HDMI` interface. This object includes methods such as `display`, `set_img`, `set_graph_rect`, `set_graph_word`, and `close`. Detailed descriptions are provided below:

## display
<font color='Blue'>【Function Description】</font>

Initializes the display module and configures display parameters.

<font color='Blue'>【Function Declaration】</font>  

```python
Display.display(chn, width, height, vot_intf, vot_out_mode)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter    | Description                              | Value Range         |
| ------------ | ---------------------------------------- | ------------------- |
| chn          | Display output layer                     | 0: Video layer, 2: Graphics layer |
| width        | Width of the input image                 | Up to 1920          |
| height       | Height of the input image                | Up to 1080          |
| vot_intf     | Video interface output resolution        | Default: 0 (1080p)  |
| vot_out_mode | Video output interface                   | Default: 1 (HDMI output) |

<font color='Blue'>【Usage】</font> 

```python
#create display object
disp = libsrcampy.Display()

#enable display function, resolution: 1080p, interface: HDMI
ret = disp.display(0, 1920, 1080, 0, 1)
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font> 

The HDMI interface resolution on the development board is obtained from the monitor's EDID. Currently, only the following resolutions are supported: `1920x1080`, `1280x720`, `1024x600`, and `800x480`. When enabling the display module, ensure that the configured resolution matches the actual resolution of the connected monitor.

<font color='Blue'>【Reference Code】</font>  

None

## set_img

<font color='Blue'>【Function Description】</font>

Feeds display data into the display module. The data format must be `NV12`.

<font color='Blue'>【Function Declaration】</font>  

```python
Display.set_img(img)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter    | Description                              | Value Range         |
| ------------ | ---------------------------------------- | ------------------- |
| img          | Image data to be displayed               | NV12 format         |

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font> 

This interface must be used only after enabling the display functionality via the `display` interface. The input data must be in `NV12` format.

<font color='Blue'>【Reference Code】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_display():
    #create display object
    disp = libsrcampy.Display()

    #enable display function
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    fo = open("output.img", "rb")
    img = fo.read()
    fo.close()

    #send image data to display
    ret = disp.set_img(img)
    print ("Display set_img return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

test_display()
```

## set_graph_rect

<font color='Blue'>【Function Description】</font>

Draws a rectangle on the graphics layer of the display module.

<font color='Blue'>【Function Declaration】</font>

```python
Display.set_graph_rect(x0, y0, x1, y1, chn, flush, color, line_width)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter    | Description                              | Value Range         |
| ------------ | ---------------------------------------- | ------------------- |
| x0           | X-coordinate of the top-left corner of the rectangle | Within video frame dimensions |
| y0           | Y-coordinate of the top-left corner of the rectangle | Within video frame dimensions |
| x1           | X-coordinate of the bottom-right corner of the rectangle | Within video frame dimensions |
| y1           | Y-coordinate of the bottom-right corner of the rectangle | Within video frame dimensions |
| chn          | Graphics layer channel number            | Range: 2–3, default: 2 |
| flush        | Whether to clear the graphics layer buffer | 0: No, 1: Yes       |
| color        | Rectangle color                          | ARGB8888 format     |
| line_width   | Width of the rectangle border            | Range: 1–16, default: 4 |

<font color='Blue'>【Usage】</font>

```python
#enable graph layer 2
ret = disp.display(2)
print ("Display display 2 return:%d" % ret)

#set osd rectangle
ret = disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
```

<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font>

This interface must be used only after enabling the display functionality via the `display` interface.

<font color='Blue'>【Reference Code】</font>

None

## set_graph_word

<font color='Blue'>【Function Description】</font>

Draws text characters on the graphics layer of the display module.

<font color='Blue'>【Function Declaration】</font>

```python
Display.set_graph_word(x, y, str, chn, flush, color, line_width)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter    | Description                              | Value Range         |
| ------------ | ---------------------------------------- | ------------------- |
| x            | X-coordinate of the starting position for text | Within video frame dimensions |
| y            | Y-coordinate of the starting position for text | Within video frame dimensions |
| str          | Text string to be drawn                  | GB2312 encoding     |
| chn          | Graphics layer channel number            | Range: 2–3, default: 2 |
| flush        | Whether to clear the graphics layer buffer | 0: No, 1: Yes       |
| color        | Text color                               | ARGB8888 format     |
| line_width   | Line width of the text characters        | Range: 1–16, default: 1 |

<font color='Blue'>【Usage】</font>

```python
#enable graph layer 2
ret = disp.display(2)
```print ("Display display 2 return:%d" % ret)

#set osd string
string = "horizon"
ret = disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
print ("Display set_graph_word return:%d" % ret)
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font> 

This API must be used after enabling the display functionality via the `display` interface.

<font color='Blue'>【Sample Code】</font>  

None

## close

<font color='Blue'>【Function Description】</font>

Closes the display module.

<font color='Blue'>【Function Declaration】</font>  

```python
Display.close()
```

<font color='Blue'>【Parameter Description】</font>  

None

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font> 

This API must be used after enabling the display functionality via the `display` interface.

<font color='Blue'>【Sample Code】</font>  

None

## bind Interface

<font color='Blue'>【Function Description】</font>

This interface binds the output and input data streams of modules such as `Camera`, `Encoder`, `Decoder`, and `Display`. Once bound, data automatically flows between the modules without requiring further user intervention. For example, after binding `Camera` and `Display`, camera data will be automatically output to the display through the display module without calling additional APIs.

<font color='Blue'>【Function Declaration】</font>
```python
    libsrcampy.bind(src, dst)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter | Description         | Value Range |
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

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Sample Code】</font>

None

## unbind Interface

<font color='Blue'>【Function Description】</font>

Unbinds two previously bound modules.

<font color='Blue'>【Function Declaration】</font>
```python
libsrcampy.unbind(src, dst)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter | Description         | Value Range |
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
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Sample Code】</font>

None