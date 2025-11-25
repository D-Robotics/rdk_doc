---
sidebar_position: 4
---

# Display Object

The `Display` object implements video display functionality and can output image data to a monitor through the `HDMI` interface. This object includes methods such as `display`, `send_frame`, `set_rect`, `set_word`, `close`, and more. The detailed explanation of each method is as follows:

## display
<font color='Blue'>【Function Description】</font>

Initializes the display module and configures the display parameters.

<font color='Blue'>【Function Declaration】</font>


```python
Display.display([width, height])
```
<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                 | Value Range      |
| -------------- | --------------------------- | ---------------- |
| width          | Width of the input image     | Not exceeding 1920 |
| height         | Height of the input image    | Not exceeding 1080 |

<font color='Blue'>【Usage】</font>


```python
#create display object
disp = srcampy.Display()

#enable display function, solution: 1080p, interface: HDMI
ret = disp.display([1920, 1080])
```
<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>  

Currently, only `1920x1080` resolution is supported.

<font color='Blue'>【Reference Code】</font>  

None

## send_frame

<font color='Blue'>【Function Description】</font>

Inputs display data into the display module. The format must be `NV12`.

<font color='Blue'>【Function Declaration】</font>

```python
Display.send_frame(img)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description              | Value Range  |
| -------------- | ------------------------ | ------------ |
| img            | Image data to be displayed | NV12 format  |

<font color='Blue'>【Usage】</font>  

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>  

This interface must be used after enabling the display function with the `display` interface. The input data must be in `NV12` format.

<font color='Blue'>【Reference Code】</font>  



```python
import sys, os, time

import numpy as np
import cv2
from hobot_spdev import libsppydev as srcampy

def test_display():
    #create display object
    disp = srcampy.Display()

    #enable display function
    ret = disp.display([1920, 1080])
    print ("Display display 0 return:%d" % ret)

    fo = open("output.img", "rb")
    img = fo.read()
    fo.close()

    #send image data to display
    ret = disp.send_frame(img)
    print ("Display send_frame return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

test_display()
```

## set_rect

<font color='Blue'>【Function Description】</font>

Draws a rectangle on the graphical layer of the display module.

<font color='Blue'>【Function Declaration】</font>


```python
Display.set_rect(x0, y0, x1, y1, flush, color, line_width)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description                       | Value Range         |
| -------------- | --------------------------------- | ------------------- |
| x0             | The x-coordinate of the top-left corner of the rectangle | Not exceeding the video frame size |
| y0             | The y-coordinate of the top-left corner of the rectangle | Not exceeding the video frame size |
| x1             | The x-coordinate of the bottom-right corner of the rectangle | Not exceeding the video frame size |
| y1             | The y-coordinate of the bottom-right corner of the rectangle | Not exceeding the video frame size |
| flush          | Whether to clear the graphics layer buffer | 0: No, 1: Yes       |
| color          | The color of the rectangle (ARGB8888 format) | ARGB8888 format     |
| line_width     | The width of the rectangle's border | Range 1~16, default is 4 |

<font color='Blue'>【Usage】</font>


```python
#enable graph layer 2
ret = disp.display(2)
print ("Display display 2 return:%d" % ret)

#set osd rectangle
ret = disp.set_rect(100, 100, 1920, 200,  flush = 1,  color = 0xffff00ff)
```

<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>

This interface must be used after enabling the display function with the `display` interface.

<font color='Blue'>【Reference Code】</font>

None

## set_word

<font color='Blue'>【Function Description】</font>

Draws characters on the graphical layer of the display module.

<font color='Blue'>【Function Declaration】</font>


```python
Display.set_word(x, y, str,flush, color, line_width)
```
<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description                  | Value Range         |
| -------------- | ---------------------------- | ------------------- |
| x              | The x-coordinate of the starting point for drawing the character | Not exceeding the video frame size |
| y              | The y-coordinate of the starting point for drawing the character | Not exceeding the video frame size |
| str            | The character data to be drawn | GB2312 encoding     |
| flush          | Whether to clear the graphics layer buffer | 0: No, 1: Yes       |
| color          | The color of the character | ARGB8888 format     |
| line_width     | The width of the character's stroke | Range 1~16, default is 1 |

<font color='Blue'>【Usage】</font>


```python
#enable graph layer 2
ret = disp.display(2)
print ("Display display 2 return:%d" % ret)

#set osd string
string = "horizon"
ret = disp.set_word(300, 300, string.encode('gb2312'),  0, 0xff00ffff)
print ("Display set_word return:%d" % ret)
```
<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>  

This interface must be used after enabling the display function with the `display` interface.

<font color='Blue'>【Reference Code】</font>  

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
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>  

This interface must be used after enabling the display function with the `display` interface.

<font color='Blue'>【Reference Code】</font>  

None

## bind Interface

<font color='Blue'>【Function Description】</font>

This interface can bind the output and input data streams of modules such as `Camera`, `Encoder`, `Decoder`, and `Display`. After binding, data will automatically flow between the bound modules without additional user intervention. For example, after binding `Camera` and `Display`, the camera data will be automatically output to the display screen through the display module, without needing to call extra interfaces.

<font color='Blue'>【Function Declaration】</font>

```python
    srcampy.bind(src, dst)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description     | Value Range                          |
| -------------- | --------------- | ------------------------------------ |
| src            | Source data module | `Camera`, `Encoder`, `Decoder` modules |
| dst            | Destination data module | `Camera`, `Encoder`, `Decoder`, `Display` modules |

<font color='Blue'>【Usage】</font>


```python
#create camera object
cam = srcampy.Camera()
ret = cam.open_cam(-1,[1920, 1080], [1280, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = srcampy.Encoder()
ret = enc.encode(2, [1920, 1080])
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = srcampy.bind(cam, enc)
print("srcampy bind return:%d" % ret)
```
<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>

None

<font color='Blue'>【Reference Code】</font>

None

## unbind Interface

<font color='Blue'>【Function Description】</font>

This interface unbinds two previously bound modules.

<font color='Blue'>【Function Declaration】</font>

```python
srcampy.unbind(src, dst)
```
<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description     | Value Range                          |
| -------------- | --------------- | ------------------------------------ |
| src            | Source data module | `Camera`, `Encoder`, `Decoder` modules |
| dst            | Destination data module | `Camera`, `Encoder`, `Decoder`, `Display` modules |

<font color='Blue'>【Usage】</font>


```python
#create camera object
cam = srcampy.Camera()
ret = cam.open_cam(-1,[1920, 1080], [1280, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = srcampy.Encoder()
ret = enc.encode(2, [1920, 1080])
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = srcampy.bind(cam, enc)
print("srcampy bind return:%d" % ret)
#unbind, input: cam, output: enc
ret = srcampy.unbind(cam, enc)
print("srcampy unbind return:%d" % ret)
```
<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>

None

<font color='Blue'>【Reference Code】</font>

None

