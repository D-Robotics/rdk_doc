---
sidebar_position: 1
---
# Camera Object

The Camera object is used to perform image acquisition and processing functions for MIPI cameras. It includes several methods such as `open_cam`, `open_vps`, `get_img`, `set_img`, `close_cam`, etc. Here are the detailed explanations:

## open_cam

<font color='Blue'>[Function Description]</font>

Opens the specified MIPI camera channel and sets the camera output frame rate and resolution format.

<font color='Blue'>[Function Declaration]</font>

```python
Camera.open_cam(pipe_id, video_index, fps, width, height, raw_height, raw_width)
```

<font color='Blue'>[Parameter Description]</font>

| Parameter Name | Definition Description | Value Range |
| -------------- | ---------------------- | ----------- |
| pipe_id        | Pipeline channel number corresponding to the camera | Starts from 0 by default, range: 0-7 |
| video_index    | Host number corresponding to the camera, -1 for auto detection. The number can be found in the /etc/board_config.json configuration file | Values: -1, 0, 1, 2 |
| fps            | Camera image output frame rate | Depends on the camera model, default value: 30 |
| width          | Final width of the camera image output | Depends on the camera model, default value: 1920 (2560 for GC4663) |
| height         | Final height of the camera image output | Depends on the camera model, default value: 1080 (1440 for GC4663) |
| raw_height     | Width of the original RAW camera image output | Depends on the camera model, default value: 1920 (2560 for GC4663) |
| raw_width      | Height of the original RAW camera image output | Depends on the camera model, default value: 1080 (1440 for GC4663) |

<font color='Blue'>[Usage]</font>

```python
#create camera object
camera = libsrcampy.Camera()

#open MIPI Camera, fps: 30, solution: 1080p
ret = camera.open_cam(0, -1, 30, 1920, 1080)
```

<font color='Blue'>[Return Value]</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>[Notes]</font>

The `width` and `height` parameters support `list` type input, which enables the camera to output multiple sets of different resolutions. The `list` supports up to 4 groups of downsizing and 1 group of enlargement, with a scaling range between `1/8` and `1.5` times the original resolution of the camera. The usage is as follows:

```python
ret = cam.open_cam(0, -1, 30, [1920, 1280], [1080, 720])
```

The `raw_height` and `raw_width` parameters are only set when the camera is not in the default resolution. For example, when using the `IMX477` camera and want to output both 4k resolution (3840x2160) and 1080P resolution (1920x1080), you can use:

```python
cam.open_cam(0, -1, 10, [3840, 1920], [2160, 1080], 3000, 4000)
```

Supported camera resolutions are shown in the table below:

| camera | resolution |
| ---- | ----- |
|IMX219|1920x1080@30fps(default), 640x480@30fps, 1632x1232@30fps, 3264x2464@15fps(max)|
|IMX477|1920x1080@50fps(default), 1280x960@120fps, 2016x1520@40fps, 4000x3000@10fps(max)|
|OV5647|1920x1080@30fps(default), 640x480@60fps, 1280x960@30fps, 2592x1944@15fps(max)|
|F37|1920x1080@30fps(default)|
|GC4663|2560x1440@30fps(default)|

:::info Note!

When switching from `1080P` resolution to other resolutions on `IMX477`, manual reset is required. You can execute `hobot_reset_camera.py` on the board to perform the reset operation.

:::

:::info Note!

The `X3` chip has alignment requirements for the width of the `VPS` output. If the width you set does not satisfy the alignment requirement of **32**, it will be rounded up automatically. For example, if you set the output width to `720` and height to `480`, the actual width of the `VPS` output will be `736`. In the **binding** situation, this will be automatically handled in the library. In the **non-binding** situation, be aware of the width alignment when manually handling the frame buffer, and also be aware that there may be screen glitches and green lines when passing frames to the display module in such non-aligned situations.

:::

<font color='Blue'>【Reference code】</font>  

None

## open_vps

<font color='Blue'>【Function Description】</font>

Enables the vps (video process) image processing function for the specified camera channel, supporting functions such as scaling, rotation, and cropping of input images.

<font color='Blue'>【Function Declaration】</font>  

```python
Camera.open_vps(pipe_id, proc_mode, src_width, src_height, dst_width, dst_height, crop_rect, rotate, src_size, dst_size)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Definition | Value Range |
| -------------- | ---------- | ----------- |
| pipe_id | pipeline channel number corresponding to the camera | default starts from 0, range 0~7 |
| proc_mode | image processing mode configuration, supports scaling, rotation, cropping | range 1~4, representing `scaling`, `scaling+cropping`, `scaling+rotation`, `scaling+cropping+rotation` |
| src_width | image input width | depends on the camera's output width |
| src_height | image input height | depends on the camera's output height |
| dst_width | image output width | `1/8~1.5` times the input width |
| dst_height | image output height | `1/8~1.5` times the input height |
| crop_rect | cropping area width and height, input format[x, y] | within the input image size |
| rotate | rotation angle, supports up to two channel rotations | range 0~3, representing `no rotation`, `90 degrees`, `180 degrees`, `270 degrees` |
| src_size | reserved parameter | no configuration required by default |
| dst_size | reserved parameter | no configuration required by default |

<font color='Blue'>【Usage】</font> 

```python
#create camera object
camera = libsrcampy.Camera()

#enable vps function
ret = camera.open_vps(1, 1, 1920, 1080, 512, 512)
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Definition |                 
| ------------ | ---------- |
| 0            | Success    |
| -1           | Failure    |

:::info Note!
- VPS processing function supports up to 6 channel outputs, 5 for scaling down and 1 for scaling up. The scaling range is between `1/8~1.5` times the original resolution. Multiple channel configurations are passed through the input parameter `list`.
- Image cropping function starts from the top left corner of the image and crops according to the configured size.
- Image cropping is performed before scaling and rotation operations. Multiple channel configurations are passed through the input parameter `list`.
:::

:::info Note!

The `X3` chip has alignment requirements for the width of the `VPS` output. If the width you set does not meet the alignment requirement of **32**, it will be rounded up automatically. For example, if you set the output width to `720` and height to `480`, the actual output width of `VPS` will be `736`. In the **binding** situation, this situation will be automatically handled in the library **automatically**. In the case of **non-binding**, pay attention to the width alignment issue when manually handling the frame buffer, as well as the occurrence of **screen distortion and green lines** when passing the frame to the display module in this non-aligned situation.

:::

```python
#creat camera object
camera = libsrcampy.Camera()

#enable vps function
#input: 4k, output0: 1080p, output1: 720p
#ouput0 croped by [2560, 1440]
ret = camera.open_vps(0, 1, 3840, 2160, [1920, 1280], [1080, 720], [2560, 1440])
```

<font color='Blue'>【Reference Code】</font>  
N/A

## get_img

<font color='Blue'>【Description】</font>

Get the image output from the camera object. Must be called after `open_cam` or `open_vps`.

<font color='Blue'>【Function Declaration】</font> 

```python
Camera.get_img(module, width, height)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter | Definition | Range |
| -------- | ------- | ----------- |
| module   | The module from which to get the image | Default value: 2 |
| width    | The width of the image to be obtained | The output width set by `open_cam` or `open_vps` |
| height   | The height of the image to be obtained | The output height set by `open_cam` or `open_vps` |


<font color='Blue'>【Usage】</font> 

```python
cam = libsrcampy.Camera()

#open MIPI Camera, fps: 30, solution: 1080p
ret = cam.open_cam(0, 1, 30, 1920, 1080)

#wait for 1s
time.sleep(1)

#get one image from camera
img = cam.get_img(2)
```

<font color='Blue'>【Return Value】</font>

| Return Value | Definition |
| ------ | ----- |
| 0      | Success  |
| -1    | Failure   |

<font color='Blue'>【Notes】</font>

This method needs to be called after `open_cam` and `open_vps`.

<font color='Blue'>【Reference Code】</font>

```python
import sys, os, time

from hobot_vio import libsrcampy

def test_camera():
    cam = libsrcampy.Camera()

    #open MIPI camera, fps: 30, solution: 1080p
    ret = cam.open_cam(0, 1, 30, 1920, 1080)
    print("Camera open_cam return:%d" % ret)

    # wait for 1s
    time.sleep(1)

    #get one image from camera   
    img = cam.get_img(2)
    if img is not None:
        #save file
        fo = open("output.img", "wb")
        fo.write(img)
        fo.close()
        print("camera save img file success")
    else:
        print("camera save img file failed")

    #close MIPI camera
    cam.close_cam()
    print("test_camera done!!!")

test_camera()
```

## set_img

<font color='Blue'>【Function Description】</font>

Input image to the `vps` module and trigger the image processing operation.

<font color='Blue'>【Function Declaration】</font>  

```python
Camera.set_img(img)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Definition     | Value Range      |
| -------- | -------------------- | ----- |
| img      | Image data to process | Consistent with vps input size |

<font color='Blue'>【Usage】</font> 

```python
camera = libsrcampy.Camera()

#enable vps function, input: 1080p, output: 512x512
ret = camera.open_vps(1, 1, 1920, 1080, 512, 512)
print("Camera vps return:%d" % ret)

fin = open("output.img", "rb")
img = fin.read()
fin.close()

#send image to vps module
ret = vps.set_img(img)
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Definition |                 
| ------ | ----- |
| 0      | Success  |
| -1    | Failure |

<font color='Blue'>【Notes】</font> 

This interface needs to be called after `open_vps`.

<font color='Blue'>【Reference Code】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_camera_vps():
    vps = libsrcampy.Camera()

    #enable vps function, input: 1080p, output: 512x512
    ret = vps.open_vps(1, 1, 1920, 1080, 512, 512)
    print("Camera vps return:%d" % ret)

    fin = open("output.img", "rb")
    img = fin.read()
    fin.close()

    #send image data to vps
    ret = vps.set_img(img)
    print ("Process set_img return:%d" % ret)

    fo = open("output_vps.img", "wb+")

    #get image data from vps
    img = vps.get_img()
    if img is not None:
        fo.write(img)
        print("encode write image success")
    else:
        print("encode write image failed")
    fo.close()

    #close vps function
    vps.close_cam()
    print("test_camera_vps done!!!")

test_camera_vps():
```

## close_cam

<font color='Blue'>【Function Description】</font>

Disable the MIPI camera.

<font color='Blue'>【Function Prototype】</font>  

```python
Camera.close_cam()
```

<font color='Blue'>【Parameter Description】</font>  

None

<font color='Blue'>【Usage】</font>cam = libsrcampy.Camera()

```python
#open MIPI camera, fps: 30, solution: 1080p
ret = cam.open_cam(0, 1, 30, 1920, 1080)
print("Camera open_cam return:%d" % ret)

#close MIPI camera
cam.close_cam()
```

<font color='Blue'>【Return】</font>  

None

<font color='Blue'>【Notes】</font>  

None

<font color='Blue'>【Reference Code】</font>  

None