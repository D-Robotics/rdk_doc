---
sidebar_position: 1
---

# Camera Object

The Camera object is used for image capture and processing functions for MIPI Cameras. It includes methods such as `open_cam`, `open_vps`, `get_frame`, `send_frame`, and `close`. Detailed descriptions are provided below:

## open_cam

<font color='Blue'>【Function Description】</font>

Opens the specified channel of the MIPI camera and sets the camera output frame rate and resolution format.

<font color='Blue'>【Function Declaration】</font>


```python
Camera.open_cam(video_index, [width, height])
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description                                      | Value Range         |
| -------------- | ------------------------------------------------ | ------------------- |
| video_index    | The host number corresponding to the camera. -1 indicates auto-detection. The number can be found in the /etc/board_config.json configuration file. | -1, 0, 1, 2, 3       |
| fps            | Camera output frame rate.                       | Depends on the camera model, default value is 30 |
| width          | The final output width of the camera image.      | Depends on the camera model, default value is 1920 (2560 for GC4663) |
| height         | The final output height of the camera image.     | Depends on the camera model, default value is 1080 (1440 for GC4663) |

<font color='Blue'>【Usage】</font>


```python
#create camera object
camera = libsrcampy.Camera()

#open MIPI Camera, fps: 30, solution: 1080p
ret = camera.open_cam(-1,  [1920, 1080])
```
<font color='Blue'>【Return Value】</font>

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font>

The resolution output supports a 2D `list` type input, which enables multiple different resolution outputs for the camera. The `list` supports a maximum of 4 downscaled resolutions and 1 upscaled resolution, with the scaling range being between `1/8~1.5` times the original camera resolution. Usage example is as follows:

```python
ret = cam.open_cam(0, -1, 30, [[1920, 1080], [1280, 720]])
```
<font color='Blue'>【Example Code】</font>  

None

## open_vps

<font color='Blue'>【Function Description】</font>

Enables the VPS (Video Processing) functionality for the specified camera channel. This supports image operations such as scaling, rotation, and cropping.

<font color='Blue'>【Function Declaration】</font>


```python
Camera.open_vps([src_width, src_height], [dst_width, dst_height], crop_rect, rotate)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description              | Range         |
| -------------- | ------------------------ | ------------- |
| src_width      | Input image width         | Depends on the camera output width |
| src_height     | Input image height        | Depends on the camera output height |
| dst_width      | Output image width        | 1/8 to 1.5 times the input width |
| dst_height     | Output image height       | 1/8 to 1.5 times the input height |
| crop_rect      | Crop area width and height, input format [x, y] | Should not exceed the input image dimensions |
| rotate         | Rotation angle, supports up to two channel rotations | Range 0-3, representing `No rotation`, `90 degrees`, `180 degrees`, `270 degrees` |

<font color='Blue'>【Usage】</font>


```python
#create camera object
camera = libsrcampy.Camera()

#enable vps function
ret = camera.open_vps([1920, 1080],[ 512, 512])
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Description     |
| ------------ | --------------- |
| 0            | Success         |
| -1           | Failure         |

<font color='Blue'>【Notes】</font> 
- Image cropping is performed starting from the top-left corner of the image, based on the configured dimensions.
- Cropping is done before scaling or rotation. Multiple channels are configured by passing a `list` of parameters.

<font color='Blue'>【Reference Code】</font>  

None

## get_frame

<font color='Blue'>【Function Description】</font>

Retrieve the image output of the camera object. This function should be called after `open_cam` and `open_vps`.

<font color='Blue'>【Function Declaration】</font>


```python
Camera.get_frame(module, [width, height])
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                         | Range                      |
| -------------- | ----------------------------------- | -------------------------- |
| module         | The module from which to get the image | Default is 2               |
| width          | The width of the image to retrieve    | Output width set by `open_cam` or `open_vps` |
| height         | The height of the image to retrieve   | Output height set by `open_cam` or `open_vps` |

<font color='Blue'>【Usage】</font>


```python
cam = libsrcampy.Camera()

#create camera object
camera = libsrcampy.Camera()

#enable vps function
ret = camera.open_vps([1920, 1080],[ 512, 512])

#get one image from camera
img = cam.get_frame(2,[512,512])
```

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------- | ----------- |
| 0             | Success     |
| -1            | Failure     |

<font color='Blue'>【Notes】</font>  

This method must be called after `open_cam` and `open_vps`.

<font color='Blue'>【Reference Code】</font>  


```python
import sys, os, time

from hobot_spdev import libsppydev as srcampy

def test_camera():
    cam = srcampy.Camera()

    #open MIPI camera, fps: 30, solution: 1080p
    ret = cam.open_cam(-1, [1920, 1080])
    print("Camera open_cam return:%d" % ret)

    # wait for 1s
    time.sleep(1)

    #get one image from camera   
    img = cam.get_frame(2,1920, 1080)
    if img is not None:
        #save file
        fo = open("output.img", "wb")
        fo.write(img)
        fo.close()
        print("camera save img file success")
    else:
        print("camera save img file failed")
    
    #close MIPI camera
    cam.close()
    print("test_camera done!!!")

test_camera()
```

## send_frame
<font color='Blue'>【Function Description】</font>

Input an image to the `vps` module and trigger the image processing operation.

<font color='Blue'>【Function Declaration】</font>


```python
Camera.send_frame(img)
```

<font color='Blue'>【Parameter Description】</font>

| Parameter Name | Description                | Range               |
| -------------- | -------------------------- | ------------------- |
| img            | Image data to be processed | Should match vps input size |

<font color='Blue'>【Usage】</font>


```python
camera = libsrcampy.Camera()

#enable vps function, input: 1080p, output: 512x512
ret = camera.open_vps( [1920, 1080], [512, 512])
print("Camera vps return:%d" % ret)

fin = open("output.img", "rb")
img = fin.read()
fin.close()

#send image to vps module
ret = vps.send_frame(img)
```
<font color='Blue'>【Return Value】</font>  

| Return Value | Description |  
| ------------ | ----------- |  
| 0            | Success     |  
| -1           | Failure     |

<font color='Blue'>【Notes】</font>  

This interface needs to be called after `open_vps`.

<font color='Blue'>【Reference Code】</font>  


```python
import sys, os, time

import numpy as np
import cv2
from hobot_spdev import libsppydev as srcampy

def test_camera_vps():
    vps = srcampy.Camera()

    #enable vps function, input: 1080p, output: 512x512
    ret = vps.open_vps( [1920, 1080], [512, 512])
    print("Camera vps return:%d" % ret)

    fin = open("output.img", "rb")
    img = fin.read()
    fin.close()

    #send image data to vps
    ret = vps.send_frame(img)
    print ("Process send_frame return:%d" % ret)

    fo = open("output_vps.img", "wb+")

    #get image data from vps
    img = vps.get_frame()
    if img is not None:
        fo.write(img)
        print("encode write image success")
    else:
        print("encode write image failed")
    fo.close()

    #close vps function
    vps.close()
    print("test_camera_vps done!!!")

test_camera_vps():
```

## close
<font color='Blue'>【Function Description】</font>

Close the enabled MIPI camera.

<font color='Blue'>【Function Declaration】</font>


```python
Camera.close()
```

<font color='Blue'>【Parameter Description】</font>

None

<font color='Blue'>【Usage】</font>

```python
cam = libsrcampy.Camera()

#open MIPI camera, fps: 30, solution: 1080p
ret = cam.open_cam(-1,[1920, 1080])
print("Camera open_cam return:%d" % ret)

#close MIPI camera
cam.close()
```
<font color='Blue'>【Return Value】</font>

None

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Reference Code】</font>

None
