---
sidebar_position: 3
---

# Decoder Object

The Decoder object implements the video data decoding functionality and includes methods such as `decode`, `send_frame`, `get_frame`, `close`, etc. The detailed descriptions are as follows:

## decode

<font color='Blue'>【Function Description】</font>

Enables the decode module and decodes the video file.

<font color='Blue'>【Function Declaration】</font>


```python
Decoder.decode(decode_type, [width, height], file)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                | Value Range                 |
| -------------- | -------------------------- | --------------------------- |
| file           | The name of the file to decode | No specific range            |
| decode_type    | Video decode type            | Range 2-3, corresponding to `H265` and `MJPEG` |
| width          | Input image width for the decode module | Maximum 4096 |
| height         | Input image height for the decode module | Maximum 4096 |

<font color='Blue'>【Usage】</font>  


```python
#create decode object
decode = libsrcampy.Decoder()

#enable decode channel 0, solution: 1080p, format: h265
ret = dec.decode(2,[ 1920, 1080],"encode.h265")
```

<font color='Blue'>【Return Value】</font>  

The return value is a `list` data with 2 members.

| Return Value          | Description                  |
| --------------------- | ---------------------------- |
| list[0]               | 0: Decoding succeeded, -1: Decoding failed |
| list[1]               | The number of frames in the input stream file, valid when decoding is successful |

<font color='Blue'>【Note】</font>  

None

<font color='Blue'>【Reference Code】</font>  

None

## get_img

<font color='Blue'>【Function Description】</font>

Gets the output result of the decoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Decoder.get_img()
```
<font color='Blue'>【Parameter Description】</font>

None

<font color='Blue'>【Usage】</font>

```python
ret = dec.decode(2,[ 1920, 1080],"encode.h265")
print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

img = dec.get_img()
```
<font color='Blue'>【Return Value】</font>

| Return Value | Description     |
| ------------ | --------------- |
| -1           | Decoding data   |

<font color='Blue'>【Note】</font>

This interface needs to be used after calling `Decoder.decode()` to create a decoding channel.

<font color='Blue'>【Reference Code】</font>


```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_decode():
    #create decode object
    dec = libsrcampy.Decoder()

    #enable decode function
    #decode input: encode.h265, solution: 1080p, format: h265
    ret = dec.decode(2,[ 1920, 1080],"encode.h265")
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))
    
    #get decoder output
    img = dec.get_img()
    if img is not None:
        #save file
        fo = open("output.img", "wb")
        fo.write(img)
        fo.close()
        print("decode save img file success")
    else:
        print("decode save img file failed")

    dec.close()
    print("test_decode done!!!")

test_decode()
```

## set_img
<font color='Blue'>【Function Description】</font>

This function inputs a single frame of encoded data into the decoding module and performs the decoding.

<font color='Blue'>【Function Declaration】</font>  

```python
Decoder.set_img(img)
```
<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description             | Value Range  |
| -------------- | ----------------------- | ------------ |
| img            | The single frame data to be decoded | None         |
| chn            | Decoder channel number   | Range 0~31   |
| eos            | Whether the decoding data has ended | 0: Not ended, 1: Ended |

<font color='Blue'>【Usage】</font>  

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Note】</font>  

This interface needs to be used after calling `Decoder.decode()` to create a decoding channel, with the `file` parameter set to null when creating the channel.

<font color='Blue'>【Reference Code】</font>


```python
import sys, os, time

import numpy as np
import cv2
from hobot_spdev import libsppydev as srcampy

def test_cam_bind_encode_decode_bind_display():
    #camera start
    cam = srcampy.Camera()
    ret = cam.open_cam(-1, [[1920, 1080], [1280, 720]])
    print("Camera open_cam return:%d" % ret)

    #enable encoder
    enc = srcampy.Encoder()
    ret = enc.encode(2, [1920, 1080])
    print("Encoder encode return:%d" % ret)

    #enable decoder
    dec = srcampy.Decoder()
    ret = dec.decode(2,[ 1920, 1080],"")
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    ret = srcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while a < 100:
        #get encode image from encoder
        img = enc.get_frame()
        if img is not None:
            #send encode image to decoder
            dec.set_frame(img)
            print("encode get image success count: %d" % a)
        else:
            print("encode get image failed count: %d" % a)
        a = a + 1

    ret = srcampy.unbind(cam, enc)
    dec.close()
    enc.close()
    cam.close()
    print("test_cam_bind_encode_decode_bind_display done!!!")

test_cam_bind_encode_decode_bind_display()
```
## close

<font color='Blue'>【Function Description】</font>

Closes the decoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Decoder.close()
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

The `close` interface must be called when exiting the program to release resources.

<font color='Blue'>【Reference Code】</font>

None
