---
sidebar_position: 3
---
# Decoder Object

The Decoder object implements the decoding function for video data and includes several methods such as `decode`, `set_img`, `get_img`, `close`, etc. The detailed description is as follows:

## decode

<font color='Blue'>[Description]</font>

Enables the decode decoding module and decodes the video file.

<font color='Blue'>[Function Declaration]</font>  

```python
Decoder.decode(file, video_chn, decode_type, width, height)
```

<font color='Blue'>[Parameter Description]</font>  

| Parameter   | Description                | Value Range                |
| ----------- | -------------------------- | -------------------------- |
| file        | File name to be decoded     | N/A                        |
| video_chn   | Video decoder channel number   | Range: 0~31                |
| decode_type | Video decoding type        | Range: 1~3, corresponding to `H264`, `H265`, `MJPEG` respectively |
| width       | Image width for input decoding module   | Not exceeding 4096        |
| height      | Image height for input decoding module  | Not exceeding 4096        |

<font color='Blue'>[Usage]</font> 

```python
#create decode object
decode = libsrcampy.Decoder()

#enable decode channel 0, solution: 1080p, format: H264
ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
```

<font color='Blue'>[Return Value]</font>  

The return value is a `list` data with 2 members.

| Return Value          | Definition                            |
| --------------------- | ------------------------------------- |
| list[0]               | 0: decoding succeeded, -1: decoding failed     | 
| list[1]               | Number of frames in the input stream file, valid when decoding succeeds     |

<font color='Blue'>【Note】</font>

None

<font color='Blue'>【Reference Code】</font>

None

## get_img

<font color='Blue'>【Function Description】</font>

Get the output result of the decoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Decoder.get_img()
```

<font color='Blue'>【Parameter Description】</font>

None

<font color='Blue'>【Usage】</font>

```python
ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

img = dec.get_img()
```

<font color='Blue'>【Return Value】</font>

| Return Value | Definition |
| ------ | ----- |
| -1      | Decoded data  |

<font color='Blue'>【Notes】</font>

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
    #decode input: encode.h264, solution: 1080p, format: h264
    ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
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

Send a single frame of encoded data to the decoding module for decoding.

<font color='Blue'>【Function Declaration】</font>  

```python
Decoder.set_img(img, chn, eos)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter | Definition                        | Value Range |
| --------- | --------------------------------- | ----------- | 
| img       | Single frame of data to be decoded | N/A         |
| chn       | Channel number of the decoder     | Range: 0~31 |
| eos       | Whether the decoding data is complete | 0: not complete, 1: complete |

<font color='Blue'>【Usage】</font> 

N/A

<font color='Blue'>【Return Values】</font>  

| Return Value | Description |
| ------ | ---- |
| 0      | Success |
| -1    | Failure |

<font color='Blue'>【Notes】</font> 

This interface needs to be used after calling `Decoder.decode()` to create a decoding channel, and the input parameter `file` should be set to empty when creating the decoding channel.

<font color='Blue'>【Reference Code】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_cam_bind_encode_decode_bind_display():
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #enable encoder
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #enable decoder
    dec = libsrcampy.Decoder()
    ret = dec.decode("", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    ret = libsrcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while a < 100:
        #get encode image from encoder
        img = enc.get_img()
        if img is not None:
            #send encode image to decoder
            dec.set_img(img)
            print("encode get image success count: %d" % a)
        else:
            print("encode get image failed count: %d" % a)
        a = a + 1
    ret = libsrcampy.unbind(cam, enc)
    dec.close()
    enc.close()
    cam.close_cam()
    print("test_cam_bind_encode_decode_bind_display done!!!")

    test_cam_bind_encode_decode()
```

## close

<font color='Blue'>【Function Description】</font>

Close the decoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Decoder.close()
```

<font color='Blue'>【Parameter Description】</font>

None

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font>

| Return Value | Definition |
| ------ | ---- |
| 0      | Success |
| -1    | Failed |

<font color='Blue'>【Notes】</font>

It is necessary to call the `close` interface to release resources when exiting the program.

<font color='Blue'>【Reference Code】</font>

None