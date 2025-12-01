---
sidebar_position: 3
---

# Decoder Object

The Decoder object implements video data decoding functionality and includes several methods such as `decode`, `set_img`, `get_img`, and `close`. Detailed descriptions are provided below:

## decode

<font color='Blue'>【Function Description】</font>

Enables the decode module and decodes the video file.

<font color='Blue'>【Function Declaration】</font>  

```python
Decoder.decode(file, video_chn, decode_type, width, height)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                              | Value Range                     |
| -------------- | ---------------------------------------- | ------------------------------- |
| file           | Filename of the file to be decoded       | None                            |
| video_chn      | Specifies the channel number of the video decoder | Range: 0–31                    |
| decode_type    | Video decoding type                      | Range: 1–3, corresponding to `H264`, `H265`, and `MJPEG` respectively |
| width          | Width of the input image to the decoding module | Up to 4096                     |
| height         | Height of the input image to the decoding module | Up to 4096                     |

<font color='Blue'>【Usage】</font> 

```python
#create decode object
decode = libsrcampy.Decoder()

#enable decode channel 0, resolution: 1080p, format: H264
ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
```

<font color='Blue'>【Return Value】</font>  

Returns a `list` containing two elements:

| Return Value   | Description                              |
| -------------- | ---------------------------------------- |
| list[0]        | 0: Decoding succeeded; -1: Decoding failed |
| list[1]        | Number of frames in the input bitstream file (valid only when decoding succeeds) |

<font color='Blue'>【Notes】</font> 

None

<font color='Blue'>【Reference Code】</font>  

None

## get_img

<font color='Blue'>【Function Description】</font>

Retrieves the output result from the decoding module.

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

| Return Value | Description     |
| ------------ | --------------- |
| -1           | Decoded data    |

<font color='Blue'>【Notes】</font>

This interface must be used after calling `Decoder.decode()` to create a decoding channel.

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
    #decode input: encode.h264, resolution: 1080p, format: h264
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

Feeds a single encoded frame into the decoding module for decoding.

<font color='Blue'>【Function Declaration】</font>  

```python
Decoder.set_img(img, chn, eos)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                              | Value Range |
| -------------- | ---------------------------------------- | ----------- |
| img            | Single-frame data to be decoded          | None        |
| chn            | Decoder channel number                   | Range: 0–31 |
| eos            | Indicates whether decoding data has ended | 0: Not ended; 1: Ended |

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font> 

This interface must be used after calling `Decoder.decode()` to create a decoding channel, and when creating the decoding channel, the `file` parameter must be left empty.

<font color='Blue'>【Reference Code】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_cam_bind_encode_decode_bind_display():
    #camera start
    cam = libsrcampy.Camera()
    # If you know the pipe_id and video_index, you can specify the first two arguments.
    # ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])

    # If you do not know the pipe_id and video_index, you can use the following
    # code to detect them, and it will default to using the first detected camera.
    ret = cam.open_cam(0, -1, 30, [1920, 1280], [1080, 720])
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
```dec.set_img(img)
            print("encode get image success count: %d" % a)
        else:
            print("encode get image failed count: %d" % a)
        a = a + 1

    ret = libsrcampy.unbind(cam, enc)
    dec.close()
    enc.close()
    cam.close_cam()
    print("test_cam_bind_encode_decode_bind_display done!!!")

test_cam_bind_encode_decode_bind_display()
```

## close

<font color='Blue'>【Function Description】</font>

Close the decoder module.

<font color='Blue'>【Function Declaration】</font>
```python
Decoder.close()
```

<font color='Blue'>【Parameters】</font>

None

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Value】</font>

| Return Value | Description        |
| ------------ | ------------------ |
| 0            | Success            |
| -1           | Failure            |

<font color='Blue'>【Notes】</font>

The `close` interface must be called upon program exit to release resources.

<font color='Blue'>【Example Code】</font>

None