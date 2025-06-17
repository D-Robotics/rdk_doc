---
sidebar_position: 2
---
# Encoder Object

The Encoder object implements encoding and compression functions for video data. It includes several methods, such as `encode`, `encode_file`, `get_img`, `close`, etc. The detailed explanation is as follows:

## encode

<font color='Blue'>【Function Description】</font>

Configure and enable the encode encoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Encoder.encode(video_chn, encode_type , width, height, bits)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description | Value Range |
| --------- | --------------- | ------------------- |
| video_chn | Specifies the channel number of the video encoder | Range: 0~31 |
| encode_type    | Video encoding type  | Range: 1~3, corresponding to `H264`, `H265`, `MJPEG` respectively |
| width     | Width of the image input to the encoding module     | Not exceeding 4096              |
| height    | Height of the image input to the encoding module      | Not exceeding 4096              |
| bits      | Bitrate of the encoding module         | Default: 8000kbps         |

<font color='Blue'>【Usage】</font>

```python
#create encode object
encode = libsrcampy.Encoder()

#enable encode channel 0, solution: 1080p, format: H264
ret = encode.encode(0, 1, 1920, 1080)
```

<font color='Blue'>【Return Value】</font>  

| Return value | Definition |                 
| ------ | ----- |
| 0      | Success  |
| -1    | Failure   |

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Reference Code】</font>

None

## encode_file

<font color='Blue'>【Function Description】</font>

Feeds image files into enabled encoding channels and encodes them according to a predetermined format.

<font color='Blue'>【Function Declaration】</font> 

```python
Encoder.encode_file(img)
```

<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                | Value Range                     |
| -------------- | --------------------------- | ---------------------------- |
| img            | Image data to be encoded, in NV12 format | N/A                          |

<font color='Blue'>【Usage Example】</font> 

```python
fin = open("output.img", "rb")
input_img = fin.read()
fin.close()

# Input image data for encoding
ret = encode.encode_file(input_img)
```

<font color='Blue'>【Return Values】</font>  

| Return Value | Definition Description |
| ------------ | --------------------- |
| 0            | Success               |
| -1           | Failure               |

<font color='Blue'>【Notes】</font> 

None

<font color='Blue'>【Reference Code】</font>  

None

## get_img



<font color='blue'>【Function Description】</font>

Get encoded data.

<font color='blue'>【Function Declaration】</font>  

```python
Encoder.get_img()
```

<font color='blue'>【Usage】</font> 

N/A

<font color='blue'>【Parameter Description】</font>  

N/A

<font color='blue'>【Return Value】</font>  

| Return Value | Definition |                 
| ------ | ----- |
| 0      | Success  |
| -1    | Failure   |

<font color='blue'>【Attention】</font> 

This interface should be used after calling `Encoder.encode()` to create the encoding channel.

<font color='blue'>【Code Sample】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_encode():
    #create encode object
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #save encoded data to file
    fo = open("encode.h264", "wb+")
    a = 0
    fin = open("output.img", "rb")
    input_img = fin.read()
    fin.close()
    while a < 100:
        #send image data to encoder
        ret = enc.encode_file(input_img)
        print("Encoder encode_file return:%d" % ret)
        #get encoded data
        img = enc.get_img()
        if img is not None:
            fo.write(img)
            print("encode write image success count: %d" % a)
        else:
            print("encode write image failed count: %d" % a)
        a = a + 1

enc.close()
print("test_encode done!!!")

test_encode()

```

## close

<font color='Blue'>【Description】</font>

Close the enabled encoding channel.

<font color='Blue'>【Function】</font>  

```python
Encoder.close()
```

<font color='Blue'>【Parameters】</font>  

None

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return】</font>  

| Return Value | Definition |
| ------ | ----- |
| 0      | Success  |
| -1    | Failed   |

<font color='Blue'>【Note】</font> 

This interface should be used after calling `Encoder.encode()` to create the encoding channel.
<font color='Blue'>【Reference Code】</font>

None