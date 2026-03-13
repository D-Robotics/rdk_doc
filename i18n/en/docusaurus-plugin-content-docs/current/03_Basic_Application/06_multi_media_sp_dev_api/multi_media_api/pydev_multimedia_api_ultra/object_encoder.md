---
sidebar_position: 2
---

## Encoder Object

<font color='Blue'>【Function Description】</font>

Configures and enables the encoding module.

<font color='Blue'>【Function Declaration】</font>

```python
Encoder.encode(encode_type , [width, height], bits)

```
<font color='Blue'>【Parameter Description】</font>  

| Parameter Name  | Description           | Range                     |
| ----------------| --------------------- | ------------------------- |
| encode_type    | Video encoding type   | Range 2-3, corresponding to `H265`, `MJPEG` |
| width           | Image width for the encoder module | Maximum 4096             |
| height          | Image height for the encoder module | Maximum 4096             |
| bits            | Bitrate for the encoding module | Default 8000kbps          |

<font color='Blue'>【Usage】</font>


```python
#create encode object
encode = libsrcampy.Encoder()

#enable encode channel 0, solution: 1080p, format: H265
ret = encode.encode(2, [1920, 1080])
```
<font color='Blue'>【Return Value】</font>  

| Return Value | Description |                 
| ------------- | ----------- |
| 0             | Success     |
| -1            | Failure     |

<font color='Blue'>【Notes】</font>

None

<font color='Blue'>【Reference Code】</font>

None

## send_frame

<font color='Blue'>【Function Description】</font>

Input an image file to the enabled encoding channel for encoding in the specified format.

<font color='Blue'>【Function Declaration】</font>


```python
Encoder.send_frame(img)
```
<font color='Blue'>【Parameter Description】</font>  

| Parameter Name | Description                                | Value Range           |
| -------------- | ------------------------------------------ | --------------------- |
| img            | Image data to be encoded, must use NV12 format | None                 |

<font color='Blue'>【Usage】</font>  


```python
fin = open("output.img", "rb")
input_img = fin.read()
fin.close()

#input image data to encode
ret = encode.send_frame(input_img)
```
<font color='Blue'>【Return Value】</font>  

| Return Value | Description  |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font>  

None

<font color='Blue'>【Reference Code】</font>  

None

## get_frame

<font color='Blue'>【Function Description】</font>  

Retrieve the encoded data

<font color='Blue'>【Function Declaration】</font>  


```python
Encoder.get_frame()
```
<font color='Blue'>【Usage】</font>  

None

<font color='Blue'>【Parameter Description】</font>  

None

<font color='Blue'>【Return Value】</font>  

| Return Value | Description  |
| ------------ | ----------- |
| 0            | Success     |
| -1           | Failure     |

<font color='Blue'>【Notes】</font>  

This interface should be used after calling `Encoder.encode()` to create an encoding channel.

<font color='Blue'>【Reference Code】</font>  



```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_encode():
    #create encode object
    enc = libsrcampy.Encoder()
    ret = enc.encode(2, [1920, 1080])
    print("Encoder encode return:%d" % ret)

    #save encoded data to file
    fo = open("encode.h264", "wb+")
    a = 0
    fin = open("output.img", "rb")
    input_img = fin.read()
    fin.close()
    while a < 100:
        #send image data to encoder
        ret = enc.send_frame(input_img)
        print("Encoder send_frame return:%d" % ret)
        #get encoded data
        img = enc.get_frame()
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

<font color='Blue'>【功能描述】</font>

Closes the enabled encoding channel.

<font color='Blue'>【函数声明】</font>


```python
Encoder.close()
```

<font color='Blue'>【Parameter Description】</font>  

None

<font color='Blue'>【Usage】</font> 

None

<font color='Blue'>【Return Values】</font>  

| Return Value | Description                |
|--------------|----------------------------|
| 0            | Success                    |
| -1           | Failure                    |

<font color='Blue'>【Notes】</font>  

This interface should be used after the `Encoder.encode()` function has been called to create the encoding channel.

<font color='Blue'>【Reference Code】</font>  

None
