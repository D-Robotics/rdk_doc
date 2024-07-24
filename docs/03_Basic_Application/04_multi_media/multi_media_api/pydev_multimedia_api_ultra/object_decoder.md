---
sidebar_position: 3
---

# Decoder对象

Decoder对象实现了对视频数据的解码功能，包含了`decode`、`send_frame`、`get_frame`、`close`等几种方法，详细说明如下：

## decode

<font color='Blue'>【功能描述】</font>

使能decode解码模块，并对视频文件进行解码

<font color='Blue'>【函数声明】</font>  

```python
Decoder.decode(decode_type, [width, height], file)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称  | 描述           | 取值范围                    |
| --------- | --------------- | ------------------- |
| file      | 需要解码的文件名     |       无       |
| decode_type | 视频解码类型  | 范围2~3，分别对应、`H265`、`MJPEG` |
| width     | 输入解码模块的图像宽度      | 不超过4096              |
| height    | 输入解码模块的图像高度      | 不超过4096              |

<font color='Blue'>【使用方法】</font> 

```python
#create decode object
decode = libsrcampy.Decoder()

#enable decode channel 0, solution: 1080p, format: h265
ret = dec.decode(2,[ 1920, 1080],"encode.h265")
```

 <font color='Blue'>【返回值】</font>  

返回值为2个成员的`list`数据

| 返回值                | 定义描述      |
| ---------------- | ----------- |
| list[0] | 0：解码成功，-1：解码失败      | 
| list[1] | 输入码流文件的帧数，解码成功时有效     |

<font color='Blue'>【注意事项】</font> 

无

<font color='Blue'>【参考代码】</font>  

无

## get_img

<font color='Blue'>【功能描述】</font>

获取解码模块的输出结果

<font color='Blue'>【函数声明】</font>
```python
Decoder.get_img()
```

<font color='Blue'>【参数描述】</font>

无

<font color='Blue'>【使用方法】</font>

```python
ret = dec.decode(2,[ 1920, 1080],"encode.h265")
print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

img = dec.get_img()
```

<font color='Blue'>【返回值】</font>

| 返回值 | 定义描述 |
| ------ | ----- |
| -1      | 解码数据  |

<font color='Blue'>【注意事项】</font>

该接口需要在调用`Decoder.decode()`创建解码通道后使用

<font color='Blue'>【参考代码】</font>

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

<font color='Blue'>【功能描述】</font>

将单帧编码数据送入解码模块，并进行解码

<font color='Blue'>【函数声明】</font>  

```python
Decoder.set_img(img)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称 | 定义描述         | 取值范围 |
| -------- | ------------- | --- | 
| img      | 需要解码的单帧数据 | 无 |
| chn      | 解码器通道号      | 范围0~31 |
| eos      | 解码数据是否结束   | 0：未结束，1：结束 |

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

该接口需要在调用`Decoder.decode()`创建解码通道后使用，且解码通道创建时入参`file`置空

<font color='Blue'>【参考代码】</font>  

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

    test_cam_bind_encode_decode()
```

## close

<font color='Blue'>【功能描述】</font>

关闭解码模块

<font color='Blue'>【函数声明】</font>
```python
Decoder.close()
```

<font color='Blue'>【参数描述】</font>

无

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【返回值】</font>

| 返回值 | 定义描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font>

退出程序时需要调用`close`接口以释放资源。

<font color='Blue'>【参考代码】</font>

无
