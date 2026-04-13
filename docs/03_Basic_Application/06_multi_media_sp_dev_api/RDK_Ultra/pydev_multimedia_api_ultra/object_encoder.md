---
sidebar_position: 2
---

# Encoder对象

Encoder对象实现了对视频数据的编码压缩功能，包含了`encode`、`send_frame`、`get_frame`、`close`等几种方法，详细说明如下：

## encode

<font color='Blue'>【功能描述】</font>

配置并使能encode编码模块

<font color='Blue'>【函数声明】</font>

```python
Encoder.encode(encode_type , [width, height], bits)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称  | 描述           | 取值范围                    |
| --------- | --------------- | ------------------- |
| encode_type    | 视频编码类型  | 范围2~3，分别对应`H265`、`MJPEG` |
| width     | 输入编码模块的图像宽度      | 不超过4096              |
| height    | 输入编码模块的图像高度      | 不超过4096              |
| bits      | 编码模块的比特率         |    默认8000kbps         |

<font color='Blue'>【使用方法】</font>

```python
#create encode object
encode = libsrcampy.Encoder()

#enable encode channel 0, solution: 1080p, format: H265
ret = encode.encode(2, [1920, 1080])
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败   |

<font color='Blue'>【注意事项】</font>

无

<font color='Blue'>【参考代码】</font>

无

## send_frame

<font color='Blue'>【功能描述】</font>

向使能的编码通道输入图像文件，按预定格式进行编码

<font color='Blue'>【函数声明】</font> 

```python
Encoder.send_frame(img)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称 | 描述              | 取值范围                     |
| -------- | ----------------- | --------------------- |
| img      | 需要编码的图像数据，需要使用NV12格式 | 无 |

<font color='Blue'>【使用方法】</font> 

```python
fin = open("output.img", "rb")
input_img = fin.read()
fin.close()

#input image data to encode
ret = encode.send_frame(input_img)
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败   |

<font color='Blue'>【注意事项】</font> 

无

<font color='Blue'>【参考代码】</font>  

无

## get_frame

<font color='Blue'>【功能描述】</font>

获取编码后的数据

<font color='Blue'>【函数声明】</font>  

```python
Encoder.get_frame()
```

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【参数描述】</font>  

无

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败   |

<font color='Blue'>【注意事项】</font> 

该接口需要在调用`Encoder.encode()`创建编码通道后使用

<font color='Blue'>【参考代码】</font>  

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

关闭使能的编码通道。

<font color='Blue'>【函数声明】</font>  

```python
Encoder.close()
```

<font color='Blue'>【参数描述】</font>  

无

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败   |

<font color='Blue'>【注意事项】</font> 

该接口需要在调用`Encoder.encode()`创建编码通道后使用

<font color='Blue'>【参考代码】</font>  

无
