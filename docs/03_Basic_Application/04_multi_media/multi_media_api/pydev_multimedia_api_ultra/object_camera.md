---
sidebar_position: 1
---

# Camera对象

Camera对象用于完成MIPI Camera的图像采集和处理功能，包含了`open_cam`、`open_vps`、`get_frame`、`send_frame`、`close`等几种方法，详细说明如下：
## open_cam

<font color='Blue'>【功能描述】</font>  

打开指定通道的MIPI摄像头，并设置摄像头输出帧率、分辨率格式。

<font color='Blue'>【函数声明】</font>  

```python
Camera.open_cam(video_index, [width, height])
```

<font color='Blue'>【参数描述】</font>  

| 参数名称      | 定义描述                  | 取值范围    |
| ----------- | ------------------------ | --------  |
| video_index | camera对应的host编号，-1表示自动探测，编号可以查看 /etc/board_config.json 配置文件 | 取值 -1, 0 , 1, 2,3 |
| fps         | camera图像输出帧率          | 依据camera型号而定，默认值30   |
| width       | camera最终图像输出宽度    |  视camera型号而定，默认值1920（GC4663为2560）   |
| height      | camera最终图像输出高度  |    视camera型号而定，默认值1080（GC4663为1440） |

<font color='Blue'>【使用方法】</font> 

```python
#create camera object
camera = libsrcampy.Camera()

#open MIPI Camera, fps: 30, solution: 1080p
ret = camera.open_cam(-1,  [1920, 1080])
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

分辨率输出部分支持二维`list`类型输入，表示使能camera多组不同分辨率输出。`list`最多支持4组缩小，1组放大，缩放区间为camera原始分辨率的`1/8~1.5`倍之间。使用方式如下：

```python
ret = cam.open_cam(0, -1, 30, [[1920, 1080], [1280, 720]])
```

<font color='Blue'>【参考代码】</font>  

无

## open_vps

<font color='Blue'>【功能描述】</font>

使能指定camera通道的vps(video process)图像处理功能，支持对输入图像完成缩放、旋转、裁剪等功能。

<font color='Blue'>【函数声明】</font>  

```python
Camera.open_vps([src_width, src_height], [dst_width, dst_height], crop_rect, rotate)
```

<font color='Blue'>【参数描述】</font>  


| 参数名称      | 定义描述                  | 取值范围    |
| ----------- | ------------------------ | --------  |
| src_width  | 图像输入宽度                 | 视camera输出宽度而定 |
| src_height | 图像输入高度                 | 视camera输出高度而定 |
| dst_width  | 图像输出宽度 | 输入宽度的`1/8~1.5`倍 |
| dst_height | 图像输出高度 | 输入高度的`1/8~1.5`倍 |
| crop_rect  | 裁剪区域的宽高，输入格式[x, y] | 不超过输入图像尺寸 |
| rotate     | 旋转角度，最多支持两个通道旋转 | 范围0~3，分别表示`不旋转`、`90度` `180度`、`270度` |


<font color='Blue'>【使用方法】</font> 

```python
#create camera object
camera = libsrcampy.Camera()

#enable vps function
ret = camera.open_vps([1920, 1080],[ 512, 512])
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 
- 图像裁剪功能以图像左上角为原点，按照配置尺寸进行裁剪
- 图像裁剪会在缩放、旋转操作之前进行，多通道配置通过输入参数`list`传递。  

 
 


<font color='Blue'>【参考代码】</font>  

无

## get_frame

<font color='Blue'>【功能描述】</font>

获取camera对象的图像输出，需要在`open_cam`、`open_vps`之后调用

<font color='Blue'>【函数声明】</font> 

```python
Camera.get_frame(module, [width, height])
```

<font color='Blue'>【参数描述】</font>  

| 参数名称 | 定义描述                 | 取值范围     |
| -------- | ------- | ----------- |
| module   | 需要获取图像的模块 | 默认为2 |
| width    | 需要获取图像的宽度 | `open_cam`、`open_vps`设置的输出宽度 |
| height   | 需要获取图像的高度 | `open_cam`、`open_vps`设置的输出高度 |


<font color='Blue'>【使用方法】</font> 

```python
cam = libsrcampy.Camera()

#create camera object
camera = libsrcampy.Camera()

#enable vps function
ret = camera.open_vps([1920, 1080],[ 512, 512])

#get one image from camera
img = cam.get_frame(2,[512,512])
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败   |

<font color='Blue'>【注意事项】</font> 

该方法需要在`open_cam`、`open_vps`之后调用  

<font color='Blue'>【参考代码】</font>  

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

<font color='Blue'>【功能描述】</font>

向`vps`模块输入图像，并触发图像处理操作

<font color='Blue'>【函数声明】</font>  

```python
Camera.send_frame(img)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称 | 定义描述     | 取值范围      |
| -------- | -------------------- | ----- |
| img      | 需要处理的图像数据 | 跟vps输入尺寸保持一致 |

<font color='Blue'>【使用方法】</font> 

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

<font color='Blue'>【返回值】</font>  

| 返回值 | 定义描述 |                 
| ------ | ----- |
| 0      | 成功  |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

该接口需要在`open_vps`之后调用

<font color='Blue'>【参考代码】</font>  

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

<font color='Blue'>【功能描述】</font>

关闭使能的MIPI camera摄像头

<font color='Blue'>【函数声明】</font>  

```python
Camera.close()
```

<font color='Blue'>【参数描述】</font>  

无

<font color='Blue'>【使用方法】</font> 

```python
cam = libsrcampy.Camera()

#open MIPI camera, fps: 30, solution: 1080p
ret = cam.open_cam(-1,[1920, 1080])
print("Camera open_cam return:%d" % ret)

#close MIPI camera
cam.close()
```

<font color='Blue'>【返回值】</font>  

无

<font color='Blue'>【注意事项】</font> 

无

<font color='Blue'>【参考代码】</font>  

无
