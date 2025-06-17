---
sidebar_position: 4
---

# Display对象

Display对象实现了视频显示功能，可以将图像数据通过`HDMI`接口输出到显示器，该对象包含`display`、`set_img`、`set_graph_rect`、`set_graph_word`、`close`等方法，详细说明如下：

## display
<font color='Blue'>【功能描述】</font>

显示模块初始化，并配置显示参数

<font color='Blue'>【函数声明】</font>  

```python
Display.display(chn, width, height, vot_intf, vot_out_mode)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称     | 定义描述                  | 取值范围      |
| ------------ | ----------------------- | ----------------- |
| chn          | 显示输出层        | 0: 视频层，2: 图形层  |
| width        | 输入图像的宽度       | 不超过1920 |
| height       | 输入图像的高度       | 不超过1080 |
| vot_intf     | 视频接口输出分辨率 | 默认为0，1080p |
| vot_out_mode | 视频输出接口     | 默认为1，HDMI输出 |

<font color='Blue'>【使用方法】</font> 

```python
#create display object
disp = libsrcampy.Display()

#enable display function, solution: 1080p, interface: HDMI
ret = disp.display(0, 1920, 1080, 0, 1)
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

开发板HDMI接口分辨率基于显示器EDID获取，目前只支持`1920x1080`、`1280x720`、`1024x600`、`800x480`几种分辨率。使能显示模块时，需要注意配置分辨率跟显示器实际分辨率相匹配。

<font color='Blue'>【参考代码】</font>  

无

## set_img

<font color='Blue'>【功能描述】</font>

向display模块输入显示数据，格式需要为`NV12`

<font color='Blue'>【函数声明】</font>  

```python
Display.set_img(img)
```

<font color='Blue'>【参数描述】</font>  

| 参数名称     | 定义描述                  | 取值范围      |
| ------------ | ----------------------- | ----------------- |
| img          | 需要显示的图像数据        | NV12格式  |

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

该接口需要在使用`display`接口使能显示功能后使用，送入数据需要为`NV12`格式

<font color='Blue'>【参考代码】</font>  

```python
import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def test_display():
    #create display object
    disp = libsrcampy.Display()

    #enable display function
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    fo = open("output.img", "rb")
    img = fo.read()
    fo.close()

    #send image data to display
    ret = disp.set_img(img)
    print ("Display set_img return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

test_display()
```

## set_graph_rect

<font color='Blue'>【功能描述】</font>

在显示模块的图形层绘制矩形框

<font color='Blue'>【函数声明】</font>

```python
Display.set_graph_rect(x0, y0, x1, y1, chn, flush, color, line_width)
```

<font color='Blue'>【参数描述】</font>

| 参数名称   | 定义描述             |    取值范围            |
| ---------- | ----------------------- | --------- |
| x0         | 绘制矩形框左上角的坐标值x   | 不超过视频画面尺寸   |
| y0         | 绘制矩形框左上角的坐标值y   | 不超过视频画面尺寸   |
| x1         | 绘制矩形框右下角的坐标值x   | 不超过视频画面尺寸   |
| y1         | 绘制矩形框右下角的坐标值y   | 不超过视频画面尺寸   |
| chn        | 图形层通道号 |  范围2~3，默认为2     |
| flush      | 是否清零图形层buffer   | 0：否，1：是      |
| color      | 矩形框颜色设置 |  ARGB8888格式 |
| line_width | 矩形框边的宽度        | 范围1~16，默认为4      |

<font color='Blue'>【使用方法】</font>

```python
#enable graph layer 2
ret = disp.display(2)
print ("Display display 2 return:%d" % ret)

#set osd rectangle
ret = disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
```

<font color='Blue'>【返回值】</font>

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font>

该接口需要在使用`display`接口使能显示功能后使用

<font color='Blue'>【参考代码】</font>

无

## set_graph_word

<font color='Blue'>【功能描述】</font>

在显示模块的图形层绘制字符

<font color='Blue'>【函数声明】</font>

```python
Display.set_graph_word(x, y, str, chn, flush, color, line_width)
```

<font color='Blue'>【参数描述】</font>

| 参数名称   | 描述                    | 取值范围         |
| ---------- | ---------------------- | ------------- |
| x          | 绘制字符的起始坐标值x     | 不超过视频画面尺寸   |
| y          | 绘制字符的起始坐标值y   | 不超过视频画面尺寸   |
| str        | 需要绘制的字符数据 | GB2312编码 |
| chn        | 图形层通道号 |  范围2~3，默认为2     |
| flush      | 是否清零图形层buffer   | 0：否，1：是      |
| color      | 字符颜色设置 |  ARGB8888格式 |
| line_width | 字符线条的宽度        | 范围1~16，默认为1      |

<font color='Blue'>【使用方法】</font>

```python
#enable graph layer 2
ret = disp.display(2)
print ("Display display 2 return:%d" % ret)

#set osd string
string = "horizon"
ret = disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
print ("Display set_graph_word return:%d" % ret)
```

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

该接口需要在使用`display`接口使能显示功能后使用

<font color='Blue'>【参考代码】</font>  

无

## close

<font color='Blue'>【功能描述】</font>

关闭显示模块

<font color='Blue'>【函数声明】</font>  

```python
Display.close()
```

<font color='Blue'>【参数描述】</font>  

无

<font color='Blue'>【使用方法】</font> 

无

<font color='Blue'>【返回值】</font>  

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font> 

该接口需要在使用`display`接口使能显示功能后使用

<font color='Blue'>【参考代码】</font>  

无

## bind接口

<font color='Blue'>【功能描述】</font>

该接口可以把`Camera`、`Encoder`、`Decoder`、`Display`模块的输出与输入数据流进行绑定，绑定后无需用户操作，数据可在绑定模块之间自动流转。例如，绑定 `Camera` 和 `Display` 后，摄像头数据会自动通过显示模块输出到显示屏上，无需调用额外接口。

<font color='Blue'>【函数声明】</font>
```python
    libsrcampy.bind(src, dst)
```

<font color='Blue'>【参数描述】</font>

| 参数名称 | 描述         | 取值范围 |
| -------- | ------------ | --- |
| src      | 源数据模块   |`Camera`、`Encoder`、`Decoder`模块 |
| dst      | 目标数据模块 |`Camera`、`Encoder`、`Decoder`、`Display`模块|

<font color='Blue'>【使用方法】</font>

```python
#create camera object
cam = libsrcampy.Camera()
ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = libsrcampy.Encoder()
ret = enc.encode(0, 1, 1920, 1080)
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = libsrcampy.bind(cam, enc)
print("libsrcampy bind return:%d" % ret)
```

 <font color='Blue'>【返回值】</font>

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font>

无

<font color='Blue'>【参考代码】</font>

无

## unbind接口

<font color='Blue'>【功能描述】</font>

将两个绑定过的模块解绑

<font color='Blue'>【函数声明】</font>
```python
libsrcampy.unbind(src, dst)
```

<font color='Blue'>【参数描述】</font>

| 参数名称 | 描述         | 取值范围 |
| -------- | ------------ | --- |
| src      | 源数据模块   |`Camera`、`Encoder`、`Decoder`模块 |
| dst      | 目标数据模块 |`Camera`、`Encoder`、`Decoder`、`Display`模块|

<font color='Blue'>【使用方法】</font>

```python
#create camera object
cam = libsrcampy.Camera()
ret = cam.open_cam(0, 1, 30, [1920, 1280], [1080, 720])
print("Camera open_cam return:%d" % ret)

#encode start
enc = libsrcampy.Encoder()
ret = enc.encode(0, 1, 1920, 1080)
print("Encoder encode return:%d" % ret)

#bind, input: cam, output: enc
ret = libsrcampy.bind(cam, enc)
print("libsrcampy bind return:%d" % ret)

#unbind, input: cam, output: enc
ret = libsrcampy.unbind(cam, enc)
print("libsrcampy unbind return:%d" % ret)
```

 <font color='Blue'>【返回值】</font>

| 返回值 | 描述 |
| ------ | ---- |
| 0      | 成功 |
| -1    | 失败 |

<font color='Blue'>【注意事项】</font>

无

<font color='Blue'>【参考代码】</font>

无

