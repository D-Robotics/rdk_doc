---
sidebar_position: 4
---

# DISPLAY（显示模块）API

`DISPLAY` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_init_display_module | **初始化显示模块对象** |
| sp_release_display_module | **销毁显示模块对象** |
| sp_start_display | **创建视频显示通道** |
| sp_stop_display | **关闭视频显示通道** |
| sp_display_set_image | **向视频显示通道传入图像** |
| sp_display_draw_rect | **在显示通道上绘制矩形框** |
| sp_display_draw_string | **在显示通道上绘制字符串** |
| sp_get_display_resolution | **获取显示器的分辨率** |

## sp_init_display_module  

**【函数原型】**  

`void *sp_init_display_module()`

**【功能描述】**  

初始化显示模块对象，本模块支持把视频图像数据显示到 `HDMI` 接口的显示器上，并且提供在显示画面上绘制矩形框和文字的功能。

**【参数】**

无

**【返回类型】** 

成功返回 `DISPLAY` 对象指针，失败返回 NULL。

## sp_release_display_module  

**【函数原型】**  

`void sp_release_display_module(void *obj)`

**【功能描述】**  

销毁 `DISPLAY` 对象。

**【参数】**

- `obj`： 已经初始化的`DISPLAY`对象指针

**【返回类型】** 

无

## sp_start_display  

**【函数原型】**  

`int32_t sp_start_display(void *obj, int32_t width, int32_t height)`

**【功能描述】**  

创建一个显示通道，RDK Ultra开发板支持的最大分辨率为 `1920 x 1080`, 最大帧率 `60fps`。

**【参数】**

- `obj`： 已经初始化的`DISPLAY`对象指针
- `width`：显示输出分辨率 - 宽
- `height`：显示输出分辨率 - 高

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_stop_display  

**【函数原型】**  

`int32_t sp_stop_display(void *obj)`

**【功能描述】**  

关闭显示通道。

**【参数】**

- `obj`： 已经初始化的`DISPLAY`对象指针

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_display_set_image  

**【函数原型】**  

`int32_t sp_display_set_image(void *obj, char *addr, int32_t size)`

**【功能描述】**  

送入一帧图像到显示模块， 图像格式只支持 `NV12` 的 `YUV` 图像。

**【参数】**

- `obj`：已经初始化的`DISPLAY`对象指针
- `addr`：图像数据，图像格式只支持 `NV12`。
- `size`：图像数据大小，计算公式为： width * height * 3 / 2

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_display_draw_rect  

**【函数原型】**  

`int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t flush, int32_t color, int32_t line_width)`

**【功能描述】**  

在显示模块的图形层绘制矩形框。

**【参数】**

- `obj`： 已经初始化的`DISPLAY`对象指针
- `x0`：绘制矩形框第一个坐标的x值
- `y0`：绘制矩形框第一个坐标的y值
- `x1`：绘制矩形框第二个坐标的x值
- `y1`：绘制矩形框第二个坐标的y值
- `flush`：是否清零当前图形层buffer
- `color`：矩形框颜色（颜色格式为ARGB8888）
- `line_width`：矩形框的线宽

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_display_draw_string  

**【函数原型】**  

`int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t flush, int32_t color, int32_t line_width)`

**【功能描述】**  

在显示模块的图形层绘制矩形框。

**【参数】**

- `obj`： 已经初始化的`DISPLAY`对象指针
- `x`：绘制字符串起始坐标的x值
- `y`：绘制字符串起始坐标的y值
- `str`：需要绘制的字符串（需要是GB2312编码）
- `flush`：是否清零当前图形层buffer
- `color`：矩形框颜色（颜色格式为ARGB8888）
- `line_width`：文字的线宽

**【返回类型】** 

成功返回 0，失败返回 -1



## sp_get_display_resolution  

**【函数原型】**  

`void sp_get_display_resolution(int32_t *width, int32_t *height)`

**【功能描述】**  

获取当前接入的显示器分辨率。

**【参数】**

- `width`： 需要获取的分辨率 - 宽
- `height`：需要获取的分辨率 - 高

**【返回类型】** 

无。

:::note

目前仅支持`1920x1080@60Fps`格式

:::