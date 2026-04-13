---
sidebar_position: 3
---

# DECODER（解码模块）API

`DECODER` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_init_decoder_module | **初始化解码模块对象** |
| sp_release_decoder_module | **销毁解码模块对象** |
| sp_start_decode | **创建图像解码通道** |
| sp_stop_decode | **关闭图像解码通道** |
| sp_decoder_get_image | **从解码通道获取解码后的图像帧** |
| sp_decoder_set_image | **向解码通道传入需要解码的码流数据** |

:::note

RDK Ultra **不**支持`H264`编解码

:::

## sp_init_decoder_module  

**【函数原型】**  

`void *sp_init_decoder_module()`

**【功能描述】**  

初始化解码模块对象，在使用解码模块时需要调用获得操作句柄，支持H264、H265和Mjpeg格式的视频码流。

**【参数】**

无。

**【返回类型】** 

成功返回`DECODER`对象，失败返回 NULL。

## sp_release_decoder_module  

**【函数原型】**  

`void sp_release_decoder_module(void *obj)`

**【功能描述】**  

销毁解码模块对象。

**【参数】**

 - `obj`: 调用初始化接口时得到的对象指针。

**【返回类型】**  

无

## sp_start_decode  

**【函数原型】**  

`int sp_start_decode(void *decoder_object,const char *stream_file, int32_t type, int32_t width, int32_t height)`

**【功能描述】**  

创建一个解码通道，设置解码的码流类型、图像帧分辨率。

**【参数】**

- `obj`： 已经初始化的`DECODER`对象指针
- `stream_file`：当 `stream_file` 设置为一个码流文件名时，表示对这个码流文件进行解码，例如设置H265的码流文件“stream.h265”, 当 `stream_file` 传入空字符串时，表示解码的数据流需要通过调用 `sp_decoder_set_image` 传入。
- `type`：解码的数据类型，支持`SP_ENCODER_H265` 和 `SP_ENCODER_MJPEG`。
- `width`：解码出来的图像帧的分辨率 - 宽
- `height`：解码出来的图像帧的分辨率 - 高

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_stop_decode  

**【函数原型】**  

`int32_t sp_stop_decode(void *obj)`

**【功能描述】**  

关闭解码通道。

**【参数】**

- `obj`： 已经初始化的`DECODER`对象指针

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_decoder_get_image  

**【函数原型】**  

`int32_t sp_decoder_get_image(void *obj, char *image_buffer)`

**【功能描述】**  

从解码通道获取解码后的图像帧数据，返回的图像数据格式为 `NV12` 的 `YUV` 图像。

**【参数】**

- `obj`：已经初始化的`DECODER`对象指针
- `image_buffer`：返回的图像帧数据，这个buffer大小与图像分辨率的关系为 width * height * 3 / 2。

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_decoder_set_image  

**【函数原型】**  

`int32_t sp_decoder_set_image(void *obj, char *image_buffer, int32_t size, int32_t eos)`

**【功能描述】**  

向已经打开的解码通道送入码流数据。
如果是解码 H264 或 H265 码流，需要先发送3-5帧数据，让解码器完成帧缓存后，再获取解码帧数据。
如果解码 H264 码流，首先第一帧送入解码的数据需要是 sps 和 pps 的描述信息，否者解码器会报错退出。

**【参数】**

- `obj`： 已经初始化的`DECODER`对象指针。
- `image_buffer`：码流数据指针。
- `size`：码流数据大小。
- `eos`：是否是最后一帧数据。

**【返回类型】** 

成功返回 0，失败返回 -1

