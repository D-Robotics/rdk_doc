---
sidebar_position: 2
---

# ENCODER（编码模块）API

`ENCODER` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_init_encoder_module | **初始化编码模块对象** |
| sp_release_encoder_module | **销毁编码模块对象** |
| sp_start_encode | **创建图像编码通道** |
| sp_stop_encode | **关闭图像编码通道** |
| sp_encoder_set_frame | **向编码通道传入图像帧** |
| sp_encoder_get_stream | **从编码通道获取编码好的码流** |

:::note

RDK Ultra **不**支持`H264`编解码

:::

## sp_init_encoder_module  

**【函数原型】**  

`void *sp_init_encoder_module()`

**【功能描述】**  

初始化编码模块对象，在使用编码模块时需要调用获得操作句柄。

**【参数】**

无

**【返回类型】**  

成功返回一个`ENCODER`对象指针，失败返回`NULL`。

## sp_release_encoder_module  

**【函数原型】**  

`void sp_release_encoder_module(void *obj)`

**【功能描述】**  

销毁编码模块对象。

**【参数】**

 - `obj`: 调用初始化接口时得到的对象指针。

**【返回类型】**  

无

## sp_start_encode  

**【函数原型】**  

`int32_t sp_start_encode(void *obj, int32_t type, int32_t width, int32_t height, int32_t bits)`

**【功能描述】**  

创建一路图像编码通道，支持最多创建 `32` 路编码，编码类型支持 `H264`, `H265` 和 `MJPEG`。

**【参数】**

- `obj`： 已经初始化的`ENCODER`对象指针
- `type`：图像编码类型，支持 `SP_ENCODER_H264`，`SP_ENCODER_H265` 和 `SP_ENCODER_MJPEG`。
- `width`：输入给编码通道的图像数据分辨率-宽
- `height`：输入给编码通道的图像数据分辨率-高
- `bits`：编码码率，常用值为 512, 1024, 2048, 4096, 8192, 16384 等码率（单位 Mbps），其他值也可以，码率也大编码的图像越清晰，压缩率越小，码流数据越大。

**【返回类型】**  

成功返回 0，失败返回 -1

## sp_stop_encode  

**【函数原型】**  

`int32_t sp_stop_encode(void *obj)`

**【功能描述】**  

关闭打开的编码通道。

**【参数】**

- `obj`： 已经初始化的`ENCODER`对象指针

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_encoder_set_frame  

**【函数原型】**  

`int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)`

**【功能描述】**  

向编码通道传入需要编码的图像帧数据，格式必须为 `NV12`。

**【参数】**

- `obj`： 已经初始化的`ENCODER`对象指针
- `frame_buffer`：需要编码的图像帧数据，必须是 `NV12` 格式，分辨率必须和调用`sp_start_encode`接口是的图像帧分辨率一致。
- `size`：图像帧数据大小，`NV12` 格式的图像的大小计算公式为 width * height * 3 / 2。

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_encoder_get_stream  

**【函数原型】**  

`int32_t sp_encoder_get_stream(void *obj, char *stream_buffer)`

**【功能描述】**  

从编码通道获取编码好的码流数据。

**【参数】**

- `obj`： 已经初始化的`ENCODER`对象指针
- `stream_buffer`：获取成功后，码流数据会存在本buffer中。此buffer的大小需要根据编码分辨率和码率进行调整。

**【返回类型】** 

成功返回码流数据的size，失败返回 -1
