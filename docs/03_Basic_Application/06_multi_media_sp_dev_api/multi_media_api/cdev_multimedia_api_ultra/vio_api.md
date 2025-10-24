---
sidebar_position: 1
---

# VIO（视频输入）API

`VIO` 模块提供操作 `MIPI` 摄像头和操作图像处理的功能。

`VIO` API提供了以下的接口：

| 函数 | 功能 |
| ---- | ----- |
| sp_init_vio_module | **初始化VIO对象** |
| sp_release_vio_module | **销毁VIO对象** |
| sp_open_camera | **打开摄像头** |
| sp_open_vps | **打开VPS** |
| sp_vio_close | **关闭摄像头** |
| sp_vio_get_frame | **获取视频图像帧** |
| sp_vio_set_frame | **发送视频图像帧给vps模块** |


## sp_init_vio_module  

**【函数原型】**  

`void *sp_init_vio_module()`

**【功能描述】**  

初始化`VIO`对象，创建操作句柄。在其他接口调用前必须执行。

**【参数】**

无

**【返回类型】**  

成功返回一个`VIO`对象指针，失败返回`NULL`

## sp_release_vio_module  

**【函数原型】**  

`void sp_release_vio_module(void *obj)`

**【功能描述】**  

销毁`VIO`对象。

**【参数】**

- `obj`： 调用初始化接口时得到的`VIO`对象指针。

**【返回类型】**  

无

## sp_open_camera  

**【函数原型】**  

`int32_t sp_open_camera(void *obj, int32_t chn_num, int32_t *width, int32_t *height)`

**【功能描述】**  

初始化接入到RDK X3上的MIPI摄像头。
设置输出分辨率，支持设置最多5组分辨率，其中只有1组可以放大，4组可以缩小。最大支持放大到原始图像的1.5倍，最小支持缩小到原始图像的1/8。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `chn_num`：设置输出多少种不同分辨率的图像，最大为5，最小为1。
- `width`：配置输出宽度的数组地址
- `height`：配置输出高度的数组地址

**【返回类型】** 

成功返回 0，失败返回 -1


## sp_open_vps  

**【函数原型】**  

`int32_t sp_open_vps(void *obj, int32_t chn_num,int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)`

**【功能描述】**  

打开一路图像处理模块，支持对输入的图像完成缩小、放大、旋转、裁剪任务。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `chn_num`：设置输出图像数量，最大为5，与设置的目标高宽数组大小有关
- `src_width`：原始帧宽度
- `src_height`：原始帧高度
- `dst_width`：配置目标输出宽度的数组地址
- `dst_height`：配置目标输出高度的数组地址
- `crop_x`：裁剪区域的左上角x坐标集合，不使用裁剪功能时，传入`NULL`
- `crop_y`：裁剪区域的左上角y坐标集合，不使用裁剪功能时，传入`NULL`
- `crop_width`：裁剪区域的宽度，不使用裁剪功能时，传入`NULL`
- `crop_height`：裁剪区域的高度，不使用裁剪功能时，传入`NULL`
- `rotate`：旋转角度集合，目前支持`ROTATION_90` 90°、`ROTATION_180` 180°和`ROTATION_270` 270°，不使用旋转功能时，传入`NULL`

**【返回类型】**  

成功返回 0，失败返回 -1

## sp_vio_close  

**【函数原型】**  

`int32_t sp_vio_close(void *obj)`

**【功能描述】**  

根据传入的 `obj` 是打开的 `camera` 还是 `vps`决定关闭camera还是vps模块。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针  

**【返回类型】**  

成功返回 0，失败返回 -1

## sp_vio_get_frame  

**【函数原型】**  

`int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【功能描述】**  

获取指定分辨率的图像帧数据（分辨率在打开模块时需要传入，否则会获取失败）。返回数据格式为 `NV12` 的 `YUV` 图像。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `frame_buffer`：已经预分配内存的buffer指针，用于保存获取出来的图片，目前获取到的图像都是`NV12`格式，所以预分配内存大小可以由公式`高 * 宽 * 3 / 2 `，也可以利用提供的宏定义 `FRAME_BUFFER_SIZE(w, h)`进行内存大小计算
- `width`：`image_buffer`保存图片的宽，必须是在`sp_open_camera`或者`sp_open_vps`配置好的输出宽
- `height`：`image_buffer`保存图片的高，必须是在`sp_open_camera`或者`sp_open_vps`配置好的输出高
- `timeout`：获取图片的超时时间，单位为`ms`，一般设置为`2000`

**【返回类型】**  

成功返回 0，失败返回 -1 

## sp_vio_get_raw  

**【函数原型】**  

`int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【功能描述】**  

获取摄像头的raw图数据

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `frame_buffer`：已经预分配内存的buffer指针，用于保存获取出来的raw图，预分配内存字节大小可以由公式`(高 * 宽 * 图像深度)/8`计算得出
- `width`：获取raw图时传`NULL`
- `height`：获取raw图时传`NULL`
- `timeout`：获取图片的超时时间，单位为`ms`，一般设置为`2000`

**【返回类型】**  

成功返回 0，失败返回 -1 

## sp_vio_get_yuv  

**【函数原型】**  

`int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【功能描述】**  

获取摄像头的ISP模块的YUV数据

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `frame_buffer`：已经预分配内存的buffer指针，用于保存获取出来的图片，目前获取到的图像都是`NV12`格式，所以预分配内存大小可以由公式`高 * 宽 * 3 / 2 `，也可以利用提供的宏定义 `FRAME_BUFFER_SIZE(w, h)`进行内存大小计算
- `width`：获取ISP的YUV数据时传`NULL`
- `height`：获取ISP的YUV数据传`NULL`
- `timeout`：获取图片的超时时间，单位为`ms`，一般设置为`2000`

**【返回类型】**  

成功返回 0，失败返回 -1 

## sp_vio_set_frame  

**【函数原型】**  

`int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size)`

**【功能描述】**  

在使用`vps`模块功能时，源数据需要通过调用本接口送入，`frame_buffer`里面的数据必须是 `NV12` 格式的图像数据，分辨率必须和调用`sp_open_vps`接口是的原始帧分辨率一致。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `image_buffer`：需要处理的图像帧数据，必须是 `NV12` 格式的图像数据，分辨率必须和调用`sp_open_vps`接口是的原始帧分辨率一致。
- `size`: 帧大小

**【返回类型】**  

成功返回 0，失败返回 -1

