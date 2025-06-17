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
| sp_open_camera_v2 | **指定分辨率打开摄像头** |
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

`int32_t sp_open_camera(void *obj, const int32_t pipe_id, const int32_t video_index, int32_t chn_num, int32_t *width, int32_t *height)`

**【功能描述】**  

初始化接入到RDK X3上的MIPI摄像头。
设置输出分辨率，支持设置最多5组分辨率，其中只有1组可以放大，4组可以缩小。最大支持放大到原始图像的1.5倍，最小支持缩小到原始图像的1/8。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `pipe_id`：支持多组数据输入，建议填0
- `video_index`：camera对应的host编号，-1表示自动探测，编号可以查看 /etc/board_config.json 配置文件
- `chn_num`：设置输出多少种不同分辨率的图像，最大为5，最小为1。
- `width`：配置输出宽度的数组地址
- `height`：配置输出高度的数组地址

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_open_camera_v2  

**【函数原型】**  

`int32_t sp_open_camera_v2(void *obj, const int32_t pipe_id, const int32_t video_index, int32_t chn_num, sp_sensors_parameters *parameters, int32_t *width, int32_t *height)`

**【功能描述】**  

初始化接入到RDK X3上的MIPI摄像头。  
支持指定摄像头原始输出RAW的分辨率大小，通过`sp_sensors_parameters`设置。  
支持设置输出分辨率，支持设置最多5组分辨率，其中只有1组可以放大，4组可以缩小。最大支持放大到原始图像的1.5倍，最小支持缩小到原始图像的1/8。

目前支持的摄像头分辨率见下表：

| camera | 分辨率 |
| ---- | ----- |
|IMX219|1920x1080@30fps(default), 640x480@30fps, 1632x1232@30fps, 3264x2464@15fps(max)|
|IMX477|1920x1080@50fps(default), 1280x960@120fps, 2016x1520@40fps, 4000x3000@10fps(max)|
|OV5647|1920x1080@30fps(default), 640x480@60fps, 1280x960@30fps, 2592x1944@15fps(max)|
|F37|1920x1080@30fps(default)|
|GC4663|2560x1440@30fps(default)|

:::info 注意！

`IMX477`从`1080P`的分辨率切换至其它分辨率需要进行手动复位，可以在板端执行`hobot_reset_camera.py`完成复位操作。

:::

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `pipe_id`：支持多组数据输入，建议填0
- `video_index`：camera对应的host编号，-1表示自动探测，编号可以查看 /etc/board_config.json 配置文件
- `chn_num`：设置输出多少种不同分辨率的图像，最大为5，最小为1。
- `parameters`：camera RAW输出相关结构体，用于指定分辨率和帧率
- `width`：配置输出宽度的数组地址
- `height`：配置输出高度的数组地址

`sp_sensors_parameters`结构体成员见下表：

| 数据类型 | 成员 | 注释 |
| ---- | ----- | ----- |
|int32_t|raw_height|摄像头输出RAW的高度|
|int32_t|raw_width|摄像头输出RAW的宽度|
|int32_t|fps|摄像头输出的帧率|

:::info 注意！

`X3`芯片对于`VPS`输出的宽度是有对齐需求的，如果您设置的宽度不满足**32**对齐，则会自动向上取整。例如您设置了  
宽度为`720`，高度为`480`的输出，实际`VPS`输出的宽度会是`736`。**绑定情况**下这种情况会在调用库里面**自动**处理  
对于**非绑定情况**下手动处理帧buffer时就要注意宽度对齐问题，以及在这种非对齐情况下将帧传给显示模块时会出现**花屏、绿线**的情况。

:::

**【返回类型】** 

成功返回 0，失败返回 -1

## sp_open_vps  

**【函数原型】**  

`int32_t sp_open_vps(void *obj, const int32_t pipe_id, int32_t chn_num, int32_t proc_mode, int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height, int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)`

**【功能描述】**  

打开一路图像处理模块，支持对输入的图像完成缩小、放大、旋转、裁剪任务。

**【参数】**

- `obj`： 已经初始化的`VIO`对象指针
- `pipe_id`：支持多次打开，通过`pipe_id`进行区分。
- `chn_num`：设置输出图像数量，最大为5，与设置的目标高宽数组大小有关
- `proc_mod`：处理模式，当前支持：`SP_VPS_SCALE` 仅缩放、`SP_VPS_SCALE_CROP` 缩放并裁剪、`SP_VPS_SCALE_ROTATE` 缩放并旋转、`SP_VPS_SCALE_ROTATE_CROP` 缩放之后旋转并裁剪
- `src_width`：原始帧宽度
- `src_height`：原始帧高度
- `dst_width`：配置目标输出宽度的数组地址
- `dst_height`：配置目标输出高度的数组地址
- `crop_x`：裁剪区域的左上角x坐标集合，当`proc_mod`没有设置裁剪功能时，传入`NULL`
- `crop_y`：裁剪区域的左上角y坐标集合，当`proc_mod`没有设置裁剪功能时，传入`NULL`
- `crop_width`：裁剪区域的宽度，当`proc_mod`没有设置裁剪功能时，传入`NULL`
- `crop_height`：裁剪区域的高度，当`proc_mod`没有设置裁剪功能时，传入`NULL`
- `rotate`：旋转角度集合，目前支持`ROTATION_90` 90°、`ROTATION_180` 180°和`ROTATION_270` 270°，当`proc_mod`没有设置旋转功能时，传入`NULL`

:::info 注意！

`X3`芯片对于`VPS`输出的宽度是有对齐需求的，如果您设置的宽度不满足**32**对齐，则会自动向上取整。例如您设置了  
宽度为`720`，高度为`480`的输出，实际`VPS`输出的宽度会是`736`。**绑定情况**下这种情况会在调用库里面**自动**处理  
对于**非绑定情况**下手动处理帧buffer时就要注意宽度对齐问题，以及在这种非对齐情况下将帧传给显示模块时会出现**花屏、绿线**的情况。

:::

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

