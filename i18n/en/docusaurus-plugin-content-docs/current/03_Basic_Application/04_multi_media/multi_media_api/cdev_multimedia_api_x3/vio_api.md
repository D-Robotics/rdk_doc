---
sidebar_position: 1
---
# VIO(Camera Input) API

The `VIO` module provides functions for operating `MIPI` cameras and image processing.

The `VIO` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_init_vio_module | **Initialize the VIO object** |
| sp_release_vio_module | **Destroy the VIO object** |
| sp_open_camera | **Open the camera** |
| sp_open_camera_v2 | **Open the camera with specified resolution** |
| sp_open_vps | **Open VPS** |
| sp_vio_close | **Close the camera** |
| sp_vio_get_frame | **Get video frame** |
| sp_vio_set_frame | **Send video frame to VPS module** |


## sp_init_vio_module  

**【Function prototype】**  

`void *sp_init_vio_module()`

**【Description】**  

Initialize the `VIO` object and create the operation handle. This must be called before calling other interfaces.

**【Parameters】**

None

**【Return type】**  

If successful, returns a pointer to the `VIO` object. If failed, returns `NULL`.

## sp_release_vio_module  

**【Function prototype】**  

`void sp_release_vio_module(void *obj)`

**【Description】**  

Destroy the `VIO` object.**【Parameters】**

- `obj`: Pointer to the `VIO` object obtained when calling the initialization interface.

**【Return Type】**

Void

## sp_open_camera  

**【Function Prototype】**  

`int32_t sp_open_camera(void *obj, const int32_t pipe_id, const int32_t video_index, int32_t chn_num, int32_t *width, int32_t *height)`

**【Description】**  

Initialize the MIPI camera connected to RDK X3.  
Set the output resolution, supporting up to 5 sets of resolutions, where only 1 set can enlarge and 4 sets can shrink.  
The maximum enlargement is 1.5 times the original image, and the minimum shrinkage is 1/8 of the original image.

**【Parameters】**

- `obj`: Pointer to the initialized `VIO` object.
- `pipe_id`: Supports multiple sets of input data, it is recommended to fill in 0.
- `video_index`: The host index corresponding to the camera, -1 means automatic detection, and the index can be viewed in the /etc/board_config.json configuration file.
- `chn_num`: Set the number of different resolutions of the output image, with a maximum of 5 and a minimum of 1.
- `width`: Array address for configuring the output width.
- `height`: Array address for configuring the output height.

**【Return Type】**

Returns 0 if successful, -1 if failed.

## sp_open_camera_v2  

**【Function Prototype】**  

`int32_t sp_open_camera_v2(void *obj, const int32_t pipe_id, const int32_t video_index, int32_t chn_num, sp_sensors_parameters *parameters, int32_t *width, int32_t *height)`

**【Description】**  

Initialize the MIPI camera connected to RDK X3.  
Support specifying the resolution size of the original RAW output of the camera by setting `sp_sensors_parameters`.  
Support setting the output resolution, up to 5 groups of resolutions, where only 1 group can enlarge and 4 groups can shrink.  
The maximum enlargement is 1.5 times the original image, and the minimum shrinkage is 1/8 of the original image.

List of currently supported camera resolutions:

| Camera | Resolution |
| ------ | ---------- |
|IMX219|1920x1080@30fps(default), 640x480@30fps, 1632x1232@30fps, 3264x2464@15fps(max)|
|IMX477|1920x1080@50fps(default), 1280x960@120fps, 2016x1520@40fps, 4000x3000@10fps(max)|
|OV5647|1920x1080@30fps(default), 640x480@60fps, 1280x960@30fps, 2592x1944@15fps(max)|
|F37|1920x1080@30fps(default)|
|GC4663|2560x1440@30fps(default)|

:::info Note!

Switching from `1080P` resolution to other resolutions for `IMX477` requires manual reset. You can use `hobot_reset_camera.py` script on the board to perform the reset operation.

:::

**【Parameters】**

- `obj`: Pointer to the initialized `VIO` object
- `pipe_id`: Supports multiple sets of input data, recommended to fill in 0
- `video_index`: Host number corresponding to the camera, -1 represents automatic detection. You can check the `/etc/board_config.json` configuration file for the number
- `chn_num`: Set the number of different resolution images to output, maximum is 5, minimum is 1
- `parameters`: Camera RAW output related structure, used to specify the resolution and frame rate
- `width`: Address of array for configuring output width
- `height`: Address of array for configuring output height

Members of the `sp_sensors_parameters` structure are shown in the table below:

| Data Type | Member | Description |
| ---- | ----- | ----- |
|int32_t|raw_height|Height of the camera's RAW output|
|int32_t|raw_width|Width of the camera's RAW output|
|int32_t|fps|Frame rate of the camera's output|

:::info Note!

The `X3` chip has alignment requirements for the width of the `VPS` output. If the width you set does not meet the alignment requirement of **32**, it will be automatically rounded up. For example, if you set the output width to `720` and height to `480`, the actual width of the `VPS` output will be `736`. In the case of **binding**, this situation will be **automatically** handled by the library. However, in the case of **non-binding**, you need to pay attention to the width alignment issue when manually processing frame buffers, and there may be **screen distortion and green lines** when passing the frame to the display module in this non-aligned situation.

:::

**【Return Type】** 

Returns 0 if successful, -1 if failed

## sp_open_vps  

**【Function Prototype】**  

`int32_t sp_open_vps(void *obj, const int32_t pipe_id, int32_t chn_num, int32_t proc_mode, int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height, int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)`

**【Function Description】**  

Opens an image processing module, supports resizing, zooming, rotating, and cropping tasks for the input image.

**【Parameters】** 

- `obj`: Initialized `VIO` object pointer
- `pipe_id`: Differentiates multiple openings through `pipe_id`.
- `chn_num`: Sets the number of output images, with a maximum of 5, depending on the size of the target height array set.
- `proc_mod`: Processing mode, currently supported: `SP_VPS_SCALE` scale only, `SP_VPS_SCALE_CROP` scale and crop, `SP_VPS_SCALE_ROTATE` scale and rotate, `SP_VPS_SCALE_ROTATE_CROP` scale, rotate, and crop
- `src_width`: Original frame width
- `src_height`: Original frame height
- `dst_width`: Array address for configuring the target output width
- `dst_height`: Array address for configuring the target output height
- `crop_x`: Left-top X coordinates of the crop area. When `proc_mod` does not set the crop function, `NULL` is passed.
- `crop_y`: Left-top Y coordinates of the crop area. When `proc_mod` does not set the crop function, `NULL` is passed.
- `crop_width`: Width of the crop area. When `proc_mod` does not set the crop function, `NULL` is passed.
- `crop_height`: Height of the crop area. When `proc_mod` does not set the crop function, `NULL` is passed.
- `rotate`: Rotation angle collection, currently supports `ROTATION_90` 90°, `ROTATION_180` 180°, and `ROTATION_270` 270°. When `proc_mod` does not set the rotation function, `NULL` is passed.

:::info Note!

The `X3` chip has alignment requirements for the width of `VPS` output. If the width you set does not meet the 32 alignment, it will be rounded up automatically. For example, if you set the output width to `720` and height to `480`, the actual `VPS` output width will be `736`. In the case of binding, this situation will be automatically handled inside the library when called. In the case of non-binding, pay attention to the width alignment when manually processing the frame buffer, and there will be screen flashing and green lines when passing the frame to the display module in this non-aligned situation.

:::

【Return Type】  
Return 0 if successful, -1 if failed

## sp_vio_close  

【Function Prototype】  

`int32_t sp_vio_close(void *obj)`

【Function Description】  

Determines whether to close the camera or the VPS module based on the input `obj`.

【Parameters】

- `obj`: Initialized `VIO` object pointer  

【Return Type】  
Return 0 if successful, -1 if failed

## sp_vio_get_frame  

【Function Prototype】

`int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【Description】**  

Get the image frame data with the specified resolution (the resolution needs to be passed in when opening the module, otherwise the retrieval will fail). The returned data format is YUV image in NV12 format.

**【Parameters】**

- `obj`: Initialized VIO object pointer
- `frame_buffer`: Pre-allocated buffer pointer for saving the retrieved image. The size of the pre-allocated memory can be calculated using the formula `height * width * 3 / 2` or using the provided macro definition `FRAME_BUFFER_SIZE(w, h)`.
- `width`: Width of the image to be saved in `frame_buffer`. It must be the output width configured in `sp_open_camera` or `sp_open_vps`.
- `height`: Height of the image to be saved in `frame_buffer`. It must be the output height configured in `sp_open_camera` or `sp_open_vps`.
- `timeout`: Timeout for retrieving the image, in milliseconds. Generally set to 2000.

**【Return Type】**  

Returns 0 on success, -1 on failure.

## sp_vio_get_raw  

**【Function Prototype】**  

`int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【Description】**  

Get the raw image data from the camera.

**【Parameters】**

- `obj`: Initialized VIO object pointer.
- `frame_buffer`: Pre-allocated buffer pointer for saving the retrieved raw image. The size of the pre-allocated memory can be calculated using the formula `(height * width * bit depth)/8`.
- `width`: Null for getting raw image.
- `height`: Null for getting raw image.
- `timeout`: Timeout for retrieving the image, in milliseconds. Generally set to 2000.

**【Return Type】**  

Returns 0 on success, -1 on failure.

## sp_vio_get_yuv  

**【Function Prototype】**  

`int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**【Description】**  

Get the YUV data from the ISP module of the camera.

**【Parameters】**

- `obj`: Initialized pointer to the `VIO` object
- `frame_buffer`: Pointer to the pre-allocated buffer used to save the captured image. Currently, the captured images are in `NV12` format, so the buffer size can be calculated using the formula `height * width * 3 / 2`, or by utilizing the provided macro definition `FRAME_BUFFER_SIZE(w, h)`.
- `width`: Set to `NULL` when fetching ISP's YUV data
- `height`: Set to `NULL` when fetching ISP's YUV data
- `timeout`: Timeout period for capturing the image, in milliseconds. Generally set to `2000`.

**【Return Type】**  

Returns 0 on success, -1 on failure.

## sp_vio_set_frame  

**【Function Prototype】**  

`int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size)`

**【Function Description】**  

When using the functionality of the `vps` module, the raw data needs to be passed in through this interface. The data in `frame_buffer` must be in the `NV12` format and have the same resolution as the original frame resolution when calling the `sp_open_vps` interface.

**【Parameters】**

- `obj`: Initialized pointer to the `VIO` object
- `image_buffer`: Image frame data to be processed. It must be in the `NV12` format and have the same resolution as the original frame resolution when calling the `sp_open_vps` interface.
- `size`: Size of the frame

**【Return Type】**  

Returns 0 on success, -1 on failure.