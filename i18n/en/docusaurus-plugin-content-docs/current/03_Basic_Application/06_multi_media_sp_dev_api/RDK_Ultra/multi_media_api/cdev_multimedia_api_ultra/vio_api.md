---
sidebar_position: 1
---
# VIO (Video Input/Output) API

The `VIO` module provides functions for operating MIPI cameras and handling image processing. Below is a summary of the available `VIO` APIs:

| Function               | Description                                                        |
|------------------------|--------------------------------------------------------------------|
| sp_init_vio_module      | **Initialize VIO object**                                          |
| sp_release_vio_module   | **Release VIO object**                                             |
| sp_open_camera          | **Open MIPI camera**                                               |
| sp_open_vps             | **Open VPS (Video Processing System)**                             |
| sp_vio_close            | **Close camera or VPS module**                                     |
| sp_vio_get_frame        | **Get video frame data**                                           |
| sp_vio_set_frame        | **Send video frame data to VPS**                                   |

## sp_init_vio_module

**Function Declaration**  
`void *sp_init_vio_module()`

**Description**  
Initializes the `VIO` object and creates the handler. This function must be called before using other interfaces.

**Parameters**  
None

**Return Type**  
- Returns a pointer to the `VIO` object on success.  
- Returns `NULL` on failure.

## sp_release_vio_module

**Function Declaration**  
`void sp_release_vio_module(void *obj)`

**Description**  
Releases the `VIO` object.

**Parameters**  
- `obj`: The pointer to the `VIO` object obtained from the initialization interface.

**Return Type**  
None

## sp_open_camera

**Function Declaration**  
`int32_t sp_open_camera(void *obj, int32_t chn_num, int32_t *width, int32_t *height)`

**Description**  
Initializes the MIPI camera connected to the RDK X3. Allows configuring the output resolution. A maximum of 5 sets of resolutions can be specified, with one set for enlargement and four for reduction.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `chn_num`: The number of different resolutions to output (up to 5).
- `width`: The array for output width.
- `height`: The array for output height.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_open_vps

**Function Declaration**  
`int32_t sp_open_vps(void *obj, int32_t chn_num, int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height, int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)`

**Description**  
Opens a video processing module that supports resizing, rotating, and cropping of the input image.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `chn_num`: The number of output images (up to 5).
- `src_width`: The original frame width.
- `src_height`: The original frame height.
- `dst_width`: The array for the target output width.
- `dst_height`: The array for the target output height.
- `crop_x`: The array for the X coordinate of the crop region (use `NULL` if cropping is not needed).
- `crop_y`: The array for the Y coordinate of the crop region (use `NULL` if cropping is not needed).
- `crop_width`: The array for the width of the crop region (use `NULL` if cropping is not needed).
- `crop_height`: The array for the height of the crop region (use `NULL` if cropping is not needed).
- `rotate`: The array for rotation angles (`ROTATION_90`, `ROTATION_180`, `ROTATION_270`), or `NULL` if no rotation is required.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_vio_close

**Function Declaration**  
`int32_t sp_vio_close(void *obj)`

**Description**  
Closes the camera or VPS module based on the `obj` parameter.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_vio_get_frame

**Function Declaration**  
`int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**Description**  
Retrieves a video frame with the specified resolution. The frame data will be in the `NV12` YUV format.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `frame_buffer`: A pre-allocated buffer pointer for storing captured images. Currently, the acquired images are all in `NV12` format, so the pre-allocated memory size can be calculated using the formula `height * width * 3 / 2`, or by utilizing the provided macro `FRAME_BUFFER_SIZE(w, h)` for memory size calculation.
- `width`: The width of the image (must match the configured output width in `sp_open_camera` or `sp_open_vps`).
- `height`: The height of the image (must match the configured output height in `sp_open_camera` or `sp_open_vps`).
- `timeout`: The timeout for obtaining the image (in milliseconds, typically set to `2000`).

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_vio_get_raw

**Function Declaration**  
`int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**Description**  
Retrieves the raw image data from the camera.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `frame_buffer`: A pre-allocated memory buffer pointer used to store the retrieved raw image. The pre-allocated memory size in bytes can be calculated using the formula `(height * width * image depth) / 8`.
- `width`: Set to `NULL` when retrieving raw data.
- `height`: Set to `NULL` when retrieving raw data.
- `timeout`: The timeout for obtaining the image (in milliseconds, typically set to `2000`).

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_vio_get_yuv

**Function Declaration**  
`int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)`

**Description**  
Retrieves the YUV data from the ISP module of the camera.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `frame_buffer`: A pre-allocated memory buffer pointer used to store the retrieved images. Currently, all acquired images are in `NV12` format, so the pre-allocated memory size can be calculated using the formula `height * width * 3 / 2`. Alternatively, the provided macro definition `FRAME_BUFFER_SIZE(w, h)` can also be used to compute the memory size.
- `width`: Set to `NULL` when retrieving ISP YUV data.
- `height`: Set to `NULL` when retrieving ISP YUV data.
- `timeout`: The timeout for obtaining the image (in milliseconds, typically set to `2000`).

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_vio_set_frame

**Function Declaration**  
`int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size)`

**Description**  
Sends the video frame to the VPS module. The `frame_buffer` should contain data in the `NV12` format, and the resolution must match the one configured in `sp_open_vps`.

**Parameters**  
- `obj`: The pointer to the initialized `VIO` object.
- `frame_buffer`: The image data in `NV12` format.
- `size`: The size of the frame.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.
