---
sidebar_position: 4
---
# DISPLAY (Display Module) API

The `DECODER` module provides interfaces for handling video decoding tasks. The available functions are as follows:

| Function                     | Description                                               |
|------------------------------|-----------------------------------------------------------|
| sp_init_display_module        | **Initialize display module object**                  |
| sp_release_display_module     | **Destroy display module object**                     |
| sp_start_display               | **Create video display channel**                       |
| sp_stop_display                | **Close video display channel**                      |
| sp_display_set_image          | **Send image to video display channel**|
| sp_display_draw_rect          | **Draw rectangle on display channel**      |
| sp_display_draw_string          | **Draw string on display channel**|
| sp_get_display_resolution          | **Get display resolution**      |


## sp_init_display_module

**Function Declaration**  
`void *sp_init_display_module()`

**Description**  
Initialize display module object. This module supports displaying video image data to monitors connected via the `HDMI` interface, and provides functionality to draw rectangles and text on the display screen.

**Parameters**  
None

**Return Type**  
- Returns pointer to `DISPLAY` object on success, NULL on failure..

## sp_release_display_module

**Function Declaration**  
`void sp_release_display_module(void *obj)`

**Description**  
Destroy `DISPLAY` object..

**Parameters**  
- `obj`: Pointer to initialized `DISPLAY` object

**Return Type**  
None

## sp_start_display

**Function Declaration**  
`int32_t sp_start_display(void *obj, int32_t width, int32_t height)`

**Description**  
Create a display channel. The RDK Ultra development board supports maximum resolution of `1920 x 1080` and maximum frame rate of `60fps`.

**Parameters**  
- `obj`: The pointer to the initialized `DISPLAY` object.
- `width`: Display output resolution - width.
- `height`: Display output resolution - height.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_stop_display

**Function Declaration**  
`int32_t sp_stop_display(void *obj)`

**Description**  
Close display channel.

**Parameters**  
- `obj`: The pointer to the initialized `DISPLAY` object.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_display_set_image

**Function Declaration**  
`int32_t sp_display_set_image(void *obj, char *addr, int32_t size)`

**Description**  
Send one frame of image to display module. Only `NV12` format `YUV` images are supported.

**Parameters**  
- `obj`: The pointer to the initialized `DISPLAY` object.
- `addr`: Image data (only NV12 format supported)
- `size`: Image data size, calculated as: width * height * 3 / 2

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_display_draw_rect

**Function Declaration**  
`int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t flush, int32_t color, int32_t line_width)`

**Description**  
Draw rectangle on the graphics layer of the display module.

**Parameters**  
- `obj`: The pointer to the initialized `DISPLAY` object
- `x0`: X-coordinate of first point of rectangle
- `y0`: Y-coordinate of first point of rectangle
- `x1`: X-coordinate of second point of rectangle
- `y1`: Y-coordinate of second point of rectangle
- `flush`: Whether to clear current graphics layer buffer
- `color`: Rectangle color (ARGB8888 format)
- `line_width`: Rectangle line width

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_display_draw_string

**Function Declaration**  
`int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t flush, int32_t color, int32_t line_width)`

**Description**  
Draw string on the graphics layer of the display module.

**Parameters**  
- `obj`: The pointer to the initialized `DISPLAY` object
- `x`: X-coordinate of starting point for string drawing
- `y`: Y-coordinate of starting point for string drawing
- `str`: String to draw (must be GB2312 encoded)
- `flush`: Whether to clear current graphics layer buffer
- `color`: Text color (ARGB8888 format)
- `line_width`: Text line width

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_get_display_resolution

**Function Declaration**  
`void sp_get_display_resolution(int32_t *width, int32_t *height)`

**Description**  
Get resolution of currently connected display.

**Parameters**  
- `width`: Resolution width to obtain
- `height`: Resolution height to obtain

**Return Type**  
None  

:::note

Currently only 1920x1080@60Fps format is supported

:::