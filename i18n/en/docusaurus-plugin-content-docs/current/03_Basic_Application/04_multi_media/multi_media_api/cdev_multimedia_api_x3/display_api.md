---
sidebar_position: 4
---

# DISPLAY API

The `DISPLAY` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_init_display_module | **Initialize the display module object** |
| sp_release_display_module | **Destroy the display module object** |
| sp_start_display | **Create a video display channel** |
| sp_stop_display | **Close the video display channel** |
| sp_display_set_image | **Pass an image to the video display channel** |
| sp_display_draw_rect | **Draw a rectangle on the display channel** |
| sp_display_draw_string | **Draw a string on the display channel** |
| sp_get_display_resolution | **Get the resolution of the display** |

## sp_init_display_module

**【Function prototype】**

`void *sp_init_display_module()`

**【Description】**

Initialize the display module object. This module supports displaying video image data on a display connected to the `HDMI` interface, and provides functions to draw rectangles and text on the display.

**【Parameters】**

None

**【Return type】**

Returns a pointer to the `DISPLAY` object if successful, or NULL if failed.

## sp_release_display_module

**【Function prototype】**

`void sp_release_display_module(void *obj)`

**【Description】**

Destroy the `DISPLAY` object.

**【Parameters】**

- `obj`: Pointer to the initialized `DISPLAY` object.

**【Return Type】** 

None

## sp_start_display 

**【Function Prototype】** 

`int32_t sp_start_display(void *obj, int32_t chn, int32_t width, int32_t height)`

**【Function Description】** 

Create a display channel. The RDK X3 development board supports 4 channels, including 2 video layers and 2 graphic layers. The maximum supported resolution is `1920 x 1080` with a maximum frame rate of `60fps`.

**【Parameters】**

- `obj`: Initialized `DISPLAY` object pointer
- `chn`: Channel number, supports 0-3. If using a desktop system, channel 0 is used for the graphical system, so applications should use channel 1. Channels 2 and 3 are generally used to draw rectangles or overlay text information.
- `width`: Display output resolution - width
- `height`: Display output resolution - height

**【Return Type】** 

Successful: 0, Failure: -1

## sp_stop_display 

**【Function Prototype】** 

`int32_t sp_stop_display(void *obj)`

**【Function Description】** 

Stop the display channel.

**【Parameters】**

- `obj`: Initialized `DISPLAY` object pointer

**【Return Type】** 

Successful: 0, Failure: -1

## sp_display_set_image 

**【Function Prototype】** 

`int32_t sp_display_set_image(void *obj, char *addr, int32_t size, int32_t chn)`

**[Function Description]**

Display image data in `addr` on display channel `chn`. The image format only supports `NV12` YUV images.

**[Parameters]**

- `obj`: Initialized `DISPLAY` object pointer
- `addr`: Image data, image format only supports `NV12`
- `size`: Image data size, calculated by: width * height * 3 / 2
- `chn`: Display channel corresponding to the channel number used by the `sp_start_display` interface.

**[Return Type]**

Returns 0 on success, -1 on failure.

## sp_display_draw_rect

**[Function Prototype]**

`int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t chn, int32_t flush, int32_t color, int32_t line_width)`

**[Function Description]**

Draws a rectangle in the graphics layer of the display module.

**[Parameters]**

- `obj`: Initialized `DISPLAY` object pointer
- `x0`: x value of the first coordinate of the rectangle
- `y0`: y value of the first coordinate of the rectangle
- `x1`: x value of the second coordinate of the rectangle
- `y1`: y value of the second coordinate of the rectangle
- `chn`: display output layer, 2~3 for graphics layer
- `flush`: whether to clear the current graphic layer buffer
- `color`: color of the rectangle (color format is ARGB8888)
- `line_width`: line width of the rectangle

**[Return Type]**

Returns 0 on success, -1 on failure.

## sp_display_draw_string

**[Function Prototype]**

`int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t chn, int32_t flush, int32_t color, int32_t line_width)`

**[Function Description]**

Draws a string in the graphics layer of the display module.**【Parameters】**

- `obj`: Initialized `DISPLAY` object pointer
- `x`: x-value of the starting coordinate for drawing the string
- `y`: y-value of the starting coordinate for drawing the string
- `str`: String to be drawn (encoded in GB2312)
- `chn`: Display output layer, 2~3 for graphic layers
- `flush`: Whether to clear the current graphic layer buffer
- `color`: Color of the rectangle (color format in ARGB8888)
- `line_width`: Line width of the text

**【Return Type】** 

Returns 0 if successful, -1 if failed

## sp_get_display_resolution

**【Function Prototype】** 

`void sp_get_display_resolution(int32_t *width, int32_t *height)`

**【Description】** 

Gets the resolution of the connected display.

**【Parameters】**

- `width`: Width of the resolution to be obtained
- `height`: Height of the resolution to be obtained

**【Return Type】** 

None.