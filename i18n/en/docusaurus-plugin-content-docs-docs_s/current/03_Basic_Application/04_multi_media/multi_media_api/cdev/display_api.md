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
| sp_display_set_image | **Send image data to the video display channel** |
| sp_display_draw_rect | **Draw a rectangle on the display channel** |
| sp_display_draw_string | **Draw a string on the display channel** |
| sp_get_display_resolution | **Get the resolution of the connected display** |

## sp_init_display_module  

**[Function Prototype]**  

`void *sp_init_display_module()`

**[Description]**  

Initializes the display module object. This module supports displaying video image data on an HDMI-connected monitor and provides functionality to draw rectangles and text overlays on the displayed image.

**[Parameters]**

None

**[Return Type]** 

Returns a pointer to the `DISPLAY` object on success; returns NULL on failure.

## sp_release_display_module  

**[Function Prototype]**  

`void sp_release_display_module(void *obj)`

**[Description]**  

Destroys the `DISPLAY` object.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object

**[Return Type]** 

None

## sp_start_display  

**[Function Prototype]**  

`int32_t sp_start_display(void *obj, int32_t chn, int32_t width, int32_t height)`

**[Description]**  

Creates a display channel. The RDK S100 development board supports 4 channels: 2 video layers and 2 graphics layers. The maximum supported resolution is `1920 x 1080` at up to `60fps`.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object  
- `chn`: Channel number (0–3). If using a desktop system, channel 0 is reserved for the graphical system, so applications should use channel 1. Channels 2 and 3 are typically used for drawing rectangles or overlaying text.  
- `width`: Display output resolution – width  
- `height`: Display output resolution – height  

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_stop_display  

**[Function Prototype]**  

`int32_t sp_stop_display(void *obj)`

**[Description]**  

Closes the display channel.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_display_set_image  

**[Function Prototype]**  

`int32_t sp_display_set_image(void *obj, char *addr, int32_t size, int32_t chn)`

**[Description]**  

Displays the image data located at `addr` on display channel `chn`. Only `NV12` format `YUV` images are supported.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object  
- `addr`: Image data buffer (only `NV12` format is supported)  
- `size`: Image data size, calculated as: width * height * 3 / 2  
- `chn`: Display channel, corresponding to the channel number used in `sp_start_display`  

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_display_draw_rect  

**[Function Prototype]**  

`int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t chn, int32_t flush, int32_t color, int32_t line_width)`

**[Description]**  

Draws a rectangle on the graphics layer of the display module.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object  
- `x0`: X-coordinate of the rectangle's first corner  
- `y0`: Y-coordinate of the rectangle's first corner  
- `x1`: X-coordinate of the rectangle's second corner  
- `y1`: Y-coordinate of the rectangle's second corner  
- `chn`: Display output layer; layers 2–3 are graphics layers  
- `flush`: Whether to clear the current graphics layer buffer  
- `color`: Rectangle color (in ARGB8888 format)  
- `line_width`: Line width of the rectangle  

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_display_draw_string  

**[Function Prototype]**  

`int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t chn, int32_t flush, int32_t color, int32_t line_width)`

**[Description]**  

Draws a string on the graphics layer of the display module.

**[Parameters]**

- `obj`: Pointer to an initialized `DISPLAY` object  
- `x`: X-coordinate of the string's starting position  
- `y`: Y-coordinate of the string's starting position  
- `str`: String to be drawn (must be encoded in GB2312)  
- `chn`: Display output layer; layers 2–3 are graphics layers  
- `flush`: Whether to clear the current graphics layer buffer  
- `color`: Text color (in ARGB8888 format)  
- `line_width`: Line width of the text  

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_get_display_resolution  

**[Function Prototype]**  

`void sp_get_display_resolution(int32_t *width, int32_t *height)`

**[Description]**  

Retrieves the resolution of the currently connected display.

**[Parameters]**

- `width`: Output parameter for the display width  
- `height`: Output parameter for the display height  

**[Return Type]** 

None.