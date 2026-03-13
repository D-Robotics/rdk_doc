---
sidebar_position: 2
---
# ENCODER (Encoder Module) API

The `ENCODER` API provides the following interfaces:

| Function | Function Description |
| ---- | ----- |
| sp_init_encoder_module | **Initialize encoder module object** |
| sp_release_encoder_module | **Destroy encoder module object** |
| sp_start_encode | **Create image encoding channel** |
| sp_stop_encode | **Close image encoding channel** |
| sp_encoder_set_frame | **Pass image frames to encoding channel** |
| sp_encoder_get_stream | **Get encoded stream from encoding channel** |

## sp_init_encoder_module

**[Function Prototype]**

`void *sp_init_encoder_module()`

**[Function Description]**

Initialize the encoder module object. This function needs to be called to obtain a handle when using the encoder module.

**[Parameters]**

None

**[Return Type]**

Returns a pointer to the `ENCODER` object on success, `NULL` on failure.

## sp_release_encoder_module

**[Function Prototype]**

`void sp_release_encoder_module(void *obj)`

**[Function Description]**

Destroy the encoder module object.

**[Parameters]**

- `obj`: Pointer to the object obtained when initializing the interface.

**[Return Type]**

None

## sp_start_encode  

**[Function Prototype]**  

`int32_t sp_start_encode(void *obj, int32_t chn, int32_t type, int32_t width, int32_t height, int32_t bits)`

**[Function Description]**  

Create an image encoding channel, supports up to 32 channels, and supports encoding types of `H264`, `H265`, and `MJPEG`.

**[Parameters]**

- `obj`: Pointer to the initialized `ENCODER` object
- `chn`: Channel number to create, supports 0 ~ 31
- `type`: Image encoding type, supports `SP_ENCODER_H264`, `SP_ENCODER_H265`, and `SP_ENCODER_MJPEG`.
- `width`: Width of the image data resolution to be input to the encoding channel
- `height`: Height of the image data resolution to be input to the encoding channel
- `bits`: Encoding bitrate, commonly used values are 512, 1024, 2048, 4096, 8192, 16384, etc. (unit: Mbps), other values are also possible. The higher the bitrate, the clearer the encoded image, the smaller the compression ratio, and the larger the stream data.

**[Return Type]**  

Returns 0 on success, -1 on failure

## sp_stop_encode  

**[Function Prototype]**  

`int32_t sp_stop_encode(void *obj)`

**[Function Description]**  

Close the opened encoding channel.

**[Parameters]**

- `obj`: Pointer to the initialized `ENCODER` object

**[Return Type]** 

Returns 0 on success, -1 on failure

## sp_encoder_set_frame  

**[Function Prototype]**  

`int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)`

**[Function Description]**

Pass the image frame data that needs to be encoded to the encoding channel, and the format must be `NV12`.

**[Parameters]**

- `obj`: Initialized `ENCODER` object pointer
- `frame_buffer`: Image frame data to be encoded, must be in `NV12` format, and the resolution must be consistent with the image frame resolution of the `sp_start_encode` interface call.
- `size`: Size of the image frame data, the calculation formula for the size of the `NV12` format image is `(width * height * 3) / 2`.

**[Return Type]**

Returns 0 for success, -1 for failure.

## sp_encoder_get_stream

**[Function Prototype]**

`int32_t sp_encoder_get_stream(void *obj, char *stream_buffer)`

**[Function Description]**

Get the encoded bitstream data from the encoding channel.

**[Parameters]**

- `obj`: Initialized `ENCODER` object pointer
- `stream_buffer`: After a successful call, the bitstream data will be stored in this buffer. The size of this buffer needs to be adjusted according to the encoding resolution and bitrate.

**[Return Type]**

Returns the size of the bitstream data for success, -1 for failure.