---
sidebar_position: 2
---

# ENCODER API

The `ENCODER` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_init_encoder_module | **Initialize the encoder module object** |
| sp_release_encoder_module | **Destroy the encoder module object** |
| sp_start_encode | **Create an image encoding channel** |
| sp_stop_encode | **Close an image encoding channel** |
| sp_encoder_set_frame | **Feed an image frame into the encoding channel** |
| sp_encoder_get_stream | **Retrieve the encoded bitstream from the encoding channel** |

## sp_init_encoder_module  

**[Function Prototype]**  

`void *sp_init_encoder_module()`

**[Description]**  

Initializes the encoder module object. This function must be called to obtain a handle before using the encoder module.

**[Parameters]**

None

**[Return Type]**  

Returns a pointer to an `ENCODER` object on success, or `NULL` on failure.

## sp_release_encoder_module  

**[Function Prototype]**  

`void sp_release_encoder_module(void *obj)`

**[Description]**  

Destroys the encoder module object.

**[Parameters]**

- `obj`: Pointer to the object obtained from the initialization function.

**[Return Type]**  

None

## sp_start_encode  

**[Function Prototype]**  

`int32_t sp_start_encode(void *obj, int32_t chn, int32_t type, int32_t width, int32_t height, int32_t bits)`

**[Description]**  

Creates an image encoding channel. Supports up to 32 concurrent encoding channels. Supported encoding types include `H264`, `H265`, and `MJPEG`.

**[Parameters]**

- `obj`: Pointer to an initialized `ENCODER` object.
- `chn`: Encoding channel number to create, supporting values from 0 to 31.
- `type`: Image encoding type. Supported values are `SP_ENCODER_H264`, `SP_ENCODER_H265`, and `SP_ENCODER_MJPEG`.
- `width`: Width of the input image resolution for the encoding channel.
- `height`: Height of the input image resolution for the encoding channel.
- `bits`: Encoding bitrate. Common values include 512, 1024, 2048, 4096, 8192, 16384 (unit: kbps). Other values are also acceptable. Higher bitrates yield clearer images with lower compression ratios and larger bitstream sizes.

**[Return Type]**  

Returns 0 on success, -1 on failure.

## sp_stop_encode  

**[Function Prototype]**  

`int32_t sp_stop_encode(void *obj)`

**[Description]**  

Closes an opened encoding channel.

**[Parameters]**

- `obj`: Pointer to an initialized `ENCODER` object.

**[Return Type]**  

Returns 0 on success, -1 on failure.

## sp_encoder_set_frame  

**[Function Prototype]**  

`int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)`

**[Description]**  

Feeds an image frame into the encoding channel for encoding. The input frame must be in `NV12` format.

**[Parameters]**

- `obj`: Pointer to an initialized `ENCODER` object.
- `frame_buffer`: Image frame data to be encoded. Must be in `NV12` format and match the resolution specified when calling `sp_start_encode`.
- `size`: Size of the image frame data. For `NV12` format, the size is calculated as `(width * height * 3) / 2`.

**[Return Type]**  

Returns 0 on success, -1 on failure.

## sp_encoder_get_stream  

**[Function Prototype]**  

`int32_t sp_encoder_get_stream(void *obj, char *stream_buffer)`

**[Description]**  

Retrieves the encoded bitstream data from the encoding channel.

**[Parameters]**

- `obj`: Pointer to an initialized `ENCODER` object.
- `stream_buffer`: Upon successful retrieval, the encoded bitstream data will be stored in this buffer. The buffer size should be adjusted according to the encoding resolution and bitrate.

**[Return Type]**  

Returns the size of the bitstream data on success, -1 on failure.