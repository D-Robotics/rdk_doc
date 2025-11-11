---
sidebar_position: 3
---

# DECODER API

The `DECODER` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_init_decoder_module | **Initialize decoder module object** |
| sp_release_decoder_module | **Destroy decoder module object** |
| sp_start_decode | **Create an image decoding channel** |
| sp_stop_decode | **Close an image decoding channel** |
| sp_decoder_get_image | **Retrieve a decoded image frame from the decoding channel** |
| sp_decoder_set_image | **Feed encoded stream data into the decoding channel** |

## sp_init_decoder_module  

**[Function Prototype]**  

`void *sp_init_decoder_module()`

**[Description]**  

Initializes a decoder module object. This function must be called to obtain a handle before using the decoder module. It supports video streams in H264, H265, and Mjpeg formats.

**[Parameters]**

None.

**[Return Type]** 

Returns a `DECODER` object on success; returns NULL on failure.

## sp_release_decoder_module  

**[Function Prototype]**  

`void sp_release_decoder_module(void *obj)`

**[Description]**  

Destroys the decoder module object.

**[Parameters]**

 - `obj`: Pointer to the object obtained from the initialization function.

**[Return Type]**  

None.

## sp_start_decode  

**[Function Prototype]**  

`int32_t sp_start_decode(void *obj, const char *stream_file, int32_t video_chn, int32_t type, int32_t width, int32_t height)`

**[Description]**  

Creates a decoding channel and configures its channel ID, stream type to decode, and output image resolution.

**[Parameters]**

- `obj`: Pointer to an initialized `DECODER` object.
- `stream_file`: When set to a valid stream filename (e.g., "stream.h264" for an H264 stream), the decoder will decode this file. If an empty string is passed, the stream data must be provided later via `sp_decoder_set_image`.
- `video_chn`: Decoding channel number, supporting values from 0 to 31.
- `type`: Stream type to decode. Supported values: `SP_ENCODER_H264`, `SP_ENCODER_H265`, and `SP_ENCODER_MJPEG`.
- `width`: Width of the decoded image frame.
- `height`: Height of the decoded image frame.

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_stop_decode  

**[Function Prototype]**  

`int32_t sp_stop_decode(void *obj)`

**[Description]**  

Closes the decoding channel.

**[Parameters]**

- `obj`: Pointer to an initialized `DECODER` object.

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_decoder_get_image  

**[Function Prototype]**  

`int32_t sp_decoder_get_image(void *obj, char *image_buffer)`

**[Description]**  

Retrieves decoded image frame data from the decoding channel. The returned image data is in `NV12` format (`YUV`).

**[Parameters]**

- `obj`: Pointer to an initialized `DECODER` object.
- `image_buffer`: Buffer to store the returned image frame data. The required buffer size is `(width * height * 3) / 2`.

**[Return Type]** 

Returns 0 on success; returns -1 on failure.

## sp_decoder_set_image  

**[Function Prototype]**  

`int32_t sp_decoder_set_image(void *obj, char *image_buffer, int32_t chn, int32_t size, int32_t eos)`

**[Description]**  

Feeds encoded stream data into an opened decoding channel.  
For H264 or H265 streams, you must first send 3–5 frames to allow the decoder to fill its internal frame buffers before retrieving decoded frames.  
For H264 streams, the first frame sent must contain SPS and PPS header information; otherwise, the decoder will report an error and exit.

**[Parameters]**

- `obj`: Pointer to an initialized `DECODER` object.
- `image_buffer`: Pointer to the encoded stream data.
- `chn`: Decoder channel number, which must correspond to a channel previously opened via `sp_start_decode`.
- `size`: Size of the stream data.
- `eos`: Indicates whether this is the last frame of data.

**[Return Type]** 

Returns 0 on success; returns -1 on failure.