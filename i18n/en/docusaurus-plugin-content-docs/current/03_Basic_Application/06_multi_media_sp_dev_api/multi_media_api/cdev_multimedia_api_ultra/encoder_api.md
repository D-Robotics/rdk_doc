---
sidebar_position: 2
---
# ENCODER（Encoding Module）API

The `ENCODER` module provides interfaces for handling video encoding tasks. The available functions are as follows:

| Function                   | Description                                                |
|----------------------------|------------------------------------------------------------|
| sp_init_encoder_module      | **Initialize the encoder module object**                   |
| sp_release_encoder_module   | **Destroy the encoder module object**                      |
| sp_start_encode             | **Create a video encoding channel**                        |
| sp_stop_encode              | **Close the video encoding channel**                       |
| sp_encoder_set_frame        | **Send image frames to the encoding channel**              |
| sp_encoder_get_stream       | **Retrieve encoded stream data from the encoding channel** |

> **Note**: RDK Ultra **does not support H264 encoding/decoding**.

## sp_init_encoder_module

**Function Declaration**  
`void *sp_init_encoder_module()`

**Description**  
Initializes the encoder module and creates an operation handle. This function must be called before using the encoding module.

**Parameters**  
None

**Return Type**  
- Returns a pointer to the `ENCODER` object on success.  
- Returns `NULL` on failure.

## sp_release_encoder_module

**Function Declaration**  
`void sp_release_encoder_module(void *obj)`

**Description**  
Releases the encoder module object.

**Parameters**  
- `obj`: The pointer to the `ENCODER` object obtained from initialization.

**Return Type**  
None

## sp_start_encode

**Function Declaration**  
`int32_t sp_start_encode(void *obj, int32_t type, int32_t width, int32_t height, int32_t bits)`

**Description**  
Creates a video encoding channel. It supports up to `32` encoding channels, with encoding types including `H264`, `H265`, and `MJPEG`.

**Parameters**  
- `obj`: The pointer to the initialized `ENCODER` object.
- `type`: The encoding type, which can be `SP_ENCODER_H264`, `SP_ENCODER_H265`, or `SP_ENCODER_MJPEG`.
- `width`: The input image width for encoding.
- `height`: The input image height for encoding.
- `bits`: The encoding bitrate in Mbps (common values: 512, 1024, 2048, 4096, 8192, 16384, etc.).

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_stop_encode

**Function Declaration**  
`int32_t sp_stop_encode(void *obj)`

**Description**  
Closes the opened encoding channel.

**Parameters**  
- `obj`: The pointer to the initialized `ENCODER` object.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_encoder_set_frame

**Function Declaration**  
`int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)`

**Description**  
Sends image frame data to the encoding channel. The frame data must be in the `NV12` format.

**Parameters**  
- `obj`: The pointer to the initialized `ENCODER` object.
- `frame_buffer`: The image frame data to be encoded, which must be in the `NV12` format. The resolution should match the one specified in `sp_start_encode`.
- `size`: The size of the image frame data, calculated as `width * height * 3 / 2` for `NV12` format.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_encoder_get_stream

**Function Declaration**  
`int32_t sp_encoder_get_stream(void *obj, char *stream_buffer)`

**Description**  
Retrieves the encoded stream data from the encoding channel.

**Parameters**  
- `obj`: The pointer to the initialized `ENCODER` object.
- `stream_buffer`: The buffer where the encoded stream data will be stored. The size of this buffer must be adjusted based on the encoding resolution and bitrate.

**Return Type**  
- Returns the size of the encoded stream data on success.  
- Returns `-1` on failure.
