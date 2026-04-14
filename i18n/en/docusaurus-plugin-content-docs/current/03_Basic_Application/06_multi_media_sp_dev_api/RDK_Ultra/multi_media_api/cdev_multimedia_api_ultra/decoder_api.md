---
sidebar_position: 3
---
# DECODER（Decoding Module）API

The `DECODER` module provides interfaces for handling video decoding tasks. The available functions are as follows:

| Function                     | Description                                               |
|------------------------------|-----------------------------------------------------------|
| sp_init_decoder_module        | **Initialize the decoder module object**                  |
| sp_release_decoder_module     | **Destroy the decoder module object**                     |
| sp_start_decode               | **Create a video decoding channel**                       |
| sp_stop_decode                | **Close the video decoding channel**                      |
| sp_decoder_get_image          | **Retrieve decoded image frame from the decoding channel**|
| sp_decoder_set_image          | **Send encoded stream data to the decoding channel**      |

> **Note**: RDK Ultra **does not support H264 encoding/decoding**.

## sp_init_decoder_module

**Function Declaration**  
`void *sp_init_decoder_module()`

**Description**  
Initializes the decoder module and creates an operation handle. This function must be called before using the decoding module. The module supports decoding video streams in H264, H265, and MJPEG formats.

**Parameters**  
None

**Return Type**  
- Returns a pointer to the `DECODER` object on success.  
- Returns `NULL` on failure.

## sp_release_decoder_module

**Function Declaration**  
`void sp_release_decoder_module(void *obj)`

**Description**  
Releases the decoder module object.

**Parameters**  
- `obj`: The pointer to the `DECODER` object obtained from initialization.

**Return Type**  
None

## sp_start_decode

**Function Declaration**  
`int sp_start_decode(void *decoder_object, const char *stream_file, int32_t type, int32_t width, int32_t height)`

**Description**  
Creates a decoding channel and sets the stream type and image frame resolution for decoding.

**Parameters**  
- `obj`: The pointer to the initialized `DECODER` object.
- `stream_file`: If provided as a stream file (e.g., `stream.h265`), it will decode the stream from this file. If an empty string is provided, the stream data will be passed through `sp_decoder_set_image`.
- `type`: The stream type to be decoded, which can be `SP_ENCODER_H265` or `SP_ENCODER_MJPEG`.
- `width`: The decoded image width.
- `height`: The decoded image height.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_stop_decode

**Function Declaration**  
`int32_t sp_stop_decode(void *obj)`

**Description**  
Closes the decoding channel.

**Parameters**  
- `obj`: The pointer to the initialized `DECODER` object.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_decoder_get_image

**Function Declaration**  
`int32_t sp_decoder_get_image(void *obj, char *image_buffer)`

**Description**  
Retrieves the decoded image frame data from the decoding channel. The image is returned in `NV12` YUV format.

**Parameters**  
- `obj`: The pointer to the initialized `DECODER` object.
- `image_buffer`: The buffer where the decoded image frame will be stored. The buffer size is `width * height * 3 / 2` for `NV12` format.

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.

## sp_decoder_set_image

**Function Declaration**  
`int32_t sp_decoder_set_image(void *obj, char *image_buffer, int32_t size, int32_t eos)`

**Description**  
Sends stream data to an already opened decoding channel.  
- For H264 or H265 decoding, send 3–5 frames initially to allow the decoder to cache the frames before retrieving decoded frames.
- For H264, the first frame should contain the SPS and PPS descriptors; otherwise, the decoder will return an error and stop.

**Parameters**  
- `obj`: The pointer to the initialized `DECODER` object.
- `image_buffer`: The pointer to the stream data.
- `size`: The size of the stream data.
- `eos`: A flag indicating whether this is the last frame (`1` for last frame, `0` otherwise).

**Return Type**  
- Returns `0` on success.  
- Returns `-1` on failure.
