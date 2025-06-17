---
sidebar_position: 3
---
# DECODER API

The `DECODER` API provides the following interfaces:

| Function | Description |
| ---- | ----- |
| sp_init_decoder_module | **Initialize the decoder module object** |
| sp_release_decoder_module | **Destroy the decoder module object** |
| sp_start_decode | **Create an image decoding channel** |
| sp_stop_decode | **Close the image decoding channel** |
| sp_decoder_get_image | **Retrieve the decoded image frame from the decoding channel** |
| sp_decoder_set_image | **Pass the encoded stream data to the decoding channel** |

## sp_init_decoder_module  

**【Function Prototype】**  

`void *sp_init_decoder_module()`

**【Description】**  

Initialize the decoder module object. You need to call this function to obtain the handle when using the decoder module. It supports H264, H265, and Mjpeg formats of video streams.

**【Parameters】**

None.

**【Return Type】** 

If successful, it returns a `DECODER` object. If failed, it returns NULL.

## sp_release_decoder_module  

**【Function Prototype】**  

`void sp_release_decoder_module(void *obj)`

**【Description】**  

Destroy the decoder module object.

**【Parameters】**

- `obj`: The object pointer obtained when calling the initialization interface.

**【Return Type】**

None

## sp_start_decode  

**【Function Prototype】**  

`int32_t sp_start_decode(void *obj, const char *stream_file, int32_t video_chn, int32_t type, int32_t width, int32_t height)`

**【Description】**  

Create a decoding channel and set the channel number, the type of the decoding stream, and the resolution of the image frame.

**【Parameters】**

- `obj`: Pointer to the initialized `DECODER` object
- `stream_file`: When `stream_file` is set to the name of a stream file, it means that the stream file needs to be decoded, for example, setting the stream file of H264 as "stream.h264". When `stream_file` is set to an empty string, it means that the decoding data stream needs to be passed in by calling `sp_decoder_set_image`.
- `video_chn`: Decoding channel number, supports 0-31.
- `type`: Type of the decoding data, supports `SP_ENCODER_H264`, `SP_ENCODER_H265`, and `SP_ENCODER_MJPEG`.
- `width`: Width of the decoded image frame resolution
- `height`: Height of the decoded image frame resolution

**【Return Type】** 

Returns 0 if successful, -1 if failure.

## sp_stop_decode  

**【Function Prototype】**  

`int32_t sp_stop_decode(void *obj)`

**【Description】**  

Close the decoding channel.

**【Parameters】**

- `obj`: Pointer to the initialized `DECODER` object

**【Return Type】** 

Returns 0 if successful, -1 if failure.

## sp_decoder_get_image  

**【Function Prototype】**  

`int32_t sp_decoder_get_image(void *obj, char *image_buffer)`

**[Function Description]**

Obtain the decoded image frame data from the decoding channel, and the returned image data format is a `YUV` image in `NV12` format.

**[Parameters]**

- `obj`: Pointer to the initialized `DECODER` object.
- `image_buffer`: The returned image frame data. The size of this buffer is determined by the resolution of the image, which is `(width * height * 3) / 2`.

**[Return Type]**

Returns 0 if successful, and -1 if failed.

## sp_decoder_set_image

**[Function Prototype]**

`int32_t sp_decoder_set_image(void *obj, char *image_buffer, int32_t chn, int32_t size, int32_t eos)`

**[Function Description]**

Feeds the bitstream data into an opened decoding channel. If decoding H264 or H265 bitstream, it is required to send 3-5 frames of data first to allow the decoder to complete frame buffering before obtaining the decoded frame data. If decoding H264 bitstream, the first frame of data sent for decoding needs to be the description of sps and pps, otherwise the decoder will report an error and exit.

**[Parameters]**

- `obj`: Pointer to the initialized `DECODER` object.
- `image_buffer`: Pointer to the bitstream data.
- `chn`: Decoder channel number, which needs to be the channel number that has been opened by calling `sp_start_decode`.
- `size`: Size of the bitstream data.
- `eos`: Whether it is the last frame of data.

**[Return Type]**

Returns 0 if successful, and -1 if failed.