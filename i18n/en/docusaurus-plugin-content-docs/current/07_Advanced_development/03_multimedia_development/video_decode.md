---
sidebar_position: 10
---
# 7.3.10 Video Decoding
## Overview
The video decoding module supports hardware decoding of H.264/H.265/JPEG/MJPEG. This module supports multi-channel real-time encoding, with each channel being independent. Common use cases include smart boxes and classroom recording.

## Function Description

### Basic Specifications

The decoding specifications supported by X3 are as follows:

| Hardware Decoding Module | Maximum Number of Channels | Supported Protocols                       | Resolution Support                                           | Maximum Performance                                          |
| ----------------------- | ------------------------- | ----------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| VPU<br/>JPU              | 32                        | **VPU:**<br/>H.264/H.265<br/>**JPU:**<br/>JPEG/MJPEG | **VPU:**<br/>- H264:<br/>max 8192×8192<br/>min：32×32<br/>- H265:<br/>max 8192×8192<br/>Min: 8×8<br/>**JPU:**<br/>- JPEG/MJPEG:<br/>max 32768×32768<br/>min:16×16 | **H264/H265:**<br/>3840×2160 @60fps<br/>**JPEG/MJPEG:**<br/>YUV4:2:0 290M pixel/sec<br/> |

### Bitstream Transmission Mode

The X3 video decoder supports the frame-by-frame transmission (VIDEO_MODE_FRAME) mode of the bitstream:

Each time the user sends a complete frame of the bitstream to the decoder, the decoder considers that the frame of the bitstream has ended and starts decoding the image. Therefore, it is necessary to ensure that each time the send interface is called, the bitstream sent must be a complete frame, otherwise decoding errors may occur.

The bitstream transmission mode enMode is defined in the decoding channel attribute structure VDEC_CHN_ATTR_S. Users can use the HB_VDEC_SetChnAttr() function to configure the decoding channel attribute.

### Image Output Mode

According to the H.264/H.265 protocol, the decoded image may not be output immediately after decoding. The X3 video decoder can achieve faster output by setting different image output modes. The image output modes include the following:

- Decode Order: The decoded image is output in the order of decoding.
- Display Order: The decoded image is output according to the H.264/H.265 protocol.

According to the H.264/H.265 protocol, the decoding order of the video may not be the same as the output order (i.e. the display order) of the video. For example, when decoding B frames, the previous and subsequent P frames are required as parameters, so the P frames after the B frames are decoded before the B frames, but the B frames are output before the P frames. Outputting in decode order is a necessary condition for rapid output. If the user selects output in decode order, it is necessary to ensure that the decoding order of the bitstream is the same as the display order.

- Combining the frame-by-frame transmission mode with the output in decode order can achieve rapid decoding and output. The user must ensure that each time a complete frame of the bitstream is sent, the decoding order of the bitstream is the same as the display order.

- Combining the frame-by-frame transmission mode with the output in display order, it is important to set the stream_end flag to HB_TRUE when setting the end of the bitstream for the last frame; otherwise, the current frame is considered incomplete and decoding exceptions may occur.

The image output mode enOutPutOrder is defined in the decoding channel attribute structure VDEC_CHN_ATTR_S. Users can use the HB_VDEC_SetChnAttr() function to configure the decoding channel attribute.

### Timestamp (PTS) Processing
When the VDEC module sends the bitstream in frame-sending (VIDEO_MODE_FRAME) mode, the timestamp (PTS) of the decoded output image is the PTS provided by the user in the send bitstream interface (HB_VDEC_SendStream). The decoder does not modify this value.

### Bitstream Buffer Configuration Mode
The decoding bitstream buffer configuration supports external mode and internal mode.

- External mode: The user initializes the Video Pool pool by calling the HB_VP_Init() function, and then can choose the public pool or create a private pool of mmz memory using the HB_SYS_Alloc() function to store the bitstreams to be decoded. The number of ion memory buffers created by the user is recommended to be consistent with the number of stream buffers set in the decoding channel (u32StreamBufCnt). Each time a decoding is performed, the virtual address of the allocated ion memory buffer is assigned to the vir_ptr field of the VIDEO_STREAM_S structure, and the size of the stream to be decoded is assigned to the size field of the VIDEO_STREAM_S structure. The decoder actually decodes the bitstreams by rotating the buffers, that is, taking turns to use the buffer number of the bitstream. If the image output mode is set to display order, all frames within a GOP need to be read before decoding can start.

  Set the bExternalBitStreamBuff field of the VDEC_CHN_ATTR_S structure in the decoder channel attribute to HB_TRUE to use the external buffer mode.- Internal mode: Users can use other tools such as FFMPEG to split the bitstream (usually read in frame mode). In this case, users do not need to apply for VB cache. They only need to pass the address of the buffer after the bitstream is split to the vir_ptr field of the VIDEO_STREAM_S structure. The system will automatically copy it to the address space of the stream buffer allocated by the encoder, saving the use of VB.

  Use external buffer mode by setting the bExternalBitStreamBuff field in the VDEC_CHN_ATTR_S structure of the decoder channel attribute to HB_FALSE.

### Advanced skip frame decoding
Users can control whether to use skip frame decoding by setting the enDecMode field in the VDEC_CHN_ATTR_S structure of the decoder channel attributes (default is not enabled, i.e. all frames are decoded). It is possible to choose to decode only IRAP frames or only parameter frames, please refer to frame skip settings for more details.

### Bandwidth optimization mode decoding
Users can control whether to use bandwidth optimization during decoding by setting the bandwidth_Opt field in the VDEC_CHN_ATTR_S structure of the decoder channel attributes (default is enabled).
This mode supports VPU ignoring the compressed non-referenced frames or linear format non-display frames written into the frame buffer, thus saving bandwidth.

### Decoder binding
After the decoder is bound, HB_VDEC_GetFrame cannot be called again.

### Notes

- When decoding H.264/H.265, the first frame must provide sps, pps, idr. If only sps is provided, an error will occur, and the error message is `FAILED TO DEC_PIC_HDR: ret(1), SEQERR(00005000)`.

## API Reference
```C
HB_VDEC_CreateChn: Create a video decoding channel.
HB_VDEC_DestroyChn: Destroy a video decoding channel.
HB_VDEC_StartRecvStream: The decoder starts receiving bitstream sent by the user.
HB_VDEC_StopRecvStream: The decoder stops receiving bitstream sent by the user.
HB_VDEC_ResetChn: Reset the decoding channel.
HB_VDEC_SendStream: Send bitstream data to the video decoding channel.
HB_VDEC_GetFrame: Get the decoded image of the video decoding channel.
HB_VDEC_ReleaseFrame: Release the decoded image of the video decoding channel.
HB_VDEC_GetFd: Get the device file handle of the video decoding channel.
HB_VDEC_CloseFd: Close the device file handle of the video decoding channel.
HB_VDEC_SetChnAttr: Set the decoding channel parameters.
HB_VDEC_GetChnAttr: Get the decoding channel parameters.
HB_VDEC_QueryStatus: Query the decoding status.
HB_VDEC_GetUserData: Get the user data.
HB_VDEC_ReleaseUserData: Release the user data.
```

### HB_VDEC_CreateChn
[Function declaration]
```C
int32_t HB_VDEC_CreateChn(VDEC_CHN VdChn, const VDEC_CHN_ATTR_S *pstAttr)
```
[Function description]
> Create a video decoding channel.

[Parameter description]

|    Parameter Name    |                   Description                   | Input/Output |
| :------------------: | :---------------------------------------------: | :----------: |
|  VdChn   | Channel number for encoding.<br/>Valid range: [0, VDEC_MAX_CHN_NUM). |   Input    |
| pstAttr  |                 Pointer to the channel attribute of the decoder.                 |   Input    |

【Return Value】

| Return Value |              Description |
| :----: | :-----------------|
|   0    |               Success |
|  Non-zero   | Failure, returns error code. |

【Notes】
> None

【Reference Code】
> Reference code for HB_VDEC_ResetChn.

### HB_VDEC_DestroyChn
【Function Declaration】
```C
int32_t HB_VDEC_DestroyChn(VDEC_CHN VdChn);
```
【Function Description】
> Destroy video decoding channel.

【Parameter Description】

| Parameter Name |                        Description                        | Input/Output |
| :------: | :------------------------------------------------: | :-------: |
|  VdChn   | Channel number for encoding.<br/>Valid range: [0, VDEC_MAX_CHN_NUM). |   Input    |

【Return Value】

| Return Value |              Description |
| :----: | :-----------------|
|   0    |               Success |
|  Non-zero   | Failure, returns error code. |

【Notes】
> None

【Reference Code】
> Reference code for HB_VDEC_ResetChn.



### HB_VDEC_StartRecvStream
**Function Declaration**
```C
int32_t HB_VDEC_StartRecvStream(VDEC_CHN VdChn);
```
**Function Description**
> Starts the decoder to receive user-sent streams.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn        | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**
> See HB_VDEC_ResetChn for a reference.

### HB_VDEC_StopRecvStream
**Function Declaration**
```C
int32_t HB_VDEC_StopRecvStream(VDEC_CHN VdChn);
```
**Function Description**
> Stops the decoder from receiving user-sent streams.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn        | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**
> See HB_VDEC_ResetChn for a reference.

### HB_VDEC_ResetChn
**Function Declaration**
```C
int32_t HB_VDEC_ResetChn(VDEC_CHN VdChn);
```
**Function Description**
> Resets the video decoding channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn        | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**
```C
VDEC_CHN VDEC_Chn = 0;
int32_t s32Ret = 0;
int32_t Width = 1920;
int32_t Height = 1080;
VDEC_CHN_ATTR_S m_VdecChnAttr;
memset(&m_VdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
m_VdecChnAttr.enType = PT_H264;
m_VdecChnAttr.enMode = VIDEO_MODE_FRAME;
m_VdecChnAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
m_VdecChnAttr.u32FrameBufCnt = 10;
m_VdecChnAttr.u32StreamBufCnt = 10;
m_VdecChnAttr.u32StreamBufSize = Width * Height * 1.5;
m_VdecChnAttr.bExternalBitStreamBuf = HB_TRUE;

if (m_VdecChnAttr.enType == PT_H265) {
    m_VdecChnAttr.stAttrH265.bandwidth_Opt = HB_TRUE;
    m_VdecChnAttr.stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
    m_VdecChnAttr.stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    m_VdecChnAttr.stAttrH265.cra_as_bla = HB_FALSE;
    m_VdecChnAttr.stAttrH265.dec_temporal_id_mode = 0;
    m_VdecChnAttr.stAttrH265.target_dec_temporal_id_plus1 = 2;
}
if (m_VdecChnAttr.enType == PT_H264) {
    m_VdecChnAttr.stAttrH264.bandwidth_Opt = HB_TRUE;
    m_VdecChnAttr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
    m_VdecChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
}

s32Ret = HB_VDEC_CreateChn(VDEC_Chn, &m_VdecChnAttr);
HB_VDEC_SetChnAttr(VDEC_Chn, &m_VdecChnAttr);
HB_VDEC_StartRecvStream(VDEC_Chn);
HB_VDEC_StopRecvStream(VDEC_Chn);
HB_VDEC_ResetChn(VDEC_Chn);
HB_VDEC_DestroyChn(VDEC_Chn);
```

### HB_VDEC_SendStream
**Function Declaration**
```C
int32_t HB_VDEC_SendStream(VDEC_CHN VdChn, const VIDEO_STREAM_S *pstStream, int32_t s32MilliSec);
```
**Function Description**
> Sends stream data to the video decoding channel.

**Parameter Descriptions**

| Parameter Name   | Description | Input/Output |
| :--------------: | :----------: | :---------: |
|    VdChn         | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |
|   pstStream     | Pointer to decoded stream data. |   Input    |
| s32MilliSec      | Timeout for sending the stream.<br/>Range: [-1, +∞)<br/>-1: Blocking.<br/>0: Non-blocking.<br/>Positive: Timeout in milliseconds. |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**

### HB_VDEC_GetFrame
**Function Declaration**
```C
int32_t HB_VDEC_GetFrame(VDEC_CHN VdChn, VIDEO_FRAME_S *pstFrameInfo, int32_t s32MilliSec);
```
**Function Description**
> Retrieves a decoded image from the video decoding channel.

**Parameter Descriptions**

| Parameter Name   | Description | Input/Output |
| :--------------: | :----------: | :---------: |
|    VdChn         | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |
| pstFrameInfo     | Pointer to the retrieved decoded image information. |   Input    |
| s32MilliSec      | Timeout for retrieving the frame.<br/>Range: [-1, +∞)<br/>-1: Blocking.<br/>0: Non-blocking.<br/>Positive: Timeout in milliseconds. |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**

### HB_VDEC_ReleaseFrame
**Function Declaration**
```C
int32_t HB_VDEC_ReleaseFrame(VDEC_CHN VdChn, const VIDEO_FRAME_S *pstFrameInfo);
```
**Function Description**
> Releases a decoded image from the video decoding channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    VdChn       | Video decoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |
| pstFrameInfo   | Pointer to the decoded image information. |   Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|      0     |   Success   |
| Non-zero   | Failure, returns an error code. |

**Notes**
> None

**Reference Code**



### HB_VDEC_GetFd
【Function Declaration】
```C
int32_t HB_VDEC_GetFd(VDEC_CHN VdChn, int32_t *fd);
```
【Function Description】
> Get the device file handle corresponding to the decoding channel.

【Parameter Description】

| Parameter Name |                        Description                        | Input/Output |
| :------: | :------------------------------------------------: | :-------: |
|  VdChn   | Encoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM) |   Input    |
|    fd    |               Returns the file handle of the encoding channel.               |   Output    |

【Return Value】

| Return Value |               Description |
| :----: | :-----------------|
|   0    |               Success |
|  Non-zero   | Failure, returns error code. |

【Notes】
> None

【Reference Code】
```C
    VDEC_CHN VDEC_Chn = 0;
    int32_t fd = 0;
    int32_t s32Ret = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;
    VDEC_CHN_ATTR_S m_VdecChnAttr ;
    memset(&m_VdecChnAttr , 0, sizeof(VDEC_CHN_ATTR_S));
    m_VdecChnAttr.enType = PT_H264;
    m_VdecChnAttr.enMode = VIDEO_MODE_FRAME;
    m_VdecChnAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
    m_VdecChnAttr.u32FrameBufCnt = 10;
    m_VdecChnAttr.u32StreamBufCnt = 10;
    m_VdecChnAttr.u32StreamBufSize = Width * Height * 1.5;
    m_VdecChnAttr.bExternalBitStreamBuf  = HB_TRUE;
    if (m_VdecChnAttr.enType == PT_H265) {
        m_VdecChnAttr.stAttrH265.bandwidth_Opt = HB_TRUE;
        m_VdecChnAttr.stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
        m_VdecChnAttr.stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
        m_VdecChnAttr.stAttrH265.cra_as_bla = HB_FALSE;
        m_VdecChnAttr.stAttrH265.dec_temporal_id_mode = 0;
        m_VdecChnAttr.stAttrH265.target_dec_temporal_id_plus1 = 2;
    }
    if (m_VdecChnAttr.enType == PT_H264) {
        m_VdecChnAttr.stAttrH264.bandwidth_Opt = HB_TRUE;
        m_VdecChnAttr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
        m_VdecChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    }

    s32Ret = HB_VDEC_CreateChn(VDEC_Chn, &m_VdecChnAttr);
    HB_VDEC_SetChnAttr(VDEC_Chn, &m_VdecChnAttr);
    HB_VDEC_GetFd(VDEC_Chn, &fd);
    HB_VDEC_CloseFd(VDEC_Chn, fd);
    s32Ret = HB_VDEC_DestroyChn(VDEC_Chn);
```

### HB_VDEC_CloseFd
【Function Declaration】
```C
int32_t HB_VDEC_CloseFd(VDEC_CHN VdChn, int32_t fd);
```
【Function Description】
> Close the device file handle corresponding to the decoding channel.

【Parameter Description】

| Parameter Name |                      Description                      | Input/Output |
| :------------: | :---------------------------------------------------: | :----------: |
|     VdChn      |         Encoding channel number.<br/>Range: [0, VDEC_MAX_CHN_NUM).        |    Input     |
|       fd       |                 Set the encoding channel file handle.                |    Input     |

【Return Value】

| Return Value |                         Description                          |
| :----------: | :----------------------------------------------------------: |
|       0      |                           Success                            |
|     Non-zero    |      Failed, returns error code.     |

【Precautions】
> None

【Reference Code】
> Reference code for HB_VDEC_GetFd



### HB_VDEC_GetChnAttr
**Function Declaration**
```C
int32_t HB_VDEC_GetChnAttr(VDEC_CHN VdChn, VDEC_CHN_ATTR_S *pstAttr);
```
**Function Description**
> Retrieves video decoding channel parameters.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn        | Channel ID. <br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input     |
| pstAttr        | Pointer to decoded channel attributes. |   Output    |

**Return Values**

| Return Value | Description |
| :---------: | :-----------|
|    0       | Success     |
| Non-zero    | Failure, returns an error code. |

**Note**
> None

**Reference Code**
```C
    VDEC_CHN VDEC_Chn = 0;
    int32_t s32Ret = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VDEC_CHN_ATTR_S m_VdecChnAttr;
    memset(&m_VdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
    m_VdecChnAttr.enType = PT_H264;
    m_VdecChnAttr.enMode = VIDEO_MODE_FRAME;
    m_VdecChnAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
    m_VdecChnAttr.u32FrameBufCnt = 10;
    m_VdecChnAttr.u32StreamBufCnt = 10;
    m_VdecChnAttr.u32StreamBufSize = Width * Height * 1.5;
    m_VdecChnAttr.bExternalBitStreamBuf = HB_TRUE;

    if (m_VdecChnAttr.enType == PT_H265) {
        m_VdecChnAttr.stAttrH265.bandwidth_Opt = HB_TRUE;
        m_VdecChnAttr.stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
        m_VdecChnAttr.stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
        m_VdecChnAttr.stAttrH265.cra_as_bla = HB_FALSE;
        m_VdecChnAttr.stAttrH265.dec_temporal_id_mode = 0;
        m_VdecChnAttr.stAttrH265.target_dec_temporal_id_plus1 = 2;
    }
    if (m_VdecChnAttr.enType == PT_H264) {
        m_VdecChnAttr.stAttrH264.bandwidth_Opt = HB_TRUE;
        m_VdecChnAttr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
        m_VdecChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    }

    s32Ret = HB_VDEC_CreateChn(VDEC_Chn, &m_VdecChnAttr);
    HB_VDEC_SetChnAttr(VDEC_Chn, &m_VdecChnAttr);
    HB_VDEC_GetChnAttr(VDEC_Chn, &VdecChnAttr);
    s32Ret = HB_VDEC_DestroyChn(VDEC_Chn);
```

### HB_VDEC_SetChnAttr
**Function Declaration**
```C
int32_t HB_VDEC_SetChnAttr(VDEC_CHN VdChn, const VDEC_CHN_ATTR_S *pstAttr);
```
**Function Description**
> Sets video decoding channel parameters.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn        | Channel ID. <br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input     |
| pstAttr        | Pointer to decoded channel attributes. |   Input    |

**Return Values**

| Return Value | Description |
| :---------: | :-----------|
|    0       | Success     |
| Non-zero    | Failure, returns an error code. |

**Note**
> None

**Reference Code**
> HB_VDEC_GetChnAttr reference code

### HB_VDEC_QueryStatus
**Function Declaration**
```C
int32_t HB_VDEC_QueryStatus(VDEC_CHN VdChn, VDEC_CHN_STATUS_S *pstStatus);
```
**Function Description**
> Queries the status of a decoding channel.

**Parameter Descriptions**

| Parameter Name  | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   VdChn         | Decoding channel ID. <br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input    |
| pstStatus      | Pointer to channel status. |   Input    |

**Return Values**

| Return Value | Description |
| :---------: | :-----------|
|    0       | Success     |
| Non-zero    | Failure, returns an error code. |

**Note**
> None

**Reference Code**

### HB_VDEC_GetUserData
**Function Declaration**
```C
int32_t HB_VDEC_GetUserData(VDEC_CHN VdChn, VDEC_USERDATA_S *pstUserData, int32_t s32MilliSec);
```
**Function Description**
> Retrieves video decoding channel user data.

**Parameter Descriptions**

| Parameter Name   | Description | Input/Output |
| :--------------: | :----------: | :---------: |
|    VdChn        | Channel ID. <br/>Range: [0, VDEC_MAX_CHN_NUM). |   Input     |
| pstUserData      | Pointer to user data. |   Input    |
| s32MilliSec     | Timeout in milliseconds. |   Input    |

**Return Values**

| Return Value | Description |
| :---------: | :-----------|
|    0       | Success     |
| Non-zero    | Failure, returns an error code. |

**Note**
> None

**Reference Code**



### HB_VDEC_ReleaseUserData
【Function Declaration】
```C
int32_t HB_VDEC_ReleaseUserData(VDEC_CHN VdChn, VDEC_USERDATA_S *pstUserData);
```
【Function Description】
> Release user data for decoding channel.

【Parameter Description】

|  Parameter Name |                  Description                 | Input/Output |
| :-------------: | :-----------------------------------------: | :----------: |
|     VdChn       | Encoding channel ID.<br/>Range: [0, VDEC_MAX_CHN_NUM). |    Input     |
|  pstUserData    |          Pointer to user data             |    Input     |

【Return Value】

| Return Value |          Description         |
| :----------: | :---------------------------: |
|      0       |             Success             |
|    Non-zero   | Failure, returning error code |

【Notes】
> None

【Reference Code】Please refer to the code of HB_VDEC_GetFd.

## Data Structures
### VIDEO_MODE_E
【Description】
> Defines the method of sending video stream.

【Structure Definition】
```C
typedef enum HB_VIDEO_MODE_E {
    VIDEO_MODE_FRAME = 1,
    VIDEO_MODE_BUTT
} VIDEO_MODE_E;
```
【Member Description】

|       Member      |            Meaning             |
| :---------------: | :----------------------------: |
| VIDEO_MODE_FRAME  | Send the video stream by frame. |

### VIDEO_OUTPUT_ORDER_E
【Description】
> Defines the enumeration of video decoding output order.

【Structure Definition】
```C
typedef enum HB_VIDEO_OUTPUT_ORDER_E {
    VIDEO_OUTPUT_ORDER_DISP = 0,
    VIDEO_OUTPUT_ORDER_DEC,
    VIDEO_OUTPUT_ORDER_BUTT
} VIDEO_OUTPUT_ORDER_E;
```
【Member Description】

|      Member     |   Meaning   |
| :-------------: | :---------: |
| VIDEO_OUTPUT_ORDER_DISP | Display order output. |
| VIDEO_OUTPUT_ORDER_DEC  | Decoding order output. |

### VIDEO_DEC_MODE_E
【Description】
> Defines the enumeration of video decoding modes.

【Structure Definition】
```C
typedef enum HB_VIDEO_DEC_MODE_E {
    VIDEO_DEC_MODE_NORMAL = 0,
    VIDEO_DEC_MODE_IRAP,
    VIDEO_DEC_MODE_REF,
    VIDEO_DEC_MODE_THUMB,
    VIDEO_DEC_MODE_BUTT 
} VIDEO_DEC_MODE_E;
```
[Member Description]

|       Member        |                  Meaning                  |
| :-----------------: | :---------------------------------------: |
| VIDEO_DEC_MODE_NORMAL |             Decode IPB frame              |
|  VIDEO_DEC_MODE_IRAP  |            Decode IRAP frame              |
|   VIDEO_DEC_MODE_REF   |         Decode reference frame           |
| VIDEO_DEC_MODE_THUMB | Decode IRAP frame without DPB |

### VDEC_ATTR_H264_S

[Description]
> Define H264 decoding parameters.

[Structure Definition]
```C
typedef struct HB_VDEC_ATTR_H264_S {
    VIDEO_DEC_MODE_E enDecMode;
    VIDEO_OUTPUT_ORDER_E enOutputOrder;
    HB_BOOL bandwidth_Opt;
} VDEC_ATTR_H264_S;
```

[Member Description]

|    Member    |                                                Meaning                                                 |
| :----------: | :---------------------------------------------------------------------------------------------------: |
|  enDecMode   |                        Decoding mode, normal decoding or skip frame mode decoding.                      |
| enOutputOrder |                     Decoding image output order, decoding sequence output or display sequence output.                      |
| bandwidth_Opt |   Enable bandwidth saving mode, this mode supports VPU to ignore compress format non-reference frame or linear format non-display frame written into frame buffer, in order to save bandwidth. |

### VDEC_ATTR_H265_S

[Description]
> Define H265 decoding parameters.

[Structure Definition]
```C
typedef struct HB_VDEC_ATTR_H265_S {
    VIDEO_DEC_MODE_E enDecMode;
    VIDEO_OUTPUT_ORDER_E enOutputOrder;
    HB_BOOLcra_as_bla;
    HB_BOOL bandwidth_Opt;
    uint32_t dec_temporal_id_mode;
    uint32_t target_dec_temporal_id_plus1;
} VDEC_ATTR_H265_S;
```

[Member Description]

|      Member       |                                                   Meaning                                                   |
|:--------------------------:|:-----------------------------------------------------------------------------------------------------------------------------------------:|
|          enDecMode         |                                                      Decoding mode, normal decoding or skip frame mode decoding.                                                      |
|        enOutputOrder       |                                                   Decoding image output order, decoding order output or display order output.                                                    |
|          cra_as_bla        |                                                                  Enable CRA as BLA processing                                                                   |
|        bandwidth_Opt       |   Enable bandwidth saving mode, this mode supports VPU to ignore writing compressed format non-reference frames or linear format non-display frames into the frame buffer, in order to save bandwidth.   |
|     dec_temporal_id_mode   |                                                         Specify the selection mode of temporal id. 0 is absolute value mode, 1 is relative value mode.                                                         |
| target_dec_temporal_id_plus1 | When the value is 0x0, tempral ID in any range will decode the image<br/>When the value is within the range of [0x1~0x6], if tempral ID is less than or equal to TARGET_DEC_TEMP_ID, the image will be decoded. |

### VDEC_ATTR_MJPEG_S
[Description]
```C
Define MJPEG decoding parameters.
```
[Structure definition]
```C
typedef struct HB_VDEC_ATTR_MJPEG_S {
    CODEC_ROTATION_E enRotation;
    MIRROR_FLIP_E enMirrorFlip;
    VIDEO_CROP_INFO_S stCropCfg;
} VDEC_ATTR_MJPEG_S;
```
[Member description]

|     Member      |               Meaning               |
| :-------------: | :---------------------------------: |
|  enRotation     | Specify the rotation angle, including 0, 90, 180, 270 |
| enMirrorFlip;   |           Specify the mirror mode           |
|   stCropCfg     |           Specify the decoding area           |

### VDEC_ATTR_JPEG_S
[Description]
> Define JPEG decoding parameters.
[Structure definition]
```C
typedef struct HB_VDEC_ATTR_JPEG_S {
    CODEC_ROTATION_E enRotation;
    MIRROR_FLIP_E enMirrorFlip;
    VIDEO_CROP_INFO_S stCropCfg;
} VDEC_ATTR_JPEG_S;
```
[Member description]

|     Member      |               Meaning               |
| :-------------: | :---------------------------------: |
|  enRotation     | Specify the rotation angle, including 0, 90, 180, 270 |
| enMirrorFlip;   |           Specify the mirror mode           |
|   stCropCfg     |           Specify the decoding area           |

### VDEC_CHN_ATTR_S
[Description]> Definition of decoding channel attributes.

【Structure Definition】
```C
typedef struct HB_VDEC_CHN_ATTR_S {
    PAYLOAD_TYPE_E enType;
    VIDEO_MODE_E enMode;
    PIXEL_FORMAT_E enPixelFormat;
    uint32_t u32StreamBufSize;
    uint32_t u32StreamBufCnt;
    HB_BOOL bExternalBitStreamBuf；
    uint32_t u32FrameBufCnt;
    uint32_t vlc_buf_size;
    union {
        VDEC_ATTR_H264_S stAttrH264;
        VDEC_ATTR_H265_S stAttrH265;
        VDEC_ATTR_MJPEG_S stAttrMjpeg;
        VDEC_ATTR_JPEG_S stAttrJpeg;
    };
} VDEC_CHN_ATTR_S;
```
【Member Description】

|          Member         |            Meaning             |
| :---------------------: | :----------------------------: |
|         enType          |   Type of stream, such as H264 or H265, etc.   |
|         enMode          |       Decoding mode, only supports frame mode       |
|     enPixelFormat      |      Decoding output pixel format       |
|    u32StreamBufSize     | Size of stream buffer used for decoding   |
|    u32StreamBufCnt      | Number of stream buffers used for decoding   |
|  bExternalBitStreamBuf  | Whether to use external or internal buffer for stream decoding |
|    u32FrameBufCnt       |   Number of frame buffers used for decoding |
|     vlc_buf_size        |       Size of decoder vlc buffer       |
| stAttrH264/stAttrH265<br/>stAttrMjpeg/stAttrJpeg |   Encoder attributes for different protocols    |

### HB_VDEC_USERDATA_S
【Description】
> Definition of decoding user data structure.

【Structure Definition】
```C
typedef struct HB_VDEC_USERDATA_S {
    HB_BOOL  bValid;
    uint32_t   u32Len;
    uint64_t   u64PhyAddr;
    uint8_t*   pu8Addr;
} VDEC_USERDATA_S;
```
【Member Description】|   Member   |        Meaning         |
| :--------: | :--------------------: |
|  bValid    |     Enable/Disable     |
|  u32Len    |    Length of Data      |
| u64PhyAddr | Physical Address of Data |
|  pu8Addr   |  Virtual Address of Data |

### HB_VDEC_CHN_STATUS_S
【Description】
> Define the structure of the decoding channel status.

【Structure Definition】
```C
typedef struct HB_VDEC_CHN_STATUS_S {
	uint32_t cur_input_buf_cnt;
	uint64_t cur_input_buf_size;
	uint64_t cur_output_buf_cnt;
	uint64_t cur_output_buf_size;
	uint32_t left_recv_frame;
	uint32_t left_enc_frame;
	uint32_t total_input_buf_cnt;
	uint32_t total_output_buf_cnt;
	int32_t pipeline;
	int32_t channel_port_id;
} VDEC_CHN_STATUS_S;
```
【Member Description】

|       Member       |          Meaning           |
| :----------------: | :------------------------: |
|  cur_input_buf_cnt |  Number of input streams not decoded at present  |
| cur_input_buf_size |    Size of input stream buffer at present    |
| cur_output_buf_cnt | Number of completed decoded streams at present |
| cur_output_buf_size |    Size of decoded stream buffer at present    |
|   left_recv_frame  | Number of remaining frames to be received |
|    left_enc_frame  | Number of remaining frames to be encoded |
| total_input_buf_cnt |   Total number of input streams |
| total_output_buf_cnt |   Total number of decoded streams  |
|      pipeline      |            pipeline             |
|   channel_port_id  |           channel id            |

## Error Codes
The error codes for VDEC are as follows:

| Error Code | Macro Definition        | Description              |
| :--------: | :--------------------- | :----------------------- |
| -269024256 | HB_ERR_VDEC_UNKNOWN    | Unknown error            |
| -269024257 | HB_ERR_VDEC_NOT_FOUND  | VDEC channel not found   |
| -269024258 | HB_ERR_VDEC_OPEN_FAIL  | Failed to open VDEC channel |
| -269024259 | HB_ERR_VDEC_RESPONSE_TIMEOUT | No response from VDEC channel || -269024260 | HB_ERR_VDEC_INIT_FAIL              | Failed to initialize VDEC module |
| -269024261 | HB_ERR_VDEC_OPERATION_NOT_ALLOWDED | Operation not allowed             |
| -269024262 | HB_ERR_VDEC_NOMEM                  | Insufficient VDEC memory          |
| -269024263 | HB_ERR_VDEC_NO_FREE_CHANNEL        | No free VDEC channel              |
| -269024264 | HB_ERR_VDEC_ILLEGAL_PARAM          | Invalid parameter                 |
| -269024265 | HB_ERR_VDEC_INVALID_CHNID          | Invalid channel ID                |
| -269024266 | HB_ERR_VDEC_INVALID_BUF            | Invalid buffer block              |
| -269024267 | HB_ERR_VDEC_INVALID_CMD            | Invalid command                   |
| -269024268 | HB_ERR_VDEC_WAIT_TIMEOUT           | Timeout waiting                   |
| -269024269 | HB_ERR_VDEC_FILE_OPERATION_FAIL    | Failed to operate on file          |
| -269024270 | HB_ERR_VDEC_PARAMS_SET_FAIL        | Failed to set parameters           |
| -269024271 | HB_ERR_VDEC_PARAMS_GET_FAIL        | Failed to get parameters           |
| -269024272 | HB_ERR_VDEC_EXIST                  | VDEC channel already exists        |
| -269024273 | HB_ERR_VDEC_UNEXIST                | VDEC channel does not exist        |
| -269024274 | HB_ERR_VDEC_NULL_PTR               | Null pointer                      |

## Reference Code
For example codes related to the VDEC, please refer to [sample_video_codec](./multimedia_samples#sample_video_codec).