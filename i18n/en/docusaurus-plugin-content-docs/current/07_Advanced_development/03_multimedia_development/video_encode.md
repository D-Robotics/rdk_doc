---
sidebar_position: 9
---
# 7.3.9 Video Encoding
## Overview
The video encoding module implements hardware encoding protocols such as H.264/H.265/JPEG/MJPEG. This module supports real-time encoding of multiple channels, with each channel being independent. Common use cases include single-channel recording, multi-channel recording, single-channel VIO video streaming, multi-channel VIO video streaming, recording + VIO video streaming, etc.

## Function Description

### Basic Specifications

The encoding specifications supported by X3 are as follows:

![image-20220329224946556](./image/video_encode/image-20220329224946556.png)

H.264/H.265 protocol encoding performance is as follows:

- H.264 decoding supports a maximum resolution of 8192 x 8192, with a minimum resolution of 256 x 128, and a minimum decoding resolution of 32 x 32.
- H.265 decoding supports a maximum resolution of 8192 x 8192, with a minimum resolution of 256 x 128, and a minimum decoding resolution of 8 x 8.
- The stride of H.264/H.265 is aligned to 32 bytes, while the width and height are aligned to 8 bytes. If they are not aligned, it is recommended to use VIDEO_CROP_INFO_S to perform corresponding cropping.
- Both H.264/H.265 have real-time multi-stream encoding capabilities.
- The highest capability supports 4K@60fps.
- ROI encoding with QP map (allows users to select regions of interest in the picture, after enabling ROI function, important or moving regions will be encoded with high-quality lossless coding, while the bitrate and image quality of regions that do not move or are not selected will be reduced, achieving standard definition video compression, or even not transmitting this part of the video)
- Supports rotation and mirroring.
- Multi-instance processing, up to 32 instances.

JPEG protocol encoding capabilities are as follows:
- Encoding and decoding resolution range from 16 x 16 to 32768 x 32768.
- MJPEG and JPEG stride are aligned to 32 bytes, width is aligned to 16 bytes, and height is aligned to 8 bytes.
- For YUV 4:2:0 format (e.g. NV12), the highest capability is 4K@30fps.
- JPEG Baseline and Extended sequential ISO/IEC 10918-1.
- Supports one or three color components, each component can have 8-bit or 12-bit sampling.
- Supports YUV 4:0:0, 4:2:0, 4:2:2, 4:4:0, and 4:4:4 color formats.
- Supports encoding and decoding ROI.
- Supports slice encoding.
- Supports rotation and mirroring.
- Multi-instance, with a maximum support of 64 instances.

### Encoding and Decoding Channels
An encoding and decoding channel refers to a specific type of encoding and decoding instance. The user parameters, configuration, and resources of different encoding and decoding channels can be independent of each other, allowing for multiple channels of video encoding and decoding with different specifications to cover various business scenarios.

### Bitrate Control
Bitrate control mainly refers to the control of encoding bitrate. Bitrate control is for continuous video encoding streams, and for a changing scene, if you want to achieve stable image quality, the encoding bitrate will fluctuate. If you want to achieve stable encoding bitrate, the image quality will fluctuate. X3 supports the following bitrate control methods for H.264, H.265, and MJPEG protocols:

- H.264/H.265 supports CBR, VBR, AVBR, FixQp, and QpMap five types of bitrate control modes for encoding channels.
- MJPEG encoding channel supports FixQp bitrate control mode.

CBR can ensure a stable overall encoding bitrate.

VBR ensures a stable image quality during encoding.AVBR considers both bitrate and image quality, generating a bitrate and image quality relatively stable bitstream;

FixQp fixes the QP value for each I frame, P frame, and B frame;

QPMAP assigns a QP value for each block in a frame, where the block size is 16x16 for H264 and 32x32 for H265.

For CBR and AVBR, the encoder internally finds a suitable QP value for each frame image to ensure a constant bitrate.

The encoder supports three levels of rate control internally, which are frame level, CTU/MB level, and subCTU/subMB level. The frame level control mainly generates a QP value for each frame image based on the target bitrate to ensure a constant bitrate; the CTU/MB level control generates a QP value for each block based on the target bitrate of each 64x64 CTU or 16x16 MB, which can achieve better bitrate control, but frequent QP value adjustments may cause unstable image quality; the subCTU/subMB level control generates a QP value for each 32x32 subCTU or 8x8 subMB, with complex blocks receiving higher QP values and static blocks receiving lower QP values, because the human eye is more sensitive to static regions compared to complex areas. The detection of complex and static regions mainly relies on internal hardware modules. This level of control is mainly for improving subjective image quality while ensuring a constant bitrate, and it results in higher SSIM scores but lower PSNR scores.

CBR, VBR, and AVBR can enable QPMAP, and the actual value for each block region is obtained by the following formula:

![image-20220329234019920](./image/video_encode/image-20220329234019920.png)

MQP is the value in the ROI map, RQP is the value obtained by the internal bitrate control of the encoder, and ROIAvaQP is the average QP value in the ROI map.

### GOP Structure

The GOP structure table can define a set of periodic GOP structures that will be used throughout the encoding process. The elements in a single structure table are shown in the following table. It is possible to specify the reference frames for this image. If the reference frames specified for the frames following the IDR frame are the data frames before the IDR frame, the encoder will automatically handle this situation so that they do not reference other frames. Users do not need to be concerned about this situation. When defining a custom GOP structure, users need to specify the number of structure tables. A maximum of 8 structure tables can be defined, and the order of the structure tables needs to be arranged in decoding order.

| Element        | Description                                                  |
| :------------- | :----------------------------------------------------------- |
| Type           | Slice type (I, P or B)                                       |
| POC            | Display order of the frame within a GOP, ranging from 1 to GOP size |
| QPoffset       | A quantization parameter of the picture in the custom GOP     |
| NUM_REF_PIC_L0 | Flag to use multi-reference picture for P picture. It is valid only if PIC_TYPE is P |
| temporal_id    | Temporal layer of the frame. A frame cannot predict from a frame with a higher temporal ID (0~6) |
| 1st_ref_POC    | The POC of the 1st reference picture of L0                   |
| 2nd_ref_POC    | The POC of the 1st reference picture of L1 in case that Type is equal to B. The POC of the 2nd reference picture of L0 in case that Type is equal to P. Note that reference_L1 can have the same POC as reference in B slice. But for compression efficiency, it is recommended that reference_L1 have a different POC from reference_L0. |

#### GOP Predefined Structures

![VENC_GOP_structure](./image/video_encode/ss_venc_gop_structure.png)


The following table provides 8 predefined GOP structures.

| Index | GOP Structure | Low Delay (encoding order and display order are the same) | GOP Size | Encoding Order | Minimum Source Frame Buffer | Minimum Decoded Picture Buffer | Intra Period (I Frame Interval) Requirement |
| :---: | :-----------: | :-------------------------------------------------------: | :------: | :------------: | :-------------------------: | :---------------------------: | :-----------------------------------------: |
|   1   |       I       |                             Yes                             |    1     |  I0-I1-I2…    |              1              |               1               |                                             |
|   2   |       P       |                             Yes                             |    1     |  P0-P1-P2…    |              1              |               2               |                                             |
|   3   |       B       |                             Yes                             |    1     |  B0-B1-B2…    |              1              |               3               |                                             |
|   4   |      BP       |                              No                             |    2     | B1-P0-B3-P2…  |              4              |               3               |               Multiple of 2               |
|   5   |     BBBP      |                              No                             |    4     | B2-B1-B3-P0…  |              7              |               4               |               Multiple of 4               |
|   6   |     PPPP      |                             Yes                             |    4     | P0-P1-P2-P3…  |              1              |               2               |                                             |
|   7   |     BBBB      |                             Yes                             |    4     | B0-B1-B2-B3…  |              1              |               3               |                                             |
|   8   |   BBBBBBBB    |                              No                             |    8     |B3-B2-B4-B1-B6-B5-B7-B0…|          12          |              5              |               Multiple of 8               |

Where: [image available for reference, not shown here]- GOP Preset1
  - Only I frames, no inter-reference frames
  - Low latency
  ![VENC_GOP_preset1](./image/video_encode/ss_venc_gop_preset1.png)
  
- GOP Preset2
  - Only I frames and P frames
  - P frames reference two forward reference frames
  - Low latency
  ![VENC_GOP_preset2](./image/video_encode/ss_venc_gop_preset2.png)
  
- GOP Preset3
  - Only I frames and B frames
  - B frames reference two forward reference frames
  - Low latency
  ![VENC_GOP_preset3](./image/video_encode/ss_venc_gop_preset3.png)
  
- GOP Preset4
  - I frames, P frames, and B frames
  - P frames reference two forward reference frames
  - B frames reference one forward reference frame and one backward reference frame
  ![VENC_GOP_preset4](./image/video_encode/ss_venc_gop_preset4.png)
  
- GOP Preset5
  - I frames, P frames, and B frames
  - P frames reference two forward reference frames
  - B frames reference one forward reference frame and one backward reference frame, where the backward reference frame can be a P frame or a B frame
  ![VENC_GOP_preset5](./image/video_encode/ss_venc_gop_preset5.png)
  
- GOP Preset 6
  - Only I frames and P frames
  - P frames reference two forward reference frames
  - Low latency
  ![VENC_GOP_preset6](./image/video_encode/ss_venc_gop_preset6.png)
  
- GOP Preset 7
  - Only I frames and B frames
  - B frames reference two forward reference frames
  - Low latency
  ![VENC_GOP_preset7](./image/video_encode/ss_venc_gop_preset7.png)
  
- GOP Preset 8
  - Only I frames and B frames
  - B frames reference one forward reference frame and one backward reference frame
  ![VENC_GOP_preset8](./image/video_encode/ss_venc_gop_preset8.png)

#### Relationship between GOP and I frame period
The following figure shows the relationship between GOP structure and I frame period.

![VENC_GOP_i-frame](./image/video_encode/ss_venc_gop_i-frame.png)

### ROI

The implementation of ROI encoding is similar to QPMAP, and users need to set the QP value for each block according to the raster scan direction. The following figure shows an example of ROI map for H265. For H264 encoding, the size of each block is 16x16, while in H265, it is 32x32. In the ROI map table, each QP value occupies one byte, ranging from 0 to 51.

ROI encoding can work together with CBR and AVBR. When CBR or AVBR is not enabled, the actual QP value for each block region is the value specified in the ROI map. When CBR or AVBR is enabled, the actual value for each block region is obtained by the following formula:

![image-20220405152959958](./image/video_encode/image-20220405152959958.png)

MQP is the value in the ROI map, RQP is the value obtained by the encoder's internal rate control, and ROIAvaQP is the average QP value in the ROI map.

![VENC_H265_ROI_map](./image/video_encode/ss_venc_h265_roi_map.png)

### Intra Refresh
Intra Refresh mode improves fault tolerance by periodically inserting intra-coded MB/CTUs into non-I frames. It provides more repair points for the decoder to avoid image corruption caused by temporal errors. Users can specify the number of continuous rows, columns, or step size of MB/CTUs to force the encoder to insert intra-coded units. Users can also specify the size of intra-coded units, which will be determined internally by the encoder.

### Long-term reference frame
Users can specify the period of long-term reference frames and the cycle of referring long-term reference frames, as shown in the following figure.

![VENC_long_reference_frame](./image/video_encode/ss_venc_long_reference_frame.png)

### Smart background encoding
In video surveillance scenarios, the background is often static. Therefore, it is desired that the encoder can either ignore the background region or use less bitrate to encode it when detecting a background region. In actual scenarios, due to the presence of noise in the camera image, it is not easy to detect the background region. In many cases, the ISP needs to notify the encoder when it detects a background region, which consumes additional bandwidth and system computing resources.

H264 and H265 encoding provide integrated smart background encoding modes inside the codec. This mode fully utilizes internal hardware modules and on-the-fly processing, without consuming additional bandwidth and system resources. The following figure shows the working mode of background detection. In the smart background encoding mode, the internal hardware module compares each block unit with the corresponding block unit of the reference frame to determine whether the block is part of the background.

For background region judgment, users can set the maximum pixel difference value (recommended value 8) and the average pixel difference value (recommended value 1). Users can also adjust the Lambda parameter to influence the mode selection in encoding. When a background region is detected, the encoder internally increases the corresponding Lambda value for each block unit, making the encoder more likely to use ignore mode to encode the block unit. For Lambda control, users can set lambdaQP (recommended value 32) and deltaQP (recommended value 3), and the final Lambda value is calculated according to the following formula:

![image-20220405153105331](./image/video_encode/image-20220405153105331.png)

QP_TO_LAMBDA_TABLE is the Lambda conversion table, which is also used for Lambda conversion in non-background regions.

![VENC_smart_bg_encoding](./image/video_encode/ss_venc_smart_bg_encoding.png)

It should be noted that Smart background encoding does not work when ROI encoding is enabled. The amount of bandwidth saved by this mode is closely related to the set bitrate and I-frame interval. The larger the bitrate and I-frame interval, the more bandwidth can be saved. In addition, in this mode, frames with better image quality can be set as long-term reference frames to improve the quality of the background image and save bitrate.

### Frame skip setting
Users can use the interface to set the encoding mode of the next input image as the skip mode. This mode is only valid for non-I frames. In skip mode, the encoder internally ignores the input frame and uses the reconstructed frame of the previous frame to generate the reconstruction frame of the current input. The input frame is then encoded as a P frame.

## API Reference
```C
HB_VENC_CreateChn: Create an encoding channel.
HB_VENC_DestroyChn: Destroy an encoding channel.
HB_VENC_ResetChn: Reset an encoding channel.
HB_VENC_StartRecvFrame: Start the encoding channel to receive input images.
HB_VENC_StopRecvFrame: Stop the encoding channel from receiving input images.
HB_VENC_SetChnAttr: Set the encoding attributes of an encoding channel.
HB_VENC_GetChnAttr: Get the encoding attributes of an encoding channel.
HB_VENC_GetStream: Get the encoded stream.
HB_VENC_ReleaseStream: Release the stream buffer.
HB_VENC_SendFrame: Support the user to send raw images for encoding.
HB_VENC_RequestIDR: Request an IDR frame.
HB_VENC_SetRoiAttr: Set the ROI encoding configuration of an encoding channel.
HB_VENC_GetRoiAttr: Get the ROI encoding configuration of an encoding channel.HB_VENC_SetH264SliceSplit: Set slice splitting configuration for H.264 encoding.
HB_VENC_GetH264SliceSplit: Get slice splitting configuration for H.264 encoding.
HB_VENC_SetH264IntraPred: Set frame intra-prediction configuration for H.264 encoding.
HB_VENC_GetH264IntraPred: Get frame intra-prediction configuration for H.264 encoding.
HB_VENC_SetH264Trans: Set transform and quantization configuration for H.264 encoding.
HB_VENC_GetH264Trans: Get transform and quantization configuration for H.264 encoding.
HB_VENC_SetH264Entropy: Set entropy coding configuration for H.264 encoding.
HB_VENC_GetH264Entropy: Get entropy coding configuration for H.264 encoding.
HB_VENC_SetH264Dblk: Set deblocking configuration for H.264 encoding.
HB_VENC_GetH264Dblk: Get deblocking configuration for H.264 encoding.
HB_VENC_SetH264Vui: Set VUI configuration for H.264 encoding.
HB_VENC_GetH264Vui: Get VUI configuration for H.264 encoding.
HB_VENC_SetH265Vui: Set VUI parameters for H.265 encoding channel.
HB_VENC_GetH265Vui: Get VUI configuration for H.265 encoding channel.
HB_VENC_SetRcParam: Set advanced parameters for channel bitrate control.
HB_VENC_GetRcParam: Get advanced parameters for channel bitrate control.
HB_VENC_SetRefParam: Set advanced skip frame reference parameters for H.264/H.265 encoding channels.
HB_VENC_GetRefParam: Get advanced skip frame reference parameters for H.264/H.265 encoding channels.
HB_VENC_SetH265SliceSplit: Set slice splitting configuration for H.265 encoding.
HB_VENC_GetH265SliceSplit: Get slice splitting configuration for H.265 encoding.
HB_VENC_SetH265PredUnit: Set PU configuration for H.265 encoding.
HB_VENC_GetH265PredUnit: Get PU configuration for H.265 encoding.
HB_VENC_SetH265Trans: Set transform and quantization configuration for H.265 encoding.
HB_VENC_GetH265Trans: Get transform and quantization configuration for H.265 encoding.
HB_VENC_SetH265Dblk: Set deblocking configuration for H.265 encoding.
HB_VENC_GetH265Dblk: Get deblocking configuration for H.265 encoding.
HB_VENC_SetH265Sao: Set SAO configuration for H.265 encoding.
HB_VENC_GetH265Sao: Get SAO configuration for H.265 encoding.
HB_VENC_GetIntraRefresh: Get parameters for P frame refreshing Islice.
HB_VENC_SetIntraRefresh: Set parameters for P frame refreshing Islice.
HB_VENC_SetCuPrediction: Set tendency for CU mode prediction.
HB_VENC_GetCuPrediction: Get configuration for CU mode prediction.
HB_VENC_GetFd: Get device file handle for encoding channel.
HB_VENC_CloseFd: Close device file handle for encoding channel.
HB_VENC_QueryStatus: Query status of encoding channel.
HB_VENC_InserUserData: Insert user data.
HB_VENC_SendFrameEx: Send raw image and QpMap table for encoding.
```



### HB_VENC_CreateChn
**Function Declaration**
```C
int32_t HB_VENC_CreateChn(VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstAttr);
```
**Function Description**
> Creates an encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                                                       | Input/Output |
| :------------- | :--------------------------------------------------------------------------------------------------------- | :----------- |
|   VeChn        | Channel number for encoding.<br/>Range: [0, VENC_MAX_CHN_NUM).<br/>H264/H265 supports up to 32 channels, JPEG/MJPEG up to 64 channels. |     Input    |
|   pstAttr      | Pointer to the encoding channel attributes                                                                                        |     Input    |

**Return Values**

| Return Value | Description |
| :----------: | :---------- |
|       0      |         Success |
| Non-zero     | Failure, refer to error codes. |

**Note**
> None

**Reference Code**
> See HB_VENC_GetStream for reference code.



### HB_VENC_DestroyChn
**Function Declaration**
```C
int32_t HB_VENC_DestroyChn(VENC_CHN VeChn);
```
**Function Description**
> Destroys an encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                            | Input/Output |
| :------------- | :----------------------------------------------------- | :----------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |

**Return Values**

| Return Value | Description                                      |
| :---------: | :----------------------------------------------- |
|     0      | Success                                          |
| Non-zero    | Failure, refer to the error code.                |

**Note**
> None

**Reference Code**
> HB_VENC_GetStream reference code

### HB_VENC_ResetChn
**Function Declaration**
```C
int32_t HB_VENC_ResetChn(VENC_CHN VeChn);
```
**Function Description**
> Resets an encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                            | Input/Output |
| :------------- | :----------------------------------------------------- | :----------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |

**Return Values**

| Return Value | Description                                      |
| :---------: | :----------------------------------------------- |
|     0      | Success                                          |
| Non-zero    | Failure, refer to the error code.                |

**Note**
> None

**Reference Code**
> HB_VENC_GetStream reference code

### HB_VENC_StartRecvFrame
**Function Declaration**
```C
int32_t HB_VENC_StartRecvFrame(VENC_CHN VeChn, const VENC_RECV_PIC_PARAM_S *pstRecvParam);
```
**Function Description**
> Enables receiving input images for the encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :----------------------------------------------------- | :----------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
| pstRecvParam   | Pointer to the receive image parameter structure, used to specify the number of frames to be received. | Input       |

**Return Values**

| Return Value | Description                                      |
| :---------: | :----------------------------------------------- |
|     0      | Success                                          |
| Non-zero    | Failure, refer to the error code.                |

**Note**
> This function must be called after setting channel attributes with HB_VENC_SetChnAttr.

**Reference Code**
> HB_VENC_GetStream reference code

### HB_VENC_StopRecvFrame
**Function Declaration**
```C
int32_t HB_VENC_StopRecvFrame(VENC_CHN VeChn);
```
**Function Description**
> Stops receiving input images for the encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :----------------------------------------------------- | :----------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |

**Return Values**

| Return Value | Description                                      |
| :---------: | :----------------------------------------------- |
|     0      | Success                                          |
| Non-zero    | Failure, refer to the error code.                |

**Note**
> None

**Reference Code**
> HB_VENC_GetStream reference code

### HB_VENC_SetChnAttr
**Function Declaration**
```C
int32_t HB_VENC_SetChnAttr(VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstChnAttr);
```
**Function Description**
> Sets the encoding attributes for an encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :----------------------------------------------------- | :----------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
| pstChnAttr     | Pointer to the encoding channel attribute structure.      |      Input   |

**Return Values**

| Return Value | Description                                      |
| :---------: | :----------------------------------------------- |
|     0      | Success                                          |
| Non-zero    | Failure, refer to the error code.                |

**Note**
> A channel must be created using HB_VENC_CreateChn before calling this function.

**Reference Code**
> HB_VENC_GetStream reference code



### HB_VENC_GetChnAttr
【Function Declaration】
```C
int32_t HB_VENC_GetChnAttr(VENC_CHN VeChn, VENC_CHN_ATTR_S *pstChnAttr);
```
【Function Description】
> Get the encoding attributes of the encoding channel.

【Parameter Description】

|  Parameter Name  | Description                                       |  Input/Output  |
| :--------------: | :------------------------------------------------ | :------------: |
|      VeChn       | Encoding channel number.<br/>Value range: [0, VENC_MAX_CHN_NUM) |     Input      |
|   pstChnAttr     | Pointer to encoding channel attributes            |     Input      |

【Return Value】

|  Return Value  |      Description     |
| :------------: | :------------------: |
|       0        |        Success       |
|  Non-zero      |    Failure. Refer to error code.  |

【Notes】
> None

【Reference code】
> Reference code for HB_VENC_GetStream

### HB_VENC_GetStream
【Function Declaration】
```C
int32_t HB_VENC_GetStream(VENC_CHN VeChn, VIDEO_STREAM_S *pstStream, int32_t s32MilliSec);
```
【Function Description】
> Get the encoding stream.

【Parameter Description】

|  Parameter Name   | Description                                                                                     |  Input/Output  |
| :---------------: | :--------------------------------------------------------------------------------------------- | :------------: |
|      VeChn        | Encoding channel number.<br/>Value range: [0, VENC_MAX_CHN_NUM)                               |     Input      |
|    pstStream      | Pointer to the stream structure                                                                |     Input      |
|    s32MilliSec    | Timeout for getting the stream.<br/>Value range: [-1, + ∞ )<br/> -1: Block.<br/> 0: Non-block.<br/> Greater than 0: Timeout duration. |     Input      |

【Return Value】| Return Value | Description       |
| :----------: | :---------------- |
|      0       | Success           |
|    Non-zero  | Failure, see error code. |

【Notes】
> None

【Reference Code】
```C
    VENC_CHN VeChn = 0;
    int32_t s32Ret = 0;
    int32_t Width = 640;
    int32_t Height = 480;
    FILE *inFile;

    char *inputFileName = "./venc/yuv/input_640x480_yuv420p.yuv";
    inFile = fopen(inputFileName, "rb");
    ASSERT_NE(inFile, nullptr);

    char* mmz_vaddr[10];
    int32_t i = 0;
    for (i=0;i<10;i++) {
        mmz_vaddr[i] = NULL;
    }
    uint64_t mmz_paddr[10];
    memset(mmz_paddr, 0, sizeof(mmz_paddr));

    int32_t mmz_size = Width * Height * 3 / 2;

    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);

    s32Ret = HB_VP_Init();
    if (s32Ret != 0) {
        printf("vp_init fail s32Ret = %d !\n",s32Ret);
    }

    for (i = 0; i < 10; i++) {
        s32Ret = HB_SYS_Alloc(&mmz_paddr[i], (void **)&mmz_vaddr[i], mmz_size);
        if (s32Ret == 0) {
            printf("mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d \n", mmz_paddr[i], mmz_vaddr[i],i);
        }
    }

    int32_t s32ReadLen = 0;
    for (i = 0; i < 10; i++) {
        s32ReadLen = fread(mmz_vaddr[i], 1, mmz_size, inFile);printf("s32ReadLen = %d !!!!!\n", s32ReadLen);
    if (s32ReadLen == 0) {
        printf("read over !!!\n");
    }
}
/* if (inFile) fclose(inFile); */

VENC_CHN_ATTR_S m_VencChnAttr;
memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
m_VencChnAttr.stVencAttr.enType = PT_H264;
m_VencChnAttr.stVencAttr.u32PicWidth = Width;
m_VencChnAttr.stVencAttr.u32PicHeight = Height;
m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
pstRcParam->stH264Cbr.u32BitRate = 3000;
pstRcParam->stH264Cbr.u32FrameRate = 30;
pstRcParam->stH264Cbr.u32IntraPeriod = 30;
pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);  // config

VENC_RECV_PIC_PARAM_S pstRecvParam;
pstRecvParam.s32RecvPicNum = 0;  // unchangable
s32Ret = HB_VENC_StartRecvFrame(VeChn, &pstRecvParam);
VIDEO_FRAME_S pstFrame;
VIDEO_STREAM_S pstStream;
memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

pstFrame.stVFrame.width = Width;
pstFrame.stVFrame.height = Height;
pstFrame.stVFrame.size = mmz_size;

int32_t offset = Width * Height;
for (i=0;i<10;i++) {
    pstFrame.stVFrame.phy_ptr[0] = mmz_paddr[i];
    pstFrame.stVFrame.phy_ptr[1] = mmz_paddr[i] + offset;
    pstFrame.stVFrame.phy_ptr[2] = mmz_paddr[i] + offset * 5 / 4;
    pstFrame.stVFrame.vir_ptr[0] = mmz_vaddr[i];
    pstFrame.stVFrame.vir_ptr[1] = mmz_vaddr[i] + offset;
    pstFrame.stVFrame.vir_ptr[2] = mmz_vaddr[i] + offset * 5 / 4;\# If i is equal to 9
if (i == 9) {
    pstFrame.stVFrame.frame_end = HB_TRUE;
}

s32Ret = HB_VENC_SendFrame(VeChn, &pstFrame, 3000);
usleep(300000);

s32Ret = HB_VENC_GetStream(VeChn, &pstStream, 3000);
EXPECT_EQ(s32Ret, (int32_t)0);
printf("i = %d   pstStream.pstPack.size = %d !!!!!\n", i, pstStream.pstPack.size);
s32Ret = HB_VENC_ReleaseStream(VeChn, &pstStream);
}

s32Ret = HB_VENC_StopRecvFrame(VeChn);
s32Ret = HB_VENC_DestroyChn(VeChn);
for (i = 0; i < 10; i++) {
    s32Ret = HB_SYS_Free(mmz_paddr[i], mmz_vaddr[i]);
    if (s32Ret == 0) {
        printf("mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", mmz_paddr[i],
            mmz_vaddr[i], i);
    }
}
s32Ret = HB_VP_Exit();
if (s32Ret == 0) printf("vp exit ok!\n");
printf("GetStream_Test\n");
if (inFile) fclose(inFile);
```

### HB_VENC_ReleaseStream
【Function Declaration】
```C
int32_t HB_VENC_ReleaseStream(VENC_CHN VeChn, VIDEO_STREAM_S *pstStream);
```
【Function Description】
> Release stream buffer.

【Parameter Description】

| Parameter | Description                                        | Input/Output |
| :-------: | :------------------------------------------------- | :----------: |
|   VeChn   | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |    Input     |
| pstStream | Pointer of stream structure                       |    Input     |

【Return Value】

| Return Value |            Description |
| :----------: | :-------------------- |
|      0       |            Success |
|   Non-zero   | Fail, see error code |

【Notes】Please translate the following Chinese parts into English, while keeping the original format and content:

> None

【Reference Code】
> HB_VENC_GetStream reference code

### HB_VENC_SendFrame
【Function Declaration】
```C
int32_t HB_VENC_SendFrame(VENC_CHN VeChn, VIDEO_FRAME_S *pstFrame ,int32_t s32MilliSec);
```
【Function Description】
> Supports sending original images for encoding.

【Parameter Description】

| Parameter | Description                                                                                            | Input/Output |
| :-------: | :---------------------------------------------------------------------------------------------------- | :----------: |
|   VeChn   | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)                                              |    Input     |
| pstFrame  | Pointer to the structure of the original image information.                                            |    Input     |
|s32MilliSec| Timeout for obtaining the bitstream.<br/>Range: [-1, +∞)<br/>-1: Block.<br/> 0: Non-block.<br/> >0: Timeout.|    Input     |

【Return Value】

| Return Value |                Description |
| :----------: | :------------------------ |
|      0       |            Success        |
|   Non-zero   | Failure, see error code.  |

【Notes】
> None

【Reference Code】
> HB_VENC_GetStream reference code



### HB_VENC_RequestIDR
**Function Declaration**
```C
int32_t HB_VENC_RequestIDR(VENC_CHN VeChn);
```
**Function Description**
> Requests an IDR frame.

**Parameter Descriptions**

| Parameter Name | Description                                      | Input/Output |
| :------------- | :------------------------------------------------ | :----------: |
| VeChn          | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|      0      |   Success  |
| Non-zero    | Failure, refer to error codes. |

**Note**
> None

**Reference Code**
```C
    VENC_CHN VeChn = 0;
    int32_t s32Ret = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_RequestIDR(VeChn);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```




### HB_VENC_SetRoiAttr
**Function Declaration**
```C
int32_t HB_VENC_SetRoiAttr(VENC_CHN VeChn, const VENC_ROI_ATTR_S *pstRoiAttr);
```
**Function Description**
> Sets the ROI (Region of Interest) encoding configuration for a video coding channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                                           | Input/Output |
| :------------: | :---------------------------------------------------------------------------------------------------- | :----------: |
|    VeChn      | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)                                            |     Input    |
|  pstRoiAttr   | Pointer to ROI area parameters                                                                              |     Input    |

**Return Values**

| Return Value | Description                                                                                         |
| :---------: | :-------------------------------------------------------------------------------------------------- |
|      0      | Successful                                                                                           |
| Non-zero    | Failure; see error codes for details                                                               |

**Caution**
> None

**Reference Code**

HB_VENC_GetRoiAttr reference code


### HB_VENC_GetRoiAttr
**Function Declaration**
```C
int32_t HB_VENC_GetRoiAttr(VENC_CHN VeChn, VENC_ROI_ATTR_S *pstRoiAttr);
```
**Function Description**
> Retrieves the ROI (Region of Interest) encoding configuration for a video coding channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                                           | Input/Output |
| :------------: | :---------------------------------------------------------------------------------------------------- | :----------: |
|    VeChn      | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)                                            |     Input    |
|  pstRoiAttr   | Pointer to the ROI configuration that will receive the retrieved data                                  |     Output   |

**Return Values**

| Return Value | Description                                                                                         |
| :---------: | :-------------------------------------------------------------------------------------------------- |
|      0      | Successful                                                                                           |
| Non-zero    | Failure; see error codes for details                                                               |

**Caution**
> None

**Example Code**
```C
    VENC_CHN VeChn = 0;
    int32_t s32Ret = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;
    VENC_ROI_ATTR_S pstRoiAttrTest1;
    memset(&pstRoiAttrTest1, 0, sizeof(VENC_ROI_ATTR_S));
    uint8_t stroi_map_array1[100] = {0};
    pstRoiAttrTest1.roi_map_array = stroi_map_array1;
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    VENC_ROI_ATTR_S pstRoiAttrTest2;
    memset(&pstRoiAttrTest1, 0, sizeof(VENC_ROI_ATTR_S));
    uint8_t stroi_map_array[100] = {0};
    pstRoiAttrTest2.roi_enable = HB_TRUE;
    pstRoiAttrTest2.roi_map_array = stroi_map_array;
    pstRoiAttrTest2.roi_map_array_count = 100;

    s32Ret = HB_VENC_SetRoiAttr(VeChn, &pstRoiAttrTest2);
    s32Ret = HB_VENC_GetRoiAttr(VeChn, &pstRoiAttrTest1);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_SetH264SliceSplit
**Function Declaration**
```C
int32_t HB_VENC_SetH264SliceSplit(VENC_CHN VeChn, const VENC_H264_SLICE_SPLIT_S *pstSliceSplit);
```
**Function Description**
> Configures the H.264 encoding slice splitting parameters.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :------------------------------------------------------ | :----------: |
|      VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
|   pstSliceSplit  | H.264 stream slice splitting parameters                |     Input    |

**Return Values**

| Return Value | Description |
| :---------: | :----------|
|      0      | Success     |
| Non-zero    | Failure, refer to error code. |

**Caution**
> None

**Reference Code**
HB_VENC_GetH264SliceSplit Reference Code

### HB_VENC_GetH264SliceSplit
**Function Declaration**
```C
int32_t HB_VENC_GetH264SliceSplit(VENC_CHN VeChn, VENC_H264_SLICE_SPLIT_S *pstSliceSplit);
```
**Function Description**
> Retrieves the H.264 encoding slice splitting configuration.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :------------------------------------------------------ | :----------: |
|      VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
|   pstSliceSplit  | H.264 stream slice splitting parameters                |     Output   |

**Return Values**

| Return Value | Description |
| :---------: | :----------|
|      0      | Success     |
| Non-zero    | Failure, refer to error code. |

**Caution**
> None

**Reference Code**
```C
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;
    int32_t s32Ret = 0;

    VENC_H264_SLICE_SPLIT_S pstSliceSplit1;
    memset(&pstSliceSplit1, 0, sizeof(VENC_H264_SLICE_SPLIT_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);  // config

    pstSliceSplit1.h264_slice_mode = HB_TRUE;
    pstSliceSplit1.h264_slice_arg = 10;
    pstSliceSplit1.slice_loop_filter_across_slices_enabled_flag = HB_TRUE;
    s32Ret = HB_VENC_SetH264SliceSplit(VeChn, &pstSliceSplit1);

    VENC_H264_SLICE_SPLIT_S pstSliceSplit2;
    memset(&pstSliceSplit2, 0, sizeof(VENC_H264_SLICE_SPLIT_S));
    s32Ret = HB_VENC_GetH264SliceSplit(VeChn, &pstSliceSplit2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_SetH264IntraPred
**Function Declaration**
```C
int32_t HB_VENC_SetH264IntraPred(VENC_CHN VeChn, const VENC_H264_INTRA_PRED_S *pstH264IntraPred);
```
**Function Description**
> Configures the intra prediction settings for H.264 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :-------------------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input   |
| pstH264IntraPred | H.264 protocol's intra prediction configuration          |     Input   |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success     |
| Non-zero   | Failure, refer to error codes. |

**Note**
> None

**Reference Code**
HB_VENC_GetH264IntraPred reference code

### HB_VENC_GetH264IntraPred
**Function Declaration**
```C
int32_t HB_VENC_GetH264IntraPred(VENC_CHN VeChn, VENC_H264_INTRA_PRED_S *pstH264IntraPred);
```
**Function Description**
> Retrieves the intra prediction configuration for H.264 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :-------------------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input   |
| pstH264IntraPred | H.264 protocol's intra prediction configuration          |     Output  |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success     |
| Non-zero   | Failure, refer to error codes. |

**Note**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H264_INTRA_PRED_S pstH264IntraPred1;
    memset(&pstH264IntraPred1, 0, sizeof(VENC_H264_INTRA_PRED_S));

    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    pstH264IntraPred1.constrained_intra_pred_flag = HB_TRUE;
    s32Ret = HB_VENC_SetH264IntraPred(VeChn, &pstH264IntraPred1);

    VENC_H264_INTRA_PRED_S pstH264IntraPred2;
    memset(&pstH264IntraPred2, 0, sizeof(VENC_H264_INTRA_PRED_S));
    s32Ret = HB_VENC_GetH264IntraPred(VeChn, &pstH264IntraPred2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```




### HB_VENC_SetH264Trans
**Function Declaration**
```C
int32_t HB_VENC_SetH264Trans(VENC_CHN VeChn, const VENC_H264_TRANS_S *pstH264Trans);
```
**Function Description**
> Configures the H.264 encoding transformation and quantization settings.

**Parameter Descriptions**

| Parameter Name | Description                                          | Input/Output |
| :-------------: | :-------------------------------------------------- | :---------: |
|    VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |    Input   |
|  pstH264Trans  | H.264 protocol-specific channel transformation and quantization attributes |    Input   |

**Return Values**

| Return Value | Description                            |
| :----------: | :-------------------------------------- |
|      0       | Success                                |
| Non-zero     | Failure; refer to error codes         |

**Note**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H264_TRANS_S pstH264Trans1;
    memset(&pstH264Trans1, 0, sizeof(VENC_H264_TRANS_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    s32Ret = HB_VENC_GetRcParam(VeChn, &m_VencChnAttr.stRcAttr);
    m_VencChnAttr.stRcAttr.stH264Cbr.u32BitRate = 3000;
    m_VencChnAttr.stRcAttr.stH264Cbr.u32FrameRate = 30;
    m_VencChnAttr.stRcAttr.stH264Cbr.u32IntraPeriod = 30;
    m_VencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize = 3000;

    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    pstH264Trans1.chroma_cb_qp_offset = 5;
    pstH264Trans1.chroma_cr_qp_offset = 5;
    pstH264Trans1.transform_8x8_enable = HB_TRUE;
    pstH264Trans1.user_scaling_list_enable = 1;
    s32Ret = HB_VENC_SetH264Trans(VeChn, &pstH264Trans1);
    VENC_H264_TRANS_S pstH264Trans2;
    memset(&pstH264Trans2, 0, sizeof(VENC_H264_TRANS_S));
    s32Ret = HB_VENC_GetH264Trans(VeChn, &pstH264Trans2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH264Vui
**Function Declaration**
```C
int32_t HB_VENC_GetH264Vui(VENC_CHN VeChn, VENC_H264_VUI_S *pstH264Vui);
```
**Function Description**
> Retrieves the VUI configuration for H.264 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                                                                     | Input/Output |
| :------------- | :----------------------------------------------------------------------------------------------- | :----------- |
|   VeChn        | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)                                      |     Input    |
| pstH264Vui      | Pointer to the H.264 protocol's Vui parameters for the encoding channel                       |     Output   |

**Return Values**

| Return Value | Description                                                                                   |
| :---------: | :--------------------------------------------------------------------------------------------- |
|     0      | Success                                                                                       |
| Non-zero    | Failure, refer to the error code.                                                               |

**Note**
> None

**Reference Code**
HB_VENC_SetH264Vui reference code

### HB_VENC_SetH265Vui
**Function Declaration**
```C
int32_t HB_VENC_SetH265Vui(VENC_CHN VeChn, const VENC_H265_VUI_S *pstH265Vui);
```
**Function Description**
> Sets the VUI configuration for H.265 encoding channels.

**Parameter Descriptions**

| Parameter Name | Description                                                                                     | Input/Output |
| :------------- | :----------------------------------------------------------------------------------------------- | :----------- |
|   VeChn        | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)                                      |     Input    |
| pstH265Vui      | Pointer to the H.265 protocol's Vui parameters for the encoding channel                       |     Input    |

**Return Values**

| Return Value | Description                                                                                   |
| :---------: | :--------------------------------------------------------------------------------------------- |
|     0      | Success                                                                                       |
| Non-zero    | Failure, refer to the error code.                                                               |

**Note**
> None

**Reference Code**
```C
        int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_VUI_S pstH265Vui1;
    memset(&pstH265Vui1, 0, sizeof(VENC_H265_VUI_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    pstH265Vui1.stVuiTimeInfo.num_ticks_poc_diff_one_minus1 = 1;
    pstH265Vui1.stVuiTimeInfo.num_units_in_tick = 2000;
    pstH265Vui1.stVuiTimeInfo.time_scale = 50000;
    s32Ret = HB_VENC_SetH265Vui(VeChn, &pstH265Vui1);

    VENC_H265_VUI_S pstH265Vui2;
    memset(&pstH265Vui2, 0, sizeof(VENC_H265_VUI_S));
    s32Ret = HB_VENC_GetH265Vui(VeChn, &pstH265Vui2);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```


### HB_VENC_GetH265Vui
**Function Declaration**
```C
int32_t HB_VENC_GetH265Vui(VENC_CHN VeChn, VENC_H265_VUI_S *pstH265Vui);
```
**Function Description**
> Retrieves the VUI configuration for an H.265 encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                      | Input/Output |
| :------------- | :------------------------------------------------ | :----------- |
|   VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) | Input       |
| pstH265Vui     | Pointer to the H.265 VUI parameters              | Output      |

**Return Values**

| Return Value | Description                              |
| :---------- | :----------------------------------------|
|    0        | Success                                  |
| Non-zero    | Failure, refer to error codes             |

**Note**
> None

**Reference Code**
> Refer to the HB_VENC_SetH265Vui reference code for more details.

### HB_VENC_SetRcParam
**Function Declaration**
```C
int32_t HB_VENC_SetRcParam(VENC_CHN VeChn, const VENC_RC_ATTR_S *pstRcParam);
```
**Function Description**
> Sets advanced rate control parameters for the encoding channel.

**Parameter Descriptions**

| Parameter Name | Description                                      | Input/Output |
| :------------- | :------------------------------------------------ | :----------- |
|   VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) | Input       |
| pstRcParam     | Pointer to the advanced rate controller parameters | Input       |

**Return Values**

| Return Value | Description                              |
| :---------- | :----------------------------------------|
|    0        | Success                                  |
| Non-zero    | Failure, refer to error codes             |

**Note**
> None

**Reference Code**
> For more information, consult the HB_VENC_SetRcParam reference code.



### HB_VENC_GetRcParam
**Function Declaration**
```C
int32_t HB_VENC_GetRcParam(VENC_CHN VeChn, VENC_RC_ATTR_S *pstRcParam);
```
**Function Description**
> Retrieves the advanced parameters for the channel's rate control.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :--------------------------------------------------------------- | :-------: |
|   VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |    Input   |
| pstRcParam     | Pointer to the advanced rate control parameters for the encoder channel |    Input   |

**Return Values**

| Return Value | Description |
| :----------: | :----------|
|      0       | Success     |
| Non-zero     | Failure; refer to error codes. |

**Note**
> None

**Reference Code**

### HB_VENC_SetRefParam
**Function Declaration**
```C
int32_t HB_VENC_SetRefParam(VENC_CHN VeChn, const VENC_REF_PARAM_S *pstRefParam);
```
**Function Description**
> Sets the advanced frame skipping reference parameters for H.264/H.265 encoding channels.

**Parameter Descriptions**

| Parameter Name   | Description                                             | Input/Output |
| :--------------: | :----------------------------------------------- | :-------: |
|     VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |    Input   |
| pstRefParam      | Pointer to the H.264/H.265 advanced frame skipping reference parameters |    Input   |

**Return Values**

| Return Value | Description |
| :----------: | :----------|
|      0       | Success     |
| Non-zero     | Failure; refer to error codes. |

**Note**
> None

**Example Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_REF_PARAM_S pstRefParam_test1;
    memset(&pstRefParam_test1, 0x00, sizeof(VENC_REF_PARAM_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    pstRefParam_test1.use_longterm = HB_TRUE;
    pstRefParam_test1.longterm_pic_period = 30;
    pstRefParam_test1.longterm_pic_using_period = 20;
    s32Ret = HB_VENC_SetRefParam(VeChn, &pstRefParam_test1);

    VENC_REF_PARAM_S pstRefParam_test2;
    memset(&pstRefParam_test2, 0x00, sizeof(VENC_REF_PARAM_S));

    s32Ret = HB_VENC_GetRefParam(VeChn, &pstRefParam_test2);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetRefParam
**Function Declaration**
```C
int32_t HB_VENC_GetRefParam(VENC_CHN VeChn, VENC_REF_PARAM_S *pstRefParam);
```
**Function Description**
> Retrieves the advanced frame skipping reference parameters for H.264/H.265 encoding channels.

**Parameter Descriptions**

| Parameter Name | Description                                            | Input/Output |
| :------------- | :------------------------------------------------------ | :----------: |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
| pstRefParam   | Advanced H.264/H.265 frame skipping reference parameters     |     Output   |

**Return Values**

| Return Value | Description |
| :---------: | :----------|
|     0      |         Success        |
| Non-zero   | Failure; refer to error codes. |

**Note**
> None

**Reference Code**
> Reference code for HB_VENC_SetRefParam

### HB_VENC_EnableIDR
**Function Declaration**
```C
int32_t HB_VENC_EnableIDR(VENC_CHN VeChn, HB_BOOL bEnableIDR);
```
**Function Description**
> Enables IDR frames.

**Parameter Descriptions**

**Return Values**

| Return Value | Description |
| :---------: | :----------|
|     0      |         Success        |
| Non-zero   | Failure; refer to error codes. |

**Note**
> None

**Reference Code**
> None (No available example)

### HB_VENC_SetH265SliceSplit
**Function Declaration**
```C
int32_t HB_VENC_SetH265SliceSplit(VENC_CHN VeChn, const VENC_H265_SLICE_SPLIT_S *pstSliceSplit);
```
**Function Description**
> Sets H.265 slice splitting configurations.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :------------------------------------------------------ | :----------: |
|     VeChn     | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
| pstSliceSplit | H.265 stream slice splitting parameters                  |     Input    |

**Return Values**

| Return Value | Description |
| :---------: | :----------|
|     0      |         Success        |
| Non-zero   | Failure; refer to error codes. |

**Note**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_SLICE_SPLIT_S pstSliceSplit1;
    memset(&pstSliceSplit1, 0, sizeof(VENC_H265_SLICE_SPLIT_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    pstSliceSplit1.h265_dependent_slice_arg = 1;
    pstSliceSplit1.h265_dependent_slice_mode = 1;
    pstSliceSplit1.h265_independent_slice_arg = 1;
    pstSliceSplit1.h265_independent_slice_mode = 1;
    s32Ret = HB_VENC_SetH265SliceSplit(VeChn, &pstSliceSplit1);

    VENC_H265_SLICE_SPLIT_S pstSliceSplit2;
    memset(&pstSliceSplit2, 0, sizeof(VENC_H265_SLICE_SPLIT_S));
    s32Ret = HB_VENC_GetH265SliceSplit(VeChn, &pstSliceSplit2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH265SliceSplit
**Function Declaration**
```C
int32_t HB_VENC_GetH265SliceSplit(VENC_CHN VeChn, VENC_H265_SLICE_SPLIT_S *pstSliceSplit);
```
**Function Description**
> Retrieves the configuration for H.265 slice splitting.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :------------------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
|  pstSliceSplit | H.265 stream's slice splitting parameters                |     Output  |

**Return Values**

| Return Value | Description                            |
| :---------: | :-------------------------------------- |
|       0     | Success                                |
| Non-zero    | Failure; see error codes for details.   |

**Note**
> None

**Reference Code**
HB_VENC_SetH265SliceSplit reference code

### HB_VENC_SetH265PredUnit
**Function Declaration**
```C
int32_t HB_VENC_SetH265PredUnit(VENC_CHN VeChn, const VENC_H265_PU_S *pstPredUnit);
```
**Function Description**
> Sets the configuration for H.265 prediction unit (PU).

**Parameter Descriptions**

|  Parameter Name   | Description                                             | Input/Output |
| :---------------: | :------------------------------------------------------- | :----------: |
|       VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
|  pstPredUnit      | H.265 protocol's PU configuration for the encoding channel |      Input   |

**Return Values**

| Return Value | Description                            |
| :---------: | :-------------------------------------- |
|       0     | Success                                |
| Non-zero    | Failure; see error codes for details.   |

**Note**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_PU_S pstPredUnit1;
    memset(&pstPredUnit1, 0, sizeof(VENC_H265_PU_S));

    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    pstPredUnit1.constrained_intra_pred_flag = 1;
    pstPredUnit1.intra_nxn_enable = 0;
    pstPredUnit1.max_num_merge = 1;
    pstPredUnit1.strong_intra_smoothing_enabled_flag = 1;

    s32Ret = HB_VENC_SetH265PredUnit(VeChn, &pstPredUnit1);

    VENC_H265_PU_S pstPredUnit2;
    memset(&pstPredUnit2, 0, sizeof(VENC_H265_PU_S));
    s32Ret = HB_VENC_GetH265PredUnit(VeChn, &pstPredUnit2);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH265PredUnit
**Function Declaration**
```C
int32_t HB_VENC_GetH265PredUnit(VENC_CHN VeChn, VENC_H265_PU_S *pstPredUnit);
```
**Function Description**
> Retrieves the configuration for prediction units in H.265 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                                  | Input/Output |
| :------------- | :------------------------------------------------------------ | :----------: |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
|  pstPredUnit   | H.265 protocol's PU configuration                             |     Output   |

**Return Values**

| Return Value | Description                                    |
| :----------: | :--------------------------------------------- |
|      0       | Successful                                     |
| Non-zero     | Failure; refer to error codes.                  |

**Notes**
> None

**Reference Code**
> See HB_VENC_SetH265PredUnit reference code

### HB_VENC_SetH265Trans
**Function Declaration**
```C
int32_t HB_VENC_SetH265Trans(VENC_CHN VeChn, const VENC_H265_TRANS_S *pstH265Trans);
```
**Function Description**
> Sets the transformation and quantization configurations for H.265 encoding.

**Parameter Descriptions**

|   Parameter Name   | Description                                                  | Input/Output |
| :----------------: | :------------------------------------------------------------ | :----------: |
|      VeChn         | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
|  pstH265Trans     | H.265 protocol's transformation and quantization settings     |     Input    |

**Return Values**

| Return Value | Description                                    |
| :----------: | :--------------------------------------------- |
|      0       | Successful                                     |
| Non-zero     | Failure; refer to error codes.                  |

**Notes**
> None

**Example Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_TRANS_S pstH265Trans1;
    memset(&pstH265Trans1, 0, sizeof(VENC_H265_TRANS_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    pstH265Trans1.chroma_cb_qp_offset = 6;
    pstH265Trans1.chroma_cr_qp_offset = 6;
    pstH265Trans1.user_scaling_list_enable = HB_TRUE;

    s32Ret = HB_VENC_SetH265Trans(VeChn, &pstH265Trans1);
    VENC_H265_TRANS_S pstH265Trans2;
    memset(&pstH265Trans2, 0, sizeof(VENC_H265_TRANS_S));
    s32Ret = HB_VENC_GetH265Trans(VeChn, &pstH265Trans2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH265Trans
**Function Declaration**
```C
int32_t HB_VENC_GetH265Trans(VENC_CHN VeChn, VENC_H265_TRANS_S *pstH265Trans);
```
**Function Description**
> Retrieves the H.265 encoding transformation and quantization configurations.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :------------------------------------------------------- | :----------: |
|    VeChn       | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
|  pstH265Trans  | H.265 protocol's encoding channel transformation/quantization settings |     Output   |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|      0      |     Success    |
| Non-zero   |  Failure; refer to error codes. |

**Note**
> None

**Reference Code**
HB_VENC_SetH265Trans reference code

### HB_VENC_SetH265Dblk
**Function Declaration**
```C
int32_t HB_VENC_SetH265Dblk(VENC_CHN VeChn, const VENC_H265_DBLK_S *pstH265Dblk);
```
**Function Description**
> Sets the H.265 encoding deblocking configuration.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :-------------: | :------------------------------------------------------- | :----------: |
|    VeChn       | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |      Input   |
| pstH265Dblk    | H.265 protocol's encoding channel deblocking configuration |     Input   |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|      0      |     Success    |
| Non-zero   |  Failure; refer to error codes. |

**Note**
> None

**Example Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_DBLK_S pstH265Dblk1;
    memset(&pstH265Dblk1, 0, sizeof(VENC_H265_DBLK_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;

    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    pstH265Dblk1.slice_beta_offset_div2 = 3;
    pstH265Dblk1.slice_tc_offset_div2 = 3;
    pstH265Dblk1.slice_deblocking_filter_disabled_flag = 1;
    s32Ret = HB_VENC_SetH265Dblk(VeChn, &pstH265Dblk1);
    VENC_H265_DBLK_S pstH265Dblk2;
    memset(&pstH265Dblk2, 0, sizeof(VENC_H265_DBLK_S));
    s32Ret = HB_VENC_GetH265Dblk(VeChn, &pstH265Dblk2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH265Dblk
**Function Declaration**
```C
int32_t HB_VENC_GetH265Dblk(VENC_CHN VeChn, VENC_H265_DBLK_S *pstH265Dblk);
```
**Function Description**
> Retrieves the deblocking configuration for H.265 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                    | Input/Output |
| :------------- | :---------------------------------------------- | :----------: |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
| pstH265Dblk    | H.265 protocol's deblocking configuration pointer |     Output   |

**Return Values**

| Return Value | Description                              |
| :----------- | :----------------------------------------|
|      0       | Successful                                |
| Non-zero     | Failure, refer to error codes            |

**Notes**
> None

**Reference Code**
> See reference code for HB_VENC_SetH265Dblk

### HB_VENC_SetH265Sao
**Function Declaration**
```C
int32_t HB_VENC_SetH265Sao(VENC_CHN VeChn, const VENC_H265_SAO_S *pstH265Sao);
```
**Function Description**
> Sets the SAO (Sample Adaptive Offset) configuration for H.265 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :------------------------------------------------------- | :----------: |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input    |
| pstH265Sao     | H.265 protocol's SAO configuration                     |     Input    |

**Return Values**

| Return Value | Description                              |
| :----------- | :----------------------------------------|
|      0       | Successful                                |
| Non-zero     | Failure, refer to error codes            |

**Notes**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_H265_SAO_S pstH265Sao1;
    memset(&pstH265Sao1, 0, sizeof(VENC_H265_SAO_S));

    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;

    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    pstH265Sao1.sample_adaptive_offset_enabled_flag = 1;
    s32Ret = HB_VENC_SetH265Sao(VeChn, &pstH265Sao1);

    VENC_H265_SAO_S pstH265Sao2;
    memset(&pstH265Sao2, 0, sizeof(VENC_H265_SAO_S));
    s32Ret = HB_VENC_GetH265Sao(VeChn, &pstH265Sao2);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetH265Sao
**Function Declaration**
```C
int32_t HB_VENC_GetH265Sao(VENC_CHN VeChn, VENC_H265_SAO_S *pstH265Sao);
```
**Function Description**
> Retrieves the SAO configuration for H.265 encoding.

**Parameter Descriptions**

| Parameter Name | Description                                                                                              | Input/Output |
| :------------: | -------------------------------------------------------------------------------------------------------- | :----------: |
|    VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM)                                                       |     Input    |
|  pstH265Sao   | Pointer to the H.265 protocol's SAO configuration for the encoding channel                                      |     Output   |

**Return Values**

| Return Value | Description                                                                                           |
| :---------: | --------------------------------------------------------------------------------------------------- |
|     0      | Success                                                                                               |
| Non-zero   | Failure; see error codes for details.                                                                   |

**Notes**
> None

**Reference Code**

HB_VENC_SetH265Sao reference code

### HB_VENC_SetIntraRefresh
**Function Declaration**
```C
int32_t HB_VENC_SetIntraRefresh(VENC_CHN VeChn, const HB_VENC_INTRA_REFRESH_S *pstIntraRefresh);
```
**Function Description**
> Sets the parameters for I slice refresh in P frames.

**Parameter Descriptions**

| Parameter Name     | Description                                                                                             | Input/Output |
| :----------------- | ------------------------------------------------------------------------------------------------------- | :----------: |
|      VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM)                                                   |     Input    |
|  pstIntraRefresh  | Pointer to the structure containing I slice refresh settings                                               |     Input    |

**Return Values**

| Return Value | Description                                                                                           |
| :---------: | --------------------------------------------------------------------------------------------------- |
|     0      | Success                                                                                               |
| Non-zero   | Failure; see error codes for details.                                                                   |

**Notes**
> None

**Reference Code**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_INTRA_REFRESH_S pstIntraRefresh1;
    memset(&pstIntraRefresh1, 0, sizeof(VENC_INTRA_REFRESH_S));

    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    pstIntraRefresh1.bRefreshEnable = HB_TRUE;
    pstIntraRefresh1.enIntraRefreshMode = INTRA_REFRESH_COLUMN;
    pstIntraRefresh1.u32RefreshNum = 2;
    s32Ret = HB_VENC_SetIntraRefresh(VeChn, &pstIntraRefresh1);
    VENC_INTRA_REFRESH_S pstIntraRefresh2;
    memset(&pstIntraRefresh2, 0, sizeof(VENC_INTRA_REFRESH_S));
    s32Ret = HB_VENC_GetIntraRefresh(VeChn, &pstIntraRefresh2);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```


### HB_VENC_GetIntraRefresh
**Function Declaration**
```C
int32_t HB_VENC_GetIntraRefresh(VENC_CHN VeChn, VENC_INTRA_REFRESH_S *pstIntraRefresh);
```
**Function Description**
> Retrieves the parameters for the I slice in a P frame.

**Parameter Descriptions**

| Parameter Name | Description                                                                                     | Input/Output |
| :------------- | :----------------------------------------------------------------------------------------------- | :----------- |
|     VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM)                                     |    Input     |
|  pstIntraRefresh | Structure containing the I slice refresh parameters                                             |    Output    |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success     |
| Non-zero    | Failure; see error codes. |

**Note**
> None

**Reference Code (HB_VENC_SetIntraRefresh example)**
> HB_VENC_SetIntraRefresh Reference Code



### HB_VENC_SetCuPrediction
【Function Declaration】
```C
int32_t HB_VENC_SetCuPrediction(VENC_CHN VeChn, const VENC_CU_PREDICTION_S * pstCuPrediction);
```
【Function Description】
> Set the tendency of CU mode prediction.

【Parameter Description】

| Parameter Name | Description                                         | Input/Output |
| :------------: | :-------------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |    Input     |
| pstCuPrediction   | Parameters for selecting the tendency of CU mode     |    Input    |

【Return Value】

| Return Value | Description |
| :----------: | :---------- |
|      0       | Success     |
|    Non-zero  | Failure, see error code. |

【Notes】
> None

【Reference Code】
```
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;

    VENC_CU_PREDICTION_S pstCuPrediction1;
    memset(&pstCuPrediction1, 0, sizeof(VENC_CU_PREDICTION_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H265;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH265Cbr.u32BitRate = 3000;
    pstRcParam->stH265Cbr.u32FrameRate = 30;
    pstRcParam->stH265Cbr.u32IntraPeriod = 30;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);

    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    pstCuPrediction1.mode_decision_enable = HB_TRUE;
    pstCuPrediction1.pu04_delta_rate = 2;
    pstCuPrediction1.cu32_merge_delta_rate = 3;
    s32Ret = HB_VENC_SetCuPrediction(VeChn, &pstCuPrediction1);

    VENC_CU_PREDICTION_S pstCuPrediction2;
    memset(&pstCuPrediction2, 0, sizeof(VENC_CU_PREDICTION_S));
    s32Ret = HB_VENC_GetCuPrediction(VeChn, &pstCuPrediction2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```



### HB_VENC_GetCuPrediction
**Function Declaration:**
```C
int32_t HB_VENC_GetCuPrediction(VENC_CHN VeChn, VENC_CU_PREDICTION_S * pstCuPrediction);
```
**Function Description:**
> Retrieves the tendency configuration for CU modes.

**Parameter Descriptions:**

| Parameter Name | Description                                                                                   | Input/Output |
| :------------- | :--------------------------------------------------------------------------------------------- | :----------- |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM)                                      |    Input    |
| pstCuPrediction | Pointer to the CU prediction preference parameters                                            |    Output   |

**Return Values:**

| Return Value | Description                           |
| :---------: | :---------------------------------- |
|      0      | Successful                           |
| Non-zero    | Failure, refer to error codes        |

**Caution:**
> None

**Reference Code:**
> See the reference code for HB_VENC_SetCuPrediction

### HB_VENC_SetJpegParam
**Function Declaration:**
```C
int32_t HB_VENC_SetJpegParam(VENC_CHN VeChn, const VENC_JPEG_PARAM_S * pstJpegParam);
```
**Function Description:**
> Sets advanced parameters for JPEG protocol encoding channels.

**Parameter Descriptions:**

| Parameter Name | Description                                                                                         | Input/Output |
| :------------- | :-------------------------------------------------------------------------------------------------- | :----------- |
|    VeChn       | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM)                                          |    Input    |
| pstJpegParam  | Pointer to the JPEG encoding channel attributes                                                       |    Input    |

**Return Values:**

| Return Value | Description                           |
| :---------: | :---------------------------------- |
|      0      | Successful                           |
| Non-zero    | Failure, refer to error codes        |

**Caution:**
> None

**Reference Code:**
> Refer to the reference code for HB_VENC_SetJpegParam


### HB_VENC_GetJpegParam
【Function Declaration】
```C
int32_t HB_VENC_GetJpegParam(VENC_CHN VeChn, VENC_JPEG_PARAM_S * pstJpegParam);
```
【Description】
> Get the advanced parameter settings for JPEG protocol encoding channel.【Parameter Description】

|   Parameter Name   | Description                                             | Input/Output |
| :----------------: | :------------------------------------------------------ | :----------: |
|      VeChn         | Channel number of the encoding.<br/>Range: [0, VENC_MAX_CHN_NUM) |    Input     |
|   pstJpegParam     | Pointer to the encoding channel attributes                 |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :--------- |
|      0       | Success     |
|   Non-zero   | Failure, see error code |

【Note】
> None

【Reference Code】
> Reference code for HB_VENC_SetJpegParam

### HB_VENC_SetMjpegParam
【Function Declaration】
```C
int32_t HB_VENC_SetJpegParam(VENC_CHN VeChn, const VENC_MJPEG_PARAM_S * pstMjpegParam);
```
【Description】
> Set advanced parameters of MJPEG protocol encoding channel.

【Parameter Description】

|   Parameter Name    | Description                                             | Input/Output |
| :-----------------: | :------------------------------------------------------ | :----------: |
|       VeChn         | Channel number of the encoding.<br/>Range: [0, VENC_MAX_CHN_NUM) |    Input     |
|   pstMjpegParam     | Pointer to the encoding channel attributes                 |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :--------- |
|      0       | Success     |
|   Non-zero   | Failure, see error code |

【Note】
> None

【Reference Code】
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn0 = 0;
    VENC_CHN VeChn1 = 1;

    int32_t Width = 1920;
    int32_t Height = 1080;
    VENC_JPEG_PARAM_S pstJpegParam;
    VENC_MJPEG_PARAM_S pstMjpegParam;
    memset(&pstJpegParam, 0, sizeof(VENC_JPEG_PARAM_S));
    memset(&pstMjpegParam, 0, sizeof(VENC_MJPEG_PARAM_S));

    VENC_CHN_ATTR_S m0_VencChnAttr;
    memset(&m0_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m0_VencChnAttr.stVencAttr.enType = PT_JPEG;
    m0_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m0_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m0_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m0_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m0_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m0_VencChnAttr.stVencAttr.enPixelFormat = pixFmt;
    m0_VencChnAttr.stVencAttr.u32BitStreamBufferCount = 1;
    m0_VencChnAttr.stVencAttr.u32FrameBufferCount = 2;
    m0_VencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
    m0_VencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
    m0_VencChnAttr.stVencAttr.stAttrJpeg.quality_factor = 0;
    m0_VencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
    m0_VencChnAttr.stVencAttr.u32BitStreamBufSize = 4096*1096;
    VENC_CHN_ATTR_S m1_VencChnAttr;
    memset(&m1_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m1_VencChnAttr.stVencAttr.enType = PT_MJPEG;
    m1_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m1_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m1_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m1_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m1_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m1_VencChnAttr.stVencAttr.enPixelFormat = pixFmt;
    m1_VencChnAttr.stVencAttr.u32BitStreamBufferCount = 1;
    m1_VencChnAttr.stVencAttr.u32FrameBufferCount = 2;
    m1_VencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
    m1_VencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
    m1_VencChnAttr.stVencAttr.stAttrJpeg.quality_factor = 0;
    m1_VencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
    m1_VencChnAttr.stVencAttr.u32BitStreamBufSize = 4096*1096;

    s32Ret = HB_VENC_CreateChn(VeChn0, &m0_VencChnAttr);
    s32Ret = HB_VENC_CreateChn(VeChn1, &m1_VencChnAttr);
    HB_VENC_SetJpegParam(VeChn0, &pstJpegParam);
    HB_VENC_GetJpegParam(VeChn0, &pstJpegParam);
    HB_VENC_SetMjpegParam(VeChn1, &pstMjpegParam);
    HB_VENC_GetMjpegParam(VeChn1, &pstMjpegParam);
    s32Ret = HB_VENC_DestroyChn(VeChn0);
    s32Ret = HB_VENC_DestroyChn(VeChn1);
```

### HB_VENC_GetMjpegParam
【Function Declaration】
```C
int32_t HB_VENC_GetMjpegParam(VENC_CHN VeChn, VENC_MJPEG_PARAM_S *pstMjpegParam);
```
【Function Description】
> Get advanced parameter settings of MJPEG protocol encoding channel.

【Parameter Description】

| Parameter Name | Description                                               | Input/Output |
| :------------: | :-------------------------------------------------------- | :-----------: |
|     VeChn      | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |     Input     |
| pstMjpegParam  | Pointer to the attributes of encoding channel             |     Input     |

【Return Value】

| Return Value | Description     |
| :----------: | :-------------- |
|      0       | Success         |
|     Non-0     | Failed, see error code |

【Notes】
> None

【Reference Code】
> Reference code of HB_VENC_SetMjpegParam



### HB_VENC_GetFd
**Function Declaration:**
```C
int32_t HB_VENC_GetFd(VENC_CHN VeChn, int32_t *fd)
```
**Function Description:**
> Retrieves the file descriptor for the encoding channel.

**Parameter Descriptions:**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :--------------------------------------------------------------- | :-------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |   Input    |
| fd             | Pointer to return the encoding channel file descriptor         |   Input    |

**Return Values:**

| Return Value | Description |
| :---------: | :---------: |
|     0      | Success    |
| Non-zero    | Failure, refer to error codes. |

**Note:**
> None

**Reference Code:**
```C
    int32_t s32Ret = 0;
    VENC_CHN VeChn = 0;
    int32_t Width = 1920;
    int32_t Height = 1080;
    int32_t Test_fd1 = 0;
    int32_t Test_fd2 = 0;

    VENC_CU_PREDICTION_S pstCuPrediction1;
    memset(&pstCuPrediction1, 0, sizeof(VENC_CU_PREDICTION_S));
    VENC_CHN_ATTR_S m_VencChnAttr;
    memset(&m_VencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    m_VencChnAttr.stVencAttr.enType = PT_H264;
    m_VencChnAttr.stVencAttr.u32PicWidth = Width;
    m_VencChnAttr.stVencAttr.u32PicHeight = Height;
    m_VencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    m_VencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    m_VencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_profile = 0;
    m_VencChnAttr.stVencAttr.stAttrH264.h264_level = 0;
    m_VencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    m_VencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    m_VencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    VENC_RC_ATTR_S *pstRcParam = &(m_VencChnAttr.stRcAttr);
    s32Ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
    pstRcParam->stH264Cbr.u32BitRate = 3000;
    pstRcParam->stH264Cbr.u32FrameRate = 30;
    pstRcParam->stH264Cbr.u32IntraPeriod = 30;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;

    s32Ret = HB_VENC_CreateChn(VeChn, &m_VencChnAttr);
    HB_VENC_SetChnAttr(VeChn, &m_VencChnAttr);
    s32Ret = HB_VENC_GetFd(VeChn, &Test_fd1);
    s32Ret = HB_VENC_GetFd(VeChn, &Test_fd2);
    s32Ret = HB_VENC_CloseFd(VeChn, Test_fd1);
    s32Ret = HB_VENC_CloseFd(VeChn, Test_fd2);
    s32Ret = HB_VENC_DestroyChn(VeChn);
```

### HB_VENC_CloseFd
**Function Declaration:**
```C
int32_t HB_VENC_CloseFd(VENC_CHN VeChn, int32_t fd)
```
**Function Description:**
> Closes the device file handle associated with the encoding channel.

**Parameter Descriptions:**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :--------------------------------------------------------------- | :-------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |   Input    |
| fd             | Input encoding channel file descriptor                     |   Input    |

**Return Values:**

| Return Value | Description |
| :---------: | :---------: |
|     0      | Success    |
| Non-zero    | Failure, refer to error codes. |

**Note:**
> None

**Reference Code (HB_VENC_GetFd):**
> See the previous reference code for HB_VENC_GetFd.

### HB_VENC_QueryStatus
**Function Declaration:**
```C
int32_t HB_VENC_QueryStatus(VENC_CHN VeChn, VENC_CHN_STATUS_S *pstStatus)
```
**Function Description:**
> Queries the status of the encoding channel.

**Parameter Descriptions:**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :--------------------------------------------------------------- | :-------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |   Input    |
| pstStatus      | Pointer to the encoding channel status structure            |   Input    |

**Return Values:**

| Return Value | Description |
| :---------: | :---------: |
|     0      | Success    |
| Non-zero    | Failure, refer to error codes. |

**Note:**
> None

**Reference Code:**

### HB_VENC_InserUserData
**Function Declaration:**
```C
int32_t HB_VENC_InserUserData(VENC_CHN VeChn, uint8_t *pu8Data,
                            uint32_t u32Len)
```
**Function Description:**
> Inserts user data into the encoding channel.

**Parameter Descriptions:**

| Parameter Name | Description                                             | Input/Output |
| :------------- | :--------------------------------------------------------------- | :-------: |
| VeChn          | Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM) |   Input    |
| pu8Data        | Pointer to user data                                      |   Input    |
| u32Len         | Length of user data                                       |   Input    |

**Return Values:**

| Return Value | Description |
| :---------: | :---------: |
|     0      | Success    |
| Non-zero    | Failure, refer to error codes. |

**Note:**
> None

**Reference Code:**
> See the previous reference code for HB_VENC_GetFd, as no separate reference code is provided for this function.



### HB_VENC_SetChnParam
【Function Declaration】
```C
int32_t HB_VENC_SetChnParam(VENC_CHN VeChn, const VENC_CHN_PARAM_S *pstChnParam)
```
【Function Description】
> Set frame rate control parameters for the encoding channel.

【Parameter Description】

|  Parameter Name  | Description                                                | Input/Output |
| :--------------: | :-------------------------------------------------------- | :----------: |
|      VeChn       | Encoding channel number.<br/>Value Range: [0, VENC_MAX_CHN_NUM) |     Input    |
|   pstChnParam    | Frame rate control parameters                             |     Input    |

【Return Value】

| Return Value |       Description      |
| :----------: | :--------------------- |
|      0       |        Success         |
|     Non-0    | Failure, see error code. |【Notice】
>  N/A

【Reference Code】
> N/A

### HB_VENC_GetChnParam
【Function Declaration】
```C
int32_t HB_VENC_GetChnParam(VENC_CHN VeChn, VENC_CHN_PARAM_S *pstChnParam)
```
【Function Description】
> Get frame rate control parameters of the encoding channel.

【Parameter Description】

|  Parameter Name  |                   Description                        |   Input/Output   |
| :--------------: | :-------------------------------------------------- | :--------------: |
|      VeChn       |        Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)     |       Input       |
|   pstChnParam    |        Frame rate control parameters        |      Output       |

【Return Value】

| Return Value |                   Description                    |
| :----------: | :---------------------------------------------- |
|      0       |                     Success                      |
|   Non-zero   |              Failure, see error code.              |

【Notice】
>  N/A

【Reference Code】
> N/A

### HB_VENC_SetModParam
【Function Declaration】
```C
int32_t HB_VENC_SetModParam(VENC_CHN VeChn, const VENC_PARAM_MOD_S *pstModParam)
```
【Function Description】
> Set whether VPS, SPS, PPS, and IDR of the encoding channel are output in one frame.

【Parameter Description】

|  Parameter Name  |                   Description                        |   Input/Output   |
| :--------------: | :-------------------------------------------------- | :--------------: |
|      VeChn       |        Encoding channel number.<br/>Range: [0, VENC_MAX_CHN_NUM)     |       Input       |
|   pstModParam    |        Pointer to ModParam                |       Input       |【Return Value】

| Return Value | Description |
| :----------: | :---------- |
|     0        | Success     |
|   Non-zero   | Failure, see error code. |

【Note】

> None

【Reference Code】

### HB_VENC_GetModParam
【Function Declaration】
```C
int32_t HB_VENC_GetModParam(VENC_CHN VeChn, VENC_PARAM_MOD_S *pstModParam)
```
【Function Description】
> Get whether the encoding channel VPS, SPS, PPS, IDR output in one frame.

【Parameter Description】

|   Parameter Name   | Description                                    | Input/Output |
| :---------------:  | :--------------------------------------------  | :-----------:|
|       VeChn        | Encoding channel number. <br/>Range: [0, VENC_MAX_CHN_NUM) |    Input     |
|    pstModParam     | Pointer to ModParam                            |   Output     |

【Return Value】

| Return Value | Description |
| :----------: | :---------- |
|     0        | Success     |
|   Non-zero   | Failure, see error code. |

【Note】

> None

【Reference Code】

### HB_VENC_SendFrameEx
【Function Declaration】
```C
int32_t HB_VENC_SendFrameEx(VENC_CHN VeChn, const USER_FRAME_INFO_S *pstFrame, int32_t s32MilliSec)
```
【Function Description】
> User sends the original image and its QpMap table for encoding.

【Parameter Description】

|   Parameter Name   | Description | Input/Output |
| :---------: | :----------------------------------------------- | :-------: |
|    VeChn    | Channel number for encoding. <br/>Value range: [0, VENC_MAX_CHN_NUM) |   Input    |
|  pstFrame   | Pointer to the structure of original image information.                           |   Input    |
| s32MilliSec | Timeout                                     |   Input    |

【Return Value】

| Return Value |               Description |
| :----: | :-----------------|
|   0    |               Success |
|  Non-zero   | Failure, see error code. |

【Notes】
> None

【Reference Code】

### HB_VENC_SetAverageQp
【Function Declaration】
```C
int32_t HB_VENC_SetAverageQp(VENC_CHN VeChn, int averageQp)
```
【Functional Description】
> Set relative Qpmap averageQp.

【Parameter Description】

| Parameter Name  | Description                                             | Input/Output |
| :-------: | :----------------------------------------------- | :-------: |
|   VeChn   | Channel number for encoding. <br/>Value range: [0, VENC_MAX_CHN_NUM) |   Input    |
| averageQp | Relative QPMAP averageqp                              |   Input    |

【Return Value】

| Return Value |               Description |
| :----: | :-----------------|
|   0    |               Success |
|  Non-zero   | Failure, see error code. |

【Notes】
> None

【Reference Code】

### HB_VENC_GetAverageQp
【Function Declaration】
```C
int32_t HB_VENC_GetAverageQp(VENC_CHN VeChn, int *averageQp)
```
【Functional Description】> Obtain the relative QPmap average QP.

[Parameter Description]

| Parameter Name | Description                                     | Input/Output |
| :-------------:| :---------------------------------------------- | :----------: |
|    VeChn       | Encoding channel number.<br/>Value range: [0, VENC_MAX_CHN_NUM) |   Input    |
|  averageQp     | Relative QPmap average QP                      |   Output    |

[Return Value]

| Return Value |           Description |
| :----------: | :------------------- |
|      0       |         Success      |
|   Non-zero   |    Failure. See the error code for details. |

[Note]
> None

[Sample Code]

### HB_VENC_Set3DNRParam
[Function Declaration]
```C
int32_t HB_VENC_Set3DNRParam(VENC_CHN VeChn, VENC_3DNR_PARAMS *param)
```
[Function Description]
> Set H265 3DNR parameters.

[Parameter Description]

| Parameter Name | Description                                     | Input/Output |
| :-------------:| :---------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number.<br/>Value range: [0, VENC_MAX_CHN_NUM) |   Input    |
|     param      | Pointer to 3DNR parameters                        |   Input    |

[Return Value]

| Return Value |           Description |
| :----------: | :------------------- |
|      0       |         Success      |
|   Non-zero   |    Failure. See the error code for details. |

[Note]
> None

[Sample Code]

### HB_VENC_Get3DNRParam
[Function Declaration]【Function Description】
```C
int32_t HB_VENC_Get3DNRParam(VENC_CHN VeChn, VENC_3DNR_PARAMS *param)
```
【Description】
> Get the H265 3DNR parameters.

【Parameter Description】

| Parameter Name | Description                                              | Input/Output |
| :------------: | :------------------------------------------------------- | :----------: |
|     VeChn      | Encoding channel number. <br/>Value range: [0, VENC_MAX_CHN_NUM) |     Input    |
|     param      | 3DNR parameter pointer                                   |     Input    |

【Return Value】

| Return Value | Description |
| :----------: | :--------- |
|      0       |   Success   |
|     non-zero     | Failure, see error code |

【Notes】
> None.

【Reference Code】

## Data Structure
Variables in the structure cannot be adjusted dynamically and need to be set before HB_VENC_SetChnAttr. Variables that can be set dynamically can be set at any time.

### HB_PIXEL_FORMAT_E
【Description】
> Definition of the enumeration of encoder input image types.

【Structure Definition】
```C
typedef enum HB_PIXEL_FORMAT_E {
    HB_PIXEL_FORMAT_NONE = -1,
    HB_PIXEL_FORMAT_YUV420P,
    HB_PIXEL_FORMAT_NV12,
    HB_PIXEL_FORMAT_NV21,
    HB_PIXEL_FORMAT_YUV422P,
    HB_PIXEL_FORMAT_NV16,
    HB_PIXEL_FORMAT_NV61,
    HB_PIXEL_FORMAT_YUYV422,
    HB_PIXEL_FORMAT_YVYU422,
    HB_PIXEL_FORMAT_UYVY422,
    HB_PIXEL_FORMAT_VYUY422,
    HB_PIXEL_FORMAT_YUV444,
    HB_PIXEL_FORMAT_YUV444P,
    HB_PIXEL_FORMAT_NV24,
    HB_PIXEL_FORMAT_NV42,
    HB_PIXEL_FORMAT_YUV440P,
    HB_PIXEL_FORMAT_YUV400,
    HB_PIXEL_FORMAT_TOTAL,
} PIXEL_FORMAT_E;
```


### PAYLOAD_TYPE_E
【Description】
> Defines the enumeration of encoder types.

【Structure Definition】
```C
typedef enum {
    PT_PCMU = 0,
    PT_1016 = 1,
    PT_G721 = 2,
    PT_GSM = 3,
    PT_G723 = 4,
    PT_DVI4_8K = 5,
    PT_DVI4_16K = 6,
    PT_LPC = 7,
    PT_PCMA = 8,
    PT_G722 = 9,
    PT_S16BE_STEREO = 10,
    PT_S16BE_MONO = 11,
    PT_QCELP = 12,
    PT_CN = 13,
    PT_MPEGAUDIO = 14,
    PT_G728 = 15,
    PT_DVI4_3 = 16,
    PT_DVI4_4 = 17,
    PT_G729 = 18,
    PT_G711A = 19,
    PT_G711U = 20,
    PT_G726 = 21,
    PT_G729A = 22,
    PT_LPCM = 23,
    PT_CelB = 25,
    PT_JPEG = 26,
    PT_CUSM = 27,
    PT_NV = 28,
    PT_PICW = 29,
    PT_CPV = 30,
    PT_H261 = 31,
    PT_MPEGVIDEO = 32,
    PT_MPEG2TS = 33,
    PT_H263 = 34,
    PT_SPEG = 35,
    PT_MPEG2VIDEO = 36,
    PT_AAC = 37,
    PT_WMA9STD = 38,
    PT_HEAAC = 39,
    PT_PCM_VOICE = 40,
    PT_PCM_AUDIO = 41,
    PT_AACLC = 42,
    PT_MP3 = 43,
    PT_ADPCMA = 49,
    PT_AEC = 50,
    PT_X_LD = 95,
    PT_H264 = 96,
    PT_D_GSM_HR = 200,
    PT_D_GSM_EFR = 201,
    PT_D_L8 = 202,
    PT_D_RED = 203,
    PT_D_VDVI = 204,
    PT_D_BT656 = 220,
    PT_D_H263_1998 = 221,
    PT_D_MP1S = 222,
    PT_D_MP2P = 223,
    PT_D_BMPEG = 224,
    PT_MP4VIDEO = 230,
    PT_MP4AUDIO = 237,
    PT_VC1 = 238,
    PT_JVC_ASF = 255,
    PT_D_AVI = 256,
    PT_DIVX3 = 257,
    PT_AVS = 258,
    PT_REAL8 = 259,
    PT_REAL9 = 260,
    PT_VP6 = 261,
    PT_VP6F = 262,
    PT_VP6A = 263,
    PT_SORENSON = 264,
    PT_H265 = 265,
    PT_MAX = 266,
    PT_AMR = 1001,
    PT_MJPEG = 1002,
    PT_AMRWB = 1003,
    PT_BUTT
} PAYLOAD_TYPE_E;
```

【Member Description】

### HB_ROTATION_E
【Description】
> Define the rotation angle enumeration.

【Structure Definition】
```C
typedef enum HB_CODEC_ROTATION_S {
    CODEC_ROTATION_0 = 0,```CODEC_ROTATION_90 = 1,
    CODEC_ROTATION_180 = 2,
    CODEC_ROTATION_270 = 3,
    ROTATION_BUTT
} CODEC_ROTATION_E;
```
【Member explanation】

|       Member        |       Meaning       |
| :-----------------: | :-----------------: |
|  CODEC_ROTATION_0   | No rotation, 0 degrees rotation. |
|  CODEC_ROTATION_90  |     90 degrees rotation.      |
| CODEC_ROTATION_180  |     180 degrees rotation.     |
| CODEC_ROTATION_270  |     270 degrees rotation.     |

### MIRROR_FLIP_E
【Description】
> Defines an enumeration of mirror flip methods.

【Structure definition】
```C
typedef enum HB_MIRROR_FLIP_E {
    DIRECTION_NONE = 0,
    VERTICAL = 1,
    HORIZONTAL = 2,
    HOR_VER = 3,
    DIRECTION_BUTT,
} MIRROR_FLIP_E;
```
【Member explanation】

|    Member    |        Meaning       |
| :----------: | :------------------: |
| DIRECTION_NONE |    No mirror operation    |
|  HORIZONTAL   | Mirror operation along the horizontal direction |
|   VERTICAL    | Mirror operation along the vertical direction |
|   HOR_VER     | Mirror operation along both horizontal and vertical directions |

### HB_VENC_H264_PROFILE_E
【Description】
> Defines H264 profile enumeration.

【Structure definition】
```C
typedef enum HB_VENC_H264_PROFILE_E {
    HB_H264_PROFILE_UNSPECIFIED,
    HB_H264_PROFILE_BP,
    HB_H264_PROFILE_MP,
    HB_H264_PROFILE_EXTENDED,
    HB_H264_PROFILE_HP,
    HB_H264_PROFILE_HIGH10,
	HB_H264_PROFILE_HIGH422,
	HB_H264_PROFILE_HIGHT444
} VENC_H264_PROFILE_E;
```


### HB_VENC_H264_LEVEL
【Description】
> Defines the H264 level enumeration.

【Structure Definition】
```C
typedef enum HB_VENC_H264_LEVEL {
	HB_H264_LEVEL_UNSPECIFIED,
	HB_H264_LEVEL1 = 10,
	HB_H264_LEVEL1b = 9,
	HB_H264_LEVEL1_1 = 11,
	HB_H264_LEVEL1_2 = 12,
	HB_H264_LEVEL1_3 = 13,
	HB_H264_LEVEL2 = 20,
	HB_H264_LEVEL2_1 = 21,
	HB_H264_LEVEL2_2 = 22,
	HB_H264_LEVEL3 = 30,
	HB_H264_LEVEL3_1 = 31,
	HB_H264_LEVEL3_2 = 32,
	HB_H264_LEVEL4 = 40,
	HB_H264_LEVEL4_1 = 41,
	HB_H264_LEVEL4_2 = 42,
	HB_H264_LEVEL5 = 50,
	HB_H264_LEVEL5_1 = 51,
	HB_H264_LEVEL5_2 = 52,
} HB_H264_LEVEL_E;
```

### HB_VENC_H265_LEVEL
【Description】
> Defines the H265 level enumeration.

【Structure Definition】
```C
typedef enum HB_VENC_H265_LEVEL {
    HB_H265_LEVEL_UNSPECIFIED,
    HB_H265_LEVEL1 = 30,
    HB_H265_LEVEL2 = 60,
    HB_H265_LEVEL2_1 = 63,
    HB_H265_LEVEL3 = 90,
    HB_H265_LEVEL3_1 = 93,
    HB_H265_LEVEL4 = 120,
    HB_H265_LEVEL4_1 = 123,
    HB_H265_LEVEL5 = 150,
    HB_H265_LEVEL5_1 = 153,
} HB_H265_LEVEL_E;
```



### CODEC_RECT_S
**Description**
> Defines a structure for rectangle region information.

**Struct Definition**
```C
typedef struct HB_CODEC_RECT_S {
    int32_t s32X;
    int32_t s32Y;
    uint32_t u32Width;
    uint32_t u32Height;
} CODEC_RECT_S;
```
**Member Descriptions**

|   Member    |  Meaning  |
| :-------: | :----: |
|   s32X    | Horizontal coordinate |
|   s32Y    | Vertical coordinate |
| u32Width  | Width |
| u32Height | Height |

### VENC_ATTR_H264_S
**Description**
> Defines a structure for H.264 encoding attributes.

**Struct Definition**
```C
typedef struct HB_VENC_ATTR_H264_S {
    VENC_H264_PROFILE_E h264_profile;
    HB_H264_LEVEL_E h264_level;
} VENC_ATTR_H264_S;
```
**Member Descriptions**

|     Member     |             Meaning             |
| :----------: | :--------------------------: |
| h264_profile | H264 profile, cannot be dynamically configured. |
|  h264_level  | H264 level, cannot be dynamically configured. |

### VENC_ATTR_H265_S
**Description**
> Defines a structure for H.265 encoding attributes.

**Struct Definition**
```C
typedef struct HB_VENC_ATTR_H265_S {
    HB_BOOL main_still_picture_profile_enable;
    int32_t s32h265_tier;
    HB_BOOL transform_skip_enabled_flag;
    uint32_t lossless_mode;
    uint32_t tmvp_Enable;
    uint32_t wpp_Enable;
    HB_H265_LEVEL_E h265_level;
} VENC_ATTR_H265_S;
```
**Member Descriptions**

|               Member                |                         Meaning                          |
| :-------------------------------: | :---------------------------------------------------: |
| main_still_picture_profile_enable | Enables H265 main still picture profile, cannot be dynamically configured. |
|           s32h265_tier            | Sets H265 tier information, cannot be dynamically configured. |
|    transform_skip_enabled_flag    | Enables transform skip for intra CU, cannot be dynamically configured. |
|           lossless_mode           | Enables lossless encoding mode, cannot be dynamically configured. |
|            tmvp_Enable            | Enables temporal motion vector prediction, cannot be dynamically configured. |
|            wpp_Enable             | Enables wpp, cannot be dynamically configured. |
|            h265_level             | H265 level, cannot be dynamically configured. |



### VENC_ATTR_MJPEG_S

**Description**
> Defines the structure for MJPEG encoding attributes.

**Structure Definition**
```C
typedef struct HB_VENC_ATTR_MJPEG_S {
    uint32_t restart_interval;
    HB_BOOL huff_table_valid;
    uint8_t huff_luma_dc_bits[16];
    uint8_t huff_luma_dc_val[16];
    uint8_t huff_luma_ac_bits[16];
    uint8_t huff_luma_ac_val[256];
    uint8_t huff_chroma_dc_bits[16];
    uint8_t huff_chroma_ac_bits[16];
    uint8_t huff_chroma_dc_val[16];
    uint8_t huff_chroma_ac_val[256];
    HB_BOOL extended_sequential;
} VENC_ATTR_MJPEG_S;
```

**Member Descriptions**

| **Member**          |                          **Meaning**                           |
| :------------------: | :--------------------------------------------------------: |
| restart_interval   | The number of MCU (Motion Compensation Unit) in an independent scan sequence, not dynamically configurable. |
| huff_table_valid   | Enables/disables the Huffman table, not dynamically configurable. |
| huff_luma_dc_bits  | Huffman brightness DC bit length table, not dynamically configurable. |
| huff_luma_dc_val   | Huffman brightness DC huffvalue table, not dynamically configurable. |
| huff_luma_ac_bits  | Huffman brightness AC bit length table, not dynamically configurable. |
| huff_luma_ac_val   | Huffman brightness AC huffvalue table, not dynamically configurable. |
| huff_chroma_dc_bits | Huffman chroma DC bit length table, not dynamically configurable. |
| huff_chroma_ac_bits | Huffman chroma AC bit length table, not dynamically configurable. |
| huff_chroma_dc_val  | Huffman chroma DC huffvalue table, not dynamically configurable. |
| huff_chroma_ac_val  | Huffman chroma AC huffvalue table, not dynamically configurable. |
| extended_sequential | A 12-bit mode, not dynamically configurable. |



### VENC_ATTR_JPEG_S
**Description**
> Defines the structure for JPEG encoding attributes.

**Structure Definition**
```C
typedef struct HB_VENC_ATTR_JPEG_S {
    HB_BOOL dcf_enable;
    uint32_t restart_interval;
    uint32_t quality_factor;
    HB_BOOL huff_table_valid;
    uint8_t huff_luma_dc_bits[16];
    uint8_t huff_luma_dc_val[16];
    uint8_t huff_luma_ac_bits[16];
    uint8_t huff_luma_ac_val[256];
    uint8_t huff_chroma_dc_bits[16];
    uint8_t huff_chroma_ac_bits[16];
    uint8_t huff_chroma_dc_val[16];
    uint8_t huff_chroma_ac_val[256];
    HB_BOOL extended_sequential;
} VENC_ATTR_JPEG_S;
```
**Member Descriptions**

| **Member**       | **Meaning**                                                                                   |
| :--------------: | :-----------------------------------------------------------------------------------------------: |
| dcf_enable       | Enables DCF (Difference Coefficient Format), not dynamically configurable.                        |
| restart_interval | Number of MCU (Minimum Coding Units) in an independent scan sequence, not configurable.          |
| quality_factor   | Quality factor, larger values result in lower compression rate, less loss, and better quality, dynamic. |
| huff_table_valid | Enables Huffman tables, not configurable.                                                       |
| huff_luma_dc_bits | Huffman luma DC bit length table, not configurable.                                            |
| huff_luma_dc_val | Huffman luma DC huffvalue table, not configurable.                                             |
| huff_luma_ac_bits | Huffman luma AC bit length table, not configurable.                                           |
| huff_luma_ac_val | Huffman luma AC huffvalue table, not configurable.                                             |
| huff_chroma_dc_bits | Huffman chroma DC bit length table, not configurable.                                          |
| huff_chroma_ac_bits | Huffman chroma AC bit length table, not configurable.                                         |
| huff_chroma_dc_val | Huffman chroma DC huffvalue table, not configurable.                                          |
| huff_chroma_ac_val | Huffman chroma AC huffvalue table, not configurable.                                         |
| extended_sequential | 12-bit mode, not configurable.                                                                |



### VENC_ATTR_S
**Description**
> Defines the structure for encoder attributes.

**Struct Definition**
```C
typedef struct HB_VENC_ATTR_S {
    PAYLOAD_TYPE_E enType;
    uint32_t u32PicWidth;
    uint32_t u32PicHeight;
    PIXEL_FORMAT_E enPixelFormat;
    uint32_t u32FrameBufferCount;
    uint32_t u32BitStreamBufferCount;
    HB_BOOL bExternalFreamBuffer;
    uint32_t u32BitStreamBufSize;
    CODEC_ROTATION_E enRotation;
    MIRROR_FLIP_E enMirrorFlip;
    VIDEO_CROP_INFO_S stCropCfg;
    HB_BOOL bEnableUserPts;
    uint32_t vlc_buf_size;
    int32_t s32BufJoint;
    int32_t s32BufJointSize;
    union {
        VENC_ATTR_H264_S stAttrH264;
        VENC_ATTR_H265_S stAttrH265;
        VENC_ATTR_MJPEG_S stAttrMjpeg;
        VENC_ATTR_JPEG_S stAttrJpeg;
    };
} VENC_ATTR_S;
```
**Member Descriptions**

| **Member** | **Meaning** |
| --- | --- |
| enType | Encoding protocol type, cannot be dynamically configured. |
| u32PicWidth | Encoded image width, cannot be dynamically configured. |
| u32PicHeight | Encoded image height, cannot be dynamically configured. |
| enPixelFormat | Pixel format, cannot be dynamically configured. |
| u32FrameBufferCount | Number of input FrameBuffer caches, cannot be dynamically configured. |
| u32BitStreamBufferCount | Number of output bitstream buffer zones, cannot be dynamically configured. |
| HB_BOOL bExternalFreamBuffer | Whether to use user-provided input buffer, cannot be dynamically configured. |
| u32BitStreamBufSize | Size of the output bitstream buffer, cannot be dynamically configured. |
| enRotation | Rotation property, cannot be dynamically configured. |
| enMirrorFlip | Mirroring property, cannot be dynamically configured. |
| stCropCfg | Cropping configuration, cannot be dynamically configured. |
| bEnableUserPts | Whether to use user-provided PTS, cannot be dynamically configured. |
| vlc_buf_size | Sets the size of the encoding task's VLC buffer, cannot be dynamically configured. |
| s32BufJoint | Whether to use contiguous memory caching for multiple frames, cannot be dynamically configured. |
| s32BufJointSize | Size of contiguous memory, range 4MB - 50MB |
| stAttrH264/stAttrMjpeg<br/>stAttrJpeg/stAttrH265 | Encoder attributes specific to a certain protocol, cannot be dynamically configured. |



### VENC_RC_MODE_E
**Description**
> Enum for RC (Rate Control) modes.

**Struct Definition**
```C
typedef enum HB_VENC_RC_MODE_E {
    VENC_RC_MODE_NONE = -1,
    VENC_RC_MODE_H264CBR = 1,
    VENC_RC_MODE_H264VBR,
    VENC_RC_MODE_H264AVBR,
    VENC_RC_MODE_H264FIXQP,
    VENC_RC_MODE_H264QPMAP,
    VENC_RC_MODE_H265CBR,
    VENC_RC_MODE_H265VBR,
    VENC_RC_MODE_H265AVBR,
    VENC_RC_MODE_H265FIXQP,
    VENC_RC_MODE_H265QPMAP,
    VENC_RC_MODE_MJPEGFIXQP,
    VENC_RC_MODE_BUTT,
} VENC_RC_MODE_E;
```
**Member Descriptions**

|       Member       |             Meaning              |
| :-----------------: | :------------------------------: |
| VENC_RC_MODE_H264CBR | H264 Constant Bit Rate (CBR) mode. |
| VENC_RC_MODE_H264VBR | H264 Variable Bit Rate (VBR) mode. |
| VENC_RC_MODE_H264AVBR | H264 Adaptive Variable Bit Rate (AVBR) mode. |
| VENC_RC_MODE_H264FIXQP | H264 Fixed Quantization Parameter (Fixqp) mode. |
| VENC_RC_MODE_H264QPMAP | H.264 QP Mapping mode. |
| VENC_RC_MODE_MJPEGFIXQP | MJPEG Fixed Quantization Parameter (Fixqp) mode. |
| VENC_RC_MODE_H265CBR | H265 Constant Bit Rate (CBR) mode. |
| VENC_RC_MODE_H265VBR | H265 Variable Bit Rate (VBR) mode. |
| VENC_RC_MODE_H265AVBR | H265 Adaptive Variable Bit Rate (AVBR) mode. |
| VENC_RC_MODE_H265FIXQP | H265 Fixed Quantization Parameter (Fixqp) mode. |
| VENC_RC_MODE_H265QPMAP | H265 QP Mapping mode. |



### VENC_H264_CBR_S
**Description**
> Defines the structure for H.264 encoding channel Constant Bit Rate (CBR) attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H264_CBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32BitRate;
    uint32_t u32FrameRate;
    uint32_t u32InitialRcQp;
    uint32_t u32VbvBufferSize;
    HB_BOOL bMbLevelRcEnable;
    uint32_t u32MaxIQp;
    uint32_t u32MinIQp;
    uint32_t u32MaxPQp;
    uint32_t u32MinPQp;
    uint32_t u32MaxBQp;
    uint32_t u32MinBQp;
    HB_BOOL bHvsQpEnable;
    int32_t s32HvsQpScale;
    uint32_t u32MaxDeltaQp;
    HB_BOOL bQpMapEnable;
} VENC_H264_CBR_S;
```
**Member Descriptions**

| **Member**      | **Meaning**                                                                                   |
| :-------------: | :----------------------------------------------------------------------------------------------- |
| u32IntraPeriod  | Intra-frame interval, dynamically configurable.                                          |
| u32IntraQp       | I-frame QP value, dynamically configurable, lower values indicate better image quality.         |
| u32BitRate      | Target average bitrate, in kbps, dynamically configurable.                                 |
| u32FrameRate    | Target frame rate, in fps, dynamically configurable.                                      |
| u32InitialRcQp  | Initial QP value for rate control, fixed and not dynamically configurable (must be within [0,51]). |
| u32VbvBufferSize | Actual VBV buffer size, proportional to bit_rate * vbv_buffer_size / 1000 (kb). Smaller sizes improve rate control accuracy but reduce image quality, while larger sizes improve quality but lead to more bitrate fluctuations. Dynamic configuration allowed. |
| bMbLevelRcEnable | Enables H264 rate control at macroblock level, providing higher precision but potentially lower image quality. ROI encoding incompatible; disabled when ROI is enabled. Not dynamically configurable. |
| u32MaxIQp        | Maximum I-frame QP value, dynamically configurable.                                          |
| u32MinIQp        | Minimum I-frame QP value, dynamically configurable.                                          |
| u32MaxPQp        | Maximum P-frame QP value, dynamically configurable.                                          |
| u32MinPQp        | Minimum P-frame QP value, dynamically configurable.                                          |
| u32MaxBQp        | Maximum B-frame QP value, dynamically configurable.                                          |
| u32MinBQp        | Minimum B-frame QP value, dynamically configurable.                                          |
| bHvsQpEnable    | Enables H264 rate control at sub-macroblock level for improved subjective image quality.          |
| s32HvsQpScale   | Effective when hvs_qp_enable is enabled, represents the QP scaling factor, dynamically configurable. |
| u32MaxDeltaQp    | Maximum deviation range for HVS QP values when hvs_qp_enable is enabled, dynamically configurable. |
| bQpMapEnable    | Enables QP map, dynamically configurable.                                                  |

Note: The rate control module adjusts QP values based on the configured bitrate. If the calculated bitrate is less than the set value, it reduces QP, potentially resulting in an image below expectations if QP falls below the minimum (qpmin). If the bitrate is higher than the set value, it increases QP, but if it exceeds the maximum (qpmax), QP cannot be increased further, causing the bitrate to remain above the target.



### VENC_H264_VBR_S
**Description**
> Defines the structure for H.264 encoding channel's VBR attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H264_VBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32FrameRate;
    HB_BOOL bQpMapEnable;
} VENC_H264_VBR_S;
```
**Member Descriptions**

| **Member** | **Meaning** |
| --- | --- |
| u32IntraPeriod | I-frame interval, dynamically configurable. |
| u32IntraQp | I-frame QP value, dynamically configurable. |
| u32FrameRate | Frame rate, dynamically configurable. |
| bQpMapEnable | Enables Qp map, dynamically configurable. |

### VENC_H264_AVBR_S
**Description**
> Defines the structure for H.264 encoding channel's AVBR attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H264_AVBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32BitRate;
    uint32_t u32FrameRate;
    uint32_t u32InitialRcQp;
    uint32_t u32VbvBufferSize;
    HB_BOOL bMbLevelRcEnable;
    uint32_t u32MaxIQp;
    uint32_t u32MinIQp;
    uint32_t u32MaxPQp;
    uint32_t u32MinPQp;
    uint32_t u32MaxBQp;
    uint32_t u32MinBQp;
    HB_BOOL bHvsQpEnable;
    int32_t s32HvsQpScale;
    uint32_t u32MaxDeltaQp;
    HB_BOOL bQpMapEnable;
} VENC_H264_AVBR_S;
```
**Member Descriptions**

| **Member** | **Meaning** |
| --- | --- |
| u32IntraPeriod | I-frame interval, dynamically configurable. |
| u32IntraQp | I-frame QP value, dynamically configurable. |
| u32BitRate | Target average bit rate, in kbps, dynamically configurable. |
| u32FrameRate | Target frame rate, in fps, dynamically configurable. |
| u32InitialRcQp | Initial QP value for rate control, fixed and not configurable (encoder decides if out of range [0,51]). |
| u32VbvBufferSize | Actual VBV buffer size in bytes, calculated as bit_rate * vbv_buffer_size / 1000 (kb). Affects coding quality and rate control accuracy; smaller buffer size gives higher precision but lower quality, larger buffer size improves quality but may have more bitrate fluctuations. |
| bMbLevelRcEnable | Enables macroblock-level rate control in H264, higher precision but may reduce image quality. Not configurable when ROI encoding is enabled. |
| u32MaxIQp | Maximum I-frame QP value, dynamically configurable. |
| u32MinIQp | Minimum I-frame QP value, dynamically configurable. |
| u32MaxPQp | Maximum P-frame QP value, dynamically configurable. |
| u32MinPQp | Minimum P-frame QP value, dynamically configurable. |
| u32MaxBQp | Maximum B-frame QP value, dynamically configurable. |
| u32MinBQp | Minimum B-frame QP value, dynamically configurable. |
| bHvsQpEnable | Enables sub-macroblock level rate control, adjusts sub-block QP to improve subjective image quality. |
| s32HvsQpScale | Effective when hvs_qp_enable is enabled, represents QP scaling factor, dynamically configurable. |
| u32MaxDeltaQp | When hvs_qp_enable is enabled, specifies the maximum deviation range for HVS QP values, dynamically configurable. |
| bQpMapEnable | Enables Qp map, dynamically configurable. |



### VENC_H264_FIXQP_S
**Description**
> Defines the structure for the H.264 encoding channel's FIXQP property.

**Structure Definition**
```C
typedef struct HB_VENC_H264_FIXQP_S {
    uint32_t u32IntraPeriod;
    uint32_t u32FrameRate;
    uint32_t u32IQp;
    uint32_t u32PQp;
    uint32_t u32BQp;
} VENC_H264_FIXQP_S;
```
**Member Descriptions**

|   Member   |                     Meaning                     |
| :--------: | :---------------------------------------------: |
| u32IntraPeriod | Period between I-frames, dynamically configurable. |
|  u32FrameRate  | Target frame rate in fps, dynamically configurable. |
|     u32IQp     | Forced QP value for I-frames, dynamically configurable. |
|     u32PQp     | Forced QP value for P-frames, dynamically configurable. |
|     u32BQp     | Forced QP value for B-frames, dynamically configurable. |

### VENC_H264_QPMAP_S
**Description**
> Defines the structure for the H.264 encoding channel's QPMAP property.

**Structure Definition**
```C
typedef struct HB_VENC_H264_QPMAP_S {
    uint32_t u32IntraPeriod;
    uint32_t u32FrameRate;
    unsigned char*  u32QpMapArray;
    uint32_t u32QpMapArrayCount;
} VENC_H264_QPMAP_S;
```
**Member Descriptions**

|    Member    |                                                  Meaning                                                  |
| :---------: | :---------------------------------------------------------------------------------------------: |
|   u32IntraPeriod   | Interval between I-frames, dynamically configurable.                                          |
|    u32FrameRate    | Target frame rate in fps, dynamically configurable.                                          |
|   u32QpMapArray    | An array specifying QP values for each 16x16 macroblock, with one byte per QP value, ordered raster-scan. |
| u32QpMapArrayCount | Size of the QP map array, dynamically configurable.                                            |



### VENC_H265_CBR_S
**Description**
> Defines the structure for H.265 CBR encoding channel attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H265_CBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32BitRate;
    uint32_t u32FrameRate;
    uint32_t u32InitialRcQp;
    uint32_t u32VbvBufferSize;
    HB_BOOL bCtuLevelRcEnable;
    uint32_t u32MaxIQp;
    uint32_t u32MinIQp;
    uint32_t u32MaxPQp;
    uint32_t u32MinPQp;
    uint32_t u32MaxBQp;
    uint32_t u32MinBQp;
    HB_BOOL bHvsQpEnable;
    int32_t s32HvsQpScale;
    uint32_t u32MaxDeltaQp;
    HB_BOOL bQpMapEnable;
} VENC_H265_CBR_S;
```
**Member Descriptions**

| **Member** | **Meaning** |
| :---------: | :----------: |
| u32IntraPeriod | I-frame interval, dynamically configurable. |
| u32IntraQp | I-frame QP value, dynamically configurable. |
| u32BitRate | Target average bitrate, in kbps, dynamically configurable. |
| u32FrameRate | Target frame rate, in fps, dynamically configurable. |
| u32InitialRcQp | Initial QP value for rate control, when outside [0, 51] range, the encoder will decide the initial value, non-dynamically configurable. |
| u32VbvBufferSize | Actual VBV buffer size, calculated as bit_rate * vbv_buffer_size / 1000 (kb). This buffer size affects coding image quality and rate control precision. Smaller buffer sizes offer higher precision but lower image quality; larger buffers provide better image quality but with more bitrate fluctuations. Dynamically configurable. |
| bMbLevelRcEnable | Enables H.264 rate control at macroblock level, offering higher precision but potentially sacrificing image quality. ROI encoding is not compatible with this mode, and it disables automatically when ROI encoding is enabled. Dynamically configurable. |
| u32MaxIQp | Maximum QP value for I-frames, dynamically configurable. |
| u32MinIQp | Minimum QP value for I-frames, dynamically configurable. |
| u32MaxPQp | Maximum QP value for P-frames, dynamically configurable. |
| u32MinPQp | Minimum QP value for P-frames, dynamically configurable. |
| u32MaxBQp | Maximum QP value for B-frames, dynamically configurable. |
| u32MinBQp | Minimum QP value for B-frames, dynamically configurable. |
| bHvsQpEnable | Enables H.264 rate control at sub-macroblock level for improved subjective image quality. Dynamically configurable. |
| s32HvsQpScale | Valid when hvs_qp_enable is enabled, represents the QP scaling factor, dynamically configurable. |
| u32MaxDeltaQp | When hvs_qp_enable is enabled, specifies the maximum deviation range for HVS QP values. Dynamically configurable. |
| bQpMapEnable | Enables QP mapping, dynamically configurable. |



### VENC_H265_VBR_S
**Description**
> Defines the structure for H.265 encoding channel's Variable Bit Rate (VBR) attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H265_VBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32FrameRate;
    HB_BOOL bQpMapEnable;
} VENC_H265_VBR_S;
```
**Member Descriptions**

| Member | Meaning |
| --- | --- |
| u32IntraPeriod | I-frame interval, dynamically configurable. |
| u32IntraQp | I-frame QP value, dynamically configurable. |
| u32FrameRate | Frame rate, dynamically configurable. |
| bQpMapEnable | Enables or disables Qp map, dynamically configurable. |

### VENC_H265_AVBR_S
**Description**
> Defines the structure for H.265 encoding channel's Average Bit Rate (ABR) attributes.

**Structure Definition**
```C
typedef struct HB_VENC_H265_AVBR_S {
    uint32_t u32IntraPeriod;
    uint32_t u32IntraQp;
    uint32_t u32BitRate;
    uint32_t u32FrameRate;
    uint32_t u32InitialRcQp;
    uint32_t u32VbvBufferSize;
    HB_BOOL bCtuLevelRcEnable;
    uint32_t u32MaxIQp;
    uint32_t u32MinIQp;
    uint32_t u32MaxPQp;
    uint32_t u32MinPQp;
    uint32_t u32MaxBQp;
    uint32_t u32MinBQp;
    HB_BOOL bHvsQpEnable;
    int32_t s32HvsQpScale;
    uint32_t u32MaxDeltaQp;
    HB_BOOL bQpMapEnable;
} VENC_H265_AVBR_S;
```
**Member Descriptions**

| Member | Meaning |
| --- | --- |
| u32IntraPeriod | I-frame interval, dynamically configurable. |
| u32IntraQp | I-frame QP value, dynamically configurable. |
| u32BitRate | Target average bitrate in kbps, dynamically configurable. |
| u32FrameRate | Target frame rate in fps, dynamically configurable. |
| u32InitialRcQp | Initial QP value for rate control, if not in [0, 51] range, encoder will decide internally, dynamically configurable. |
| u32VbvBufferSize | Actual VBV buffer size in bits, proportional to bit_rate * vbv_buffer_size / 1000 (kb). This buffer size affects image quality and rate control precision. Higher buffer sizes improve image quality but can cause larger bitrate fluctuations, dynamically configurable. |
| bCtuLevelRcEnable | Enables H.264 rate control at CTU level, providing higher precision but potentially lower image quality. This feature cannot be used with ROI encoding; it is automatically disabled when ROI encoding is enabled, dynamically configurable. |
| u32MaxIQp | Maximum I-frame QP value, dynamically configurable. |
| u32MinIQp | Minimum I-frame QP value, dynamically configurable. |
| u32MaxPQp | Maximum P-frame QP value, dynamically configurable. |
| u32MinPQp | Minimum P-frame QP value, dynamically configurable. |
| u32MaxBQp | Maximum B-frame QP value, dynamically configurable. |
| u32MinBQp | Minimum B-frame QP value, dynamically configurable. |
| bHvsQpEnable | Enables H.264 rate control at sub-CTU level, improving subjective image quality. |
| s32HvsQpScale | Effective when hvs_qp_enable is enabled, represents the QP scaling factor, dynamically configurable. |
| u32MaxDeltaQp | Maximum deviation range for HVS QP values when hvs_qp_enable is enabled, dynamically configurable. |
| bQpMapEnable | Enables or disables Qp map, dynamically configurable. |



### VENC_H265_FIXQP_S
**Description**
> Defines the structure for the H.265 encoding channel's Fixqp attribute.

**Structure Definition**
```C
typedef struct HB_VENC_H265_FIXQP_S {
    uint32_t u32IntraPeriod;
    uint32_t u32FrameRate;
    uint32_t u32IQp;
    uint32_t u32PQp;
    uint32_t u32BQp;
} VENC_H265_FIXQP_S;
```
**Member Descriptions**

|       Member       |                             Meaning                             |
| :-----------------: | :-------------------------------------------------------------: |
| u32IntraPeriod    | Interval between I-frames, dynamically configurable.         |
| u32FrameRate      | Target frame rate in frames per second (fps), dynamically configurable. |
| u32IQp            | Forced I-frame QP value, dynamically configurable.          |
| u32PQp            | Forced P-frame QP value, dynamically configurable.          |
| u32BQp            | Forced B-frame QP value, dynamically configurable.          |

### VENC_H265_QPMAP_S
**Description**
> Defines the structure for the H.265 encoding channel's QPMAP attribute.

**Structure Definition**
```C
typedef struct HB_VENC_H265_QPMAP_S {
    uint32_t u32IntraPeriod;
    uint32_t u32FrameRate;
    unsigned char * u32QpMapArray;
    uint32_t u32QpMapArrayCount;
} VENC_H265_QPMAP_S;
```
**Member Descriptions**

|        Member        |                                                                                           Meaning                                                                                           |
| :-----------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------: |
|   u32IntraPeriod   | Interval between I-frames, dynamically configurable.                                                                                                 |
|    u32FrameRate    | Target frame rate in frames per second (fps), dynamically configurable.                                                                               |
|   u32QpMapArray    | A specified QP map table, where each 16x16 macroblock is assigned a QP value, with one byte per QP and sorted in raster scan order, dynamically configurable. |
| u32QpMapArrayCount | The size of the QP map table, dynamically configurable.                                                                                               |



### VENC_MJPEG_FIXQP_S
**Description**
> Defines the structure for MJPEG encoding channel's Fixqp attribute.

**Structure Definition**
```C
typedef struct HB_VENC_MJPEG_FIXQP_S {
    uint32_t u32FrameRate; // Target frame rate, in fps, can be dynamically configured.
    uint32_t u32QualityFactor; // Quantization factor, when this value is 100, image quality loss is minimal but compression rate is low; when it is 1, image quality loss is significant but compression rate is high, can be dynamically configured.
} VENC_MJPEG_FIXQP_S;
```
**Member Descriptions**

|      Member       |                            Meaning                            |
| :---------------: | :---------------------------------------------------------: |
|   u32FrameRate    | Target frame rate, in frames per second (fps), configurable. |
| u32QualityFactor | Quantization factor; 100 gives minimal quality loss but low compression, 1 offers higher compression with more noticeable quality loss, configurable. |

### VENC_RC_ATTR_S
**Description**
> Defines the attributes of the encoding channel's rate controller.

**Structure Definition**
```C
typedef struct HB_VENC_RC_ATTR_S {
    VENC_RC_MODE_E enRcMode; // RC mode, not configurable.
    union {
        VENC_H264_CBR_S stH264Cbr; // H.264 protocol CBR mode attributes.
        VENC_H264_VBR_S stH264Vbr; // H.264 protocol VBR mode attributes.
        VENC_H264_AVBR_S stH264AVbr; // H.264 protocol AVBR mode attributes.
        VENC_H264_FIXQP_S stH264FixQp; // H.264 protocol Fixqp mode attributes.
        VENC_H264_QPMAP_S stH264QpMap; // H.264 protocol QpMap mode attributes.
        VENC_MJPEG_FIXQP_S stMjpegFixQp; // MJPEG protocol Fixqp mode attributes.
        VENC_H265_CBR_S stH265Cbr; // H.265 protocol CBR mode attributes.
        VENC_H265_VBR_S stH265Vbr; // H.265 protocol VBR mode attributes.
        VENC_H265_AVBR_S stH265AVbr; // H.265 protocol AVBR mode attributes.
        VENC_H265_FIXQP_S stH265FixQp; // H.265 protocol Fixqp mode attributes.
        VENC_H265_QPMAP_S stH265QpMap; // H.265 protocol QpMap mode attributes.
    };
} VENC_RC_ATTR_S;
```
**Member Descriptions**

|     Member     |                              Meaning                              |
| :------------: | :-------------------------------------------------------------: |
|   enRcMode     | RC mode, which is not configurable.                            |
|  stH264Cbr     | Attributes for H.264 protocol CBR mode.                        |
|  stH264Vbr     | Attributes for H.264 protocol VBR mode.                        |
|  stH264AVbr    | Attributes for H.264 protocol AVBR mode.                       |
| stH264FixQp    | Attributes for H.264 protocol Fixqp mode.                      |
| stH264QpMap    | Attributes for H.264 protocol QpMap mode.                     |
| stMjpegFixQp  | Attributes for MJPEG protocol Fixqp mode.                      |
|  stH265Cbr     | Attributes for H.265 protocol CBR mode.                        |
|  stH265Vbr     | Attributes for H.265 protocol VBR mode.                        |
|  stH265AVbr    | Attributes for H.265 protocol AVBR mode.                       |
| stH265FixQp    | Attributes for H.265 protocol Fixqp mode.                      |
| stH265QpMap    | Attributes for H.265 protocol QpMap mode.                     |



### VENC_GOP_PICTURE_CUSTOM_S
**Description**
> Defines the data structure for a custom GOP (Group of Pictures) structure.

**Struct Definition**
```C
typedef struct HB_VENC_GOP_PICTURE_CUSTOM_S {
    uint32_t u32PictureType;
    int32_t s32PocOffset;
    uint32_t u32PictureQp;
    int32_t s32NumRefPictureL0;
    int32_t s32RefPocL0;
    int32_t s32RefPocL1;
    uint32_t u32TemporalId;
} VENC_GOP_PICTURE_CUSTOM_S;
```
**Member Descriptions**

|        Member        |                                                  Description                                                  |
| :-----------------: | :---------------------------------------------------------------------------------------------: |
|   u32PictureType   | The image frame type, which is not dynamically configurable.                                      |
|    s32PocOffset    | The POC (Picture Order Count) value of the image, which is not dynamically configurable.       |
|    u32PictureQp    | The QP (Quantization Parameter) value of the image, which is not dynamically configurable.    |
| s32NumRefPictureL0 | The number of L0 reference frames for the current image; valid only when pic_type=1, not dynamic. |
|    s32RefPocL0     | The POC value of the L0 reference frame for the current image, not dynamically configurable.    |
|    s32RefPocL1     | If pic_type=2, this is the POC value of the L1 reference frame for the current image; if pic_type=1, it's the POC value of the L0 reference frame, not dynamic. |
|   u32TemporalId    | The temporal ID of the image, which is not dynamically configurable.                          |



### VENC_GOP_CUSTOM_S
**Description**
> Defines the parameters set for a custom GOP structure.

**Structure Definition**
```C
typedef struct HB_VENC_GOP_CUSTOM_S {
    uint32_t u32CustomGopSize;
    VENC_GOP_PICTURE_CUSTOM_S stCustomGopPicture[CUSTOM_MAX_GOP_NUM];
} VENC_GOP_CUSTOM_S;
```
**Member Descriptions**

|       Member       |                        Meaning                        |
| :-----------------: | :--------------------------------------------------: |
|  u32CustomGopSize  | Size of the custom GOP (0-8), not dynamically configurable. |
| stCustomGopPicture | Array of custom GOP picture structures, not dynamic. |

### VENC_GOP_ATTR_S
**Description**
> Defines GOP parameters, allowing users to choose between pre-defined GOP structures and custom GOP structures.

**Structure Definition**
```C
typedef struct HB_VENC_GOP_ATTR_S {
    int32_t s32DecodingRefreshType;
    uint32_t u32GopPresetIdx;
    VENC_GOP_CUSTOM_S stCustomGopParam;
} VENC_GOP_ATTR_S;
```
**Member Descriptions**

|         Member         |                                 Meaning                                  |
| :--------------------: | :-------------------------------------------------------------------: |
| s32DecodingRefreshType | Specifies the specific type of IDR frame, valid only for H265 codec, not configurable. |
|    u32GopPresetIdx     | Chooses a pre-defined GOP structure; 0 indicates using a custom GOP structure, not configurable. |
|    stCustomGopParam    | Custom GOP structure, valid only when u32GopPresetIdx=0, not configurable. |



### VENC_CHN_ATTR_S
**Description**
> Defines the structure for encoding channel attributes.

**Structure Definition**
```C
typedef struct HB_VENC_CHN_ATTR_S {
    VENC_ATTR_S stVencAttr;
    VENC_RC_ATTR_S stRcAttr;
    VENC_GOP_ATTR_S stGopAttr;
} VENC_CHN_ATTR_S;
```
**Member Descriptions**

|    Member    |              Meaning               |
| :----------: | :---------------------------------: |
| stVencAttr   | Encoder attribute settings.        |
|  stRcAttr    | Rate control attribute settings.   |
| stGopAttr    | Structure containing GOP mode settings. |

### VENC_JPEG_PARAM_S
**Description**
> Defines the high-level parameters for JPEG protocol encoding channel.

**Structure Definition**
```C
typedef struct HB_VENC_JPEG_PARAM_S {
    uint32_t u32Qfactor;
    uint8_t  u8LumaQuantTable[64];
    uint8_t u8ChromaQuantTable[64];
    uint16_t u16LumaQuantEsTable[64];
    uint16_t u16ChromaQuantEsTable[64];
    uint32_t u32RestartInterval;
    VIDEO_CROP_INFO_S stCropCfg;
} VENC_JPEG_PARAM_S;
```
**Member Descriptions**

|        Member        |                               Description                               |
| :------------------: | :--------------------------------------------------------------------: |
|      u32Qfactor      | Refer to RFC2435 for meaning; system default is 90. Range: [1, 99].     |
|   u8LumaQuantTable   | 8-bit Y component quantization table<br/>Range: [0, 255].                |
|  u8ChromaQuantTable  | 8-bit CbCr component quantization table<br/>Range: [0, 255].             |
|  u16LumaQuantEsTable | 12-bit Y component quantization table<br/>Range: [0, 255].               |
| u16ChromaQuantEsTable | 12-bit CbCr component quantization table<br/>Range: [0, 255].             |
|  u32RestartInterval | Restart interval: [0, (picwidth+15)>>4 x(picheight+15)>>4 x 2]         |
|       stCropCfg      | Parameters for cropping configuration.                                    |



### VENC_MJPEG_PARAM_S
**Description**
> Defines the structure for advanced parameters of the MJPEG protocol encoding channel.

**Structure Definition**
```C
typedef struct HB_VENC_MJPEG_PARAM_S {
    uint8_t  u8LumaQuantTable [64];
    uint8_t  u8ChromaQuantTable [64];
    uint16_t  u16LumaQuantEsTable [64];
    uint16_t u16ChromaQuantEsTable[64];
    uint32_t u32RestartInterval;
} VENC_MJPEG_PARAM_S;
```
**Member Descriptions**

|        Member         |                                Meaning                                |
| :--------------------: | :--------------------------------------------------------------: |
|   u8LumaQuantTable    | 8-bit Y-quantization table<br/>Range: [0, 255].               |
|  u8ChromaQuantTable   | 8-bit CbCr-quantization table<br/>Range: [0, 255].             |
|  u16LumaQuantEsTable  | 12-bit Y-quantization table<br/>Range: [0, 255].              |
| u16ChromaQuantEsTable | 12-bit CbCr-quantization table<br/>Range: [0, 255].             |
|  u32RestartInterval   | u32RestartInterval: [0, (picwidth+15)>>4 x(picheight+15)>>4 x 2] |

### VENC_INTRA_REFRESH_MODE_E
**Description**
> Defines the P-frame intra-refresh mode for ISlice.

**Structure Definition**
```C
typedef enum HB_VENC_INTRA_REFRESH_MODE_E
{
    INTRA_REFRESH_ROW = 0,
    INTRA_REFRESH_COLUMN,
    INTRA_REFRESH_STEP_SIZE,
    INTRA_REFRESH_ADAPTIVE,
    INTRA_REFRESH_BUTT
} VENC_INTRA_REFRESH_MODE_E;
```
**Member Descriptions**

|        Member         |                Meaning                 |
| :--------------------: | :----------------------------------: |
|    INTRA_REFRESH_ROW    | Refresh by rows (row-based).         |
|  INTRA_REFRESH_COLUMN   | Refresh by columns (column-based).    |
| INTRA_REFRESH_STEP_SIZE | MB/CTU step size for refresh.        |
| INTRA_REFRESH_ADAPTIVE  | Adaptive refresh of MB/CTUs in a frame. |



### VENC_INTRA_REFRESH_S
**Description**
>P-frame ISlice control parameters.

**Struct Definition**
```C
typedef struct HB_VENC_INTRA_REFRESH_S
{
    HB_BOOL bRefreshEnable;
    VENC_INTRA_REFRESH_MODE_E enIntraRefreshMode;
    uint32_t u32RefreshNum;
} VENC_INTRA_REFRESH_S;
```
**Member Descriptions**

|        Member        |                            Meaning                            |
| :-----------------: | :--------------------------------------------------------: |
|   bRefreshEnable   | Enables the ISlice refresh function. Default: 0<br/>0: Disable<br/>1: Enable |
| enIntraRefreshMode | I macroblock refresh mode, either row-by-row or column-by-column. |
|   u32RefreshNum    | Number of rows or columns to refresh during each I macroblock. |

### VENC_H264_ENTROPY_S
**Description**
>Defines H.264 entropy encoding parameters.

**Struct Definition**
```C
typedef struct HB_VENC_H264_ENTROPY_S
{
    uint32_t u32EntropyEncMode;
} VENC_H264_ENTROPY_S;
```
**Member Descriptions**

|       Member        |           Meaning           |
| :---------------: | :----------------------: |
| u32EntropyEncMode | Entropy encoding mode, dynamically configurable. |

### VENC_H264_DBLK_S
**Description**
>Defines H.264 deblocking filter parameters.

**Struct Definition**
```C
typedef struct HB_VENC_H264_DBLK_S
{
    uint32_t disable_deblocking_filter_idc;
    int32_t slice_alpha_c0_offset_div2;
    int32_t slice_beta_offset_div2;
} VENC_H264_DBLK_S;
```
**Member Descriptions**

|             Member              |                                Meaning                                 |
| :---------------------------: | :-----------------------------------------------------------------: |
| disable_deblocking_filter_idc | Range: [0, 2], default 0; see H.264 specification for details, dynamically configurable. |
|  slice_alpha_c0_offset_div2   | Range: [-6, 6], default 0; see H.264 specification for details, dynamically configurable. |
|    slice_beta_offset_div2     | Range: [-6, 6], default 0; see H.264 specification for details, dynamically configurable. |



### VENC_H265_DBLK_S
**Description**
> Defines parameters for H265 deblocking filter.

**Structure Definition**
```C
typedef struct HB_VENC_H265_DBLK_S
{
    uint32_t slice_deblocking_filter_disabled_flag;
    int32_t slice_beta_offset_div2;
    int32_t slice_tc_offset_div2;
    uint32_t slice_loop_filter_across_slices_enabled_flag;
} VENC_H265_DBLK_S;
```
**Member Descriptions**

|                 Member                 |                           Meaning                           |
| :----------------------------------: | :-------------------------------------------------------------: |
| slice_deblocking_filter_disabled_flag | Default is 0.<br/>Range: 0 or 1. Configurable dynamically. |
|            slice_tc_offset_div2            | Default is 0.<br/>Range: [-6, 6]. Configurable dynamically. |
|           slice_beta_offset_div2           | Default is 0.<br/>Range: [-6, 6]. Configurable dynamically. |
| slice_loop_filter_across_slices_enabled_flag | Enables slice boundary filtering. Configurable dynamically. |

### VENC_VUI_H264_TIME_INFO_S
**Description**
> Defines H264 Timing parameters.

**Structure Definition**
```C
typedef struct HB_VENC_H264_VUI_TIME_INFO_S
{
    uint32_t fixed_frame_rate_flag;
    uint32_t num_units_in_tick;
    uint32_t time_scale;
}VENC_VUI_H264_TIME_INFO_S;
```
**Member Descriptions**

|         Member         |                             Meaning                             |
| :--------------------: | :--------------------------------------------------------------: |
| fixed_frame_rate_flag | Refer to the H.264 standard; system default is 0. Range: 0 or 1. | Not configurable parameter. |
|   num_units_in_tick   | Follows H264 specifications; not configurable parameter.        |
|      time_scale       | Follows H264 specifications; not configurable parameter.        |



### VENC_H264_VUI_S
**Description**
> Defines the structure for the H.264 protocol's VUI (Video Usability Information) structure.

**Structure Definition**
```C
typedef struct HB_VENC_H264_VUI_S
{
    VENC_VUI_H264_TIME_INFO_S stVuiTimeInfo;
} VENC_H264_VUI_S;
```
**Member Descriptions**

|   Member    |    Meaning    |
| :---------: | :-----------: |
| stVuiTimeInfo | H264 Timing Parameters |

### VENC_VUI_H265_TIME_INFO_S
**Description**
> Defines the H265 Timing Parameters.

**Structure Definition**
```C
typedef struct HB_VENC_VUI_H265_TIME_INFO_S
{
    uint32_t num_units_in_tick;
    uint32_t time_scale;
    uint32_t num_ticks_poc_diff_one_minus1;
} VENC_VUI_H265_TIME_INFO_S;
```
**Member Descriptions**

|        Member        |                Meaning                |
| :------------------: | :----------------------------------: |
| num_units_in_tick    | Follows H265 specification, not dynamic. |
|      time_scale      | Follows H265 specification, not dynamic. |
| num_ticks_poc_diff_one_minus1 | Follows H265 specification, not dynamic. |

### VENC_H265_VUI_S
**Description**
> Defines the structure for the H.265 protocol's VUI (Video Usability Information) structure.

**Structure Definition**
```C
typedef struct HB_VENC_H265_VUI_S
{
    VENC_VUI_H265_TIME_INFO_S stVuiTimeInfo;
} VENC_H265_VUI_S
```
**Member Descriptions**

|   Member    |      Meaning      |
| :---------: | :---------------: |
| stVuiTimeInfo | H265 Timing Parameters |



### VENC_H265_SAO_S
**Description**
> Defines the structure for the H.265 protocol encoding channel's Sample Adaptive Offset (SAO) feature.

**Structure Definition**
```C
typedef struct HB_VENC_H265_SAO_S
{
    uint32_t sample_adaptive_offset_enabled_flag;
} VENC_H265_SAO_S;
```
**Member Descriptions**

|          Member           |              Meaning              |
| :------------------------: | :------------------------------: |
| sample_adaptive_offset_enabled_flag | Flag indicating whether SAO is enabled |

### VENC_H264_SLICE_SPLIT_S
**Description**
> Defines the structure for the H.264 protocol encoding channel's SLICE Splitting feature.

**Structure Definition**
```C
typedef struct HB_VENC_H264_SLICE_SPLIT_S
{
    int32_t h264_slice_mode;
    int32_t h264_slice_arg;
} VENC_H264_SLICE_SPLIT_S;
```
**Member Descriptions**

|        Member         |                            Meaning                            |
| :--------------------: | :----------------------------------------------------------: |
| h264_slice_mode       | Slice splitting mode, dynamically configurable |
| h264_slice_arg        | Slice parameter, dynamically configurable, represents the number of macroblocks, starting from the top-left corner, with 16x16 pixels each. The image is divided into macroblocks for encoding. The maximum value is (h+15)/16 * (w+15)/16, where h and w are height and width, respectively. |

### VENC_H265_SLICE_SPLIT_S
**Description**
> Defines the structure for the H.265 protocol encoding channel's SLICE Splitting feature.

**Structure Definition**
```C
typedef struct HB_VENC_H265_SLICE_SPLIT_S
{
    int32_t h265_independent_slice_mode;
    int32_t h265_independent_slice_arg;
    int32_t h265_dependent_slice_mode;
    int32_t h265_dependent_slice_arg;
} VENC_H265_SLICE_SPLIT_S
```
**Member Descriptions**

|            Member             |                                            Meaning                                            |
| :-------------------------: | :-------------------------------------------------------------------------------------------: |
| h265_independent_slice_mode | Encoding mode for independent slices<br/>0: Disable, 1: Enable (dynamically configurable). |
| h265_independent_slice_arg  | Size of independent slices in coding CTUs, range [0, 2^16-1], dynamically configurable. |
| h265_dependent_slice_mode  | Mode for dependent slices<br/>0: Disable<br/>1: Slice unit is coding CTU<br/>2: Slice unit is byte, dynamically configurable. |
| h265_dependent_slice_arg   | Size of dependent slices, range [0, 2^16-1], dynamically configurable.                      |



### VENC_H264_INTRA_PRED_S
**Description**
> Defines the structure for H.264 protocol encoding channel's intra prediction.

**Structure Definition**
```C
typedef struct HB_VENC_H264_INTRA_PRED_S
{
    uint32_t constrained_intra_pred_flag;
} VENC_H264_INTRA_PRED_S
```
**Member Descriptions**

|          Member           |                     Meaning                     |
| :------------------------: | :---------------------------------------------: |
| constrained_intra_pred_flag | Default is 0.<br/>Range: 0 or 1. Dynamic configuration supported. |

### VENC_H265_PU_S
**Description**
> Defines the structure for H.265 protocol encoding channel's PU parameters.

**Structure Definition**
```C
typedef struct HB_VENC_H265_PU_S
{
    uint32_t intra_nxn_enable;
    uint32_t max_num_merge;
    uint32_t constrained_intra_pred_flag;
    uint32_t strong_intra_smoothing_enabled_flag;
} VENC_H265_PU_S
```
**Member Descriptions**

|                             Member                             |                      Meaning                      |
| :-----------------------------------------------------------: | :-----------------------------------------------: |
|                                      intra_nxn_enable                                      | Enables intra NxN PUs in intra CUs, dynamically configurable |
|                                        max_num_merge                                        | Specifies the number of merge candidates in RDO, dynamically configurable |
|                                 constrained_intra_pred_flag                                 | Default is 0<br/>Range: 0 or 1.<br/>Dynamically configurable |
| strong_intra_smoothing_enabled_flag<br/>Default is 0.<br/>Range: 0 or 1.<br/>Dynamically configurable |



### VENC_H264_TRANS_S
**Description**
> Defines the parameters for H264 Transform.

**Struct Definition**
```C
typedef struct HB_VENC_H264_TRANS_S {
    uint32_t transform_8x8_enable;
    int32_t chroma_cb_qp_offset;
    int32_t chroma_cr_qp_offset;
    uint32_t user_scaling_list_enable;
    uint8_t scaling_list_4x4[HB_VENC_SL_MATRIX_NUM][16];
    uint8_t scaling_list_8x8[2][64];
} VENC_H264_TRANS_S;
```
**Member Descriptions**

|          Member          |                              Meaning                             |
| :-----------------------: | :-------------------------------------------------------------: |
|   transform_8x8_enable   | Enables 8x8 transform, dynamically configurable.               |
|   chroma_cb_qp_offset    | Specifies the QP offset for the cb component, dynamically configurable. |
|   chroma_cr_qp_offset    | Specifies the QP offset for the cr component, dynamically configurable. |
| user_scaling_list_enable | Enables user-defined scaling list, not dynamically configurable. |
|     scaling_list_4x4     | Specifies 4x4 block correlation coefficients, 16 coefficients per element, non-dynamic. |
|     scaling_list_8x8     | Specifies 8x8 block correlation coefficients, 64 coefficients per element, non-dynamic. |

### VENC_H265_TRANS_S
**Description**
> Defines the H265 Transform parameters.

**Struct Definition**
```C
typedef struct HB_VENC_H265_TRANSFORM_PARAMS {
    int32_t chroma_cb_qp_offset;
    int32_t chroma_cr_qp_offset;
    uint32_t user_scaling_list_enable;
    uint8_t scaling_list_4x4[HB_VENC_SL_MATRIX_NUM][16];
    uint8_t scaling_list_8x8[HB_VENC_SL_MATRIX_NUM][64];
    uint8_t scaling_list_16x16[HB_VENC_SL_MATRIX_NUM][64];
    uint8_t scaling_list_32x32[2][64];
    uint8_t scaling_list_dc_16x16[HB_VENC_SL_MATRIX_NUM];
    uint8_t scaling_list_dc_32x32[2];
} VENC_H265_TRANS_S;
```
**Member Descriptions**

|          Member          |                             Meaning                             |
| :-----------------------: | :--------------------------------------------------------------: |
|   chroma_cb_qp_offset    | Specifies the QP offset for the cb component, dynamically configurable. |
|   chroma_cr_qp_offset    | Specifies the QP offset for the cr component, dynamically configurable. |
| user_scaling_list_enable | Enables user-defined scaling list, not dynamically configurable. |
|     scaling_list_4x4     | Specifies 4x4 block correlation coefficients, 16 coefficients per element, non-dynamic. |
|     scaling_list_8x8     | Specifies 8x8 block correlation coefficients, 64 coefficients per element, non-dynamic. |
|    scaling_list_16x16    | Specifies 16x16 block correlation coefficients, 64 coefficients per element, non-dynamic. |
|    scaling_list_32x32    | Specifies 32x32 block correlation coefficients, 64 coefficients per element, non-dynamic. |
|  scaling_list_dc_16x16   | Specifies the DC coefficients for 16x16 blocks, non-dynamic. |
|  scaling_list_dc_32x32   | Specifies the DC coefficients for 32x32 blocks, non-dynamic. |



### VENC_CU_PREDICTION_S
**Description**
> Defines parameters for internal encoding mode decision in H264/H265.

**Structure Definition**
```C
typedef struct HB_VENC_CU_PREDICTION_S
{
    int32_t mode_decision_enable;
    uint32_t pu04_delta_rate;
    uint32_t pu08_delta_rate;
    uint32_t pu16_delta_rate;
    uint32_t pu32_delta_rate;
    uint32_t pu04_intra_planar_delta_rate;
    uint32_t pu04_intra_dc_delta_rate;
    uint32_t pu04_intra_angle_delta_rate;
    uint32_t pu08_intra_planar_delta_rate;
    uint32_t pu08_intra_dc_delta_rate;
    uint32_t pu08_intra_angle_delta_rate;
    uint32_t pu16_intra_planar_delta_rate;
    uint32_t pu16_intra_dc_delta_rate;
    uint32_t pu16_intra_angle_delta_rate;
    uint32_t pu32_intra_planar_delta_rate;
    uint32_t pu32_intra_dc_delta_rate;
    uint32_t pu32_intra_angle_delta_rate;
    uint32_t cu08_intra_delta_rate;
    uint32_t cu08_inter_delta_rate;
    uint32_t cu08_merge_delta_rate;
    uint32_t cu16_intra_delta_rate;
    uint32_t cu16_inter_delta_rate;
    uint32_t cu16_merge_delta_rate;
    uint32_t cu32_intra_delta_rate;
    uint32_t cu32_inter_delta_rate;
    uint32_t cu32_merge_delta_rate;
} VENC_CU_PREDICTION_S;
```
**Member Descriptions**

| Member            | Description                                                                                   |
| :-----------------: | :-----------------------------------------------------------------------------------------------: |
| mode_decision_enable | Enables mode selection, dynamically configurable.                                           |
| pu04_delta_rate    | Dynamic configuration for 4x4 block cost delta.                                                 |
| pu08_delta_rate    | Dynamic configuration for 8x8 block cost delta.                                                 |
| pu16_delta_rate    | Dynamic configuration for 16x16 block cost delta.                                              |
| pu32_delta_rate    | Dynamic configuration for 32x32 block cost delta.                                              |
| pu04_intra_planar_delta_rate | Dynamic configuration for 4x4 intra-planar rate delta in frame prediction mode.                  |
| pu04_intra_dc_delta_rate | Dynamic configuration for 4x4 intra DC rate delta in frame prediction mode.                     |
| pu04_intra_angle_delta_rate | Dynamic configuration for 4x4 intra angle rate delta in frame prediction mode.                |
| pu08_intra_planar_delta_rate | Dynamic configuration for 8x8 intra-planar rate delta in frame prediction mode.               |
| pu08_intra_dc_delta_rate | Dynamic configuration for 8x8 intra DC rate delta in frame prediction mode.                    |
| pu08_intra_angle_delta_rate | Dynamic configuration for 8x8 intra angle rate delta in frame prediction mode.               |
| pu16_intra_planar_delta_rate | Dynamic configuration for 16x16 intra-planar rate delta in frame prediction mode.           |
| pu16_intra_dc_delta_rate | Dynamic configuration for 16x16 intra DC rate delta in frame prediction mode.                |
| pu16_intra_angle_delta_rate | Dynamic configuration for 16x16 intra angle rate delta in frame prediction mode.             |
| pu32_intra_planar_delta_rate | Dynamic configuration for 32x32 intra-planar rate delta in frame prediction mode.          |
| pu32_intra_dc_delta_rate | Dynamic configuration for 32x32 intra DC rate delta in frame prediction mode.                |
| pu32_intra_angle_delta_rate | Dynamic configuration for 32x32 intra angle rate delta in frame prediction mode.             |
| cu08_intra_delta_rate | Dynamic configuration for 8x8 intra rate delta between frames.                                 |
| cu08_inter_delta_rate | Dynamic configuration for 8x8 inter rate delta between frames.                               |
| cu08_merge_delta_rate | Dynamic configuration for 8x8 merge rate delta between frames.                             |
| cu16_intra_delta_rate | Dynamic configuration for 16x16 intra rate delta between frames.                           |
| cu16_inter_delta_rate | Dynamic configuration for 16x16 inter rate delta between frames.                          |
| cu16_merge_delta_rate | Dynamic configuration for 16x16 merge rate delta between frames.                         |
| cu32_intra_delta_rate | Dynamic configuration for 32x32 intra rate delta between frames.                           |
| cu32_inter_delta_rate | Dynamic configuration for 32x32 inter rate delta between frames.                          |
| cu32_merge_delta_rate | Dynamic configuration for 32x32 merge rate delta between frames.                         |



### VIDEO_CROP_INFO_S
**Description**
> Defines the cropping parameters.

**Struct Definition**
```C
typedef struct HB_VIDEO_CROP_INFO_S
{
    HB_BOOL bEnable; // Enable cropping.
    CODEC_RECT_S stRect; // Cropping region.
} VIDEO_CROP_INFO_S;
```
**Member Descriptions**

| Member | Description |
| :----: | :---------- |
| bEnable | Whether to perform cropping. <br/>Range: [HB_FALSE, HB_TRUE] |
| stRect  | Cropping area, where s32X and s32Y are 8-byte aligned, while u32Width and u32Height for H.264/H.265 are 2-byte aligned, and for mjpeg/jpeg, u32Width and u32Height are 1-byte aligned |

### VIDEO_FRAME_PACK_S
**Description**
> Defines the image frame structure.

**Struct Definition**
```C
typedef struct HB_VIDEO_FRAME_PACK_S {
    hb_char* vir_ptr[3]; // Image frame virtual address pointers
    uint64_t phy_ptr[3]; // Image frame physical addresses
    uint32_t size; // Image frame size
    uint32_t width; // Image width
    uint32_t height; // Image height
    PIXEL_FORMAT_E pix_format; // Image pixel format
    int32_t stride; // Image horizontal span
    int32_t vstride; // Image vertical span
    int32_t fd[3]; // Image ION memory handles
    uint64_t pts; // Image PTS
    HB_BOOL frame_end; // Whether the frame is the last one
    int32_t flags; // Image flags
    int32_t src_idx; // Internal buffer index
} VIDEO_FRAME_PACK_S;
```
**Member Descriptions**

| Member | Description |
| :----: | :---------- |
| vir_ptr[3] | Virtual addresses of image frames |
| phy_ptr[3] | Physical addresses of image frames |
| size | Size of the image frame |
| width | Image width |
| height | Image height |
| pix_format | Image pixel format |
| stride | Image horizontal stride |
| vstride | Image vertical stride |
| fd[3] | Image ION memory handles |
| pts | Image presentation timestamp (PTS) |
| frame_end | Whether the frame marks the end of a sequence |
| flags | Image flags |
| src_idx | Internal buffer index |



### VIDEO_FRAME_S
**Description**
> Defines the structure of an original video frame.

**Structure Definition**
```C
typedef struct HB_VIDEO_FRAME_S {
    VIDEO_FRAME_PACK_S stVFrame;
    union {
        VIDEO_FRAME_INFO_S stFrameInfo;
        VIDEO_FRAME_INFO_JPEG_S stJpegInfo;
    };
} VIDEO_FRAME_S;
```
**Member Descriptions**

|    Member     |    Meaning    |
| :-----------: | :-----------: |
|   stVFrame    | Video Frame   |
| stFrameInfo  | Video Frame Information |
| stJpegInfo   | JPEG Frame Information |

### VIDEO_FRAME_INFO_S
**Description**
> Defines the structure for video frame information.

**Structure Definition**
```C
typedef struct HB_VIDEO_FRAME_INFO_S {
    int32_t decode_result;
    int32_t frame_display_index;
    int32_t frame_decoded_index;
    uint64_t stream_start_addr;
    int32_t stream_size;
    int32_t nalu_type;
    int32_t err_mb_in_frame_decoded;
    int32_t total_mb_in_frame_decoded;
    int32_t err_mb_in_frame_display;
    int32_t total_mb_in_frame_display;
    CODEC_RECT_S display_rect;
    int32_t display_width;
    int32_t display_height;
    CODEC_RECT_S decoded_rect;
    int32_t aspect_rate_info;
    int32_t frame_rate_numerator;
    int32_t frame_rate_denominator;
    int32_t display_poc;
    int32_t decoded_poc;
    int32_t error_reason;
    int32_t warn_info;
    int32_t sequence_no;
    int32_t temporal_id;
    int32_t output_flag;
    int32_t ctu_size;
} VIDEO_FRAME_INFO_S;
```
**Member Descriptions**

|          Member           |              Meaning              |
| :------------------------: | :--------------------------------: |
|       decode_result       |         Decoding result          |
|    frame_display_index    |           Display index           |
|    frame_decoded_index    |           Decoded index           |
|    stream_start_addr;     |         Stream start address        |
|       stream_size;        |             Stream size            |
|         nalu_type         |           NALU type               |
|  err_mb_in_frame_decoded  | Error MB blocks in decoded frame |
| total_mb_in_frame_decoded | Total MB blocks in decoded frame |
|  err_mb_in_frame_display  | Error MB blocks in displayed frame |
| total_mb_in_frame_display | Total MB blocks in displayed frame |
|       display_rect        |              Display area          |
|       display_width       |              Display width           |
|      display_height       |              Display height          |
|       decoded_rect        |              Decoded area          |
|     aspect_rate_info      |            Aspect ratio info       |
|   frame_rate_numerator    |      Numerator of frame rate       |
|  frame_rate_denominator   |     Denominator of frame rate      |
|        display_poc        |            Displayed POC           |
|        decoded_poc        |            Decoded POC             |
|       error_reason        |             Error reason            |
|         warn_info         |              Warning info           |
|        sequence_no        |         Frame sequence number       |
|        temporal_id        | Temporal ID in custom GOP         |
|        output_flag        |            Output flag             |
|         ctu_size          |                 CTU size           |



### VIDEO_FRAME_INFO_JPEG_S
**Description**
> Defines the structure for video image frame information.

**Struct Definition**
```C
typedef struct HB_VIDEO_FRAME_INFO_JPEG_S {
    int32_t decode_result;
    int32_t frame_display_index;
    uint64_t stream_start_addr;
    int32_t stream_size;
    int32_t err_rst_idx;
    int32_t err_pos_x;
    int32_t err_pos_y;
    int32_t display_width;
    int32_t display_height;
} VIDEO_FRAME_INFO_JPEG_S;
```
**Member Descriptions**

|          Member          |                             Meaning                             |
| :-----------------------: | :-------------------------------------------------------------: |
|        decode_result;       | The result of the decoding process.                            |
| frame_display_index; | The index used for displaying the frame.                       |
|  stream_start_addr;       | The starting address of the stream.                            |
|        stream_size;        | The size of the stream in bytes.                               |
|      err_rst_idx;         | JPEG error restart index, available after successful decoding. |
|       err_pos_x;          | JPEG error MCU position X, available after successful decoding. |
|       err_pos_y;          | JPEG error MCU position Y, available after successful decoding. |
|     display_width;        | The width of the displayed image.                              |
|    display_height;        | The height of the displayed image.                             |



### VIDEO_STREAM_PACK_S
**Description**
> Defines information about a video stream's buffer.

**Structure Definition**
```C
typedef struct HB_VIDEO_PACK_S {
    hb_char* vir_ptr;           // Pointer to the virtual address of the framebuffer
    uint64_t phy_ptr;            // Physical address of the framebuffer
    uint32_t size;               // Total size of the framebuffer
    uint64_t pts;                // Presentation timestamp of the frame
    uint32_t fd;                 // File descriptor of the buffer
    uint32_t src_idx;            // Index of the source buffer
    HB_BOOL stream_end;          // Indicates if this is the end of the data stream
} VIDEO_STREAM_PACK_S;
```
**Member Descriptions**

|    Member    |                  Meaning                  |
| :----------: | :--------------------------------------: |
|  vir_ptr     | Pointer to the virtual address of the frame buffer |
|  phy_ptr     | Physical address of the frame buffer       |
|    size      | Total size of the frame buffer             |
|    pts       | Frame timestamp                          |
|     fd       | File descriptor of the buffer              |
|  src_idx     | Index of the source buffer                 |
| stream_end   | Whether this is the last segment in the stream |

### VIDEO_STREAM_INFO_S
**Description**
> Defines additional information for H264/H265 output streams.

**Structure Definition**
```C
typedef struct HB_VIDEO_STREAM_INFO_S {
    HB_BOOL frame_index;           // Whether the frame is indexed
    uint64_t frame_start_addr;      // Start address of the stream
    int32_t frame_size;            // Size of the stream frame
    int32_t nalu_type;             // NAL unit type
    uint32_t slice_idx;            // Slice index
    uint32_t slice_num;            // Number of slices (valid for H264)
    uint32_t dependent_slice_num;   // Number of dependent slices (valid for H265)
    uint32_t independent_slice_num; // Number of independent slices (valid for H265)
    uint32_t pic_skipped;          // Indicates if the frame was skipped
    uint32_t intra_block_num;      // Number of intra-blocks in the frame
    uint32_t skip_block_num;       // Number of blocks skipped
    uint32_t avg_mb_qp;            // Average macroblock QP
    uint32_t enc_pic_byte;         // Size of encoded image, in bytes
    int32_t enc_gop_pic_idx;       // Index of the GOP picture in encoding
    int32_t enc_pic_poc;           // Encoding picture order count
    uint32_t enc_src_idx;          // Index of the source buffer for encoding
    uint32_t enc_pic_cnt;          // Count of encoded pictures
    int32_t enc_error_reason;      // Encoding error reason
    int32_t enc_warn_info;         // Encoding warning information
    uint32_t frame_cycle;          // Period for encoding one frame
    uint32_t temporal_id;          // Output stream's temporal layer identifier
    uint32_t longterm_ref_type;    // Stream frame type, bits 1 and 0 valid<br/>
                                  // Bit 1 set indicates long-term reference frame<br/>
                                  // Bit 0 set indicates reference to a long-term reference frame.
} VIDEO_STREAM_INFO_S;
```
**Member Descriptions**

|         Member          |                                                 Meaning                                                 |
| :-------------------: | :----------------------------------------------------------------------------------------------: |
|   frame_start_addr    | Address at which the stream begins                                          |
|      frame_size       | Size of the stream frame                                                                       |
|      frame_index      | Index assigned to the reconstructed frame                                                   |
|       nalu_type       | Type of the NAL unit                                                                           |
|       slice_idx       | Index of the slice                                                                              |
|       slice_num       | Number of slices, valid for H264                                                                   |
|  dependent_slice_num  | Number of non-independent slices, valid for H265                                               |
| independent_slice_num | Number of independent slices, valid for H265                                                      |
|      pic_skipped      | Flag indicating if the frame was skipped during encoding                                          |
|    intra_block_num    | Number of intra-blocks within the frame                                                           |
|    skip_block_num     | Number of blocks that were skipped during encoding                                              |
|       avg_mb_qp       | Average macroblock quantization parameter                                                       |
|     enc_pic_byte      | Size of the encoded image, in bytes                                                               |
|    enc_gop_pic_idx    | Index of the GOP picture in the encoding process                                              |
|      enc_pic_poc      | Picture order count of the encoded image                                                         |
|      enc_src_idx      | Index of the source buffer used for encoding                                                      |
|      enc_pic_cnt      | Count of encoded pictures                                                                        |
|   enc_error_reason    | Encoding error reason                                                                            |
|     enc_warn_info     | Encoding warning information                                                                     |
|      frame_cycle      | Period for encoding one frame                                                                      |
|      temporal_id      | Identifier for the output stream's temporal domain                                                |
|   longterm_ref_type   | Stream frame type, where bit 1 and 0 are valid. Bit 1 = long-term reference frame, bit 0 = ref to long-term. |



### VIDEO_STREAM_INFO_JPEG_S
**Description**
> Defines additional information for an MJPEG/JPEG output stream.

**Structure Definition**
```C
typedef struct VIDEO_STREAM_INFO_JPEG_S {
    uint64_t frame_start_addr; // Start address of the stream
    int32_t frame_size;        // Size of the stream in bytes
    uint32_t slice_idx;         // Index of the slice within the frame
    uint32_t slice_num;         // Total number of slices in the frame
    uint32_t frame_cycle;       // Period to encode one frame
}VIDEO_STREAM_INFO_JPEG_S;
```
**Member Descriptions**

| Member         | Meaning                                                                                   |
| :-------------: | :----------------------------------------------------------------------------------------- |
| frame_start_addr | Address of the beginning of the stream in memory                                            |
| frame_size      | Size of the stream in bytes, including all slices                                              |
| slice_idx      | Index of the current slice being processed within the frame                                      |
| slice_num      | Total number of slices that make up the frame                                                   |
| frame_cycle    | Time interval between encoding consecutive frames                                             |

### VIDEO_STREAM_S
**Description**
> Defines a frame stream structure.

**Structure Definition**
```C
typedef struct HB_VIDEO_STREAM_S {
    VIDEO_STREAM_PACK_S pstPack; // Frame stream packet structure
    union {
        VIDEO_STREAM_INFO_S stStreamInfo; // Stream information
        VIDEO_STREAM_INFO_JPEG_S stJpegInfo; // JPEG stream information
    };
}VIDEO_STREAM_S;
```
**Member Descriptions**

| Member        | Meaning                                                                                   |
| :------------: | :----------------------------------------------------------------------------------------- |
| pstPack       | Structure containing details about the frame stream packet                                          |
| stStreamInfo  | Information about the general stream characteristics                                              |
| stJpegInfo    | Information specific to an MJPEG/JPEG stream (e.g., JPEG stream start address and slice details) |

### VENC_RECV_PIC_PARAM_S
**Description**
> Defines a structure for the number of consecutive frames received and encoded by the encoding channel.

**Structure Definition**
```C
typedef struct HB_VENC_RECV_PIC_PARAM_S {
    int32_t s32RecvPicNum; // Number of consecutive frames to receive and encode, range: [-1, 0) ∪ (0, ∞]
}VENC_RECV_PIC_PARAM_S;
```
**Member Description**

| Member      | Meaning                                                                                     |
| :----------: | :------------------------------------------------------------------------------------------- |
| s32RecvPicNum | The number of frames to be continuously received and encoded by the encoding channel. Negative values indicate a variable number or end of stream. |



### VENC_REF_PARAM_S
**Description**
> Defines the structure for the encoder reference parameter.

**Structure Definition**
```C
typedef struct HB_VENC_REF_PARAM_S
{
    uint32_t use_longterm;
    uint32_t longterm_pic_period;
    uint32_t longterm_pic_using_period;
} VENC_REF_PARAM_S;
```
**Member Descriptions**

|          Member          |                  Meaning                  |
| :------------------------: | :--------------------------------------: |
|       use_longterm        | Enables long-term frame reference mode, not dynamically configurable. |
|    longterm_pic_period    | Long-term picture period, dynamically configurable. |
| longterm_pic_using_period | Period of usage for long-term picture references, dynamically configurable. |

### VENC_USER_RC_ATTR_S
**Description**
> Defines user image frame information.

**Structure Definition**
```C
typedef struct HB_VENC_USER_RC_ATTR_S {
    HB_BOOL qp_map_valid;
    unsigned char *qp_map_array;
    uint32_t qp_map_array_count;
} VENC_USER_RC_ATTR_S;
```
**Member Descriptions**

|         Member         |             Meaning             |
| :--------------------: | :------------------------------: |
|    qp_map_valid       | Enables the qp map feature. |
|  qp_map_array      | Pointer to the qp map array.    |
| qp_map_array_count | Length of the qp map array.  |

### USER_FRAME_INFO_S
**Description**
> Defines user image frame information.

**Structure Definition**
```C
typedef struct HB_USER_FRAME_INFO_S {
    VIDEO_FRAME_S stUserFrame;
    VENC_USER_RC_ATTR_S stUserRcInfo;
} USER_FRAME_INFO_S;
```
**Member Descriptions**

|    Member    |            Meaning            |
| :----------: | :------------------------------: |
| stUserFrame | The image frame.                 |
| stUserRcInfo | User RC information.             |



### VENC_PARAM_MOD_S
**Description**
> Defines encoding channel frame rate control parameters.

**Structure Definition**
```C
typedef struct HB_VENC_PARAM_MOD_S {
    uint32_t u32OneStreamBuffer;
} VENC_PARAM_MOD_S;
```
**Member Descriptions**

|        Member        |                       Meaning                        |
| :-----------------: | :-----------------------------------------------: |
| u32OneStreamBuffer | Whether VPS, SPS, PPS, and IDR frames are output per frame. Default is one frame output per frame. |

### VENC_FRAME_RATE_S
**Description**
> Defines encoding channel frame rate control parameters.

**Structure Definition**
```C
typedef struct HB_VENC_FRAME_RATE_S {
    int32_t s32InputFrameRate;
    int32_t s32OutputFrameRate;
} VENC_FRAME_RATE_S;
```
**Member Descriptions**

|        Member        |                 Meaning                  |
| :----------------: | :-----------------------------------: |
| s32InputFrameRate  |              Input frame rate             |
| s32OutputFrameRate | Output frame rate, within the range [1- s32InputFrameRate] |

### VENC_CHN_PARAM_S
**Description**
> Defines parameters for an encoding channel.

**Structure Definition**
```C
typedef struct HB_VENC_CHN_PARAM_S {
    VENC_FRAME_RATE_S stFrameRate;
} VENC_CHN_PARAM_S;
```
**Member Descriptions**

|    Member     |           Meaning           |
| :---------: | :----------------------: |
| stFrameRate | Frame rate control parameter, which can be dynamically set |



### VENC_ROI_ATTR_S
**Description**
> Defines the structure for encoding ROI (Region of Interest) information.

**Structure Definition**
```C
typedef struct HB_VENC_ROI_ATTR_S {
    uint32_t roi_enable;
    uint8_t* roi_map_array;
    uint32_t roi_map_array_count;
} VENC_ROI_ATTR_S;
```
**Member Descriptions**

|        Member         |                   Description                   |
| :--------------------: | :---------------------------------------------: |
|     roi_enable        | Enables the ROI area; not dynamically configurable. |
|    roi_map_array      | ROI area QP array, dynamically configurable.    |
| roi_map_array_count  | Number of ROI QP array elements, not dynamically configurable. |

### VENC_CHN_STATUS_S
**Description**
> Defines the structure for encoding channel status.

**Structure Definition**
```C
typedef struct HB_VENC_CHN_STATUS_S {
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
} VENC_CHN_STATUS_S;
```
**Member Descriptions**

|         Member         |                        Meaning                        |
| :---------------------: | :--------------------------------------------------: |
|  cur_input_buf_cnt     | Current number of input frames yet to be encoded   |
|  cur_input_buf_size    | Current size of input frame buffers                 |
|  cur_output_buf_cnt    | Current number of encoded frames                    |
| cur_output_buf_size    | Current size of encoded buffer                      |
|   left_recv_frame      | Remaining frames to be received                       |
|    left_enc_frame      | Remaining frames to be encoded                       |
| total_input_buf_cnt    | Total number of input frames received               |
| total_output_buf_cnt  | Total number of encoded frames                       |
|       pipeline        | The current stage of the encoding pipeline           |
|   channel_port_id      | Identifier for the encoding channel                  |



### VENC_3DNR_PARAMS
**Description**
> Defines the structure for encoding channel state.

**Structure Definition**
```C
typedef struct HB_VENC_3DNR_PARAMS {
    uint32_t nr_y_enable;
    uint32_t nr_cb_enable;
    uint32_t nr_cr_enable;
    uint32_t nr_est_enable;
    uint32_t nr_intra_weightY;
    uint32_t nr_intra_weightCb;
    uint32_t nr_intra_weightCr;
    uint32_t nr_inter_weightY;
    uint32_t nr_inter_weightCb;
    uint32_t nr_inter_weightCr;
    uint32_t nr_noise_sigmaY;
    uint32_t nr_noise_sigmaCb;
    uint32_t nr_noise_sigmaCr;
} VENC_3DNR_PARAMS;
```
**Member Descriptions**

| Member | Meaning |
| :-----: | :------ |
| `nr_y_enable` | Y component noise reduction enabled |
| `nr_cb_enable` | Cb component noise reduction enabled |
| `nr_cr_enable` | Cr component noise reduction enabled |
| `nr_est_enable` | Noise estimation enabled |
| `nr_intra_weightY` | Intra-frame image Y-weighting coefficient |
| `nr_intra_weightCb` | Intra-frame image Cb-weighting coefficient |
| `nr_intra_weightCr` | Intra-frame image Cr-weighting coefficient |
| `nr_inter_weightY` | Inter-frame image Y-weighting coefficient |
| `nr_inter_weightCb` | Inter-frame image Cb-weighting coefficient |
| `nr_inter_weightCr` | Inter-frame image Cr-weighting coefficient |
| `nr_noise_sigmaY` | Y component noise standard deviation |
| `nr_noise_sigmaCb` | Cb component noise standard deviation |
| `nr_noise_sigmaCr` | Cr component noise standard deviation |

## Error Codes
The VENC error codes are as follows:

| Error Code | Macro Definition | Description |
| :---------: | :--------------: | -----------: |
| `-268958720` | `HB_ERR_VENC_UNKNOWN` | Unknown error |
| `-268958721` | `HB_ERR_VENC_NOT_FOUND` | VENC channel not found |
| `-268958722` | `HB_ERR_VENC_OPEN_FAIL` | Failed to open VENC channel |
| `-268958723` | `HB_ERR_VENC_RESPONSE_TIMEOUT` | No response from VENC channel |
| `-268958724` | `HB_ERR_VENC_INIT_FAIL` | Failed to initialize VENC module |
| `-268958725` | `HB_ERR_VENC_OPERATION_NOT_ALLOWDED` | Operation not allowed |
| `-268958726` | `HB_ERR_VENC_NOMEM` | Insufficient VENC memory |
| `-268958727` | `HB_ERR_VENC_NO_FREE_CHANNEL` | No available VENC channel |
| `-268958728` | `HB_ERR_VENC_ILLEGAL_PARAM` | Invalid parameter |
| `-268958729` | `HB_ERR_VENC_INVALID_CHNID` | Invalid channel ID |
| `-268958730` | `HB_ERR_VENC_INVALID_BUF` | Invalid buffer block |
| `-268958731` | `HB_ERR_VENC_INVALID_CMD` | Invalid command |
| `-268958732` | `HB_ERR_VENC_WAIT_TIMEOUT` | Timeout waiting |
| `-268958733` | `HB_ERR_VENC_FILE_OPERATION_FAIL` | Operation failure |
| `-268958734` | `HB_ERR_VENC_PARAMS_SET_FAIL` | Failed to set parameters |
| `-268958735` | `HB_ERR_VENC_PARAMS_GET_FAIL` | Failed to get parameters |
| `-268958736` | `HB_ERR_VENC_EXIST` | VENC channel already exists |
| `-268958737` | `HB_ERR_VENC_UNEXIST` | VENC channel does not exist |
| `-268958738` | `HB_ERR_VENC_NULL_PTR` | Null pointer |
| `-268958739` | `HB_ERR_VENC_UNSUPPORT` | Not supported |

## Referencing Code
For a sample of VENC code, you can refer to the [sample_video_codec](./multimedia_samples#sample_video_codec) section.

