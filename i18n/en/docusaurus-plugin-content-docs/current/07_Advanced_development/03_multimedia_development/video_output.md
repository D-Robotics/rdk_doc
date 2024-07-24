---
sidebar_position: 8
---
# 7.3.8 Video Output

## Overview
The Video Output (VOT) module actively reads image and graphics data from memory and outputs the image through the corresponding display output device. The supported display/write-back devices, video layers, and graphics layers of the chip are shown in the following table.

![image-20220329222211836](./image/video_output/image-20220329222211836.png)

## Functional Description

### Basic Concepts
- HD Display Device

  The SDK identifies HD devices as DHVx, where x is the index number starting from 0, indicating the number of HD devices. X3 has one HD device, DHV0.

- Video Layer

  - For the video layer fixed on each display device, the SDK uses VHVx to identify it. X3 DHV0 has one video layer, VHV0.
  - VHV0 supports zooming and has 2 channels.
  - X3 output interface supports RGB, BT1120/BT656, and MIPI. All three interfaces support a maximum output timing of 1080P@60fps.

- Video Write-Back

  The write-back device is referred to as WD. The write-back function: X3 only supports device-level write-back, capturing video data output at the device level, which can be used for display and encoding.

- Channels

  Channels are managed by video layers. Each video layer of X3 supports 2 channels.

- Graphics Layer

  X3 has 2 graphics layers, which are fixedly bound to DHV0.

- Input and Output Data Formats

  VOT supports input and output data in specified formats, where output refers to writing back data to DDR. The supported input and output data formats of X3 are shown in the table below.

|   Input Format    |   Output Format    |
| :----------------: | :----------------: |
| FORMAT_YUV422_UYVY | FORMAT_YUV422_UYVY |
| FORMAT_YUV422_VYUY | FORMAT_YUV422_VYUY |
| FORMAT_YUV422_YVYU | FORMAT_YUV422_YVYU |
| FORMAT_YUV422_YUYV | FORMAT_YUV422_YUYV |
| FORMAT_YUV422SP_UV | FORMAT_YUV420SP_UV |
| FORMAT_YUV422SP_VU | FORMAT_YUV420SP_VU |
| FORMAT_YUV420SP_UV |   FORMAT_BGR0     |
| FORMAT_YUV420SP_VU |                    || FORMAT_YUV422P_UV  |                    |
| FORMAT_YUV422P_VU  |                    |
| FORMAT_YUV420P_UV  |                    |


## API Reference

Video Output (VOT) implements functions to enable video output devices or channels, and send video data to output channels.

VOT provides the following APIs:

```C
HB_VOT_SetPubAttr: Set the public attributes of the video output device.
HB_VOT_GetPubAttr: Get the public attributes of the video output device.
HB_VOT_Enable: Enable the video output device.
HB_VOT_Disable: Disable the video output device.
HB_VOT_SetLcdBackLight: Set the LCD backlight.
HB_VOT_SetVideoLayerAttr: Set the attributes of the video layer.
HB_VOT_GetVideoLayerAttr: Get the attributes of the video layer.
HB_VOT_EnableVideoLayer: Enable the video layer.
HB_VOT_DisableVideoLayer: Disable the video layer.
HB_VOT_SetVideoLayerCSC: Set the CSC (Color Space Conversion) of the video layer.
HB_VOT_GetVideoLayerCSC: Get the CSC (Color Space Conversion) of the video layer.
HB_VOT_SetVideoLayerUpScale: Set the upscaling parameters of the video layer.
HB_VOT_GetVideoLayerUpScale: Get the upscaling parameters of the video layer.
HB_VOT_BatchBegin: Start batch processing of video layer attributes.
HB_VOT_BatchEnd: End batch processing of video layer attributes.
HB_VOT_GetScreenFrame: Get the output image of the device.
HB_VOT_ReleaseScreenFrame: Release the output image of the device.
HB_VOT_SetChnAttr: Set the attributes of the video output channel.
HB_VOT_GetChnAttr: Get the attributes of the video output channel.
HB_VOT_SetChnAttrEx: Set the advanced parameters of the video output channel.
HB_VOT_GetChnAttrEx: Get the advanced parameters of the video output channel.
HB_VOT_EnableChn: Enable the video output channel.
HB_VOT_DisableChn: Disable the video output channel.
HB_VOT_SetChnCrop: Set the cropping attributes of the channel.
HB_VOT_GetChnCrop: Get the cropping attributes of the channel.
HB_VOT_SetChnDisplayPosition: Set the display position of the channel.
HB_VOT_GetChnDisplayPosition: Get the display position of the channel.
HB_VOT_SetChnFrameRate: Set the display frame rate of the channel.
HB_VOT_GetChnFrameRate: Get the display frame rate of the channel.
HB_VOT_ShowChn: Show the channel image.
HB_VOT_HideChn: Hide the channel image.
HB_VOT_SendFrame: Send the output image.
HB_VOT_ClearChnBuf: Clear the image buffer.
HB_VOT_EnableWB: Enable write back.
HB_VOT_DisableWB: Disable write back.
HB_VOT_SetWBAttr: Set the write back attributes.
HB_VOT_GetWBAttr: Get the write back attributes.
HB_VOT_GetWBFrame: Get the write back image.HB_VOT_ReleaseWBFrame: Release write-back image.
HB_VOT_ShutDownHDMI: Turn off HDMI output image to the device, and the display module still works normally.
HB_VOT_StartHDMI: Enable HDMI output image to the device.
```

### HB_VOT_SetPubAttr
【Function Declaration】
```C
int HB_VOT_SetPubAttr(uint8_t dev, const VOT_PUB_ATTR_S *pstPubAttr);
```
【Description】
> Set the public attributes of video output.

【Parameter Description】

| Parameter Name |                   Description                   | Input/Output |
| :------------: | :---------------------------------------------: | :---------: |
|      dev       | Video output device ID. Range: 0.                |    Input    |
| pstPubAttr    | Video output device public attributes.          |    Input    |



【Return Value】

| Return Value | Description |
| :---------: | :---------: |
|     0      | Success.    |
| Non-zero   | Failure.    |


【Notes】
> None.

【Reference Code】
> Refer to HB_VOT_Enable.

### HB_VOT_GetPubAttr
【Function Declaration】
```C
int HB_VOT_GetPubAttr(uint8_t dev, VOT_PUB_ATTR_S *pstPubAttr);
```
【Description】
> Get the public attributes of video output.

【Parameter Description】

|  Parameter Name  |              Description              | Input/Output |
| :--------------: | :-----------------------------------: | :----------: |
|       dev        |    Video output device ID.<br/>Range: 0.    |    Input     |
|    pstPubAttr    |       Video output device public attributes.      |    Output    |

【Return Value】

| Return value | Description |
| :----------: | :---------: |
|      0       |   Success.  |
|    Non-0     |   Failure.  |

【Notes】
> None

【Reference Code】
> See [HB_VOT_Enable](#HB_VOT_Enable)

### HB_VOT_Enable
【Function Declaration】
```C
int HB_VOT_Enable(uint8_t dev);
```
【Function Description】
> Enable video output device.

【Parameter Description】

| Parameter Name |           Description           | Input/Output |
| :------------: | :----------------------------: | :----------: |
|      dev       | Video output device ID.<br/>Range: 0.  |    Input     |

【Return Value】

| Return value | Description |
| :----------: | :---------: |
|      0       |   Success.  |
|    Non-0     |   Failure.  |

【Notes】
> None

【Reference Code】
```C
    int ret = 0; VOT_PUB_ATTR_S stPubAttr = {};
    ret = HB_VOT_GetPubAttr(0, &stPubAttr);
    if (ret) {
        printf("HB_VOT_GetPubAttr failed.\n");
        // break;
    }
    stPubAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
    stPubAttr.u32BgColor = 0xFF7F88;
    ret = HB_VOT_SetPubAttr(0, &stPubAttr);
    if (ret) {
        printf("HB_VOT_SetPubAttr failed.\n");
        // break;
    }
    ret = HB_VOT_Enable(0);
    if (ret) {
        printf("HB_VOT_Enable failed.\n");
    }
    ret = HB_VOT_Disable(0);
    if (ret) {
        printf("HB_VOT_Disable failed.\n");
    }
```

### HB_VOT_Disable
【Function Declaration】
```C
int HB_VOT_Disable(uint8_t dev);
```
【Description】
> Disable video output device.

【Parameters】

| Parameter Name |          Description          | Input/Output |
| :------------: | :---------------------------: | :----------: |
|      dev       | Video output device ID.<br/>Range: 0. |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success   |
|   Non-zero   |   Failure   |

【Notes】
> None

【Reference Code】
> See [HB_VOT_Enable](#HB_VOT_Enable)  

### HB_VOT_SetLcdBackLight
【Function Declaration】
```C
int HB_VOT_SetLcdBackLight (uint8_t dev, uint32_t backlight);
```
【Description】
> Set LCD backlight brightness.

【Parameters】

| Parameter Name |          Description          | Input/Output |
| :------------: | :---------------------------: | :----------: |
|      dev       | Video output device ID.<br/>Range:0. |    Input     |
| backlight | Backlight brightness value. <br/>Value range: 0-10. The larger the value, the brighter the backlight.<br/>When the brightness value is 0, the screen is completely black. | Input |

【Return Value】

| Return Value | Description |
| :----: | :----: |
|   0    | Success. |
|  Non-zero   | Failure. |

【Notes】
> None

【Reference Code】
> None

### HB_VOT_SetVideoLayerAttr
【Function Declaration】
```C
int HB_VOT_SetVideoLayerAttr(uint8_t layer, const VOT_VIDEO_LAYER_ATTR_S *pstLayerAttr);
```
【Function Description】
> Set the attributes of the video layer.

【Parameter Description】

|   Parameter Name   |                Description                 | Input/Output |
| :----------: | :---------------------------------: | :-------: |
|    layer     | Video output layer id.<br/>Value range: 0. |   Input    |
| pstLayerAttr |        Attributes of the video output layer.         |   Input    |

【Return Value】

| Return Value |  Description  |
| :----: | :----: |
|   0    | Success. |
|  Non-zero   | Failure. |

【Notes】
> Device needs to be enabled first.

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerAttr
【Function Declaration】
```C
int HB_VOT_GetVideoLayerAttr(uint8_t layer,  VOT_VIDEO_LAYER_ATTR_S *pstLayerAttr);
```
【Function Description】

> Get the attributes of the video layer.

【Parameter Description】

| Parameter Name |               Description                | Input/Output |
| :------------: | :--------------------------------------: | :----------: |
|     layer      | Video output video layer ID.<br/>Range: 0 |    Input     |
| pstLayerAttr   |      Video output video layer attribute.  |    Output    |

【Return Value】

| Return Value |  Description  |
| :----------: | :-----------: |
|      0       |    Success.   |
|   Non-zero   |    Failed.    |

【Notes】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_EnableVideoLayer
【Function Declaration】
```C
int HB_VOT_EnableVideoLayer(uint8_t layer);
```
【Function Description】
> Enable the video layer.

【Parameter Description】

| Parameter Name |                 Description                 | Input/Output |
| :------------: | :-----------------------------------------: | :----------: |
|     layer      | Video output video layer ID.<br/>Range: 0.  |    Input     |

【Return Value】

| Return Value |  Description  |
| :----------: | :-----------: |
|      0       |    Success.   |
|   Non-zero   |    Failed.    |

【Notes】
> None

【Reference Code】
```C
    ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
        printf("HB_VOT_GetVideoLayerAttr failed.\n");
    }
    printf("stLayer width:%d\n", stLayerAttr.stImageSize.u32Width);
    printf("stLayer height:%d\n", stLayerAttr.stImageSize.u32Height);
    stLayerAttr.stImageSize.u32Width = 1920;
    stLayerAttr.stImageSize.u32Height = 1080;
    ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
        printf("HB_VOT_SetVideoLayerAttr failed.\n");
    }
    ret = HB_VOT_GetVideoLayerCSC(0, &stCsc);
    if (ret) {
        printf("HB_VOT_GetVideoLayerCSC failed.\n");
    }
    printf("stCsc luma :%d\n", stCsc.u32Luma);
    printf("stCsc contrast :%d\n", stCsc.u32Contrast);
    printf("stCsc hue :%d\n", stCsc.u32Hue);
    printf("stCsc satuature :%d\n", stCsc.u32Satuature);
    stCsc.u32Luma = 60;
    stCsc.u32Contrast = 60;
    stCsc.u32Hue = 60;
    stCsc.u32Satuature = 60;
    ret = HB_VOT_SetVideoLayerCSC(0, &stCsc);
    ret = HB_VOT_GetVideoLayerUpScale(0, &stUpScale);
    if (ret) {
        printf("HB_VOT_GetVideoLayerUpScale failed.\n");
    }
    printf("stUpScale src width :%d\n", stUpScale.src_width);
    printf("stUpScale src height :%d\n", stUpScale.src_height);
    printf("stUpScale tgt width :%d\n", stUpScale.tgt_width);
    printf("stUpScale tgt height :%d\n", stUpScale.tgt_height);
    printf("stUpScale pos x :%d\n", stUpScale.pos_x);
    printf("stUpScale pos y :%d\n", stUpScale.pos_y);
    stUpScale.src_width = 1280;
    stUpScale.src_height = 720;
    stUpScale.tgt_width = 1920;
    stUpScale.tgt_height = 1080;
    ret = HB_VOT_SetVideoLayerUpScale(0, &stUpScale);
    if (ret) {
        printf("HB_VOT_SetVideoLayerUpScale failed.\n");
    }
    ret = HB_VOT_EnableVideoLayer(0);
    if (ret) {
        printf("HB_VOT_EnableVideoLayer failed.\n");
    }

    ret = HB_VOT_GetChnAttr(0, 0, &stChnAttr);
    if (ret) {
        printf("HB_VOT_GetChnAttr failed.\n");
    }
    printf("stChnAttr priority :%d\n", stChnAttr.u32Priority);
    printf("stChnAttr src width :%d\n", stChnAttr.u32SrcWidth);
    printf("stChnAttr src height :%d\n", stChnAttr.u32SrcHeight);
    printf("stChnAttr s32X :%d\n", stChnAttr.s32X);
    printf("stChnAttr s32Y :%d\n", stChnAttr.s32Y);
    printf("stChnAttr u32DstWidth :%d\n", stChnAttr.u32DstWidth);
    printf("stChnAttr u32DstHeight :%d\n", stChnAttr.u32DstHeight);
    stChnAttr.u32Priority = 0;
    stChnAttr.u32SrcWidth = 1920;
    stChnAttr.u32SrcHeight = 1080;
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32DstWidth = 1920;
    stChnAttr.u32DstHeight = 1080;
    ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
    if (ret) {
        printf("HB_VOT_SetChnAttr failed.\n");
        //   break;
    }
    ret = HB_VOT_EnableChn(0, 0);
    if (ret) {
        printf("HB_VOT_EnableChn failed.\n");
    }
    ret = HB_VOT_GetChnCrop(0, 0, &stCrop);
    if (ret) {
        printf("HB_VOT_GetChnCrop failed.\n");
    }
    printf("stCrop width :%d\n", stCrop.u32Width);
    printf("stCrop height :%d\n", stCrop.u32Height);
    stCrop.u32Width = 1280;
    stCrop.u32Height = 720;
    ret = HB_VOT_SetChnCrop(0, 0, &stCrop);
    if (ret) {
        printf("HB_VOT_SetChnCrop failed.\n");
    }
    ret = HB_VOT_GetChnDisplayPosition(0, 0, &stPoint);
    if (ret) {
        printf("HB_VOT_GetChnDisplayPosition failed.\n");
    }
    printf("stPoint s32x :%d\n", stPoint.s32X);
    printf("stPoint s32y :%d\n", stPoint.s32Y);
    stPoint.s32X = 200;
    stPoint.s32Y = 200;
    ret = HB_VOT_SetChnDisplayPosition(0, 0, &stPoint);
    if (ret) {
        printf("HB_VOT_SetChnDisplayPosition failed.\n");
    }
    ret = HB_VOT_GetChnAttrEx(0, 0, &stChnAttrEx);
    if (ret) {
        printf("HB_VOT_GetChnAttrEx failed.\n");
        // break;
    }
    printf("stChnAttrEx format :%d\n", stChnAttrEx.format);
    printf("stChnAttrEx alpha_en :%d\n", stChnAttrEx.alpha_en);
    printf("stChnAttrEx alpha_sel :%d\n", stChnAttrEx.alpha_sel);
    printf("stChnAttrEx alpha :%d\n", stChnAttrEx.alpha);
    printf("stChnAttrEx keycolor :%d\n", stChnAttrEx.keycolor);
    printf("stChnAttrEx ov_mode :%d\n", stChnAttrEx.ov_mode);
    // stChnAttrEx.format = 1;
    stChnAttrEx.alpha_en = 1;
    stChnAttrEx.alpha_sel = 0;
    stChnAttrEx.alpha = 30;
    stChnAttrEx.keycolor = 0x7F88;
    stChnAttrEx.ov_mode = 1;
    ret = HB_VOT_SetChnAttrEx(0, 0, &stChnAttrEx);
    if (ret) {
        printf("HB_VOT_SetChnAttrEx failed.\n");
    }

    ret = HB_VOT_DisableVideoLayer(0);
    if (ret) {
        printf("HB_VOT_DisableVideoLayer failed.\n");
    }
```

### HB_VOT_DisableVideoLayer
【Function declaration】
```C
int HB_VOT_DisableVideoLayer(uint8_t layer);
```
【Function Description】
> Disable the video layer.

【Parameter Description】

| Parameter |                   Description                    | Input/Output |
| :-------: | :----------------------------------------------: | :----------: |
|   layer   | ID of the video output video layer.<br/>Range 0. |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success   |
|   Non-zero   |   Failure   |

【Precautions】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_SetVideoLayerCSC

【Function Declaration】
```C
int HB_VOT_SetVideoLayerCSC(uint8_t layer, const VOT_CSC_S *pstVideoCSC);
```
【Description】
> Set the image effect of the video layer output.

【Parameter Description】

|  Parameter Name   |            Description            | Input/Output |
| :---------------: | :-------------------------------: | :----------: |
|       layer       |   Video output layer ID.<br/>Range: 0.   |    Input     |
| pstVideoCSC |   Image output effect of the video.   |    Input     |

【Return Value】

| Return Value |         Description          |
| :----------: | :--------------------------: |
|      0       |           Success.           |
|   Non-zero   |          Failure.           |

【Notes】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerCSC
【Function Declaration】
```C
int HB_VOT_GetVideoLayerCSC(uint8_t layer, VOT_CSC_S *pstVideoCSC);
```
【Description】
> Get the image effect of the video layer output.

【Parameter Description】

|  Parameter Name   |            Description            | Input/Output |
| :---------------: | :-------------------------------: | :----------: |
|       layer       |   Video output layer ID.<br/>Range: 0.   |    Input     |
| pstVideoCSC |   Image output effect of the video.   |    Output    |

【Return Value】

| Return Value |         Description          |
| :----------: | :--------------------------: |
|      0       |           Success.           |
|   Non-zero   |          Failure.           |【Precautions】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_SetVideoLayerUpScale
【Function Declaration】
```C
int HB_VOT_SetVideoLayerUpScale(uint8_t layer, const VOT_UPSCALE_ATTR_S *pstUpScaleAttr);
```
【Function Description】
> Set the scaling attribute of the video layer.

【Parameter Description】

| Parameter Name |             Description             | Input/Output |
| :------------: | :---------------------------------: | :----------: |
|     layer      | Video output layer ID.<br/>Range: 0 |    Input     |
| pstUpScaleAttr |         Scaling attributes         |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success   |
|   Non-zero   |   Failed    |

【Precautions】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerUpScale
**Function Declaration**
```C
int HB_VOT_GetVideoLayerUpScale(uint8_t layer, VOT_UPSCALE_ATTR_S *pstUpScaleAttr);
```
**Function Description**
> Retrieves the video layer scaling attributes.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|     layer      | Video output layer ID. <br/>Range: 0. |   Input    |
| pstUpScaleAttr | Video layer scaling attributes. |   Output    |

**Return Values**

| Return Value | Description |
| :----------: | :----------: |
|     0       | Success.    |
| Non-zero    | Failure.    |

**Caution**
> None

**Reference Code**
> See HB_VOT_EnableVideoLayer

### HB_VOT_BatchBegin
**Function Declaration**
```C
int HB_VOT_BatchBegin(uint8_t layer);
```
**Function Description**
> Begins setting channel properties for a video layer.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|     layer      | Video output layer ID. <br/>Range: 0. |   Input    |

**Return Values**

| Return Value | Description |
| :----------: | :----------: |
|     0       | Success.    |
| Non-zero    | Failure.    |

**Caution**
> None

**Reference Code**
> None


### HB_VOT_BatchEnd
**Function Declaration**
```C
int HB_VOT_BatchEnd(uint8_t layer);
```
**Function Description**
> Ends setting channel properties for a video layer.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|     layer      | Video output layer ID. <br/>Range: 0. |   Input    |

**Return Values**

| Return Value | Description |
| :----------: | :----------: |
|     0       | Success.    |
| Non-zero    | Failure.    |

**Caution**
> None

**Reference Code**
> None


### HB_VOT_GetScreenFrame
**Function Declaration**
```C
int HB_VOT_GetScreenFrame(uint8_t layer, void *pstVFrame, int millisec);
```
**Function Description**
> Retrieves the output screen image data.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|   layer       | Video output layer ID. <br/>Range: 0. |    Input    |
| pstVFrame     | Pointer to the output screen image data information. |   Output    |
|  millisec     | Timeout in milliseconds. |    Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|    0      | Success.    |
| Non-zero   | Failure.    |

**Caution**
> This function must be used after enabling the device, video layer, and channel.

**Reference Code**
```C
    int ret;
    hb_vio_buffer_t stVFrame;

    ret = HB_VOT_GetScreenFrame(0, &stVFrame, 0);
    if (ret != 0) {
        printf("HB_VOT_GetScreenFrame failed.\n");
    }

    ret = HB_VOT_ReleaseScreenFrame(0, &stVFrame, 0);
    if (ret != 0) {
        printf("HB_VOT_ReleaseScreenFrame failed.\n");
    }
```
### HB_VOT_ReleaseScreenFrame
**Function Declaration**
```C
int HB_VOT_ReleaseScreenFrame(uint8_t layer, const void *pstVFrame);
```
**Function Description**
> Releases the output screen image data.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|   layer       | Video output layer ID. <br/>Range: 0. |    Input    |
| pstVFrame     | Pointer to the output screen image data information. |   Input    |

**Return Values**

| Return Value | Description |
| :--------: | :---------: |
|    0      | Success.    |
| Non-zero   | Failure.    |

**Caution**
> None

**Reference Code**
> See the example for HB_VOT_ReleaseScreenFrame above, as there is no separate reference code provided.



### HB_VOT_SetChnAttr
**Function Declaration**
```C
int HB_VOT_SetChnAttr(uint8_t layer, uint8_t chn, const VOT_CHN_ATTR_S *pstChnAttr);
```
**Function Description**
> Sets the properties of a video output channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   layer        | Video output layer ID (range: 0). | Input      |
|    chn         | Video output channel ID (range: [0, 4)).<br/>0 and 1 correspond to video channels; <br/>2 and 3 are graphic channels. | Input      |
| pstChnAttr    | Video output channel attributes. | Input      |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|    0       | Success.   |
| Non-zero   | Failure.   |

**Note**
> None

**Reference Code**
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetChnAttr
**Function Declaration**
```C
int HB_VOT_GetChnAttr(uint8_t layer, uint8_t chn, VOT_CHN_ATTR_S *pstChnAttr);
```
**Function Description**
> Retrieves the properties of a video output channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|   layer        | Video output layer ID (range: 0). | Input      |
|    chn         | Video output channel ID (range: [0, 4)).<br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. | Input      |
| pstChnAttr    | Video output channel attributes. | Output     |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|    0       | Success.   |
| Non-zero   | Failure.   |

**Note**
> None

**Reference Code**
> See HB_VOT_EnableVideoLayer


### HB_VOT_SetChnAttrEx
**Function Declaration**
```C
int HB_VOT_SetChnAttrEx(uint8_t layer, uint8_t chn, const VOT_CHN_ATTR_EX_S *pstChnAttrEx);
```
**Function Description**
> Sets advanced properties for a video output channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|    layer       | Video output layer ID. Range: 0. | Input      |
|     chn        | Video output channel ID. Range: [0, 4). <br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. | Input      |
| pstChnAttrEx   | Advanced video output channel properties. | Input      |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|    0       | Success.    |
| Non-zero   | Failure.    |

**Note**
> None

**Reference Code**
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetChnAttrEx
**Function Declaration**
```C
int HB_VOT_GetChnAttrEx(uint8_t layer, uint8_t chn, VOT_CHN_ATTR_EX_S *pstChnAttrEx);
```
**Function Description**
> Retrieves advanced properties for a video output channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
|    layer       | Video output layer ID. Range: 0. | Input      |
|     chn        | Video output channel ID. Range: [0, 4). <br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. | Input      |
| pstChnAttrEx   | Advanced video output channel properties. | Output     |

**Return Values**

| Return Value | Description |
| :---------: | :---------: |
|    0       | Success.    |
| Non-zero   | Failure.    |

**Note**
> None

**Reference Code**
> See HB_VOT_EnableVideoLayer



### HB_VOT_EnableChn
**Function Declaration**
```C
int HB_VOT_EnableChn(uint8_t layer, uint8_t chn);
```
**Function Description**
> Enables a video output channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                                   | Input/Output |
| :------------: | :--------------------------------------------------------------------------------------------- | :----------: |
|     layer      | ID of the video output layer. Range: 0.                                                              |    Input    |
|       chn      | ID of the video output channel. Range: [0, 4). <br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. |    Input    |

**Return Values**

| Return Value | Description                     |
| :---------: | :------------------------------ |
|      0      | Success.                        |
| Non-zero    | Failure.                        |

**Caution**
> None

**Reference Code**
> Refer to HB_VOT_EnableVideoLayer

### HB_VOT_DisableChn
**Function Declaration**
```C
int HB_VOT_DisableChn(uint8_t layer, uint8_t chn);
```
**Function Description**
> Disables a video output channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                             | Input/Output |
| :------------: | :------------------------------------------------------------------------------------- | :----------: |
|     layer      | ID of the video output layer. Range: 0.                                                   |    Input    |
|       chn      | ID of the video output channel. Range: [0, 4). <br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. |    Input    |

**Return Values**

| Return Value | Description                     |
| :---------: | :------------------------------ |
|      0      | Success.                        |
| Non-zero    | Failure.                        |

**Caution**
> None

**Reference Code**
> Refer to HB_VOT_EnableVideoLayer



### HB_VOT_SetChnCrop
**Function Declaration**
```C
int HB_VOT_SetChnCrop(uint8_t layer, uint8_t chn, const VOT_CROP_INFO_S *pstCropInfo);
```
**Function Description**
> Sets the cropping properties for the video output channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                                   | Input/Output |
| :------------- | :--------------------------------------------------------------------------------------------- | :----------- |
|    layer       | ID of the video output layer. Range: 0.                                                       |    Input     |
|     chn        | ID of the video output channel. Range: [0, 4). <br/>0 and 1 represent video channels; <br/>2 and 3 are graphic channels. |    Input     |
| pstCropInfo    | Cropping attributes for the video output channel.                                                  |    Input     |

**Return Values**

| Return Value | Description                            |
| :----------: | :-------------------------------------- |
|      0       | Success.                                |
| Non-zero value | Failure.                               |

**Caution**
> None.

**Reference Code**
> Refer to HB_VOT_EnableVideoLayer for an example.



### HB_VOT_GetChnCrop
【Function Declaration】
```C
int HB_VOT_GetChnCrop(uint8_t layer, uint8_t chn, VOT_CROP_INFO_S *pstCropInfo);
```
【Description】
> Get the cropping properties of the video output channel.

【Parameter Description】

|  Parameter Name   | Description  | Input/Output |
| :---------: | :----------------------------------------------------------------------------- | :-------: |
|    layer    | Video output video layer ID. Value range: 0.                                                |   Input    |
|     chn     | Video output channel ID. Value range: [0, 4).<br/>0 and 1 represent video channels;<br/>2 and 3 represent graphic channels. |   Input    |
| pstCropInfo | Video output channel crop attributes.                                                         |   Output    |

【Return Value】

| Return Value |  Description  |
| :----: | :----: |
|   0    | Success. |
|  Non-zero   | Failure. |

【Notice】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_SetChnDisplayPosition
【Function Declaration】
```C
int HB_VOT_SetChnDisplayPosition(uint8_t layer, uint8_t chn, const POINT_S *pstDispPos);
```
【Function Description】
> Set the display position of the video output channel.

【Parameter Description】

|  Parameter  | Description                                                                           | Input/Output |
| :--------: | :----------------------------------------------------------------------------- | :-------: |
|   layer    | Video output video layer ID. Value range: 0.                                                |   Input    |
|    chn     | Video output channel ID. Value range: [0, 4).<br/>0 and 1 represent video channels;<br/>2 and 3 represent graphic channels. |   Input    |
| pstDispPos | Video output channel display position.                                                         |   Input    |

【Return Value】

| Return Value |  Description  |
| :----: | :----: |
|   0    | Success. |
|  Non-zero   | Failure. |

【Notice】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_GetChnDisplayPosition
【Function Declaration】
```C
int HB_VOT_GetChnDisplayPosition(uint8_t layer, uint8_t chn, POINT_S *pstDispPos);
```

【Function Description】
> Get the displayed coordinates of the video output channel.

【Parameter Description】

| Parameter Name | Description | Input/Output |
| :------------: | :-------------------------------------------------------------- | :----------: |
|     layer      | Video output layer ID. Value range: 0.                          |    Input     |
|      chn       | Video output channel ID. Value range: [0, 4).<br/>0, 1 represent video channels;<br/>2, 3 represent graphic channels. |    Input     |
|  pstDispPos    | Displayed coordinates of the video output channel.              |    Output    |

【Return Value】

| Return Value | Description |
| :----------: | :----------:|
|      0       |  Successful. |
|   Non-zero   |   Failed.   |

【Note】
> None

【Reference Code】
> See HB_VOT_EnableVideoLayer

### HB_VOT_SetChnFrameRate
【Function Declaration】
```C
int HB_VOT_SetChnFrameRate(uint8_t layer, uint8_t chn, int frame_rate);
```
【Function Description】
> Set the frame rate of the video channel.

【Parameter Description】

| Parameter Name | Description                                                     | Input/Output |
| :------------: | :-------------------------------------------------------------- | :----------: |
|     layer      | Video output layer ID. Value range: 0.                          |    Input     |
|      chn       | Video output channel ID. Value range: [0, 4).<br/>0, 1 represent video channels;<br/>2, 3 represent graphic channels. |    Input     |
|  frame_rate    | Displayed frame rate of the video channel.                      |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :----------:|
|      0       |  Successful. |
|   Non-zero   |   Failed.   |

【Note】
> None

【Reference Code】
> N/A

### HB_VOT_GetChnFrameRate
【Function Declaration】
```C
int HB_VOT_GetChnFrameRate(uint8_t layer, uint8_t chn, int *pframe_rate);
```
【Description】
> Get the display frame rate of the video channel.

【Parameter Description】

|   Parameter Name   |                             Description                              | Input/Output |
| :----------------: | :------------------------------------------------------------------: | :----------: |
|       layer        |            Video output layer ID. Value range: 0.                    |    Input     |
|        chn         | Video output channel ID. Value range: [0, 4). <br/>0, 1 represents the video channel; <br/>2, 3 represents the graphic channel. |    Input     |
|    pframe_rate     |             Display frame rate of the video channel.                 |    Output    |

【Return Value】

| Return Value | Description |
| :----------: | :---------: |
|      0       |   Success.  |
|    Non-zero  |   Failure.  |

【Notes】
> N/A

【Reference Code】
> N/A



### HB_VOT_ShowChn
**Function Declaration**
```C
int HB_VOT_ShowChn(uint8_t layer, uint8_t chn);
```
**Function Description**
> Displays the specified channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :---------- | :---------- |
|    layer      | Video output layer ID. Range: 0. | Input       |
|     chn       | Video output channel ID. Range: [0, 4).<br/>0, 1 for video channels;<br/>2, 3 for graphic channels. | Input       |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success.    |
| Non-zero   | Failure.    |

**Note**
> None.

**Reference Code**
> N/A

### HB_VOT_HideChn
**Function Declaration**
```C
int HB_VOT_HideChn(uint8_t layer, uint8_t chn);
```
**Function Description**
> Hides the specified channel.

**Parameter Descriptions**

| Parameter Name | Description                                                                           | Input/Output |
| :------------: | :----------------------------------------------------------------------------- | :---------- |
|    layer      | Video output video layer ID. Range: 0.                                                | Input       |
|     chn       | Video output channel ID. Range: [0, 4).<br/>0, 1 for video channels;<br/>2, 3 for graphic channels. | Input       |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success.    |
| Non-zero   | Failure.    |

**Note**
> None.

**Reference Code**
> N/A

### HB_VOT_SendFrame
**Function Declaration**
```C
int HB_VOT_SendFrame(uint8_t layer, uint8_t chn, void *pstVFrame, int millisec);
```
**Function Description**
> Sends the video image to the specified output channel for display.

**Parameter Descriptions**

| Parameter Name | Description                                                      | Input/Output |
| :------------: | :-------------------------------------------------------- | :---------- |
|    layer      | Video output video layer ID. Range: 0.                           | Input       |
|     chn       | Video output channel ID. Range: [0, 2).<br/>0, 1 for video channels. | Input       |
|  pstVFrame    | Pointer to video data information.                                  | Input       |
|    millisec   | Timeout time. Unit: ms                                          | Input       |

**Return Values**

| Return Value | Description |
| :---------: | :---------- |
|     0      | Success.    |
| Non-zero   | Failure.    |

**Note**
> None.

**Reference Code**
> N/A




### HB_VOT_ClearChnBuf
[Function Declaration]
```C
int HB_VOT_ClearChnBuf(uint8_t layer, uint8_t chn, HB_BOOL bClrAll);
```

[Function Description]
> Empty the buffer data of the specified output channel.

[Parameter Description]

| Parameter Name | Description                                                        | Input/Output |
| :------------: | :----------------------------------------------------------------- | :----------: |
|     layer      | Video output video layer ID. Range: 0.                             |    Input     |
|      chn       | Video output channel ID. Range: [0, 4).<br/>0 and 1 represent video channels;<br/>2 and 3 represent graphic channels. |    Input     |
|    bClrAll     | Whether to empty the data in the channel buffer.                   |    Input     |

[Return Value]

| Return Value | Description |
| :----------: | :----------: |
|      0       |   Success.  |
|    Non-0     |   Failure.  |

[Attention]
> None

[Reference code]
> None

### HB_VOT_BindVps
[Function Declaration]Please translate the Chinese parts in the following content into English, while preserving the original format and content:

```C
int HB_VOT_BindVps(uint8_t vpsGroup, uint8_t vpsChn, uint8_t layer, uint8_t chn);
```
[Function Description]
> Bind the input source of video output with the output of the VPS module.

[Parameter Description]

| Parameter |            Description             | Input/Output |
| :-------: | :-------------------------------: | :----------: |
| vpsGroup  | The group of the bound VPS module |    Input     |
|  vpsChn   |  The channel of the bound VPS module  |    Input     |
|   layer   |  The layer of the bound VOT module   |    Input     |
|    chn    | The channel of the bound VOT module |    Input     |

[Return Value]

| Return Value | Description |
|:------:|:-------:|
|    0   |  Success  |
|  Non-zero  |  Failure  |

[Note]
> None

[Reference code]
> None



### HB_VOT_EnableWB
**Function Declaration**
```C
int HB_VOT_EnableWB(VOT_WB votWb);
```
**Function Description**
> Enables write-back for the video output device.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|   votWb       | Write-back device ID.<br/>Range: 0. |   Input      |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero     | Failure     |

**Caution**
> None

**Reference Code**
```C
int sample_vot_wb_init(int wb_src, int wb_format)
{
    int ret = 0;
    VOT_WB_ATTR_S stWbAttr;
    stWbAttr.wb_src = wb_src;
    stWbAttr.wb_format = wb_format;
    HB_VOT_SetWBAttr(0, &stWbAttr);
    ret = HB_VOT_EnableWB(0);
    if (ret) {
        printf("HB_VOT_EnableWB failed.\n");
        return -1;
    }
    return 0;
}
```
### HB_VOT_DisableWB
**Function Declaration**
```C
int HB_VOT_DisableWB(VOT_WB votWb);
```
**Function Description**
> Disables write-back for the video output device.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|   votWb       | Write-back device ID.<br/>Range: 0. |   Input      |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero     | Failure     |

**Caution**
> None

**Reference Code**
> None

### HB_VOT_GetWBAttr
**Function Declaration**
```C
int HB_VOT_GetWBAttr (VOT_WB votWb, VOT_WB_ATTR_S *pstWBAttr);
```
**Function Description**
> Retrieves the write-back attributes of the video output device.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|   votWb       | Write-back device ID.<br/>Range: 0. |   Input      |
|  pstWBAttr    | Write-back attribute structure. |   Output    |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero     | Failure     |

**Caution**
> None

**Reference Code**
> None

### HB_VOT_SetWBAttr
**Function Declaration**
```C
int HB_VOT_SetWBAttr (VOT_WB votWb, VOT_WB_ATTR_S *pstWBAttr);
```
**Function Description**
> Sets the write-back attributes for the video output device.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|   votWb       | Write-back device ID.<br/>Range: 0. |   Input      |
|  pstWBAttr    | Write-back attribute structure. |   Input    |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero     | Failure     |

**Caution**
> None

**Reference Code**
> See the `HB_VOT_EnableWB` example for a usage reference.



### HB_VOT_GetWBFrame
【Function Declaration】
```C
int HB_VOT_GetWBFrame (VOT_WB votWb, void* pstVFrame, int millisec);
```
【Function Description】
> Enable the writeback of the video output device.

【Parameter Description】

| Parameter Name |                    Description                    | Input/Output |
| :------------: | :------------------------------------------------: | :----------: |
|     votWb      |            ID of the writeback device.<br/>Range: 0.            |    Input     |
|   pstVFrame    | The captured writeback image frame (the pointer type should be hb_vio_buffer_t *) |    Input     |
|    millisec    |              Timeout, not available in this version.               |    Input     |

【Return Value】

| Return Value | Description |
| :----------: | :--------: |
|       0      |   Success  |
|    Non-zero   |   Failure  |

【Notice】
> None

【Reference Code】
> None



### HB_VOT_ReleaseWBFrame
**Function Declaration**
```C
int HB_VOT_ReleaseWBFrame (VOT_WB votWb, void* pstVFrame)；
```
**Function Description**
> Enables the video output device for write-back.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------- | :---------- | :---------: |
|   votWb        | Write-back device ID. <br/>Range: 0. |   Input     |
| pstVFrame      | Acquired write-back image frame. |   Input     |

**Return Values**

| Return Value | Description |
|:------------|:------------|
|    0         | Success     |
| Non-zero    | Failure     |

**Caution**
> None

**Reference Code**
> N/A

### HB_VOT_ShutDownHDMI
**Function Declaration**
```C
int HB_VOT_ShutDownHDMI(void)；
```
**Function Description**
> Closes HDMI output to the target device, such as a monitor, causing it to display black but keeping the display hardware module functioning normally.

**Parameter Descriptions**

| Parameter Name | Description  | Input/Output |
| :------------- | :----------- | :---------: |
|   void         | Empty        |   Input     |

**Return Values**

| Return Value | Description |
|:------------|:------------|
|    0         | Success     |
| Non-zero    | Failure     |

**Caution**
> Limited to HDMI display usage

**Reference Code**

### HB_VOT_StartHDMI
**Function Declaration**
```C
int HB_VOT_StartHDMI (void)；
```
**Function Description**
> Enables HDMI output of images to the target display device, to be used in conjunction with HB_VOT_ShutDownHDMI.

**Parameter Descriptions**

| Parameter Name | Description  | Input/Output |
| :------------- | :----------- | :---------: |
|   void         | Empty        |   Input     |

**Return Values**

| Return Value | Description |
|:------------|:------------|
|    0         | Success     |
| Non-zero    | Failure     |

**Caution**
> For restarting image output after HB_VOT_ShutDownHDMI

**Reference Code**
> N/A




### Example of API calling process
```C
int sample_vot_init()
{
    int ret = 0;
    VOT_PUB_ATTR_S devAttr;
    VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
    VOT_CHN_ATTR_S stChnAttr;
    VOT_CROP_INFO_S cropAttrs;
    devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
    devAttr.u32BgColor = 0x8080;
    devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;

    ret = HB_VOT_SetPubAttr(0, &devAttr);
    if (ret) {
        printf("HB_VOT_SetPubAttr failed\n");
        goto err;
    }
    ret = HB_VOT_Enable(0);
    if (ret) {
        printf("HB_VOT_Enable failed.\n");
        goto err;
    }
    ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
        printf("HB_VOT_GetVideoLayerAttr failed.\n");
        goto err;
    }
    stLayerAttr.stImageSize.u32Width  = 1920;
    stLayerAttr.stImageSize.u32Height = 1080;
    stLayerAttr.panel_type = 0;
    stLayerAttr.rotate = 0;
    stLayerAttr.dithering_flag = 0;
    stLayerAttr.dithering_en = 0;
    stLayerAttr.gamma_en = 0;
    stLayerAttr.hue_en = 0;
    stLayerAttr.sat_en = 0;
    stLayerAttr.con_en = 0;
    stLayerAttr.bright_en = 0;
    stLayerAttr.theta_sign = 0;
    stLayerAttr.contrast = 0;
    stLayerAttr.theta_abs = 0;
    stLayerAttr.saturation = 0;
    stLayerAttr.off_contrast = 0;
    stLayerAttr.off_bright = 0;
    stLayerAttr.user_control_disp = 0;
    stLayerAttr.user_control_disp_layer1 = 0;
    stLayerAttr.big_endian = 0;
    ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
    if (ret) {
        printf("HB_VOT_SetVideoLayerAttr failed.\n");
        goto err;
    }
    ret = HB_VOT_EnableVideoLayer(0);
    if (ret) {
        printf("HB_VOT_EnableVideoLayer failed.\n");
        HB_VOT_Disable(0);
        goto err;
    }
    stChnAttr.u32Priority = 2;
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32SrcWidth = 1920;
    stChnAttr.u32SrcHeight = 1080;
    stChnAttr.u32DstWidth = 1920;
    stChnAttr.u32DstHeight = 1080;
    ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
    if (ret) {
        printf("HB_VOT_SetChnAttr 0: %d\n", ret);
        HB_VOT_DisableVideoLayer(0);
        HB_VOT_Disable(0);
        goto err;
    }

    cropAttrs.u32Width = stChnAttr.u32DstWidth;  // - stChnAttr.s32X;
    cropAttrs.u32Height = stChnAttr.u32DstHeight;  //- stChnAttr.s32Y;
    ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);
    printf("HB_VOT_SetChnCrop: %d\n", ret);
    ret = HB_VOT_EnableChn(0, 0);
    if (ret) {
        printf("HB_VOT_EnableChn: %d\n", ret);
        HB_VOT_DisableVideoLayer(0);
        HB_VOT_Disable(0);
        goto err;
    }
    if (g_use_ipu) {
        ret = HB_VOT_BindVps(0, 3, 0, 0);  // 37 gdc0
    } else {
        ret = HB_VOT_BindVps(0, 11, 0, 0);  // 37 gdc0
    }

    if (ret) {
        printf("HB_VOT_BindVps: %d\n", ret);
        HB_VOT_DisableChn(0, 1);
        HB_VOT_DisableVideoLayer(0);
        HB_VOT_Disable(0);
    }
    err:
    return ret;
}

int sample_vot_deinit()
{
    int ret = 0;
    ret = HB_VOT_DisableChn(0, 0);
    if (ret) {
        printf("HB_VOT_DisableChn failed.\n");
    }
    ret = HB_VOT_DisableVideoLayer(0);
    if (ret) {
        printf("HB_VOT_DisableVideoLayer failed.\n");
    }

    ret = HB_VOT_Disable(0);
    if (ret) {
        printf("HB_VOT_Disable failed.\n");
    }
    return 0;
}
```



## Data Structures

### HB_VOT_PUB_ATTR_S
**Structure Definition**
```C
typedef struct HB_VOT_PUB_ATTR_S {
    uint32_t u32BgColor; /* Device background color in RGB format */
    VOT_OUTPUT_MODE_E enOutputMode; /* Video Output Interface (VOT) type */
    VOT_INTF_SYNC_E enIntfSync; /* Type of synchronization for the VOT interface */
    VOT_SYNC_INFO_S stSyncInfo; /* Structure containing synchronization information for the VOT interface */
} VOT_PUB_ATTR_S;
```
**Function Description**
> Defines common attributes for video output.

**Member Descriptions**

|Member| Meaning|
|---|---|
|u32BgColor| Device background color|
|enOutputMode| Type of Vo interface|
|enIntfSync| Default synchronization configuration for the interface|
|stSyncInfo| Structure holding interface synchronization details|

This structure is effective when enabling user-defined timing.



### HB_VOT_OUTPUT_MODE_E
【Structure Definition】
```C
typedef enum HB_VOT_OUTPUT_MODE_E {
    HB_VOT_OUTPUT_MIPI,
    HB_VOT_OUTPUT_BT1120,
    HB_VOT_OUTPUT_RGB888,
    HB_VOT_OUTPUT_BT656,
    HB_VOT_OUTPUT_MODE_BUTT,
} VOT_OUTPUT_MODE_E;
```
【Function Description】
> Define Video Output Mode

【Member Description】

|          Member           | Meaning                     |
| :---------------------: | :----------------------- |
|   HB_VOT_OUTPUT_MIPI    | MIPI Output                 |
|  HB_VOT_OUTPUT_BT1120   | BT1120 Output(for HDMI Display) |
|  HB_VOT_OUTPUT_RGB888   | RGB Output                  |
| HB_VOT_OUTPUT_MODE_BUTT | Number of Supported Output Modes       |

### HB_VOT_INTF_SYNC_E
【Structure Definition】
```C
typedef enum HB_VOT_INTF_SYNC_E {
    VOT_OUTPUT_1920x1080,
    VOT_OUTPUT_800x480,
    VOT_OUTPUT_720x1280,
    VOT_OUTPUT_1080x1920,
    VOT_OUTPUT_704x576，
    VOT_OUTPUT_1080P60，
    VOT_OUTPUT_1080P50，
    VOT_OUTPUT_1080P30，
    VOT_OUTPUT_1080P25，
    VOT_OUTPUT_1080P59_94，
    VOT_OUTPUT_1080P29_97，
    VOT_OUTPUT_1080I60，
    VOT_OUTPUT_1080I50，
    VOT_OUTPUT_1080I59_94，
VOT_OUTPUT_720P60,
    VOT_OUTPUT_720P50,
    VOT_OUTPUT_720P30,
    VOT_OUTPUT_720P25,
    VOT_OUTPUT_720P59_94,
    VOT_OUTPUT_720P29_97,
    VOT_OUTPUT_704x576_25,
    VOT_OUTPUT_704x480_30,
    VO_OUTPUT_USER, /* User timing. */
    VO_OUTPUT_BUTT
} VOT_INTF_SYNC_E;
```

【Function Description】
> Defines the typical video output timing modes.

【Member Description】

| Member                 | Description           |
| :--------------------  | :------------------    |
| VOT_OUTPUT_1920x1080   | 1920x1080             |
| VOT_OUTPUT_800x480     | 800x480               |
| VOT_OUTPUT_720x1280    | 720x1280              |
| VOT_OUTPUT_1080x1920   | 1080x1920             |
| VOT_OUTPUT_704x576     | 704x576               |
| VOT_OUTPUT_1080P60     | 1920x1080P@60fps      |
| VOT_OUTPUT_1080P50     | 1920x1080P@50fps      |
| VOT_OUTPUT_1080P30     | 1920x1080P@30fps      |
| VOT_OUTPUT_1080P25     | 1920x1080P@25fps      |
| VOT_OUTPUT_1080P59_94  | 1920x1080P@59.94fps   |
| VOT_OUTPUT_1080P29_97  | 1920x1080P@29.97fps   |
| VOT_OUTPUT_1080I60     | 1920x1080I@60fps      |
| VOT_OUTPUT_1080I50     | 1920x1080I@50fps      |
| VOT_OUTPUT_1080I59_94  | 1920x1080I@59.94fps   |
| VOT_OUTPUT_720P60      | 1280x720P@60fps       |
| VOT_OUTPUT_720P50      | 1280x720P@50fps       |
| VOT_OUTPUT_720P30      | 1280x720P@30fps       |
| VOT_OUTPUT_720P25      | 1280x720P@25fps       |
| VOT_OUTPUT_720P59_94   | 1280x720P@59.94fps    |
| VOT_OUTPUT_720P29_97   | 1280x720P@29.97fps    |
| VOT_OUTPUT_704x576_25  | 704x576P@25fps        |
| VOT_OUTPUT_704x480_30  | 704x480P@30fps        |
| VO_OUTPUT_USER         | User-defined timing    |

### HB_VOT_SYNC_INFO_S
【Structure Definition】
```C
typedef struct HB_VOT_SYNC_INFO_S {
    uint32_t hbp;
    uint32_t hfp;
    Tuint32_t hs;
    uint32_t vbp;
    uint32_t vfp;
    uint32_t vs;
    uint32_t vfp_cnt;
    uint32_t width;
    uint32_t height;
} VOT_SYNC_INFO_S;
```    
【Function Description】
> Defines the user-defined video output timing.

【Member Description】

| Member | Meaning                             |
| :-----: | :------------------------------- |
|   hbp   | Horizontal blanking porch                           |
|   hfp   | Horizontal front porch                           |
|   hs    | Horizontal sync signal                       |
|   vbp   | Vertical blanking porch                           |
|   vfp   | Vertical front porch                           |
|   vs    | Vertical sync signal                       |
| vfp_cnt | Currently fixed at 0xa (bt656 field is 0) |
|  width  | Screen resolution width                     |
| height  | Screen resolution height                     |

### HB_VOT_VIDEO_LAYER_ATTR_S
【Structure Definition】
```C
typedef struct HB_VOT_VIDEO_LAYER_ATTR_S {
    SIZE_S stImageSize;/* Video layer canvas size */
    uint32_t big_endian;
    uint32_t display_addr_type;
    uint32_t display_cam_no;
    uint32_t display_addr_type_layer1;
    uint32_t display_cam_no_layer1;
    int32_t dithering_flag;
    uint32_t dithering_en;
    uint32_t gamma_en;
    uint32_t hue_en;
    uint32_t sat_en;
    uint32_t con_en;
    uint32_t bright_en;
    uint32_t theta_sign;
    uint32_t contrast;
    uint32_t gamma;
    uint32_t theta_abs;
    uint32_t saturation;
    uint32_t off_contrast;
    uint32_t off_bright;
    uint32_t panel_type;
```uint32_t rotate;
    uint32_t user_control_disp;
    uint32_t user_control_disp_layer1;
} VOT_VIDEO_LAYER_ATTR_S;
```
【Functional Description】
> Defines the video layer attributes

【Member Description】

 | Member                   | Meaning                                  |
 | :----------------------- | :--------------------------------------- |
 | stImageSize              | Canvas size of the video layer            |
 | big_endian               | Endian configuration for input images of channel 2 and 3       |
 | display_addr_type        | Display type for channel 0                            |
 | display_cam_no           | Display source for channel 0                              |
 | display_addr_type_layer1 | Display type for channel 1                            |
 | display_cam_no_layer1    | Display source for channel 1                              |
 | dithering_flag           | Dithering type                            |
 | dithering_en             | Whether dithering is enabled                        |
 | gamma_en                 | Whether gamma is enabled                            |
 | hue_en                   | Whether hue is enabled                              |
 | sat_en                   | Whether saturation is enabled                              |
 | con_en                   | Whether contrast is enabled                              |
 | bright_en                | Whether brightness is enabled                           |
 | theta_sign               | Hue angle                                |
 | contrast                 | Contrast value, controls contrast together with off_contrast |
 | gamma                    | Gamma value                                  |
 | theta_abs                | Absolute value of hue angle, range: 0-8d180             |
 | saturation               | Saturation value                                    |
 | off_contrast             | Contrast offset 0 - 255                  |
 | off_bright               | Brightness offset -128-127                   |
 | panel_type               | Output type                                 |
 | rotate                   | Whether rotation is enabled                             |
 | user_control_disp        | Whether user input control is enabled for channel 0                |
 | user_control_disp_layer1 | Whether user input control is enabled for channel 1                |

### HB_VOT_CSC_S

【Structure Definition】
```C
typedef struct HB_VOT_CSC_S {
    uint32_t u32Luma;
    uint32_t u32Contrast;
    uint32_t u32Hue;
    uint32_t u32Satuature;
} VOT_CSC_S;
```
【Functional Description】
> Defines the structure for image output effect.【Member Description】

| Member       | Meaning        |
| :----------- | :----------- |
| u32Luma      | Set VO Brightness   |
| u32Contrast  | Set VO Contrast |
| u32Hue       | Set VO Hue   |
| u32Satuature | Set VO Saturation |

### HB_VOT_UPSCALE_ATTR_S
【Structure Definition】
```C
typedef struct HB_VOT_UPSCALE_ATTR_S {
    uint32_t src_width;
    uint32_t src_height;
    uint32_t tgt_width;
    uint32_t tgt_height;
    uint32_t pos_x;
    uint32_t pos_y;
    uinit32_t upscale_en;
} VOT_UPSCALE_ATTR_S;
```
【Function Description】
> Define the upscale attribute of the video layer

【Member Description】

| Member       | Meaning                 |
| :--------- | :------------------- |
| src_width  | Width of the original image for upscaling         |
| src_height | Height of the original image for upscaling        |
| tgt_width  | Width of the target image after upscaling |
| tgt_height | Height of the target image after upscaling |
| pos_x      | X-coordinate of the target image after upscaling        |
| pos_y      | Y-coordinate of the target image after upscaling        |
| upscale_en | Whether the upscaling function is enabled     |

### HB_VOT_CHN_ATTR_S
【Structure Definition】
```C
typedef struct HB_VOT_CHN_ATTR_S {
    uint32_t u32Priority;
    uint32_t u32SrcWidth;
    uint32_t u32SrcHeight;
    int32_t s32X;
    int32_t s32Y;
    uint32_t u32DstWidth;
    uint32_t u32DstHeight;
} VOT_CHN_ATTR_S;
```
【Function Description】
> Define the attributes of video output channel

【Member Description】

|     Member    | Meaning                                                     |
| :----------: | :----------------------------------------------------------- |
|  u32Priority  | Overlay priority of the video channel, with 0 being the highest priority and 3 being the lowest priority. Note that no two channels can have the same priority. If there is a duplicate, none of the channels will be displayed. |
|  u32SrcWidth  | Width of the original video channel                           |
| u32SrcHeight  | Height of the original video channel                          |
|     s32X     | X-coordinate of the video channel                             |
|     s32Y     | Y-coordinate of the video channel                             |
| u32DstWidth  | Width of the target image of the video channel                 |
| u32DstHeight | Height of the target image of the video channel                |

### HB_VOT_CHN_ATTR_EX_S
【Structure Definition】
```C
typedef struct HB_VOT_CHN_ATTR_EX_S {
    uint32_t format;
    uint32_t alpha;
    uint32_t keycolor;
    uint32_t alpha_sel;
    uint32_t ov_mode;
    uint32_t alpha_en;
} VOT_CHN_ATTR_EX_S;
```

【Function Description】
> Define the advanced attributes of the video output channel

【Member Description】

|   Member   | Meaning|
| :-------: | :--------- |
|   format   | Input video YUV format         |
|   alpha    | Alpha value                    |
| keycolor   | Channel key-color              |
| alpha_sel  | Alpha overlay algorithm selection<br/>0: Result image = (layer A * (~BR + 1) + layer B*BR) >> 8<br/>1: Result image = (layer A * (~BR + 1) + layer B*BR) >> 8<br/>Layer A is merged result output by the pipeline backwards.<br/>Layer B is the higher priority layer of current pipeline.<br/>BR is Layer B Alpha ratio(0~255) and may be from image pixel data(ARGB/RGBA) or programmable register. |
|   ov_mode  | Overlay mode<br/>00: transparent<br/>01: and<br/>10: or<br/>11: inv |
|  alpha_en  | Alpha overlay enable or not  |

### VOT_WB_ATTR_S
【Structure Definition】
```C
typedef struct HB_VOT_WBC_ATTR_S{
    int wb_src;
    int wb_format;
}VOT_WB_ATTR_S;
```
【Function Description】
> Define the write-back attributes

【Member Description】

|   Member   | Meaning       |
| :--------: | :------------------------- |
|  wb_src    | Write-back source<br/>0 overlay-alphablend output<br/>1 upscaling output<br/>2 de-output output    |
| wb_format  | Write-back format<br/>0 FORMAT_YUV422_UYVY<br/>1 FORMAT_YUV422_VYUY<br/>2 FORMAT_YUV422_YVYU<br/>3 FORMAT_YUV422_YUYV<br/>4 FORMAT_YUV420SP_UV<br/>5 FORMAT_YUV420SP_VU<br/>6 FORMAT_BGR0<br/>When wb_src is 2, and vot output is in RGB mode, only FORMAT_BGR0 is supported.  |

### VOT_CROP_INFO_S
【Structure Definition】
```C
typedef struct HB_VOT_CROP_INFO_S{
    uint32_t u32Width;
    uint32_t u32Height;
    }VOT_CROP_INFO_S;
```
【Function Description】
> Define crop properties

【Member Description】

|   Member   | Meaning               |
| :--------: | :--------------------- |
| u32Width   | Target width of the cropped image |
| u32Height  | Target height of the cropped image |

### VOT_FRAME_INFO_S
【Structure Definition】
```C
typedef struct HB_VOT_FRAME_INFO_S{   void *addr;
void *addr_uv;   unsigned int size; }VOT_FRAME_INFO_S;
```
【Function Description】
> Define image information

【Member Description】

|   Member   | Meaning                        |
| :--------: | :----------------------------- |
|   addr     | Start virtual address of the Y component of the image  |
|  addr_uv   | Start virtual address of the UV component of the image |
|   size     | Image size                      |

### HB_POINT_S
【Structure Definition】
```C
typedef struct HB_POINT_S {
    int s32X;
    int s32Y;
} POINT_S;
```
【Function Description】
> Definition of pixel point.

【Member Description】

| Member | Description       |
| :---:  | :---------------- |
| s32X   | x-coordinate of the pixel point |
| s32Y   | y-coordinate of the pixel point |

### VOT_WB
【Structure Definition】
```C
typedef uint32_t VOT_WB;
```
【Function Description】
> Definition of VOT write-back variable.

### HB_PIXEL_FORMAT_YUV_E
【Structure Definition】
```C
typedef enum HB_PIXEL_FORMAT_YUV_E {
    PIXEL_FORMAT_YUV422_UYVY = 0,
    PIXEL_FORMAT_YUV422_VYUY = 1,
    PIXEL_FORMAT_YUV422_YVYU = 2,
    PIXEL_FORMAT_YUV422_YUYV = 3,
    PIXEL_FORMAT_YUV422SP_UV = 4,
    PIXEL_FORMAT_YUV422SP_VU = 5,
    PIXEL_FORMAT_YUV420SP_UV = 6,
    PIXEL_FORMAT_YUV420SP_VU =
    PIXEL_FORMAT_YUV422P_UV = 8,
    PIXEL_FORMAT_YUV422P_VU = 9,
    PIXEL_FORMAT_YUV420P_UV = 10,
    PIXEL_FORMAT_YUV420P_VU = 11,
    PIXEL_FORMAT_YUV_BUTT = 12
} PIXEL_FORMAT_YUV_E;
```

### HB_PIXEL_FORMAT_RGB_E
【Structure Definition】
```C
typedef enum HB_PIXEL_FORMAT_RGB_E {
    PIXEL_FORMAT_8BPP = 0,
    PIXEL_FORMAT_RGB565 = 1,
    PIXEL_FORMAT_RGB888 = 2,
    PIXEL_FORMAT_RGB888P = 3,
    PIXEL_FORMAT_ARGB8888 = 4,
    PIXEL_FORMAT_RGBA8888 = 5,
    PIXEL_FORMAT_RGB_BUTT = 6
} PIXEL_FORMAT_RGB_E;
```

### HB_SIZE_S
【Structure Definition】
```C
typedef struct HB_SIZE_S {
    uint32_t u32Width;
    uint32_t u32Height;
} SIZE_S;

```
【Function Description】
> Defines the size of an image.

【Member Description】

|  Member   | Meaning |
| :-------: | :----- |
| u32Width  | Width of the image |
| u32Height | Height of the image |

## Error Codes
The error codes for VOT are as follows.

| Error Code | Macro                           | Description            |
| :--------: | :------------------------------ | :--------------------- |
|  0xa401   | HB_ERR_VOT_BUSY             | Resource is busy     |
|  0xa402   | HB_ERR_VOT_NO_MEM         | Out of memory       |
|  0xa403   | HB_ERR_VOT_NULL_PTR       | Null pointer in function parameters  |
|  0xa404   | HB_ERR_VOT_SYS_NOTREADY | System is not ready    |
|  0xa405   | HB_ERR_VOT_INVALID_DEVID | Device ID is out of range   |
|  0xa406   | HB_ERR_VOT_INVALID_CHNID | Channel ID is out of range  |
|  0xa407   | HB_ERR_VOT_ILLEGAL_PARAM | Parameters are out of range  |
|  0xa408   | HB_ERR_VOT_NOT_SUPPORT   | Operation is not supported   |
|  0xa409   | HB_ERR_VOT_NOT_PERMIT    | Operation is not allowed     |
|  0xa40a   | HB_ERR_VOT_INVALID_WBCID | WBC number is out of range   |
|  0xa40b   | HB_ERR_VOT_INVALID_LAYERID | Video layer number is out of range  |
|  0xa40c   | HB_ERR_VOT_INVALID_VIDEO_CHNID | Video layer channel number is out of range |
|  0xa40d   | HB_ERR_VOT_INVALID_BIND_VPSGROUPID | Binding VPS GROUP number is out of range |
|  0xa40e   | HB_ERR_VOT_INVALID_BIND_VPSCHNID | Binding VPS CHN number is out of range |
|  0xa40f   | HB_ERR_VOT_INVALID_FRAME_RATE | Unsupported frame rate  |
|  0xa410   | HB_ERR_VOT_DEV_NOT_CONFIG | Device is not configured |
|  0xa411   | HB_ERR_VOT_DEV_NOT_ENABLE | Device is not enabled   |
|  0xa412   | HB_ERR_VOT_DEV_HAS_ENABLED | Device is already enabled |
|  0xa413   | HB_ERR_VOT_DEV_HAS_BINDED  | Device is already bound |
|  0xa414   | HB_ERR_VOT_DEV_NOT_BINDED  | Device is not bound     || 0xa415 | HB_ERR_VOT_LAYER_NOT_ENABLE              | Video output layer is not enabled          |
| 0xa420 | HB_ERR_VOT_VIDEO_NOT_ENABLE              | Video layer is not enabled                |
| 0xa421 | HB_ERR_VOT_VIDEO_NOT_DISABLE             | Video layer is not disabled                |
| 0xa422 | HB_ERR_VOT_VIDEO_NOT_CONFIG              | Video layer is not configured                |
| 0xa423 | HB_ERR_VOT_VIDEO_HAS_BINDED              | Video layer is already binded                |
| 0xa424 | HB_ERR_VOT_VIDEO_NOT_BINDED              | Video layer is not binded                |
| 0xa430 | HB_ERR_VOT_WBC_NOT_DISABLE               | Write back device is not disabled              |
| 0xa431 | HB_ERR_VOT_WBC_NOT_CONFIG                | Write back device is not configured                |
| 0xa432 | HB_ERR_VOT_WBC_HAS_CONFIG                | Write back device is already configured                |
| 0xa433 | HB_ERR_VOT_WBC_NOT_BIND                  | Write back device is not binded                |
| 0xa434 | HB_ERR_VOT_WBC_HAS_BIND                  | Write back device is already binded                |
| 0xa435 | HB_ERR_VOT_INVALID_WBID                  | Invalid write back device number          |
| 0xa436 | HB_ERR_VOT_WB_NOT_ENABLE                 | Write back device is not enabled              |
| 0xa437 | HB_ERR_VOT_WB_GET_TIMEOUT                | Timeout when getting one frame of write back image        |
| 0xa440 | HB_ERR_VOT_CHN_NOT_DISABLE               | Channel is not disabled                  |
| 0xa441 | HB_ERR_VOT_CHN_NOT_ENABLE                | Channel is not enabled                  |
| 0xa442 | HB_ERR_VOT_CHN_NOT_CONFIG                | Channel is not configured                  |
| 0xa443 | HB_ERR_VOT_CHN_NOT_ALLOC                 | Channel resource is not allocated              |
| 0xa444 | HB_ERR_VOT_CHN_AREA_OVERLAP              | VO channel area overlap              |
| 0xa450 | HB_ERR_VOT_INVALID_PATTERN               | Invalid style                    |
| 0xa451 | HB_ERR_VOT_INVALID_POSITION              | Invalid cascade position                |
| 0xa460 | HB_ERR_VOT_WAIT_TIMEOUT                  | Wait timeout                    |
| 0xa461 | HB_ERR_VOT_INVALID_VFRAME                | Invalid video frame                  |
| 0xa462 | HB_ERR_VOT_INVALID_RECT_PARA             | Invalid rectangle parameters                |
| 0xa463 | HB_ERR_VOT_SETBEGIN_ALREADY              | BEGIN is already set                 |
| 0xa464 | HB_ERR_VOT_SETBEGIN_NOTYET               | BEGIN is not set yet                 |
| 0xa465 | HB_ERR_VOT_SETEND_ALREADY                | END is already set                   |
| 0xa466 | HB_ERR_VOT_SETEND_NOTYET                 | END is not set yet                   |
| 0xa470 | HB_ERR_VOT_GFX_NOT_DISABLE               | Graphics layer is not closed                |
| 0xa471 | HB_ERR_VOT_GFX_NOT_BIND                  | Graphics layer is not binded                |
| 0xa472 | HB_ERR_VOT_GFX_NOT_UNBIND                | Graphics layer is not unbinded              |
| 0xa473 | HB_ERR_VOT_GFX_INVALID_ID                | Graphics layer ID is out of range              |
| 0xa480 | HB_ERR_VOT_BUF_MANAGER_ILLEGAL_OPERATION | Illegal operation of Buffer manager           |

## Reference code
For examples of VO, you can refer to [sample_vot](./multimedia_samples#sample_vot) and [sample_lcd](./multimedia_samples#sample_lcd).