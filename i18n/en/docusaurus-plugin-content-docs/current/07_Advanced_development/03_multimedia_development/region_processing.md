---
sidebar_position: 7
---
# 7.3.7 Region Processing
## Overview
Users generally need to overlay OSD (On-Screen Display) on videos to display specific information (such as channel number, timestamp, etc.), and may also fill color blocks when necessary. These OSDs overlaid on the videos and color blocks obscured on the videos are collectively referred to as regions. The REGION module is used to manage these region resources uniformly.

Region management can create regions, overlay them on videos, or obscure videos. For example, in practical applications, users create a region through HB_RGN_AttachToChn and overlay this region on a specific channel (such as the US channel). When the channel is scheduled, the OSD is overlaid on the video. A region can be specified to multiple channels (such as the US channel and the DS channel) by calling the interface to set channel display attributes, and the display attributes (such as position, display status, etc.) for each channel can be different.

## Function Description
### Basic Concepts
Region Types:
- overlay: video overlay region that can draw texts, lines, etc.
- cover: video obscuring region that obscures with a solid color block.

Bitmap Filling:
- The region bitmap is overlaid onto the region memory. When using the HB_RGN_SetBitMap method, if the size of the bitmap is larger than that of the region set, the overflow part beyond the region range will be cropped off.
When using the HB_RGN_GetCanvasInfo/HB_RGN_UpdateCanvas method, write according to the obtained canvas size.

Region Attributes:
- When creating a region, some basic information of the region needs to be set, such as size, region type, etc.

Channel Display Attributes:
- When overlaying a region on a channel, specific display attributes of the channel need to be set, such as overlay position, display status, etc. If the bShow is set to false, the region will be overlaid on the channel but not displayed.

Drawing Text:
- Use HB_RGN_DrawWord to draw texts, supporting four font sizes and 15 font colors.

Drawing Lines:
- Use HB_RGN_DrawLine/HB_RGN_DrawLineArray to draw lines or multiple lines simultaneously, supporting adjustment of line thickness and line colors.

Region Inversion:
- There is a reverse color switch in the channel display attributes. If the reverse color is enabled, the color of the region will be inverted during overlay.

### Usage Example
The usage process should be as follows:
- Users create a region and set the region attributes.
- Users bind the region to a channel.
- Use HB_RGN_GetAttr/HB_RGN_SetAttr to get or modify the region attributes.
- Using the HB_RGN_SetBitMap method:
  - Use HB_RGN_DrawWord or HB_RGN_DrawLine/HB_RGN_DrawLineArray to draw texts or lines to the bitmap created by the user, and then call HB_RGN_SetBitMap to set the bitmap to the region.
  - Using the HB_RGN_GetCanvasInfo/HB_RGN_UpdateCanvas method:
  - Use HB_RGN_GetCanvasInfo to get the address, use HB_RGN_DrawWord or HB_RGN_DrawLine/HB_RGN_DrawLineArray to draw texts or lines to the obtained address, and then use HB_RGN_UpdateCanvas to update the canvas.
- Use HB_RGN_SetDisplayAttr/HB_RGN_GetDisplayAttr to get or set the channel display attributes.
- Finally, the user removes the region from the channel and destroys the region.

## API Reference
```c
int32_t HB_RGN_Create(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
int32_t HB_RGN_Destory(RGN_HANDLE Handle);
int32_t HB_RGN_GetAttr(RGN_HANDLE Handle, RGN_ATTR_S *pstRegion);
int32_t HB_RGN_SetAttr(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
int32_t HB_RGN_SetBitMap(RGN_HANDLE Handle, const RGN_BITMAP_S *pstBitmapAttr);
int32_t HB_RGN_AttachToChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_DetachFromChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn);
int32_t HB_RGN_SetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_GetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_GetCanvasInfo(RGN_HANDLE Handle, RGN_CANVAS_S *pstCanvasInfo);
int32_t HB_RGN_UpdateCanvas(RGN_HANDLE Handle);
int32_t HB_RGN_DrawWord(RGN_HANDLE Handle, const RGN_DRAW_WORD_S *pstRgnDrawWord);
int32_t HB_RGN_DrawLine(RGN_HANDLE Handle, const RGN_DRAW_LINE_S *pstRgnDrawLine);
int32_t HB_RGN_DrawLineArray(RGN_HANDLE Handle,const RGN_DRAW_LINE_S astRgnDrawLine[],uint32_t u32ArraySize);
int32_t HB_RGN_BatchBegin(RGN_HANDLEGROUP *pu32Group, uint32_t u32Num, const RGN_HANDLE handle[]);
int32_t HB_RGN_BatchEnd(RGN_HANDLEGROUP u32Group);
int32_t HB_RGN_SetColorMap(const RGN_CHN_S *pstChn, uint32_t color_map[15]);
int32_t HB_RGN_SetSta(const RGN_CHN_S *pstChn, uint8_t astStaLevel[3], RGN_STA_S astStaAttr[8]);
int32_t HB_RGN_GetSta(const RGN_CHN_S *pstChn, uint16_t astStaValue[8][4]);
int32_t HB_RGN_AddToYUV(RGN_HANDLE Handle, hb_vio_buffer_t *vio_buffer, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_SetDisplayLevel(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, uint32_t osd_level);
```


### HB_RGN_Create/HB_RGN_Destroy
**Function Declaration**
```c
int32_t HB_RGN_Create(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
int32_t HB_RGN_Destroy(RGN_HANDLE Handle);
```
**Function Description**
> Create or destroy a region.

**Parameter Descriptions**

|   Member    |                   Meaning                    |
| :---------: | :----------------------------------------: |
|  Handle     | Region handle number. Range: [0, RGN_HANDLE_MAX). |
| pstRegion   | Pointer to region attributes.                |

**Return Values**

| Return Value |                  Description                 |
| :---------: | :------------------------------------------: |
|      0      | Successfully created or destroyed.            |
| Non-zero    | Failed, refer to error codes.                |

**Notes for HB_RGN_Create:**
1. The handle is specified by the user and acts as an ID, with handle numbers within the specified range;
2. Duplicate creation is not supported;
3. Region attributes must not be empty and must be valid;
4. For Cover type regions, specify only the region type; attributes are set during HB_RGN_AttachToChn call;
5. Creation checks for maximum and minimum dimensions; supported pixel formats are detailed in RGN_PIXEL_FORMAT_E;

**Notes for HB_RGN_Destroy:**
1. The region must have been created;
2. Detach from channel using HB_RGN_DetachFromChn before calling this interface;
3. During the call, no other interfaces like HB_RGN_SetAttr or HB_RGN_SetBitMap should be used simultaneously.

**Example Code**
```c
    RGN_HANDLE handle = 0;
    RGN_ATTR_S stRegion;
    int ret;
    stRegion.enType = OVERLAY_RGN;
    stRegion.stOverlayAttr.stSize.u32Width = 640;
    stRegion.stOverlayAttr.stSize.u32Height = 128;
    stRegion.stOverlayAttr.enBgColor = FONT_KEY_COLOR;
    stRegion.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;

    ret = HB_RGN_Create(handle, &stRegion);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_GetAttr(handle, &stRegion);
    if (ret < 0) {
        return ret;
    }

    stRegion.stOverlayAttr.enBgColor = FONT_COLOR_WHITE;

    ret = HB_RGN_SetAttr(handle, &stRegion);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_Destroy(handle);
    if (ret < 0) {
        return ret;
    }
```



### HB_RGN_GetAttr/HB_RGN_SetAttr
[Function Declaration]
```c
int32_t HB_RGN_GetAttr(RGN_HANDLE Handle, RGN_ATTR_S *pstRegion);
int32_t HB_RGN_SetAttr(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
```
[Function Description]
> Get or set the region attributes;

[Parameter Description]| Parameter Name | Description                                                                      |
| :------------: | :------------------------------------------------------------------------------- |
|     Handle     | Region handle number. Value range: [0, RGN_HANDLE_MAX).                         |
|   pstRegion    | Pointer to region attribute.                                                    |

【Return Value】

| Return Value | Description                          |
| :----------: | :----------------------------------- |
|      0       | Success                              |
|    Non-zero  | Failure, see error code for details   |

【Notes】
HB_RGN_GetAttr:
1. The region must be created;
2. The pointer to the region attribute cannot be empty;
3. The region type must be Overlay, the Cover attribute is specified in HB_RGN_AttachToChn, and modified in HB_RGN_SetDisplayAttr;

HB_RGN_SetAttr:
1. The region must be created;
2. The pointer to the region attribute cannot be empty;
3. The region type must be Overlay, the Cover attribute is specified in HB_RGN_AttachToChn, and modified in HB_RGN_SetDisplayAttr;
4. The region size cannot be modified after calling HB_RGN_AttachToChn;

【Reference Code】
> Please refer to the examples of HB_RGN_Create/HB_RGN_Destory

### HB_RGN_SetBitMap
【Function Declaration】
```c
int32_t HB_RGN_SetBitMap(RGN_HANDLE Handle, const RGN_BITMAP_S *pstBitmapAttr);
```
【Function description】
> Set the bitmap to fill a region;

【Parameter description】

| Parameter Name | Description                                                                      |
| :------------: | :------------------------------------------------------------------------------- |
|     Handle     | Region handle number. Value range: [0, RGN_HANDLE_MAX).                         |
|   pstBitmap    | Pointer to bitmap attribute.                                                    |

【Return Value】

| Return Value | Description                          |
| :----------: | :----------------------------------- |
|      0       | Success                              |
|    Non-zero  | Failure, see error code for details   |【Notice】
1. The region must have been created;
2. The size of the bitmap is not consistent with the size of the region;
3. The bitmap is loaded from the (0, 0) of the region and automatically cropped when the bitmap is larger than the region;
4. The pixel format must be consistent with the pixel format of the region;
5. The bitmap attribute pointer cannot be empty;
6. Supports multiple invocations;
7. This interface is only valid for Overlay type regions;
8. After calling HB_RGN_GetCanvasInfo, calling this interface is invalid unless calling HB_RGN_UpdateCanvas to make the canvas effective.

【Reference Code】
```c
    RGN_HANDLE handle = 0;
    RGN_ATTR_S stRegion;
    int ret;

    RGN_BITMAP_S stBitmapAttr;
    stBitmapAttr.enPixelFormat = PIXEL_FORMAT_VGA_4;
    stBitmapAttr.stSize.u32Width = 640;
    stBitmapAttr.stSize.u32Height = 128;
    stBitmapAttr.pAddr = malloc(640 * 64);
    memset(stBitmapAttr.pAddr, 0xff, 640 * 64);

    RGN_CHN_S stChn;
    stChn.s32PipelineId = 0;
    stChn.enChnId = CHN_US;

    RGN_DRAW_WORD_S stDrawWord;
    stDrawWord.enFontSize = FONT_SIZE_MEDIUM;
    stDrawWord.enFontColor = FONT_COLOR_WHITE;
    stDrawWord.stPoint.u32X = 0;
    stDrawWord.stPoint.u32Y = 0;
    time_t tt = time(0);
    char str[32];
    strftime(str, sizeof(str), "%Y-%m-%d %H:%M:%S", localtime(&tt));
    stDrawWord.pu8Str = str;
    stDrawWord.bFlushEn = false;
    stDrawWord.pAddr = stBitmapAttr.pAddr;
    stDrawWord.stSize = stBitmapAttr.stSize;

    RGN_DRAW_LINE_S stDrawLine[2];
    stDrawLine[0].stStartPoint.u32X = 400;
    stDrawLine[0].stStartPoint.u32Y = 0;
    stDrawLine[0].stEndPoint.u32X = 500;
    stDrawLine[0].stEndPoint.u32Y = 100;
    stDrawLine[0].bFlushEn = false;
    stDrawLine[0].pAddr = stBitmapAttr.pAddr;
    stDrawLine[0].stSize = stBitmapAttr.stSize;
    stDrawLine[0].u32Color = FONT_COLOR_WHITE;
    stDrawLine[0].u32Thick = 4;
    memcpy(&stDrawLine[1], &stDrawLine[0], sizeof(RGN_DRAW_LINE_S));
    stDrawLine[1].stEndPoint.u32Y = 200;

    ret = HB_RGN_DrawWord(handle, &stDrawWord);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_DrawLine(handle, &stDrawLine[0]);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_DrawLineArray(handle, stDrawLine, 2);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_SetBitMap(handle, &stBitmapAttr);
    if (ret < 0) {
        return ret;
    }
```

### HB_RGN_AttachToChn/HB_RGN_DetachFromChn
【Function Declaration】
```c
int32_t HB_RGN_AttachToChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_DetachFromChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn);
```
【Function Description】
>Attach or detach the region to or from the channel.

【Parameter Description】

| Parameter Name |                   Description                   |
| :------------: | :---------------------------------------------: |
|     Handle     | The handle of the region. Range: [0, RGN_HANDLE_MAX). |
|    pstChn      |            Pointer to the channel structure.            |
|   pstChnAttr   |      Pointer to the region channel display attribute.     |

【Return Value】

| Return Value |         Description          |
| :----------: | :--------------------------: |
|      0       |            Success           |
|    Non-zero  | Failure, see error code. |

【Notes】
HB_RGN_AttachToChn:
1. The region must have been created.
2. The pointer to the channel structure and the pointer to the display attribute structure cannot be NULL.3. Each channel can have up to 32 regions overlaid.
4. The size of the region overlaid on the channel should not exceed the channel resolution.

HB_RGN_DetachFromChn:
1. The region must have been created.
2. The channel structure pointer cannot be empty.

【Reference Code】
```c
    RGN_HANDLE handle = 0;
    int ret;
    int osd_level = 0;
    RGN_CHN_ATTR_S stChnAttr;
    stChnAttr.bShow = true;
    stChnAttr.bInvertEn = false;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32X = 0;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32Y = 0;

    RGN_CHN_S stChn;
    stChn.s32PipelineId = 0;
    stChn.enChnId = CHN_US;

    ret = HB_RGN_AttachToChn(handle, &stChn, &stChnAttr);
    if (ret < 0) {
        return ret;
    }
    HB_RGN_GetDisplayAttr(handle, &stChn, &stChnAttr);
    if (ret < 0) {
        return ret;
    }
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32X = 20;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32Y = 20;
    HB_RGN_SetDisplayAttr(handle, &stChn, &stChnAttr);
    if (ret < 0) {
        return ret;
    }
    HB_RGN_SetDisplayLevel(handle, &stChn, osd_level);
    if (ret < 0) {
        return ret;
    }
    HB_RGN_DetachFromChn(handle, &stChn);
    if (ret < 0) {
        return ret;
    }
```

### HB_RGN_SetDisplayAttr/HB_RGN_GetDisplayAttr
【Function Declaration】
```c
int32_t HB_RGN_SetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_GetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, RGN_CHN_ATTR_S *pstChnAttr);
```
【Function Description】
> Get or set the display attribute of the region on the channel.

【Parameter Description】

|  Parameter Name  |               Description                |
| :--------------: | :--------------------------------------: |
|      Handle      |   Region handle number. Range: [0, RGN_HANDLE_MAX). |
|      pstChn      |        Pointer to the channel structure.         |
|    pstChnAttr    |  Pointer to the display attribute of the region channel.    |

【Return Value】

| Return Value |               Description |
| :----------: | :----------------------- |
|      0       |          Success          |
|   Non-zero   |     Failure, see error codes.   |

【Notes】
HB_RGN_SetDisplayAttr:
1. The region must be created first.
2. It is recommended to get the attribute before setting.
3. The pointer to the channel structure and the pointer to the display attribute structure cannot be empty.
4. The region must be overlaid on the channel first.
5. The size of a region of Cover type cannot be modified.

HB_RGN_GetDisplayAttr:
1. The region must be created first.
2. The pointer to the channel structure and the pointer to the display attribute structure cannot be empty.

【Reference Code】
> Please refer to the examples of HB_RGN_AttachToChn/HB_RGN_DetachFromChn.

### HB_RGN_GetCanvasInfo/HB_RGN_UpdateCanvas
【Function Declaration】
```c
int32_t HB_RGN_GetCanvasInfo(RGN_HANDLE Handle, RGN_CANVAS_S *pstCanvasInfo);
int32_t HB_RGN_UpdateCanvas(RGN_HANDLE Handle);
```
【Function Description】
> Get or update the display canvas.

【Parameter Description】

|  Parameter Name  |               Description                |
| :--------------: | :--------------------------------------: |
|      Handle      |   Region handle number. Range: [0, RGN_HANDLE_MAX). |
|  pstCanvasInfo   |        Information of the region display canvas.         |【Return Value】

| Return Value | Description |
| :----: | :-----------------|
|   0    |               Success |
|  Nonzero   | Failure, see error code. |

【Notes】
HB_RGN_GetCanvasInfo:
1. The region must have been created；
2. Similar to HB_RGN_SetBitMap, used to update the bitmap data of the overlay type region; this interface can directly operate on the internal buffer to save one memory copy；
3. This interface is mutually exclusive with HB_RGN_SetBitMap. If this interface has been used, calling HB_RGN_SetBitMap will not take effect before calling HB_RGN_UpdateCanvas;

HB_RGN_UpdateCanvas:
1. The region must have been created；
2. This interface is used with HB_RGN_GetCanvasInfo to switch buffer display after updating the data；
3. Before using this interface every time, you need to call HB_RGN_GetCanvasInfo to get the information；
【Reference Code】
```c
    RGN_HANDLE handle = 0;
    RGN_ATTR_S stRegion;
    RGN_CANVAS_S stCanvasInfo;
    int ret;
    stRegion.enType = OVERLAY_RGN;
	stRegion.stOverlayAttr.stSize.u32Width = 640;
	stRegion.stOverlayAttr.stSize.u32Height = 128;
	stRegion.stOverlayAttr.enPixelFmt = PIXEL_FORMAT_VGA_4;

    ret = HB_RGN_Create(handle, &stRegion);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_GetCanvasInfo(handle, &stCanvasInfo);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_UpdateCanvas(handle);
    if (ret < 0) {
        return ret;
    }
```

### HB_RGN_DrawWord
【Function Declaration】
```c
int32_t HB_RGN_DrawWord(RGN_HANDLE Handle, const RGN_DRAW_WORD_S *pstRgnDrawWord);
```
【Function Description】
> Generate a bitmap from the given string and address.【Parameter Description】

|    Parameter Name    |                  Description                   |
| :------------------: | :--------------------------------------------: |
|        Handle        |       Region handle number. Range: [0, RGN_HANDLE_MAX).   |
|   pstRgnDrawWord   |    Pointer to the parameters for drawing text.  |

【Return Value】

| Return Value |               Description                 |
| :----------: | :---------------------------------------: |
|       0       |                    Success                   |
|     Non-zero        |     Failed, see error code.       |

【Notes】
1. The region must have been created;
2. The pointer to the structure of attribute information and the pointer to the address cannot be empty;
3. The value of attribute information needs to be valid;
4. The write format is PIXEL_FORMAT_VGA_4;

【Sample Code】
> Please refer to the example of HB_RGN_SetBitMap.

### HB_RGN_DrawLine/HB_RGN_DrawLineArray
【Function Declaration】
```c
int32_t HB_RGN_DrawLine(RGN_HANDLE Handle, const RGN_DRAW_LINE_S *pstRgnDrawLine);
int32_t HB_RGN_DrawLineArray(RGN_HANDLE Handle,const RGN_DRAW_LINE_S astRgnDrawLine[],uint32_t u32ArraySize);
```
【Function Description】
> Draw a line or draw multiple lines at once.

【Parameter Description】

|    Parameter Name    |                  Description                   |
| :------------------: | :--------------------------------------------: |
|        Handle        |        Region handle number. Range: [0, RGN_HANDLE_MAX).   |
|   pstRgnDrawLine   |   Pointer or array of parameters for drawing lines.   |
|   u32ArraySize  |            Number of lines to draw.          |

【Return Value】

| Return Value |               Description     |
| :----------: | :---------------------------: |
|       0       |                Success               |
|     Non-zero       |     Failed, see error code.       |

【Notes】
1. The region must have been created;2. The pointer to attribute information structure and the pointer to address must not be empty;
3. The number of array elements in the HB_RGN_DrawLineArray interface must match the array;
4. The writing format is PIXEL_FORMAT_VGA_4 format;

【Reference Code】
> Please refer to the example of HB_RGN_SetBitMap

### HB_RGN_BatchBegin/HB_RGN_BatchEnd
【Function Declaration】
```c
int32_t HB_RGN_BatchBegin(RGN_HANDLEGROUP *pu32Group, uint32_t u32Num, const RGN_HANDLE handle[]);
int32_t HB_RGN_BatchEnd(RGN_HANDLEGROUP u32Group);
```
【Function Description】
> Batch update regions;

【Parameter Description】

|     Parameter Name    |                              Description                              |
| :-------------------: | :-------------------------------------------------------------------: |
|       pu32Group       |                     Batch processing group number.                    |
|         u32Num        |                  Number of regions to be batch processed.              |
|         handle        |                 Array of region handles to be batch processed.         |

【Return Value】

| Return Value |                     Description                     |
| :----------: | :-----------------------------------------------: |
|       0      |                      Success                      |
|    Non-zero  |                 Failure, see error code.           |

【Notes】
1. Regions must have been created;
2. The number of handles set by HB_RGN_BatchBegin must be equal to the length of the array and not exceed the maximum value;
3. The region type must be Overlay;
4. HB_RGN_BatchBegin must appear in pairs with HB_RGN_BatchEnd;

【Reference Code】
```c
    RGN_HANDLE handle_batch[3];
    int ret = 0;
    RGN_HANDLEGROUP group = 0;
    for (int i = 0; i < 3; i++) {
        handle_batch[i] = i;
    }
    ret = HB_RGN_BatchBegin(&group, 3, handle_batch);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_GetCanvasInfo(handle_batch[0], &stCanvasInfo);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_GetCanvasInfo(handle_batch[1], &stCanvasInfo);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_GetCanvasInfo(handle_batch[2], &stCanvasInfo);
    if (ret < 0) {
        return ret;
    }
    ret = HB_RGN_BatchEnd(group);
    if (ret < 0) {
        return ret;
}
```

### HB_RGN_SetColorMap

【Function Declaration】
```c
int32_t HB_RGN_SetColorMap(const RGN_CHN_S *pstChn, uint32_t aColorMap[15]);
```

【Description】
> Set the color palette for using colors. After use, the enumeration RGN_FONT_COLOR_E is invalid. It needs to be used after the region is attached to the channel.

【Parameter Description】

| Parameter |          Description           |
| :-------: | :----------------------------- |
|  pstChn   | Pointer to the channel struct. |
| aColorMap | The set color palette, the set color value is in RGB format. |

【Return Value】

| Return Value |          Description |
| :----------: | :------------------- |
|      0       |        Success       |
|    Non-zero  | Failure, see error code. |

【Notes】
1. The pointer to the channel struct cannot be empty.
2. The channel in the vps module needs to be enabled.
3. Set once for all channels to share.
4. The input color value is in the RGB color space.
5. CHN_GRP cannot be passed as a parameter.

【Example Code】
```c
    RGN_CHN_S stChn;
    stChn.s32PipelineId = 0;
    stChn.enChnId = CHN_US;
    uint32_t aColorMap[15] = {0xFFFFFF, 0x000000, 0x808000, 0x00BFFF, 0x00FF00,
                            0xFFFF00, 0x8B4513, 0xFF8C00, 0x800080, 0xFFC0CB,
                            0xFF0000, 0x98F898, 0x00008B, 0x006400, 0x8B0000};
    int ret;

    ret = HB_RGN_SetColorMap(&stChn, aColorMap);
    if (ret < 0) {
        return ret;
    }
```

### HB_RGN_SetSta/HB_RGN_GetSta

【Function Declaration】
```c
int32_t HB_RGN_SetSta(const RGN_CHN_S *pstChn, uint8_t astStaLevel[3], RGN_STA_S astStaAttr[8]);
int32_t HB_RGN_GetSta(const RGN_CHN_S *pstChn, uint16_t astStaValue[8][4]);
```

【Description】
> Set up to 8 areas and obtain the sum of brightness of the specified region. The region needs to be attached to the channel for use;

【Parameter Description】

|  Parameter Name |                        Description                        |
| :-------------: | :-------------------------------------------------------: |
|     pstChn      |                 Pointer to the channel struct.                 |
|  astStaLevel  |    Brightness level set (0, 255).      |
|   astStaAttr   |          Properties of the brightness region to be obtained.             |
|  astStaValue  | Number of pixels within the specified range. |

【Return Value】

| Return Value |           Description |
| :----------: | :-----------------|
|      0      |           Successful |
|   Non-zero   | Failure, see error code. |

【Precautions】
1. The channel structure cannot be empty;
2. HB_RGN_SetSta and HB_RGN_GetSta must appear in pairs;
3. HB_RGN_SetSta is used to set up to 8 area information, and HB_RGN_GetSta obtains brightness information for the specified area;

【Reference Code】
```c
RGN_CHN_S stChn;
stChn.s32PipelineId = 0;
stChn.enChnId = CHN_US;
uint16_t aOsdStaBinValue[8][4];
RGN_STA_S aOsdSta[8];
uint8_t aStaLevel[3];
int ret;
aStaLevel[0] = 60;
aStaLevel[1] = 120;
aStaLevel[2] = 180;

memset(aOsdStaBinValue, 0, 8 * 4 * sizeof(uint16_t));
for (int i = 0; i < 8; i++) {
    aOsdSta[i].u8StaEn = true;
    aOsdSta[i].u16StartX = i * 50;
    aOsdSta[i].u16StartY = 0;
    aOsdSta[i].u16Width = 50;
    aOsdSta[i].u16Height = 50;
}

ret = HB_RGN_SetSta(&stChn, aStaLevel, aOsdSta);
if (ret < 0) {
    return ret;
}

ret = HB_RGN_GetSta(&stChn, aOsdStaBinValue);
if (ret < 0) {
    return ret;
}

ret = HB_RGN_AddToYUV(Handle, vio_buffer, pstChnAttr);
if (ret != 0) {
    return ret;
}
```

### HB_RGN_AddToYUV

【Function Declaration】
```c
int32_t HB_RGN_AddToYUV(RGN_HANDLE Handle, hb_vio_buffer_t *vio_buffer, const RGN_CHN_ATTR_S *pstChnAttr);
```

【Function Description】
> Overlay the region onto a yuv420 format image.

【Parameter Description】

| Parameter Name |                    Description                     |
| :------------: | :------------------------------------------------: |
|     Handle     | Region handle number. Value range: [0, RGN_HANDLE_MAX). |
|  vio_buffer    |              Buffer pointer of yuv image.              |
|   pstChnAttr   |           Pointer to the channel attribute of the region.           |

【Return Value】

| Return Value |               Description |
| :----------: | :-----------------------: |
|      0       |               Success |
|   Non-zero   | Failure, see the error code. |

【Precautions】
1. The region must have been created.2. The pointer to the image buffer structure and the pointer to the display attribute structure cannot be empty.

【Reference Code】
```c
    RGN_HANDLE handle;
    int ret;
    hb_vio_buffer_t vio_buffer;
    RGN_CHN_ATTR_S stChnAttr;
    stChnAttr.bShow = true;
    stChnAttr.bInvertEn = false;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32X = 0;
    stChnAttr.unChnAttr.stOverlayChn.stPoint.u32Y = 0;

    ret = HB_RGN_AddToYUV(handle, &vio_buffer, &stChnAttr);
    if (ret < 0) {
        return ret;
    }
```

### HB_RGN_SetDisplayLevel
【Function Declaration】
```c
int32_t HB_RGN_SetDisplayLevel(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, uint32_t osd_level);
```
【Function Description】
> Set the display level of the region.

【Parameter Description】

| Parameter Name |                Description                |
| :------------: | :---------------------------------------: |
|     Handle     |          Region handle number.           |
|     pstChn     |          Channel structure pointer.       |
|   osd_level    | Display level of the region on the channel [0, 3]. |

【Return Value】

| Return Value | Description |
| :----------: | :--------: |
|      0       |  Success   |
|    Non-zero   |  Failure   |

【Notes】
1. The region must have been created.
2. The pointer to the channel structure cannot be empty.
3. The level ranges from 0 to 3, with 0 being the default hardware processing. If it exceeds the number of hardware processing or the channel does not support it, it will be changed to software processing. Levels 1 to 3 are processed by software.
4. Different regions on the same channel can have different display levels.

【Reference Code】
> Please refer to examples of HB_RGN_AttachToChn/HB_RGN_DetachFromChn.

## Data Structure
### RGN_SIZE_S
【Structure Definition】
```c
typedef struct HB_RGN_SIZE_ATTR_S {
    uint32_t u32Width;
    uint32_t u32Height;
} RGN_SIZE_S;
```
【Description】
> Defines the structure for size information.

【Member Description】

| Member    | Meaning |
| --------- | ------- |
| u32Width  | Width   |
| u32Height | Height  |

### RGN_POINT_S
【Structure Definition】
```c
typedef struct HB_RGN_POINT_ATTR_S {
    uint32_t u32X;
    uint32_t u32Y;
} RGN_POINT_S;
```
【Description】
> Defines the structure for coordinate information.

【Member Description】

| Member | Meaning  |
| ------ | -------- |
| u32X   | X-axis   |
| u32Y   | Y-axis   |

### RGN_RECT_S
【Structure Definition】
```c
typedef struct HB_RGN_RECT_ATTR_S {
    uint32_t u32X;
    uint32_t u32Y;
    uint32_t u32Width;
    uint32_t u32Height;
} RGN_RECT_S;
```
【Description】
> Defines the structure for rectangle information.【Member Description】

|   Member   |  Meaning  |
| :-------: | :----: |
|   u32X    | X Coordinate |
|   u32Y    | Y Coordinate |
| u32Width  |  Width  |
| u32Height |  Height  |

### RGN_OVERLAY_S
【Structure Definition】
```c
typedef struct HB_RGN_OVERLAY_ATTR_S {
    RGN_PIXEL_FORMAT_E enPixelFmt;
    RGN_FONT_COLOR_E enBgColor;
    RGN_SIZE_S stSize;
} RGN_OVERLAY_S;
```
【Function Description】
> Defines the structure of overlay region attributes

【Member Description】

|    Member    | Meaning                                                                                                           |
| :--------: | :------------------------------------------------------------------------------------------------------------- |
| enPixelFmt | Pixel format                                                                                                       |
| enBgColor  | Background color of the bitmap                                                                                                   |
|   stSize   | Size of the region<br/>PIXEL_FORMAT_VGA_4:<br/>Minimum width is 32, minimum height is 2<br/>PIXEL_FORMAT_YUV420SP:<br/>Minimum width is 2, minimum height is 2 |

### RGN_ATTR_S
【Structure Definition】
```c
typedef struct HB_RGN_ATTR_S {
    RGN_TYPE_E enType;
    RGN_OVERLAY_S stOverlayAttr;
} RGN_ATTR_S;
```
【Function Description】
> Defines the structure of region information

【Member Description】

|     Member      |      Meaning      |
| :-----------: | :------------: |
|    enType     |    Region type    |
| stOverlayAttr | Attributes of the overlay region |

### RGN_CHN_S
【Structure Definition】
```c
typedef struct HB_RGN_CHN_S
{
    uint32_t s32PipelineId;
    int32_t enChnId;
} RGN_CHN_S;
```
【Function Description】
> Defines the structure for data stream channel.

【Member Description】

|    Member    |            Meaning           |
| :----------: | :--------------------------: |
| s32PipelineId|         pipelineID           |
|   enChnId    | Channel ID within [0, CHN_MAX_NUM) range |

### RGN_OVERLAY_CHN_S
【Structure Definition】
```c
typedef struct HB_RGN_OVERLAY_CHN_ATTR_S {
    RGN_POINT_S stPoint;
} RGN_OVERLAY_CHN_S;
```
【Function Description】
> Defines the structure for attributes of overlay region display.

【Member Description】

|  Member  |   Meaning   |
| :------: | :---------: |
| stPoint  | Region position |

### RGN_COVER_CHN_S
【Structure Definition】
```c
typedef struct HB_RGN_COVER_CHN_ATTR_S {
    RGN_RECT_S stRect;
    uint32_t u32Color;
} RGN_COVER_CHN_S;
```
【Function Description】
> Defines the structure for attributes of covering region display.

【Member Description】

|   Member   |                 Meaning                 |
| :--------: | :--------------------------------------: |
|   stRect   | Region position, width and height, with the minimum width of 32 and the minimum height of 2 |
| u32Color   |              Region color                |

### RGN_CHN_U
【Structure Definition】
```c
typedef union HB_RGN_CHN_ATTR_U {
    RGN_OVERLAY_CHN_S stOverlayChn;
    RGN_COVER_CHN_S stCoverChn;
} RGN_CHN_U;
```
【Function Description】
> Define a union for the display attributes of the region channel.

【Member Description】

|   Member    |      Meaning      |
| :---------: | :---------------: |
| stOverlayChn| Overlay region display attributes |
|  stCoverChn |   Cover region display attributes |


### RGN_CANVAS_S
【Structure Definition】
```c
typedef struct HB_RGN_CANVAS_INFO_S {
    void *pAddr;
    RGN_SIZE_S stSize;
    RGN_PIXEL_FORMAT_E enPixelFmt;
} RGN_CANVAS_S;
```
【Function Description】
> Define a structure for the information of the canvas.

【Member Description】

|   Member   |    Meaning    |
| :--------: | :-----------: |
|   pAddr    |   Canvas address |
|   stSize   |   Canvas size |
| enPixelFmt | Pixel format |



### RGN_CHN_ATTR_S
**Structure Definition**
```c
typedef struct HB_RGN_CHN_ATTR_S {
    bool bShow;                     // Region visibility
    bool bInvertEn;                 // Invert color enable
    RGN_CHN_U unChnAttr;            // Channel display attribute
} RGN_CHN_ATTR_S;
```
**Function Description**
> Defines the structure for region channel display attributes.

**Member Descriptions**

|   Member    |       Meaning       |
| :---------: | :------------------: |
|   bShow    | Whether to show the region |
| bInvertEn | Enable or disable inversion |
| unChnAttr | Attributes for channel display |

### RGN_BITMAP_S
**Structure Definition**
```c
typedef struct HB_RGN_BITMAP_ATTR_S {
    RGN_PIXEL_FORMAT_E enPixelFormat; // Pixel format
    RGN_SIZE_S stSize;               // Bitmap size
    void *pAddr;                     // Bitmap address
} RGN_BITMAP_S;
```
**Function Description**
> Defines the structure for bitmap attributes.

**Member Descriptions**

|     Member      |              Meaning              |
| :-----------: | :--------------------------------: |
| enPixelFormat | Bitmap pixel format             |
|    stSize     | Size of the bitmap               |
|     pAddr     | Address of the bitmap             |

### RGN_DRAW_WORD_S
**Structure Definition**
```c
typedef struct HB_RGN_DRAW_WORD_PARAM_S {
    void *pAddr;                   // Destination address for drawing
    RGN_SIZE_S stSize;             // Size of the destination address
    RGN_POINT_S stPoint;           // Position of the text in the bitmap
    uint8_t *pu8Str;               // Pointer to the string (supporting GBK characters only)
    RGN_FONT_COLOR_E enFontColor;  // Font color
    RGN_FONT_SIZE_E enFontSize;    // Font size
    bool bFlushEn;                 // Flush the current address before drawing
} RGN_DRAW_WORD_S;
```
**Function Description**
> Defines the structure for parameters used to draw text.

**Member Descriptions**

|    Member     |                     Meaning                     |
| :---------: | :------------------------------------------------: |
|    pAddr    | Address where the text will be drawn to          |
|   stSize    | Size of the destination area for the text        |
|   stPoint   | Position of the text within the bitmap           |
|   pu8Str    | Pointer to the string (limited to GBK encoding) |
| enFontColor | Color of the font                                |
| enFontSize  | Size of the font                                 |
|  bFlushEn   | Clear the current address before drawing           |

### RGN_DRAW_LINE_S
**Structure Definition**
```c
typedef struct HB_RGN_DRAW_LINE_PARAM_S {
    void *pAddr;                      // Destination address for drawing lines
    RGN_SIZE_S stSize;                // Size of the destination address
    RGN_POINT_S stStartPoint;         // Starting point of the line
    RGN_POINT_S stEndPoint;           // Ending point of the line
    uint32_t u32Thick;                // Thickness of the line
    uint32_t u32Color;                // Color of the line
    bool bFlushEn;                    // Flush the current address before drawing
} RGN_DRAW_LINE_S;
```
**Function Description**
> Defines the structure for parameters used to draw lines.

**Member Descriptions**

|     Member     |            Meaning            |
| :----------: | :----------------------------: |
|    pAddr     | Address for drawing the line |
|    stSize    | Size of the destination area |
| stStartPoint | Start position of the line  |
|  stEndPoint  | End position of the line   |
|   u32Thick   | Width of the line           |
|   u32Color   | Color of the line           |
|   bFlushEn   | Clear the current address |

(When using batch drawing, only the `bFlushEn` property of the first structure in the array is effective)

### RGN_STA_S
**Structure Definition**
```c
typedef struct HB_RGN_STA_ATTR_S {
    uint8_t u8StaEn;           // Enable flag for the region
    uint16_t u16StartX;        // X coordinate of the region start
    uint16_t u16StartY;        // Y coordinate of the region start
    uint16_t u16Width;         // Width of the region (2-255)
    uint16_t u16Height;        // Height of the region (2-255)
} RGN_STA_S;
```
**Function Description**
> Defines the brightness statistics region attributes.

**Member Descriptions**

|   Member    |              Meaning               |
| :-------: | :---------------------------------: |
|  u8StaEn  | Enable flag for the region          |
| u16StartX | X-coordinate of the region start |
| u16StartY | Y-coordinate of the region start |
| u16Width  | Width of the region (2-255)        |
| u16Height | Height of the region (2-255)       |

### RGN_TYPE_E
**Structure Definition**
```c
typedef enum HB_RGN_TYPE_PARAM_E {
    OVERLAY_RGN, // Overlay region type
    COVER_RGN   // Cover region type
} RGN_TYPE_E;
```
**Function Description**
> Defines the region type.

**Member Descriptions**

|    Member     |        Meaning         |
| :---------: | :------------------: |
| OVERLAY_RGN | Type for overlay regions |
|  COVER_RGN  | Type for cover regions  |

### RGN_CHN_ID_E
**Structure Definition**
```c
typedef enum HB_RGN_CHN_ID_ATTR_E {
    CHN_US,      // US channel
    CHN_DS0,     // DS0 channel
    CHN_DS1,     // DS1 channel
    CHN_DS2,     // DS2 channel
    CHN_DS3,     // DS3 channel
    CHN_DS4,     // DS4 channel
    CHN_GRP,     // GROUP channel (automatically scaled if VPS performs zoom)
    CHANNEL_MAX_NUM
} RGN_CHN_ID_E;
```
**Function Description**
> Defines the channel ID.

**Member Descriptions**

|      Member       |                             Meaning                             |
| :-------------: | :----------------------------------------------------------: |
|     CHN_US      |                            US channel                            |
|     CHN_DS0     |                           DS0 channel                            |
|     CHN_DS1     |                           DS1 channel                            |
|     CHN_DS2     |                           DS2 channel                            |
|     CHN_DS3     |                           DS3 channel                            |
|     CHN_DS4     |                           DS4 channel                            |
|     CHN_GRP     | GROUP channel; scaled if VPS performs zoom before IPU |
| CHANNEL_MAX_NUM |                           Maximum number of channels                           |

### RGN_FONT_SIZE_E
**Structure Definition**
```c
typedef enum HB_RGN_FONT_SIZE_ATTR_E {
    FONT_SIZE_SMALL = 1, // Small font, 16x16
    FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
    FONT_SIZE_EXTRA_LARGE
} RGN_FONT_SIZE_E;
```
**Function Description**
> Defines font sizes.

**Member Descriptions**

|         Member          |       Meaning        |
| :-------------------: | :---------------: |
|    FONT_SIZE_SMALL    |  Small font, 16x16  |
|   FONT_SIZE_MEDIUM    |  Medium font, 32x32  |
|    FONT_SIZE_LARGE    |  Large font, 48x48  |
| FONT_SIZE_EXTRA_LARGE | Extra large font, 64x64 |

### RGN_FONT_COLOR_E
**Structure Definition**
```c
typedef enum HB_RGN_FONT_COLOR_ATTR_E {
    FONT_COLOR_WHITE = 1, // White color
    FONT_COLOR_BLACK,
    FONT_COLOR_GREY,
    FONT_COLOR_BLUE,
    FONT_COLOR_GREEN,
    FONT_COLOR_YELLOW,
    FONT_COLOR_BROWN,
    FONT_COLOR_ORANGE,
    FONT_COLOR_PURPLE,
    FONT_COLOR_PINK,
    FONT_COLOR_RED,
    FONT_COLOR_CYAN,
    FONT_COLOR_DARKBLUE,
    FONT_COLOR_DARKGREEN,
    FONT_COLOR_DARKRED,
    FONT_KEY_COLOR = 16 // Do not overlay, use original image color
} RGN_FONT_COLOR_E;
```
**Function Description**
> Defines font colors; this enumeration becomes invalid after calling HB_RGN_SetColorMap;

**Member Descriptions**

|         Member         |         Meaning         |
| :------------------: | :------------------: |
|   FONT_COLOR_WHITE   |         White         |
|   FONT_COLOR_BLACK   |         Black         |
|   FONT_COLOR_GREY    |         Grey         |
|   FONT_COLOR_BLUE    |         Blue         |
|   FONT_COLOR_GREEN   |         Green         |
|  FONT_COLOR_YELLOW   |         Yellow         |
|   FONT_COLOR_BROWN   |         Brown         |
|  FONT_COLOR_ORANGE   |        Orange        |
|  FONT_COLOR_PURPLE   |         Purple        |
|   FONT_COLOR_PINK    |        Pink         |
|    FONT_COLOR_RED    |         Red



### RGN_PIXEL_FORMAT_E
【Structure Definition】
```c
typedef enum HB_PIXEL_FORMAT_ATTR_E
{
    PIXEL_FORMAT_VGA_4,
    PIXEL_FORMAT_YUV420SP
} RGN_PIXEL_FORMAT_E;
```
【Function Description】
> Define pixel formats.

【Member Description】

|        Member         |                              Meaning                              |
| :-------------------: | :---------------------------------------------------------------: |
|  PIXEL_FORMAT_VGA_4   | 4-bit 16-color pixel format<br/>[Each pixel occupies 4 bits (0-15), which corresponds to RGN_FONT_COLOR_E as color index] |
| PIXEL_FORMAT_YUV420SP |                          YUV420SP pixel format                          |

### RGN_HANDLE
【Structure Definition】
```c
typedef int32_t RGN_HANDLE;
```


【Function Description】
> Define the region handle.

### RGN_HANDLE_MAX
【Structure Definition】
```c
#define RGN_HANDLE_MAX 256
```
【Function Description】
> Define the maximum number of region handles.

### RGN_HANDLEGROUP
【Structure Definition】
```c
typedef int32_t RGN_HANDLEGROUP;
```
【Function Description】
> Define the batch processing group number.

### RGN_GROUP_MAX
【Structure Definition】
```c
#define RGN_GROUP_MAX 8
```
【Function Description】
> Define the maximum number of batch processing.

## Error Codes
The error codes for RGN are as follows:

|   Error Code   | Macro Definition              | Description          |
| :------------: | :---------------------------- | :--------------------|
| -268762113     | HB_ERR_RGN_INVALID_CHNID      | Invalid channel ID   |
| -268762114     | HB_ERR_RGN_ILLEGAL_PARAM      | Invalid input parameter   |
| -268762115     | HB_ERR_RGN_EXIST              | Region handle already exists   |
| -268762116     | HB_ERR_RGN_UNEXIST            | Region handle does not exist   |
| -268762117     | HB_ERR_RGN_NULL_PTR           | Null pointer   |
| -268762118     | HB_ERR_RGN_NOMEM              | Out of memory   |
| -268762119     | HB_ERR_RGN_OPEN_FILE_FAIL     | Failed to open font library file   |
| -268762120     | HB_ERR_RGN_INVALID_OPERATION  | Invalid operation   |
| -268762121     | HB_ERR_RGN_PROCESS_FAIL_OSD   | Processing failed   |

## Reference Code
For example code of OSD, please refer to [sample_osd](./multimedia_samples#sample_osd).