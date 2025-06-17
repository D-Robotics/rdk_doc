---
sidebar_position: 7
---

# 7.3.7 区域处理
## 概述
用户一般都需要在视频中叠加 `OSD` 用于显示一些特定的信息（如：通道号、时间戳等），必要时还会填充色块。这些叠加在视频上的 `OSD` 和遮挡在视频上的色块统称为区域。`REGION` 模块，用于统一管理这些区域资源。

区域管理可以实现区域的创建，并叠加到视频中或对视频进行遮挡。例如，实际应用中，用户通过`HB_RGN_AttachToChn`创建一个区域，将该区域叠加到某个通道（如US通道）中。在通道进行调度时，则会将 `OSD` 叠加在视频中。一个区域支持通过调用设置通道显示属性接口指定到多个通道中（如：`US通道`和`DS通道`），且支持在每个通道的显示属性（如位置、是否显示等）都不同。

## 功能描述
### 基本概念
区域类型：
- `overlay`:视频叠加区域，绘制文字、线条等；
- `cover`：视频遮挡区域，纯色块遮挡；

位图填充：
- 将区域位图叠加到区域内存中，使用`HB_RGN_SetBitMap`方式时如果位图的大小比设定的区域大的话，将会裁剪掉超出区域范围的部分；
使用`HB_RGN_GetCanvasInfo`/`HB_RGN_UpdateCanvas`方式时需要按照获取的画布大小写入。

区域属性：
- 创建区域时需要设置区域的一些基本信息，例如大小、区域类型等。

通道显示属性：
- 将区域叠加到通道上时需要设定通道的显示属性，例如叠加的位置、是否显示等，如果设置`bShow`为`false`，将叠加到通道但是不显示区域。

绘制文字：
- 可以使用`HB_RGN_DrawWord`绘制文字，支持四种字体大小以及15种字体颜色；

绘制线条：
- 可以使用`HB_RGN_DrawLine`/`HB_RGN_DrawLineArray`绘制线条或同时多个线条，支持调整线条粗细以及线条颜色。

区域反色：
- 在通道显示属性中有反色开关，如果使能了反色，将会在叠加时使得区域的颜色反转。

### 使用示意
使用过程应该如下所示：
- 用户通过创建区域并设置区域属性；
- 用户将区域绑定到通道上；
- 通过`HB_RGN_GetAttr`/`HB_RGN_SetAttr`获取或修改区域属性；
- 使用`HB_RGN_SetBitMap`方式：
  - 使用`HB_RGN_DrawWord`或`HB_RGN_DrawLine`/`HB_RGN_DrawLineArray`绘制文字或线条到用户创建的位图中，然后调用`HB_RGN_SetBitMap`将位图设置到区域中；
  - 使用`HB_RGN_GetCanvasInfo`/`HB_RGN_UpdateCanvas`方式：
  - 使用`HB_RGN_GetCanvasInfo`获取地址，使用`HB_RGN_DrawWord`或`HB_RGN_DrawLine`/`HB_RGN_DrawLineArray`绘制文字或线条到获取的地址中，再使用`HB_RGN_UpdateCanvas`更新画布。
- 通过`HB_RGN_SetDisplayAttr`/`HB_RGN_GetDisplayAttr`获取或设置通道显示属性；
- 最后用户再将区域从通道中撤出，销毁区域；

## API参考
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

### HB_RGN_Create/HB_RGN_Destory
【函数声明】
```c
int32_t HB_RGN_Create(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
int32_t HB_RGN_Destory(RGN_HANDLE Handle);
```
【功能描述】
> 创建或销毁一块区域；

【参数描述】

|   成员    |                    含义                     |
| :-------: | :-----------------------------------------: |
|  Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstRegion |               区域属性指针。                |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
HB_RGN_Create：
1. 句柄由用户指定，意义等同于ID号，句柄号需在指定范围内；
2. 不支持重复创建；
3. 区域属性不能为空且属性需合法；
4. 创建Cover类型区域的时候只需指定区域类型，区域属性在调用HB_RGN_AttachToChn时指定；
5. 创建区域时，会进行最大最小宽高等检查，具体支持像素格式请参考RGN_PIXEL_FORMAT_E；

HB_RGN_Destory：
1. 区域必须已创建；
2. 调用此接口之前区域需先调用HB_RGN_DetachFromChn接口；
3. 调用该接口的过程中，不允许同时调用HB_RGN_SetAttr及HB_RGN_SetBitMap接口；

【参考代码】
```c
    RGN_HANDLE handle = 0;
    RGN_ATTR_S stRegion;
    int ret;
    stRegion.enType = OVERLAY_RGN;
    stRegion.stOverlayAttr.stSize.u32Width = 640;
    stRegion.stOverlayAttr.stSize.u32Height = 128;
    stRegion.stOverlayAttr.enBgColor = FONT_KEY_COLOR；
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
【函数声明】
```c
int32_t HB_RGN_GetAttr(RGN_HANDLE Handle, RGN_ATTR_S *pstRegion);
int32_t HB_RGN_SetAttr(RGN_HANDLE Handle, const RGN_ATTR_S *pstRegion);
```
【功能描述】
> 获取或设置区域属性；

【参数描述】

| 参数名称  |                    描述                     |
| :-------: | :-----------------------------------------: |
|  Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstRegion |               区域属性指针。                |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
HB_RGN_GetAttr：
1. 区域必须已创建；
2. 区域属性指针不能为空；
3. 区域类型必须为Overlay，Cover属性在HB_RGN_AttachToChn时指定、HB_RGN_SetDisplayAttr时修改；

HB_RGN_SetAttr：
1. 区域必须已创建；
2. 区域属性指针不能为空；
3. 区域类型必须为Overlay，Cover属性在HB_RGN_AttachToChn时指定、HB_RGN_SetDisplayAttr时修改；
4. 在调用HB_RGN_AttachToChn之后不可修改区域大小；

【参考代码】
> 请参见HB_RGN_Create/HB_RGN_Destory举例

### HB_RGN_SetBitMap
【函数声明】
```c
int32_t HB_RGN_SetBitMap(RGN_HANDLE Handle, const RGN_BITMAP_S *pstBitmapAttr);
```
【功能描述】
> 设置位图，填充一块区域；

【参数描述】

| 参数名称  |                    描述                     |
| :-------: | :-----------------------------------------: |
|  Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstBitmap |               位图属性指针。                |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 区域必须已创建；
2. 支持位图的大小和区域的大小不一致；
3. 位图从区域的（0，0）开始加载，位图比区域大时自动裁剪；
4. 像素格式必须和区域像素格式一致；
5. 位图属性指针不能为空；
6. 支持多次调用；
7. 此接口只对Overlay类型区域有效；
8. 调用了HB_RGN_GetCanvasInfo之后，调用本接口无效，除非调用HB_RGN_UpdateCanvas更新画布生效；

【参考代码】
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
【函数声明】
```c
int32_t HB_RGN_AttachToChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_DetachFromChn(RGN_HANDLE Handle, const RGN_CHN_S *pstChn);
```
【功能描述】
> 将区域叠加到通道或从通道中撤出；

【参数描述】

|  参数名称  |                    描述                     |
| :--------: | :-----------------------------------------: |
|   Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
|   pstChn   |              通道结构体指针。               |
| pstChnAttr |           区域通道显示属性指针。            |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
HB_RGN_AttachToChn：
1. 区域必须已创建；
2. 通道结构体指针及显示属性结构体指针不能为空；
3. 每个通道最多叠加32个区域；
4. 叠加到通道上的区域大小范围不能超过通道分辨率；

HB_RGN_DetachFromChn：
1. 区域必须已创建；
2. 通道结构体指针不能为空；

【参考代码】
```c
    RGN_HANDLE handle = 0;
    int ret;
    int osd_level = 0;
    RGN_CHN_ATTR_S stChnAttr;
    stChnAttr.bShow = true;
    stChnAttr. bInvertEn = false;
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
【函数声明】
```c
int32_t HB_RGN_SetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, const RGN_CHN_ATTR_S *pstChnAttr);
int32_t HB_RGN_GetDisplayAttr(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, RGN_CHN_ATTR_S *pstChnAttr);
```
【功能描述】
> 获取或设置区域在通道的显示属性；

【参数描述】

|  参数名称  |                    描述                     |
| :--------: | :-----------------------------------------: |
|   Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
|   pstChn   |              通道结构体指针。               |
| pstChnAttr |           区域通道显示属性指针。            |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
HB_RGN_SetDisplayAttr：
1. 区域必须已创建；
2. 建议先获取属性再设置；
3. 通道结构体指针及显示属性结构体指针不能为空；
4. 区域需先叠加到通道上；
5. Cover类型的区域大小不可修改；

HB_RGN_GetDisplayAttr:
1. 区域必须已创建；
2. 通道结构体指针及显示属性结构体指针不能为空；

【参考代码】
> 请参见HB_RGN_AttachToChn/HB_RGN_DetachFromChn举例

### HB_RGN_GetCanvasInfo/HB_RGN_UpdateCanvas
【函数声明】
```c
int32_t HB_RGN_GetCanvasInfo(RGN_HANDLE Handle, RGN_CANVAS_S *pstCanvasInfo);
int32_t HB_RGN_UpdateCanvas(RGN_HANDLE Handle);
```
【功能描述】
> 获取或更新显示画布；

【参数描述】

|   参数名称    |                    描述                     |
| :-----------: | :-----------------------------------------: |
|    Handle     | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstCanvasInfo |             区域显示画布信息。              |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
HB_RGN_GetCanvasInfo：
1. 区域必须已创建；
2. 与HB_RGN_SetBitMap类似，用于更新Overlay类型区域位图数据；此接口可以直接操作内部buffer节省一次内存拷贝；
3. 此接口与HB_RGN_SetBitMap接口互斥。如果已经使用了本接口，那么在调用HB_RGN_UpdateCanvas前，调用HB_RGN_SetBitMap不生效；

HB_RGN_UpdateCanvas：
1. 区域必须已创建；
2. 此接口配合HB_RGN_GetCanvasInfo使用，用于更新数据之后切换buffer显示；
3. 每次使用此接口之前，都需要调用HB_RGN_GetCanvasInfo获取信息；
【参考代码】
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
【函数声明】
```c
int32_t HB_RGN_DrawWord(RGN_HANDLE Handle, const RGN_DRAW_WORD_S *pstRgnDrawWord);
```
【功能描述】
> 从给定的字符串和地址生成位图；

【参数描述】

|    参数名称    |                    描述                     |
| :------------: | :-----------------------------------------: |
|     Handle     | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstRgnDrawWord |             绘制文字参数指针。              |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 区域必须已创建；
2. 属性信息结构体指针及地址指针不能为空；
3. 属性信息值需要合法；
4. 写入格式为PIXEL_FORMAT_VGA_4格式；

【参考代码】
> 请参见HB_RGN_SetBitMap举例

### HB_RGN_DrawLine/HB_RGN_DrawLineArray
【函数声明】
```c
int32_t HB_RGN_DrawLine(RGN_HANDLE Handle, const RGN_DRAW_LINE_S *pstRgnDrawLine);
int32_t HB_RGN_DrawLineArray(RGN_HANDLE Handle,const RGN_DRAW_LINE_S astRgnDrawLine[],uint32_t u32ArraySize);
```
【功能描述】
> 画线或批量画线；

【参数描述】

|    参数名称    |                    描述                     |
| :------------: | :-----------------------------------------: |
|     Handle     | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| pstRgnDrawLine |          绘制线条参数指针或数组。           |
|  u32ArraySize  |               绘制线条数量。                |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 区域必须已创建；
2. 属性信息结构体指针及地址指针不能为空；
3. HB_RGN_DrawLineArray接口中数组元素个数必须与数组匹配；
4. 写入格式为PIXEL_FORMAT_VGA_4格式；

【参考代码】
> 请参见HB_RGN_SetBitMap举例

### HB_RGN_BatchBegin/HB_RGN_BatchEnd
【函数声明】
```c
int32_t HB_RGN_BatchBegin(RGN_HANDLEGROUP *pu32Group, uint32_t u32Num, const RGN_HANDLE handle[]);
int32_t HB_RGN_BatchEnd(RGN_HANDLEGROUP u32Group);
```
【功能描述】
> 批量更新区域；

【参数描述】

| 参数名称  |                      描述                       |
| :-------: | :---------------------------------------------: |
| pu32Group | 批处理组号。<br/>取值范围：[0, RGN_GROUP_MAX)。 |
|  u32Num   |               批处理的区域数量。                |
|  handle   |             批处理的区域句柄数组。              |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 区域必须已创建；
2. HB_RGN_BatchBegin设置的句柄个数一定要等于数组的长度，且不超过最大值；
3. 区域类型一定要是Overlay类型；
4. HB_RGN_BatchBegin必须与 HB_RGN_BatchEnd成对出现；

【参考代码】
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
【函数声明】
```c
int32_t HB_RGN_SetColorMap(const RGN_CHN_S *pstChn, uint32_t aColorMap[15]);
```
【功能描述】
> 设置使用颜色的调色板，使用后RGN_FONT_COLOR_E枚举失效，需要区域attach到通道后使用；

【参数描述】

| 参数名称  |                 描述                  |
| :-------: | :-----------------------------------: |
|  pstChn   |           通道结构体指针。            |
| aColorMap | 设置的调色板，设定的颜色值为RGB格式。 |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 通道结构体指针不能为空；
2. 通道在vps模块中对用通道需要使能；
3. 设置一次即可所有通道共享；
4. 输入颜色值为RGB颜色空间；
5. 参数不能传CHN_GRP；

【参考代码】
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
【函数声明】
```c
int32_t HB_RGN_SetSta(const RGN_CHN_S *pstChn, uint8_t astStaLevel[3], RGN_STA_S astStaAttr[8]);
int32_t HB_RGN_GetSta(const RGN_CHN_S *pstChn, uint16_t astStaValue[8][4]);
```
【功能描述】
> 设置指定最多8块区域，获取指定区域的亮度总和，需要区域attach到通道后使用；

【参数描述】

|  参数名称   |                描述                |
| :---------: | :--------------------------------: |
|   pstChn    |          通道结构体指针。          |
| astStaLevel |      设置的亮度等级(0, 255)。      |
| astStaAttr  |    设置的要获取亮度区域的属性。    |
| astStaValue | 获取到的落在指定区间内的像素个数。 |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 通道结构体不能为空；
2. HB_RGN_SetSta与HB_RGN_GetSta需要成对出现；
3. HB_RGN_SetSta用于设置最多8块区域信息，HB_RGN_GetSta获取指定区域亮度信息；

【参考代码】
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
```

### HB_RGN_AddToYUV
【函数声明】
```c
int32_t HB_RGN_AddToYUV(RGN_HANDLE Handle, hb_vio_buffer_t *vio_buffer, const RGN_CHN_ATTR_S *pstChnAttr);
```
【功能描述】
> 将区域叠加到一张yuv420格式的图片上；

【参数描述】

|  参数名称  |                    描述                     |
| :--------: | :-----------------------------------------: |
|   Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
| vio_buffer |            yuv图片的buffer指针。            |
| pstChnAttr |           区域通道显示属性指针。            |

【返回值】

| 返回值 |               描述 |
| :----: | :-----------------|
|   0    |               成功 |
|  非0   | 失败，参见错误码。 |

【注意事项】
1. 区域必须已创建；
2. 图片buffer结构体指针及显示属性结构体指针不能为空；

【参考代码】
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
【函数声明】
```c
int32_t HB_RGN_SetDisplayLevel(RGN_HANDLE Handle, const RGN_CHN_S *pstChn, uint32_t osd_level);
```
【功能描述】
> 设置区域的显示层级；

【参数描述】

| 参数名称  |                    描述                     |
| :-------: | :-----------------------------------------: |
|  Handle   | 区域句柄号。取值范围：[0, RGN_HANDLE_MAX)。 |
|  pstChn   |              通道结构体指针。               |
| osd_level |       区域在通道上的显示层级[0, 3]。        |

【返回值】

| 返回值 | 描述 |
| :----: | ---: |
|   0    | 成功 |
|  非0   | 失败 |

【注意事项】
1. 区域必须已创建;
2. 通道结构体指针不能为空；
3. 设置等级范围为0-3，其中0默认硬件处理，如果超出硬件处理的个数或通道不支持则改为软件处理，1-3由软件处理；
4. 同一通道上不同区域可设置不同显示等级；

【参考代码】
> 请参见HB_RGN_AttachToChn/HB_RGN_DetachFromChn举例

## 数据结构
### RGN_SIZE_S
【结构定义】
```c
typedef struct HB_RGN_SIZE_ATTR_S{
    uint32_t u32Width;
    uint32_t u32Height;
} RGN_SIZE_S;
```
【功能描述】
> 定义大小信息的结构体

【成员说明】

|   成员    | 含义  |
| :-------: | :---: |
| u32Width  | 宽度  |
| u32Height | 高度  |

### RGN_POINT_S
【结构定义】
```c
typedef struct HB_RGN_POINT_ATTR_S {
    uint32_t u32X;
    uint32_t u32Y;
} RGN_POINT_S;
```
【功能描述】
> 定义坐标信息的结构体

【成员说明】

| 成员  |  含义  |
| :---: | :----: |
| u32X  | 横坐标 |
| u32Y  | 纵坐标 |

### RGN_RECT_S
【结构定义】
```c
typedef struct HB_RGN_RECT_ATTR_S {
    uint32_t u32X;
    uint32_t u32Y;
    uint32_t u32Width;
    uint32_t u32Height;
} RGN_RECT_S;
```
【功能描述】
> 定义矩形信息的结构体

【成员说明】

|   成员    |  含义  |
| :-------: | :----: |
|   u32X    | 横坐标 |
|   u32Y    | 纵坐标 |
| u32Width  |  宽度  |
| u32Height |  高度  |

### RGN_OVERLAY_S
【结构定义】
```c
typedef struct HB_RGN_OVERLAY_ATTR_S {
    RGN_PIXEL_FORMAT_E enPixelFmt;
    RGN_FONT_COLOR_E enBgColor;
    RGN_SIZE_S stSize;
} RGN_OVERLAY_S;
```
【功能描述】
> 定义叠加区域属性的结构体

【成员说明】

|    成员    | 含义                                                                                                           |
| :--------: | :------------------------------------------------------------------------------------------------------------- |
| enPixelFmt | 像素格式                                                                                                       |
| enBgColor  | 位图的背景色                                                                                                   |
|   stSize   | 区域大小<br/>PIXEL_FORMAT_VGA_4:<br/>最小宽为32，最小高为2<br/>PIXEL_FORMAT_YUV420SP:<br/>最小宽为2，最小高为2 |

### RGN_ATTR_S
【结构定义】
```c
typedef struct HB_RGN_ATTR_S {
    RGN_TYPE_E enType;
    RGN_OVERLAY_S stOverlayAttr;
} RGN_ATTR_S;
```
【功能描述】
> 定义区域信息的结构体

【成员说明】

|     成员      |      含义      |
| :-----------: | :------------: |
|    enType     |    区域类型    |
| stOverlayAttr | 叠加区域的属性 |

### RGN_CHN_S
【结构定义】
```c
typedef struct HB_RGN_CHN_S
{
    uint32_t s32PipelineId;
    int32_t enChnId;
} RGN_CHN_S;
```
【功能描述】
>定义数据流通道结构体

【成员说明】

|     成员      |             含义             |
| :-----------: | :--------------------------: |
| s32PipelineId |          pipelineID          |
|    enChnId    | 通道ID，范围[0，CHN_MAX_NUM) |

### RGN_OVERLAY_CHN_S
【结构定义】
```c
typedef struct HB_RGN_OVERLAY_CHN_ATTR_S {
    RGN_POINT_S stPoint;
} RGN_OVERLAY_CHN_S;
```
【功能描述】
> 定义叠加区域显示属性的结构体

【成员说明】

|  成员   |   含义   |
| :-----: | :------: |
| stPoint | 区域位置 |

### RGN_COVER_CHN_S
【结构定义】
```c
typedef struct HB_RGN_COVER_CHN_ATTR_S {
    RGN_RECT_S stRect;
    uint32_t u32Color;
} RGN_COVER_CHN_S;
```
【功能描述】
> 定义遮挡区域显示属性的结构体
【成员说明】

|   成员   |                 含义                  |
| :------: | :-----------------------------------: |
|  stRect  | 区域位置，宽高，最小宽为32，最小高为2 |
| u32Color |               区域颜色                |

### RGN_CHN_U
【结构定义】
```c
typedef union HB_RGN_CHN_ATTR_U {
    RGN_OVERLAY_CHN_S stOverlayChn;
    RGN_COVER_CHN_S stCoverChn;
} RGN_CHN_U;
```
【功能描述】
> 定义区域通道显示属性的联合体
【成员说明】

|     成员     |       含义       |
| :----------: | :--------------: |
| stOverlayChn | 叠加区域显示属性 |
|  stCoverChn  | 遮挡区域显示属性 |

### RGN_CANVAS_S
【结构定义】
```c
typedef struct HB_RGN_CANVAS_INFO_S {
    void *pAddr;
    RGN_SIZE_S stSize;
    RGN_PIXEL_FORMAT_E enPixelFmt;
} RGN_CANVAS_S;
```
【功能描述】
> 定义画布信息的结构体

【成员说明】

|    成员    |   含义   |
| :--------: | :------: |
|   pAddr    | 画布地址 |
|   stSize   | 画布大小 |
| enPixelFmt | 像素格式 |

### RGN_CHN_ATTR_S
【结构定义】
```c
typedef struct HB_RGN_CHN_ATTR_S {
    bool bShow;
    bool bInvertEn;
    RGN_CHN_U unChnAttr;
} RGN_CHN_ATTR_S;
```
【功能描述】
> 定义区域通道显示属性的结构体
【成员说明】

|   成员    |     含义     |
| :-------: | :----------: |
|   bShow   | 区域是否显示 |
| bInvertEn | 反色是否使能 |
| unChnAttr | 区域显示属性 |

### RGN_BITMAP_S
【结构定义】
```c
typedef struct HB_RGN_BITMAP_ATTR_S {
    RGN_PIXEL_FORMAT_E enPixelFormat;
    RGN_SIZE_S stSize;
    void *pAddr;
} RGN_BITMAP_S;
```
【功能描述】
> 定义位图属性的结构体

【成员说明】

|     成员      |   含义   |
| :-----------: | :------: |
| enPixelFormat | 像素格式 |
|    stSize     | 位图大小 |
|     pAddr     | 位图地址 |

### RGN_DRAW_WORD_S
【结构定义】
```c
typedef struct HB_RGN_DRAW_WORD_PARAM_S {
    void *pAddr;
    RGN_SIZE_S stSize;
    RGN_POINT_S stPoint;
    uint8_t *pu8Str;
    RGN_FONT_COLOR_E enFontColor;
    RGN_FONT_SIZE_E enFontSize;
    bool bFlushEn;
} RGN_DRAW_WORD_S;
```
【功能描述】
> 定义绘制文字参数的结构体

【成员说明】

|    成员     |                 含义                  |
| :---------: | :-----------------------------------: |
|    pAddr    |           绘制文字目标地址            |
|   stSize    |            目标地址的尺寸             |
|   stPoint   |          文字在位图中的位置           |
|   pu8Str    | 绘制的字符串，仅支持GBK编码格式的字符 |
| enFontColor |               字体颜色                |
| enFontSize  |               字体大小                |
|  bFlushEn   |        绘制时是否清空当前地址         |

### RGN_DRAW_LINE_S
【结构定义】
```c
typedef struct HB_RGN_DRAW_LINE_PARAM_S {
    void *pAddr;
    RGN_SIZE_S stSize;
    RGN_POINT_S stStartPoint;
    RGN_POINT_S stEndPoint;
    uint32_t u32Thick;
    uint32_t u32Color;
    bool bFlushEn;
} RGN_DRAW_LINE_S;
```
【功能描述】
> 定义绘制线条参数结构体

【成员说明】

|     成员     |          含义          |
| :----------: | :--------------------: |
|    pAddr     |    绘制线条目标地址    |
|    stSize    |       地址的尺寸       |
| stStartPoint |        开始坐标        |
|  stEndPoint  |        结束坐标        |
|   u32Thick   |        线条宽度        |
|   u32Color   |        线条颜色        |
|   bFlushEn   | 绘制时是否清空当前地址 |
（当使用批量绘制线条时则只有数组中第一个结构体的bFlushEn属性有效）

### RGN_STA_S
【结构定义】
```c
typedef struct HB_RGN_STA_ATTR_S {
	uint8_t u8StaEn;
	uint16_t u16StartX;
	uint16_t u16StartY;
	uint16_t u16Width;
	uint16_t u16Height;
} RGN_STA_S;
```
【功能描述】
> 定义亮度统计区域属性

【成员说明】

|   成员    |        含义         |
| :-------: | :-----------------: |
|  u8StaEn  |    区域是否使能     |
| u16StartX | 区域起始位置的X坐标 |
| u16StartY | 区域起始位置的Y坐标 |
| u16Width  | 区域宽度，（2,255） |
| u16Height | 区域高度，（2,255） |

### RGN_TYPE_E
【结构定义】
```c
typedef enum HB_RGN_TYPE_PARAM_E
{
    OVERLAY_RGN,
    COVER_RGN
} RGN_TYPE_E;
```
【功能描述】
> 定义区域类型

【成员说明】

|    成员     |     含义     |
| :---------: | :----------: |
| OVERLAY_RGN | 叠加区域类型 |
|  COVER_RGN  | 遮挡区域类型 |

### RGN_CHN_ID_E
【结构定义】
```c
typedef enum HB_RGN_CHN_ID_ATTR_E
{
	CHN_US,
    CHN_DS0,
    CHN_DS1,
	CHN_DS2,
    CHN_DS3,
    CHN_DS4,
    CHN_GRP,
    CHANNEL_MAX_NUM
}RGN_CHN_ID_E;
```
【功能描述】
> 定义通道ID

【成员说明】

|      成员       |                             含义                             |
| :-------------: | :----------------------------------------------------------: |
|     CHN_US      |                            US通道                            |
|     CHN_DS0     |                           DS0通道                            |
|     CHN_DS1     |                           DS1通道                            |
|     CHN_DS2     |                           DS2通道                            |
|     CHN_DS3     |                           DS3通道                            |
|     CHN_DS4     |                           DS4通道                            |
|     CHN_GRP     | GROUP通道，如果使用此通道，因为group操作是在进ipu之前的，所以如果vps做了放大操作，那么osd也会跟着放大 |
| CHANNEL_MAX_NUM |                           通道数量                           |

### RGN_FONT_SIZE_E
【结构定义】
```c
typedef enum HB_RGN_FONT_SIZE_ATTR_E
{
	FONT_SIZE_SMALL = 1,
	FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
	FONT_SIZE_EXTRA_LARGE
}RGN_FONT_SIZE_E;
```
【功能描述】
> 定义字体大小

【成员说明】

|         成员          |       含义        |
| :-------------------: | :---------------: |
|    FONT_SIZE_SMALL    |  小号字体，16*16  |
|   FONT_SIZE_MEDIUM    |  中号字体，32*32  |
|    FONT_SIZE_LARGE    |  大号字体，48*48  |
| FONT_SIZE_EXTRA_LARGE | 超大号字体，64*64 |

### RGN_FONT_COLOR_E
【结构定义】
```c
typedef enum HB_RGN_FONT_COLOR_ATTR_E
{
	FONT_COLOR_WHITE = 1,
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
    FONT_KEY_COLOR = 16
}RGN_FONT_COLOR_E;
```
【功能描述】
> 定义字体颜色，当调用完HB_RGN_SetColorMap后此枚举失效；

【成员说明】

|         成员         |         含义         |
| :------------------: | :------------------: |
|   FONT_COLOR_WHITE   |         白色         |
|   FONT_COLOR_BLACK   |         黑色         |
|   FONT_COLOR_GREY    |         灰色         |
|   FONT_COLOR_BLUE    |         蓝色         |
|   FONT_COLOR_GREEN   |         绿色         |
|  FONT_COLOR_YELLOW   |         黄色         |
|   FONT_COLOR_BROWN   |         棕色         |
|  FONT_COLOR_ORANGE   |        橘黄色        |
|  FONT_COLOR_PURPLE   |         紫色         |
|   FONT_COLOR_PINK    |        粉红色        |
|    FONT_COLOR_RED    |         红色         |
|   FONT_COLOR_CYAN    |         青色         |
| FONT_COLOR_DARKBLUE  |        深蓝色        |
| FONT_COLOR_DARKGREEN |        深绿色        |
|  FONT_COLOR_DARKRED  |        深红色        |
|    FONT_KEY_COLOR    | 不叠加，使用图片原色 |

### RGN_PIXEL_FORMAT_E
【结构定义】
```c
typedef enum HB_PIXEL_FORMAT_ATTR_E
{
    PIXEL_FORMAT_VGA_4,
    PIXEL_FORMAT_YUV420SP
} RGN_PIXEL_FORMAT_E;
```
【功能描述】
> 定义像素格式

【成员说明】

|         成员          |                                                  含义                                                   |
| :-------------------: | :-----------------------------------------------------------------------------------------------------: |
|  PIXEL_FORMAT_VGA_4   | 4bit的16色像素格式<br/>[此像素格式为每个像素占4个bit（0-15），作为颜色索引一一之后对应RGN_FONT_COLOR_E] |
| PIXEL_FORMAT_YUV420SP |                                            YUV420SP像素格式                                             |

### RGN_HANDLE
【结构定义】
```c
typedef int32_t RGN_HANDLE;
```
【功能描述】
> 定义区域句柄

### RGN_HANDLE_MAX
【结构定义】
```c
#define RGN_HANDLE_MAX 256
```
【功能描述】
> 定义最大区域句柄数量

### RGN_HANDLEGROUP
【结构定义】
```c
typedef int32_t RGN_HANDLEGROUP;
```
【功能描述】
> 定义批处理组号

### RGN_GROUP_MAX
【结构定义】
```c
#define RGN_GROUP_MAX 8
```
【功能描述】
> 定义最大批处理数量

## 错误码
RGN错误码如下表：

|   错误码   | 宏定义                       | 描述             |
| :--------: | :--------------------------- | :--------------- |
| -268762113 | HB_ERR_RGN_INVALID_CHNID     | 通道ID错误       |
| -268762114 | HB_ERR_RGN_ILLEGAL_PARAM     | 输入参数错误     |
| -268762115 | HB_ERR_RGN_EXIST             | 区域Handle已存在 |
| -268762116 | HB_ERR_RGN_UNEXIST           | 区域Handle不存在 |
| -268762117 | HB_ERR_RGN_NULL_PTR          | 空指针           |
| -268762118 | HB_ERR_RGN_NOMEM             | 内存不足         |
| -268762119 | HB_ERR_RGN_OPEN_FILE_FAIL    | 打开字库文件失败 |
| -268762120 | HB_ERR_RGN_INVALID_OPERATION | 无效操作         |
| -268762121 | HB_ERR_RGN_PROCESS_FAIL	OSD  | 处理失败         |

## 参考代码
OSD部分示例代码可以参考，[sample_osd](./multimedia_samples#sample_osd)。