---
sidebar_position: 8
---

# 7.3.8 视频输出

## 概述
VOT（视频输出）模块主动从内存中读取图像和图形数据，并通过相应的显示输出设备输出图像。芯片支持的显示/回写设备、视频层和图形层情况如下表所示。

![image-20220329222211836](./image/video_output/image-20220329222211836.png)

## 功能描述

### 基本概念
- 高清显示设备

  SDK将高清设备标示为DHVx，其中x为索引号，从0开始取值，标示第几路高清设备。X3有一个高清设备DHV0。

- 视频层

  - 对于固定在每个显示设备上的视频层，SDK对应采取VHVx来标示。X3 DHV0 有一个视频层VHV0。
  - VHV0 支持放大，支持2个通道
  - X3 输出接口支持 RGB、BT1120/BT656、MIPI，三种接口都支持最大输出时序 1080P@60fps

- 视频回写

​		回写设备称之为WD。回写功能：X3只支持设备级的回写，捕获设备级输出的视频数据，可用于显示和编码。

- 通道

​		通道由视频层管理，X3每个视频层支持2个通道。

- 图形层

​		X3有2个图形层，固定绑定到DHV0。

- 输入输出数据格式

​		VOT支持输入和输出指定格式的数据，其中输出是指回写数据到DDR。X3支持的输入输出数据格式见下表。

| 输入格式 | 输出格式 |
| :----------------: | :----------------: |
| FORMAT_YUV422_UYVY | FORMAT_YUV422_UYVY |
| FORMAT_YUV422_VYUY | FORMAT_YUV422_VYUY |
| FORMAT_YUV422_YVYU | FORMAT_YUV422_YVYU |
| FORMAT_YUV422_YUYV | FORMAT_YUV422_YUYV |
| FORMAT_YUV422SP_UV | FORMAT_YUV420SP_UV |
| FORMAT_YUV422SP_VU | FORMAT_YUV420SP_VU |
| FORMAT_YUV420SP_UV |    FORMAT_BGR0     |
| FORMAT_YUV420SP_VU |                    |
| FORMAT_YUV422P_UV  |                    |
| FORMAT_YUV422P_VU  |                    |
| FORMAT_YUV420P_UV  |                    |


## API参考

视频输出（VOT）实现启用视频输出设备或通道、发送视频数据到输出通道等功能。

VOT提供如下API：

```C
HB_VOT_SetPubAttr：设置视频输出设备公共属性。
HB_VOT_GetPubAttr：获取视频输出设备公共属性。
HB_VOT_Enable：启用视频输出设备。
HB_VOT_Disable：关闭视频输出设备。
HB_VOT_SetLcdBackLight：设置LCD背光
HB_VOT_SetVideoLayerAttr：设置视频层属性。
HB_VOT_GetVideoLayerAttr：获取视频层属性。
HB_VOT_EnableVideoLayer：使能视频层。
HB_VOT_DisableVideoLayer：禁止视频层。
HB_VOT_SetVideoLayerCSC：设置视频层CSC。
HB_VOT_GetVideoLayerCSC：获取视频层CSC。
HB_VOT_SetVideoLayerUpScale：设置视频层放大参数
HB_VOT_GetVideoLayerUpScale：获取视频层放大参数
HB_VOT_BatchBegin：设置视频层属性批处理开始。
HB_VOT_BatchEnd：设置视频层属性批处理结束。
HB_VOT_GetScreenFrame：获取设备输出图像。
HB_VOT_ReleaseScreenFrame：释放设备输出图像。
HB_VOT_SetChnAttr：设置视频输出通道属性。
HB_VOT_GetChnAttr：获取视频输出通道属性。
HB_VOT_SetChnAttrEx：设置视频输出通道高级参数。
HB_VOT_GetChnAttrEx：设置视频输出通道高级参数。
HB_VOT_EnableChn：使能视频输出通道。
HB_VOT_DisableChn：禁止视频输出通道。
HB_VOT_SetChnCrop：设置通道裁剪属性。
HB_VOT_GetChnCrop：获取通道裁剪属性。
HB_VOT_SetChnDisplayPosition：设置通道显示位置。
HB_VOT_GetChnDisplayPosition：获取通道显示位置。
HB_VOT_SetChnFrameRate：设置通道显示帧率。
HB_VOT_GetChnFrameRate：获取通道显示帧率。
HB_VOT_ShowChn：显示通道图像。
HB_VOT_HideChn：隐藏通道图像。
HB_VOT_SendFrame：发送输出图像。
HB_VOT_ClearChnBuf：清空图像缓存。
HB_VOT_EnableWB：使能回写。
HB_VOT_DisableWB：禁止回写。
HB_VOT_SetWBAttr：设置回写属性。
HB_VOT_GetWBAttr：获取回写属性。
HB_VOT_GetWBFrame：获取回写图像。
HB_VOT_ReleaseWBFrame：释放回写图像。
HB_VOT_ShutDownHDMI：关闭HDMI输出图像到设备，显示模块仍然正常工作。
HB_VOT_StartHDMI：启用HDMI输出图像到设备。
```

### HB_VOT_SetPubAttr
【函数声明】
```C
int HB_VOT_SetPubAttr(uint8_t dev, const VOT_PUB_ATTR_S *pstPubAttr);
```
【功能描述】
> 设置视频输出公共属性。

【参数描述】

|  参数名称  |             描述              | 输入/输出 |
| :--------: | :---------------------------: | :-------: |
|    dev     | 视频输出设备id。取值范围：0。 |   输入    |
| pstPubAttr |    视频输出设备公共属性。     |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见 HB_VOT_Enable

### HB_VOT_GetPubAttr
【函数声明】
```C
int HB_VOT_GetPubAttr(uint8_t dev, VOT_PUB_ATTR_S *pstPubAttr);
```
【功能描述】
> 获取视频输出公共属性。

【参数描述】

|  参数名称  |                描述                | 输入/输出 |
| :--------: | :--------------------------------: | :-------: |
|    dev     | 视频输出设备id。<br/>取值范围：0。 |   输入    |
| pstPubAttr |       视频输出设备公共属性。       |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见 [HB_VOT_Enable](#HB_VOT_Enable)

### HB_VOT_Enable
【函数声明】
```C
int HB_VOT_Enable(uint8_t dev);
```
【功能描述】
> 启用视频输出设备。

【参数描述】

| 参数名称 |                描述                | 输入/输出 |
| :------: | :--------------------------------: | :-------: |
|   dev    | 视频输出设备id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
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
【函数声明】
```C
int HB_VOT_Disable(uint8_t dev);
```
【功能描述】
> 禁用视频输出设备。

【参数描述】

| 参数名称 |                描述                | 输入/输出 |
| :------: | :--------------------------------: | :-------: |
|   dev    | 视频输出设备id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见 [HB_VOT_Enable](#HB_VOT_Enable)

### HB_VOT_SetLcdBackLight
【函数声明】
```C
int HB_VOT_SetLcdBackLight (uint8_t dev, uint32_t backlight);
```
【功能描述】
> 设置LCD背光亮度。

【参数描述】

| 参数名称  |                             描述                             | 输入/输出 |
| :-------: | :----------------------------------------------------------: | :-------: |
|    dev    |              视频输出设备id。<br/>取值范围：0。              |   输入    |
| backlight | 背光亮度值。<br/>取值范围0-10，值越大越亮。<br/>亮度值为0，则屏幕全黑。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_SetVideoLayerAttr
【函数声明】
```C
int HB_VOT_SetVideoLayerAttr(uint8_t layer, const VOT_VIDEO_LAYER_ATTR_S *pstLayerAttr);
```
【功能描述】
> 设置视频层属性。

【参数描述】

|   参数名称   |                描述                 | 输入/输出 |
| :----------: | :---------------------------------: | :-------: |
|    layer     | 视频输出视频层id。<br/>取值范围：0. |   输入    |
| pstLayerAttr |        视频输出视频层属性。         |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 需要先使能设备

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerAttr
【函数声明】
```C
int HB_VOT_GetVideoLayerAttr(uint8_t layer,  VOT_VIDEO_LAYER_ATTR_S *pstLayerAttr);
```
【功能描述】
> 获取视频层属性。

【参数描述】

|   参数名称   |                描述                 | 输入/输出 |
| :----------: | :---------------------------------: | :-------: |
|    layer     | 视频输出视频层id。<br/>取值范围：0. |   输入    |
| pstLayerAttr |        视频输出视频层属性。         |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_EnableVideoLayer
【函数声明】
```C
int HB_VOT_EnableVideoLayer(uint8_t layer);
```
【功能描述】
> 使能视频层。

【参数描述】

| 参数名称 |                 描述                 | 输入/输出 |
| :------: | :----------------------------------: | :-------: |
|  layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
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
【函数声明】
```C
int HB_VOT_DisableVideoLayer(uint8_t layer);
```
【功能描述】
> 禁止视频层。

【参数描述】

| 参数名称 |                 描述                 | 输入/输出 |
| :------: | :----------------------------------: | :-------: |
|  layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetVideoLayerCSC
【函数声明】
```C
int HB_VOT_SetVideoLayerCSC(uint8_t layer, const VOT_CSC_S *pstVideoCSC);
```
【功能描述】
> 设置视频层输出图像效果。

【参数描述】

|  参数名称   |                 描述                 | 输入/输出 |
| :---------: | :----------------------------------: | :-------: |
|    layer    | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstVideoCSC |        视频输出图像输出效果。        |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerCSC
【函数声明】
```C
int HB_VOT_GetVideoLayerCSC(uint8_t layer, VOT_CSC_S *pstVideoCSC);
```
【功能描述】
> 获取视频层输出图像效果。

【参数描述】

|  参数名称   |                 描述                 | 输入/输出 |
| :---------: | :----------------------------------: | :-------: |
|    layer    | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstVideoCSC |        视频输出图像输出效果。        |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetVideoLayerUpScale
【函数声明】
```C
int HB_VOT_SetVideoLayerUpScale(uint8_t layer, const VOT_UPSCALE_ATTR_S *pstUpScaleAttr);
```
【功能描述】
> 设置视频层放大属性。

【参数描述】

|    参数名称    |                 描述                 | 输入/输出 |
| :------------: | :----------------------------------: | :-------: |
|     layer      | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstUpScaleAttr |           视频层放大属性。           |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetVideoLayerUpScale
【函数声明】
```C
int HB_VOT_GetVideoLayerUpScale(uint8_t layer, VOT_UPSCALE_ATTR_S *pstUpScaleAttr);
```
【功能描述】
> 获取视频层放大属性。

【参数描述】

|    参数名称    |                 描述                 | 输入/输出 |
| :------------: | :----------------------------------: | :-------: |
|     layer      | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstUpScaleAttr |           视频层放大属性。           |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_BatchBegin
【函数声明】
```C
int HB_VOT_BatchBegin(uint8_t layer);
```
【功能描述】
> 视频层的通道设置属性的开始。

【参数描述】

| 参数名称 |                 描述                 | 输入/输出 |
| :------: | :----------------------------------: | :-------: |
|  layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_BatchEnd
【函数声明】
```C
int HB_VOT_BatchEnd(uint8_t layer);
```
【功能描述】
> 视频层的通道设置属性的结束。

【参数描述】

| 参数名称 |                 描述                 | 输入/输出 |
| :------: | :----------------------------------: | :-------: |
|  layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_GetScreenFrame
【函数声明】
```C
int HB_VOT_GetScreenFrame(uint8_t layer, void *pstVFrame, int millisec);
```
【功能描述】
> 获取输出屏幕图像数据。

【参数描述】

| 参数名称  |                 描述                 | 输入/输出 |
| :-------: | :----------------------------------: | :-------: |
|   layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstVFrame |        输出屏幕图像数据信息。        |   输出    |
| millisec  |          超时时间。单位：ms          |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 需要在使能设备、视频层、通道后使用。

【参考代码】
```C
    int ret;
    hb_vio_buffer_t stVFrame;

    ret = HB_VOT_GetScreenFrame(0, &stVFrame, 0);
    if (ret != ) {
        printf("HB_VOT_GetScreenFrame failed.\n");
    }

    ret = HB_VOT_ReleaseScreenFrame(0, & stVFrame, 0);
    if (ret != ) {
        printf("HB_VOT_ReleaseScreenFrame failed.\n");
    }
```

### HB_VOT_ReleaseScreenFrame
【函数声明】
```C
int HB_VOT_ReleaseScreenFrame(uint8_t layer, const void *pstVFrame);
```
【功能描述】
> 释放输出屏幕图像数据。

【参数描述】

| 参数名称  |                 描述                 | 输入/输出 |
| :-------: | :----------------------------------: | :-------: |
|   layer   | 视频输出视频层id。<br/>取值范围：0。 |   输入    |
| pstVFrame |        输出屏幕图像数据信息。        |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_ReleaseScreenFrame

### HB_VOT_SetChnAttr
【函数声明】
```C
int HB_VOT_SetChnAttr(uint8_t layer, uint8_t chn, const VOT_CHN_ATTR_S *pstChnAttr);
```
【功能描述】
> 设置视频输出通道属性。

【参数描述】

|  参数名称  |                                      描述                                      | 输入/输出 |
| :--------: | :----------------------------------------------------------------------------: | :-------: |
|   layer    |                        视频输出视频层id。取值范围：0。                         |   输入    |
|    chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstChnAttr |                               视频输出通道属性。                               |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetChnAttr
【函数声明】
```C
int HB_VOT_GetChnAttr(uint8_t layer, uint8_t chn, VOT_CHN _ATTR_S *pstChnAttr);
```
【功能描述】
> 获取视频输出通道属性。

【参数描述】

|  参数名称  |                                      描述                                      | 输入/输出 |
| :--------: | :----------------------------------------------------------------------------: | :-------: |
|   layer    |                        视频输出视频层id。取值范围：0。                         |   输入    |
|    chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstChnAttr |                               视频输出通道属性。                               |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetChnAttrEx
【函数声明】
```C
int HB_VOT_SetChnAttrEx(uint8_t layer, uint8_t chn, const VOT_CHN_ATTR_EX_S *pstChnAttrEx);
```
【功能描述】
> 设置视频输出通道高级属性。

【参数描述】

|   参数名称   |                                      描述                                      | 输入/输出 |
| :----------: | :----------------------------------------------------------------------------: | :-------: |
|    layer     |                        视频输出视频层id。取值范围：0。                         |   输入    |
|     chn      | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstChnAttrEx |                             视频输出通道高级属性。                             |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetChnAttrEx
【函数声明】
```C
int HB_VOT_GetChnAttrEx(uint8_t layer, uint8_t chn, VOT_CHN _ATTR_EX_S *pstChnAttrEx);
```
【功能描述】
> 获取视频输出通道高级属性。

【参数描述】

|   参数名称   |                                      描述                                      | 输入/输出 |
| :----------: | :----------------------------------------------------------------------------: | :-------: |
|    layer     |                        视频输出视频层id。取值范围：0。                         |   输入    |
|     chn      | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstChnAttrEx |                             视频输出通道高级属性。                             |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_EnableChn
【函数声明】
```C
int HB_VOT_EnableChn(uint8_t layer, uint8_t chn);
```
【功能描述】
> 启用视频输出通道。

【参数描述】

| 参数名称 |                                      描述                                      | 输入/输出 |
| :------: | :----------------------------------------------------------------------------: | :-------: |
|  layer   |                        视频输出视频层id。取值范围：0。                         |   输入    |
|   chn    | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_DisableChn
【函数声明】
```C
int HB_VOT_DisableChn(uint8_t layer, uint8_t chn);
```
【功能描述】
> 禁用视频输出通道。

【参数描述】

| 参数名称 | 描述                                                                           | 输入/输出 |
| :------: | :----------------------------------------------------------------------------- | :-------: |
|  layer   | 视频输出视频层id。取值范围：0。                                                |   输入    |
|   chn    | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetChnCrop
【函数声明】
```C
int HB_VOT_SetChnCrop(uint8_t layer, uint8_t chn,  const VOT_CROP_INFO_S *pstCropInfo);
```
【功能描述】
> 设置视频输出通道裁剪属性。

【参数描述】

|  参数名称   | 描述                                                                           | 输入/输出 |
| :---------: | :----------------------------------------------------------------------------- | :-------: |
|    layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|     chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstCropInfo | 视频输出通道裁剪属性。                                                         |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetChnCrop
【函数声明】
```C
int HB_VOT_GetChnCrop(uint8_t layer, uint8_t chn, VOT_CROP_INFO_S *pstCropInfo);
```
【功能描述】
> 获取视频输出通道裁剪属性。

【参数描述】

|  参数名称   | 描述                                                                           | 输入/输出 |
| :---------: | :----------------------------------------------------------------------------- | :-------: |
|    layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|     chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstCropInfo | 视频输出通道裁剪属性。                                                         |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetChnDisplayPosition
【函数声明】
```C
int HB_VOT_SetChnDisplayPosition(uint8_t layer, uint8_t chn, const POINT_S *pstDispPos);
```
【功能描述】
> 设置视频输出通道显示坐标。

【参数描述】

|  参数名称  | 描述                                                                           | 输入/输出 |
| :--------: | :----------------------------------------------------------------------------- | :-------: |
|   layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|    chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstDispPos | 视频输出通道显示坐标。                                                         |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_GetChnDisplayPosition
【函数声明】
```C
int HB_VOT_GetChnDisplayPosition(uint8_t layer, uint8_t chn, POINT_S *pstDispPos);
```
【功能描述】
> 获取视频输出通道显示坐标。

【参数描述】

|  参数名称  | 描述                                                                           | 输入/输出 |
| :--------: | :----------------------------------------------------------------------------- | :-------: |
|   layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|    chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pstDispPos | 视频输出通道显示坐标。                                                         |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableVideoLayer

### HB_VOT_SetChnFrameRate
【函数声明】
```C
int HB_VOT_SetChnFrameRate(uint8_t layer, uint8_t chn, int frame_rate);
```
【功能描述】
> 设置视频通道显示帧率。

【参数描述】

|  参数名称  | 描述                                                                           | 输入/输出 |
| :--------: | :----------------------------------------------------------------------------- | :-------: |
|   layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|    chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| frame_rate | 视频通道显示帧率。                                                             |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_GetChnFrameRate
【函数声明】
```C
int HB_VOT_GetChnFrameRate(uint8_t layer, uint8_t chn, int *pframe_rate);
```
【功能描述】
> 获取视频通道显示帧率。

【参数描述】

|  参数名称   | 描述                                                                           | 输入/输出 |
| :---------: | :----------------------------------------------------------------------------- | :-------: |
|    layer    | 视频输出视频层id。取值范围：0。                                                |   输入    |
|     chn     | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| pframe_rate | 视频通道显示帧率。                                                             |   输出    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_ShowChn
【函数声明】
```C
int HB_VOT_ShowChn(uint8_t layer, uint8_t chn);
```
【功能描述】
> 显示指定通道。

【参数描述】

| 参数名称 |                                      描述                                      | 输入/输出 |
| :------: | :----------------------------------------------------------------------------: | :-------: |
|  layer   |                        视频输出视频层id。取值范围：0。                         |   输入    |
|   chn    | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_HideChn
【函数声明】
```C
int HB_VOT_HideChn(uint8_t layer, uint8_t chn);
```
【功能描述】
> 隐藏指定通道。

【参数描述】

| 参数名称 | 描述                                                                           | 输入/输出 |
| :------: | :----------------------------------------------------------------------------- | :-------: |
|  layer   | 视频输出视频层id。取值范围：0。                                                |   输入    |
|   chn    | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_SendFrame
【函数声明】
```C
int HB_VOT_SendFrame(uint8_t layer, uint8_t chn, void *pstVFrame, int millisec);
```
【功能描述】
> 将视频图像送入指定输出通道显示。

【参数描述】

| 参数名称  | 描述                                                      | 输入/输出 |
| :-------: | :-------------------------------------------------------- | :-------: |
|   layer   | 视频输出视频层id。取值范围：0。                           |   输入    |
|    chn    | 视频输出通道id。取值范围：[0, 2)。<br/>0、1表示视频通道。 |   输入    |
| pstVFrame | 视频数据信息。                                            |   输入    |
| millisec  | 超时时间。单位：ms                                        |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_ClearChnBuf
【函数声明】
```C
int HB_VOT_ClearChnBuf(uint8_t layer, uint8_t chn, HB_BOOL bClrAll);
```
【功能描述】
> 清空指定输出通道的缓存buffer数据。

【参数描述】

| 参数名称 | 描述                                                                           | 输入/输出 |
| :------: | :----------------------------------------------------------------------------- | :-------: |
|  layer   | 视频输出视频层id。取值范围：0。                                                |   输入    |
|   chn    | 视频输出通道id。取值范围：[0, 4)。<br/>0、1表示视频通道；<br/>2、3是图形通道。 |   输入    |
| bClrAll  | 是否将通道buffer中的数据清空。                                                 |   输入    |

【返回值】

| 返回值 |  描述  |
| :----: | :----: |
|   0    | 成功。 |
|  非0   | 失败。 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_BindVps
【函数声明】
```C
int HB_VOT_BindVps(uint8_t vpsGroup, uint8_t vpsChn, uint8_t layer, uint8_t chn);
```
【功能描述】
> 视频输出的输入源绑定vps模块的输出。

【参数描述】

| 参数名称 |                  描述                   | 输入/输出 |
| :------: | :-------------------------------------: | :-------: |
| vpsGroup | 绑定的VPS模块的Group，取值范围：[0,4)。 |   输入    |
|  vpsChn  | 绑定的VPS模块的通道，取值范围：[0,39)。 |   输入    |
|  layer   |    绑定的VOT模块的layer，取值范围0。    |   输入    |
|   chn    | 绑定的VOT模块的通道，取值范围：[0,2)。  |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_EnableWB
【函数声明】
```C
int HB_VOT_EnableWB(VOT_WB votWb);
```
【功能描述】
> 使能视频输出设备的回写。

【参数描述】

| 参数名称 |              描述              | 输入/输出 |
| :------: | :----------------------------: | :-------: |
|  votWb   | 回写设备id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
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
【函数声明】
```C
int HB_VOT_DisableWB(VOT_WB votWb);
```
【功能描述】
> 禁止视频输出设备的回写。

【参数描述】

| 参数名称 |              描述              | 输入/输出 |
| :------: | :----------------------------: | :-------: |
|  votWb   | 回写设备id。<br/>取值范围：0。 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_GetWBAttr
【函数声明】
```C
int HB_VOT_GetWBAttr (VOT_WB votWb, VOT_WB_ATTR_S *pstWBAttr);
```
【功能描述】
> 使能视频输出设备的回写。

【参数描述】

| 参数名称  |              描述              | 输入/输出 |
| :-------: | :----------------------------: | :-------: |
|   votWb   | 回写设备id。<br/>取值范围：0。 |   输入    |
| pstWBAttr |            回写属性            |   输出    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_SetWBAttr
【函数声明】
```C
int HB_VOT_SetWBAttr (VOT_WB votWb, VOT_WB_ATTR_S *pstWBAttr);
```
【功能描述】
> 使能视频输出设备的回写。

【参数描述】

| 参数名称  |              描述              | 输入/输出 |
| :-------: | :----------------------------: | :-------: |
|   votWB   | 回写设备id。<br/>取值范围：0。 |   输入    |
| pstWBAttr |            回写属性            |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 见HB_VOT_EnableWB

### HB_VOT_GetWBFrame
【函数声明】
```C
int HB_VOT_GetWBFrame (VOT_WB votWb, void* pstVFrame, int millisec);
```
【功能描述】
> 使能视频输出设备的回写。

【参数描述】

| 参数名称  |                          描述                           | 输入/输出 |
| :-------: | :-----------------------------------------------------: | :-------: |
|   votWb   |             回写设备id。<br/>取值范围：0。              |   输入    |
| pstVFrame | 获取到的回写图像帧(传入的指针类型应为hb_vio_buffer_t *) |   输入    |
| millisec  |                  超时，本版本不可用。                   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_ReleaseWBFrame
【函数声明】
```C
int HB_VOT_ReleaseWBFrame (VOT_WB votWb, void* pstVFrame)；
```
【功能描述】
> 使能视频输出设备的回写。

【参数描述】

| 参数名称  |              描述              | 输入/输出 |
| :-------: | :----------------------------: | :-------: |
|   votWb   | 回写设备id。<br/>取值范围：0。 |   输入    |
| pstVFrame |       获取到的回写图像帧       |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VOT_ShutDownHDMI
【函数声明】
```C
int HB_VOT_ShutDownHDMI(void)；
```
【功能描述】
> 关闭HDMI输出到目标设备，例如显示器等，目标设备显示黑屏，但是显示硬件模块仍然正常工作。

【参数描述】

| 参数名称 | 描述  | 输入/输出 |
| :------: | :---: | :-------: |
|   void   |  空   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 限于HDMI显示时使用

【参考代码】

### HB_VOT_StartHDMI
【函数声明】
```C
int HB_VOT_StartHDMI (void)；
```
【功能描述】
> 使能HDMI输出图像到目标显示设备，需要与HB_VOT_ShutDownHDMI成对使用。

【参数描述】

| 参数名称 | 描述  | 输入/输出 |
| :------: | :---: | :-------: |
|   void   |  空   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 用于HB_VOT_ShutDownHDMI后，重新启动图像输出

【参考代码】

### API调用流程示例
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

## 数据结构

### HB_VOT_PUB_ATTR_S
【结构定义】
```C
typedef struct HB_VOT_PUB_ATTR_S {
    uint32_t u32BgColor;/* 设备背景色 RGB表示 */
    VOT_OUTPUT_MODE_E enOutputMode; /* VOT 接口类型 */
    VOT_INTF_SYNC_E enIntfSync; /* VOT接口时序类型 */
    VOT_SYNC_INFO_S stSyncInfo; /* VOT接口时序信息 */
} VOT_PUB_ATTR_S;
```
【功能描述】
> 定义视频输出公共属性

【成员说明】

|成员| 含义|
|:---|:---|
|u32BgColor |设备背景色|
|enOutputMode |Vo 接口类型|
|enIntfSync| 接口时序典型配置|
|stSyncInfo| 接口时序结构体|

在使能用户时序时，该结构体生效。

### HB_VOT_OUTPUT_MODE_E
【结构定义】
```C
typedef enum HB_VOT_OUTPUT_MODE_E {
    HB_VOT_OUTPUT_MIPI,
    HB_VOT_OUTPUT_BT1120,
    HB_VOT_OUTPUT_RGB888,
    HB_VOT_OUTPUT_BT656,
    HB_VOT_OUTPUT_MODE_BUTT,
} VOT_OUTPUT_MODE_E;
```
【功能描述】
> 定义视频输出模式

【成员说明】

|          成员           | 含义                     |
| :---------------------: | :----------------------- |
|   HB_VOT_OUTPUT_MIPI    | MIPI输出                 |
|  HB_VOT_OUTPUT_BT1120   | BT1120输出(用于HDMI显示) |
|  HB_VOT_OUTPUT_RGB888   | RGB输出                  |
| HB_VOT_OUTPUT_MODE_BUTT | 支持的输出方式总数       |

### HB_VOT_INTF_SYNC_E
【结构定义】
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
    VOT_OUTPUT_720P60，
    VOT_OUTPUT_720P50，
    VOT_OUTPUT_720P30，
    VOT_OUTPUT_720P25，
    VOT_OUTPUT_720P59_94，
    VOT_OUTPUT_720P29_97，
    VOT_OUTPUT_704x576_25，
    VOT_OUTPUT_704x480_30，
    VO_OUTPUT_USER, /* User timing. */
    VO_OUTPUT_BUTT
} VOT_INTF_SYNC_E;
```
【功能描述】
> 定义视频输出时序典型模式

【成员说明】

| 成员                  | 含义                |
| :-------------------- | :------------------ |
| VOT_OUTPUT_1920x1080  | 1920x1080           |
| VOT_OUTPUT_800x480    | 800x480             |
| VOT_OUTPUT_720x1280   | 720x1280            |
| VOT_OUTPUT_1080x1920  | 1080x1920           |
| VOT_OUTPUT_704x576    | 704x576             |
| VOT_OUTPUT_1080P60    | 1920x1080P@60fps    |
| VOT_OUTPUT_1080P50    | 1920x1080P@50fps    |
| VOT_OUTPUT_1080P30    | 1920x1080P@30fps    |
| VOT_OUTPUT_1080P25    | 1920x1080P@25fps    |
| VOT_OUTPUT_1080P59_94 | 1920x1080P@59.94fps |
| VOT_OUTPUT_1080P29_97 | 1920x1080P@29.97fps |
| VOT_OUTPUT_1080I60    | 1920x1080I@60fps    |
| VOT_OUTPUT_1080I50    | 1920x1080I@50fps    |
| VOT_OUTPUT_1080I59_94 | 1920x1080I@59.94fps |
| VOT_OUTPUT_720P60     | 1280x720P@60fps     |
| VOT_OUTPUT_720P50     | 1280x720P@50fps     |
| VOT_OUTPUT_720P30     | 1280x720P@30fps     |
| VOT_OUTPUT_720P25     | 1280x720P@25fps     |
| VOT_OUTPUT_720P59_94  | 1280x720P@59.94fps  |
| VOT_OUTPUT_720P29_97  | 1280x720P@29.97fps  |
| VOT_OUTPUT_704x576_25 | 704x576P@25fps      |
| VOT_OUTPUT_704x480_30 | 704x480P@30fps      |
| VO_OUTPUT_USER        | 用户自定义时序      |

### HB_VOT_SYNC_INFO_S
【结构定义】
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
【功能描述】
> 定义视频输出用户自定义时序

【成员说明】

|  成员   | 含义                             |
| :-----: | :------------------------------- |
|   hbp   | 行前沿                           |
|   hfp   | 行后沿                           |
|   hs    | 行同步信号                       |
|   vbp   | 帧前沿                           |
|   vfp   | 帧后沿                           |
|   vs    | 帧同步信号                       |
| vfp_cnt | 目前固定为0xa（bt656 field 为0） |
|  width  | 屏幕分辨率宽                     |
| height  | 屏幕分辨率高                     |

### HB_VOT_VIDEO_LAYER_ATTR_S
【结构定义】
```C
typedef struct HB_VOT_VIDEO_LAYER_ATTR_S {
    SIZE_S stImageSize;/* 视频层画布大小 */
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
    uint32_t rotate;
    uint32_t user_control_disp;
    uint32_t user_control_disp_layer1;
} VOT_VIDEO_LAYER_ATTR_S;
```
【功能描述】
> 定义视频层属性

【成员说明】

 | 成员                     | 含义                                     |
 | :----------------------- | :--------------------------------------- |
 | stImageSize              | 视频层画布大小                           |
 | big_endian               | 通道2，3输入图像格式大小端方式配置       |
 | display_addr_type        | 通道0显示类型                            |
 | display_cam_no           | 通道0显示源                              |
 | display_addr_type_layer1 | 通道1显示类型                            |
 | display_cam_no_layer1    | 通道1显示源                              |
 | dithering_flag           | dithering类型                            |
 | dithering_en             | dithering是否使能                        |
 | gamma_en                 | gamma是否使能                            |
 | hue_en                   | hue是否使能                              |
 | sat_en                   | sat是否使能                              |
 | con_en                   | con是否使能                              |
 | bright_en                | bright是否使能                           |
 | theta_sign               | hue angle                                |
 | contrast                 | contrast值，和off_contrast共同控制对比度 |
 | gamma                    | gamma值                                  |
 | theta_abs                | hue angle绝对值，范围0-8d180             |
 | saturation               | sat值                                    |
 | off_contrast             | contrast offset 0 - 255                  |
 | off_bright               | bright offset -128-127                   |
 | panel_type               | 输出类型                                 |
 | rotate                   | 是否使能旋转                             |
 | user_control_disp        | 通道0是否使能用户输入控制                |
 | user_control_disp_layer1 | 通道1是否使能用户输入控制                |

### HB_VOT_CSC_S

【结构定义】
```C
typedef struct HB_VOT_CSC_S {
    uint32_t u32Luma;
    uint32_t u32Contrast;
    uint32_t u32Hue;
    uint32_t u32Satuature;
} VOT_CSC_S;
```
【功能描述】
> 定义图像输出效果结构体

【成员说明】

| 成员         | 含义         |
| :----------- | :----------- |
| u32Luma      | 设置VO亮度   |
| u32Contrast  | 设置VO对比度 |
| u32Hue       | 设置VO色度   |
| u32Satuature | 设置VO饱和度 |

### HB_VOT_UPSCALE_ATTR_S
【结构定义】
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
【功能描述】
> 定义视频层放大属性

【成员说明】

| 成员       | 含义                 |
| :--------- | :------------------- |
| src_width  | 放大原图宽度         |
| src_height | 放大原图高度         |
| tgt_width  | 放大后的目标图像宽度 |
| tgt_height | 放大后的目标图像高度 |
| pos_x      | 放大后的x坐标        |
| pos_y      | 放大后的y坐标        |
| upscale_en | 放大功能是否使能     |

### HB_VOT_CHN_ATTR_S
【结构定义】
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
【功能描述】
> 定义视频输出通道属性

【成员说明】

|     成员     | 含义                                                         |
| :----------: | :----------------------------------------------------------- |
| u32Priority  | 视频通道叠加优先级，0优先级最高，3优先级最低，注意任何两个通道优先级不能重复，重复了所有通道都不能显示 |
| u32SrcWidth  | 视频通道原图宽度                                             |
| u32SrcHeight | 视频通道原图高度                                             |
|     s32X     | 视频通道x坐标                                                |
|     s32Y     | 视频通道y坐标                                                |
| u32DstWidth  | 视频通道目标图像宽度                                         |
| u32DstHeight | 视频通道目标图像高度                                         |

### HB_VOT_CHN_ATTR_EX_S
【结构定义】
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

【功能描述】
> 定义视频输出通道高级属性

【成员说明】

|   成员    | 含义|
| :-------: | :--------- |
|  format   | 输入视频YUV格式         |
|   alpha   | Alpha值                |
| keycolor  | 通道key-color          |
| alpha_sel | Aphpa叠加算法选择<br/>0: Result image = (layer A * (~BR + 1) + layer B*BR) >> 8<br/>1: Result image = (layer A * (~BR + 1) + layer B*BR) >> 8<br/>Layer A is merged result output by the pipeline backwards.<br/>Layer B is the higher priority layer of current pipeline.<br/>BR is Layer B Alpha ratio(0~255) and may be from image pixel data(ARGB/RGBA) or programmable register. |
|  ov_mode  | 叠加模式<br/>00: transparent<br/>01: and<br/>10: or<br/>11: inv  |
| alpha_en  | 是否是能alpha叠加  |

### VOT_WB_ATTR_S
【结构定义】
```C
typedef struct HB_VOT_WBC_ATTR_S{
    int wb_src;
    int wb_format;
    }VOT_WB_ATTR_S;
```
【功能描述】
> 定义回写属性

【成员说明】

|   成员    | 含义        |
| :-------: | :------------------------- |
|  wb_src   | 回写源<br/>0 overlay-alphablend输出<br/>1 upscaling 输出<br/>2 de-output输出    |
| wb_format | 回写格式<br/>0 FORMAT_YUV422_UYVY<br/>1 FORMAT_YUV422_VYUY<br/>2 FORMAT_YUV422_YVYU<br/>3 FORMAT_YUV422_YUYV<br/>4 FORMAT_YUV420SP_UV<br/>5 FORMAT_YUV420SP_VU<br/>6 FORMAT_BGR0<br/>wb_src为2时，vot输出为RGB模式时，只支持FORMAT_BGR0 |

### VOT_CROP_INFO_S
【结构定义】
```C
typedef struct HB_VOT_CROP_INFO_S{
    uint32_t u32Width;
    uint32_t u32Height;
    }VOT_CROP_INFO_S;
```
【功能描述】
> 定义crop属性

【成员说明】

|   成员    | 含义               |
| :-------: | :----------------- |
| u32Width  | Crop图像的目标宽度 |
| u32Height | Crop图像的目标高度 |

### VOT_FRAME_INFO_S
【结构定义】
```C
typedef struct HB_VOT_FRAME_INFO_S{   void *addr;
void *addr_uv;   unsigned int size; }VOT_FRAME_INFO_S;
```
【功能描述】
> 定义图像信息

【成员说明】

|  成员   | 含义                     |
| :-----: | :----------------------- |
|  addr   | 图像的Y分量起始虚拟地址  |
| addr_uv | 图像的UV分量起始虚拟地址 |
|  size   | 图像大小                 |

### HB_POINT_S
【结构定义】
```C
typedef struct HB_POINT_S {
    int s32X;
    int s32Y;
} POINT_S;
```
【功能描述】
> 像素点描述。

【成员说明】

| 成员  | 含义        |
| :---: | :---------- |
| s32X  | 像素点x坐标 |
| s32Y  | 像素点y坐标 |

### VOT_WB
【结构定义】
```C
typedef uint32_t VOT_WB;
```
【功能描述】
> 定义VOT回写变量。

### HB_PIXEL_FORMAT_YUV_E
【结构定义】
```C
typedef enum HB_PIXEL_FORMAT_YUV_E {
    PIXEL_FORMAT_YUV422_UYVY = 0,
    PIXEL_FORMAT_YUV422_VYUY = 1,
    PIXEL_FORMAT_YUV422_YVYU = 2,
    PIXEL_FORMAT_YUV422_YUYV = 3,
    PIXEL_FORMAT_YUV422SP_UV = 4,
    PIXEL_FORMAT_YUV422SP_VU = 5,
    PIXEL_FORMAT_YUV420SP_UV = 6,
    PIXEL_FORMAT_YUV420SP_VU = 7,
    PIXEL_FORMAT_YUV422P_UV = 8,
    PIXEL_FORMAT_YUV422P_VU = 9,
    PIXEL_FORMAT_YUV420P_UV = 10,
    PIXEL_FORMAT_YUV420P_VU = 11,
    PIXEL_FORMAT_YUV_BUTT = 12
} PIXEL_FORMAT_YUV_E;
```

### HB_PIXEL_FORMAT_RGB_E
【结构定义】
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
【结构定义】
```C
typedef struct HB_SIZE_S {
    uint32_t u32Width;
    uint32_t u32Height;
} SIZE_S;

```
【功能描述】
> 定义图像的尺寸。

【成员说明】

|   成员    | 含义     |
| :-------: | :------- |
| u32Width  | 图像的宽 |
| u32Height | 图像的高 |

## 错误码
VOT错误码如下表所示。

| 错误码 | 宏定义                                   | 描述                        |
| :----: | :--------------------------------------- | :-------------------------- |
| 0xa401 | HB_ERR_VOT_BUSY                          | 资源忙                      |
| 0xa402 | HB_ERR_VOT_NO_MEM                        | 内存不足                    |
| 0xa403 | HB_ERR_VOT_NULL_PTR                      | 函数参数中有空指针          |
| 0xa404 | HB_ERR_VOT_SYS_NOTREADY                  | 系统未初始化                |
| 0xa405 | HB_ERR_VOT_INVALID_DEVID                 | 设备ID超出合法范围          |
| 0xa406 | HB_ERR_VOT_INVALID_CHNID                 | 通道ID超出合法范围          |
| 0xa407 | HB_ERR_VOT_ILLEGAL_PARAM                 | 参数超出合法范围            |
| 0xa408 | HB_ERR_VOT_NOT_SUPPORT                   | 不支持的操作                |
| 0xa409 | HB_ERR_VOT_NOT_PERMIT                    | 操作不允许                  |
| 0xa40a | HB_ERR_VOT_INVALID_WBCID                 | WBC号超出范围               |
| 0xa40b | HB_ERR_VOT_INVALID_LAYERID               | 视频层号超出范围            |
| 0xa40c | HB_ERR_VOT_INVALID_VIDEO_CHNID           | 视频层通道号超出范围        |
| 0xa40d | HB_ERR_VOT_INVALID_BIND_VPSGROUPID       | 绑定VPS GROUP号超出范围     |
| 0xa40e | HB_ERR_VOT_INVALID_BIND_VPSCHNID         | 绑定VPS CHN号超出范围       |
| 0xa40f | HB_ERR_VOT_INVALID_FRAME_RATE            | 不支持的帧率                |
| 0xa410 | HB_ERR_VOT_DEV_NOT_CONFIG                | 设备未配置                  |
| 0xa411 | HB_ERR_VOT_DEV_NOT_ENABLE                | 设备未使能                  |
| 0xa412 | HB_ERR_VOT_DEV_HAS_ENABLED               | 设备已使能                  |
| 0xa413 | HB_ERR_VOT_DEV_HAS_BINDED                | 设备已被绑定                |
| 0xa414 | HB_ERR_VOT_DEV_NOT_BINDED                | 设备未被绑定                |
| 0xa415 | HB_ERR_VOT_LAYER_NOT_ENABLE              | 视频输出层未使能            |
| 0xa420 | HB_ERR_VOT_VIDEO_NOT_ENABLE              | 视频层未使能                |
| 0xa421 | HB_ERR_VOT_VIDEO_NOT_DISABLE             | 视频层未禁止                |
| 0xa422 | HB_ERR_VOT_VIDEO_NOT_CONFIG              | 视频层未配置                |
| 0xa423 | HB_ERR_VOT_VIDEO_HAS_BINDED              | 视频层已绑定                |
| 0xa424 | HB_ERR_VOT_VIDEO_NOT_BINDED              | 视频层未绑定                |
| 0xa430 | HB_ERR_VOT_WBC_NOT_DISABLE               | 回写设备未禁用              |
| 0xa431 | HB_ERR_VOT_WBC_NOT_CONFIG                | 回写设备未配置              |
| 0xa432 | HB_ERR_VOT_WBC_HAS_CONFIG                | 回写设备已配置              |
| 0xa433 | HB_ERR_VOT_WBC_NOT_BIND                  | 回写设备未绑定              |
| 0xa434 | HB_ERR_VOT_WBC_HAS_BIND                  | 回写设备已绑定              |
| 0xa435 | HB_ERR_VOT_INVALID_WBID                  | 不可用的回写设备号          |
| 0xa436 | HB_ERR_VOT_WB_NOT_ENABLE                 | 回写设备未使能              |
| 0xa437 | HB_ERR_VOT_WB_GET_TIMEOUT                | 获取一帧回写图像超时        |
| 0xa440 | HB_ERR_VOT_CHN_NOT_DISABLE               | 通道未禁止                  |
| 0xa441 | HB_ERR_VOT_CHN_NOT_ENABLE                | 通道未使能                  |
| 0xa442 | HB_ERR_VOT_CHN_NOT_CONFIG                | 通道未配置                  |
| 0xa443 | HB_ERR_VOT_CHN_NOT_ALLOC                 | 通道未分配资源              |
| 0xa444 | HB_ERR_VOT_CHN_AREA_OVERLAP              | VO通道区域重叠              |
| 0xa450 | HB_ERR_VOT_INVALID_PATTERN               | 无效样式                    |
| 0xa451 | HB_ERR_VOT_INVALID_POSITION              | 无效级联位置                |
| 0xa460 | HB_ERR_VOT_WAIT_TIMEOUT                  | 等待超时                    |
| 0xa461 | HB_ERR_VOT_INVALID_VFRAME                | 无效视频帧                  |
| 0xa462 | HB_ERR_VOT_INVALID_RECT_PARA             | 无效矩形参数                |
| 0xa463 | HB_ERR_VOT_SETBEGIN_ALREADY              | BEGIN已设置                 |
| 0xa464 | HB_ERR_VOT_SETBEGIN_NOTYET               | BEGIN未设置                 |
| 0xa465 | HB_ERR_VOT_SETEND_ALREADY                | END已设置                   |
| 0xa466 | HB_ERR_VOT_SETEND_NOTYET                 | END未设置                   |
| 0xa470 | HB_ERR_VOT_GFX_NOT_DISABLE               | 图形层未关闭                |
| 0xa471 | HB_ERR_VOT_GFX_NOT_BIND                  | 图形层未绑定                |
| 0xa472 | HB_ERR_VOT_GFX_NOT_UNBIND                | 图形层未解绑定              |
| 0xa473 | HB_ERR_VOT_GFX_INVALID_ID                | 图形层ID超出范围            |
| 0xa480 | HB_ERR_VOT_BUF_MANAGER_ILLEGAL_OPERATION | Buffer manger非法的工作状态 |

## 参考代码
VO部分示例代码可以参考，[sample_vot](./multimedia_samples#sample_vot)和[sample_lcd](./multimedia_samples#sample_lcd)。