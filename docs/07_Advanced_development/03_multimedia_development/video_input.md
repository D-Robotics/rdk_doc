---
sidebar_position: 4
---

# 7.3.4 视频输入
## 概述
视频输入（VIN）实现的功能：通过 MIPI Rx接口接收视频数据。VIN将接收到的数据给下一个模块VPS，同时也可存入到指定的内存区域，在此过程中，VIN可以对接收到的原始视频图像数据进行处理，实现视频数据的采集。

### 概念

视频输入设备 视频输入设备主要是指sif，图像数据接口，主要功能接收摄像头模组输出的图像数据，经过offline或者online直接输出到ISP模块进行图像处理。

- 视频输入设备

​		视频输入设备主要是指sif，图像数据接口，主要功能接收摄像头模组输出的图像数据，经过offline或者		 		online直接输出到ISP模块进行图像处理。

- 视频输入 PIPE

​		视频输入 PIPE (ISP)绑定在设备后端，负责图像处理，硬核功能配置，支持Multi context。

- 镜头畸变校正（LDC）

​		主要负责矫正图像，有时因为镜头曲面造成的图像变形，一些低端镜头容易产生图像畸变，需要根据畸变程		度对其图像进行校正。

- DIS

​		DIS 模块通过比较当前图像与前两帧图像采用不同自由度的防抖算法计算出当前图像在各个轴方向上的抖动偏		移向量，然后根据抖动偏移向量对当前图像进行校正，从而起到防抖的效果。

- DWE

​		DWE主要是将LDC和DIS集成在一起，包括LDC的畸变矫正和DIS的统计结果。

## 功能描述

VIN在软件上划分4个部分，如下图所示。

![image-20220329195124946](./image/video_input/image-20220329195124946.png)

### 视频输入设备

sif主要功能接收摄像头模组输出的图像数据，经过offline或者online直接输出到ISP模块进行图像处理。Mipi:支持RAW8/RAW10/RAW12/RAW14/RAW16 or YUV422 8bit/10bit。DVP interface: RAW8/RAW10/RAW12/RAW14/RAW16 or YUV422 8bit/10bit。最多支持8路sensor接入。

### 视频输入PIPE

Isp主要负责图像处理，硬核功能配置，支持Multi context，最多支持8路接入。主要是对图像数据进行流水线处理，输出YUV 图像格式给通道。同时PIPE也包括DIS、LDC的功能。

### 视频物理通道

VIN的PIPE 包含 2 个物理通道，物理通道0是指isp处理后的数据到ddr，或者是通过ddr给到下一级模块VPS。物理通道1是指isp处理后的数据online到VPS，VIN和VPS的绑定关系请参考“系统控制”章节。

### 绑定关系

VIN和VPS之间的绑定关系请参考“系统控制”章节 HB_SYS_SetVINVPSMode



## API参考

```c
int HB_MIPI_SetBus(MIPI_SENSOR_INFO_S *snsInfo, uint32_t busNum);
int HB_MIPI_SetPort(MIPI_SENSOR_INFO_S *snsInfo, uint32_t port);
int HB_MIPI_SensorBindSerdes(MIPI_SENSOR_INFO_S *snsInfo, uint32_t serdesIdx, uint32_t serdesPort);
int HB_MIPI_SensorBindMipi(MIPI_SENSOR_INFO_S *snsInfo, uint32_t mipiIdx);
int HB_MIPI_SetExtraMode(MIPI_SENSOR_INFO_S *snsInfo, uint32_t ExtraMode);
int HB_MIPI_InitSensor (uint32_t DevId, MIPI_SENSOR_INFO_S  *snsInfo);
int HB_MIPI_DeinitSensor (uint32_t  DevId);
int HB_MIPI_ResetSensor(uint32_t DevId);
int HB_MIPI_UnresetSensor(uint32_t DevId);
int HB_MIPI_EnableSensorClock(uint32_t mipiIdx);
int HB_MIPI_DisableSensorClock(uint32_t mipiIdx);
int HB_MIPI_SetSensorClock(uint32_t mipiIdx, uint32_t snsMclk);
int HB_MIPI_ResetMipi(uint32_t  mipiIdx);
int HB_MIPI_UnresetMipi(uint32_t  mipiIdx);
int HB_MIPI_SetMipiAttr(uint32_t  mipiIdx, MIPI_ATTR_S  mipiAttr);
int HB_MIPI_Clear(uint32_t  mipiIdx);
int HB_MIPI_ReadSensor(uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size);
int HB_MIPI_WriteSensor (uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size);
int HB_MIPI_GetSensorInfo(uint32_t devId, MIPI_SENSOR_INFO_S *snsInfo);
int HB_MIPI_SwSensorFps(uint32_t devId, uint32_t fps);
int HB_VIN_SetMipiBindDev(uint32_t devId, uint32_t mipiIdx);
int HB_VIN_GetMipiBindDev(uint32_t devId, uint32_t *mipiIdx);
int HB_VIN_SetDevAttr(uint32_t devId,  const VIN_DEV_ATTR_S *stVinDevAttr);
int HB_VIN_GetDevAttr(uint32_t devId, VIN_DEV_ATTR_S *stVinDevAttr);
int HB_VIN_SetDevAttrEx(uint32_t devId,  const VIN_DEV_ATTR_EX_S *stVinDevAttrEx);
int HB_VIN_GetDevAttrEx(uint32_t devId, VIN_DEV_ATTR_EX_S *stVinDevAttrEx);
int HB_VIN_EnableDev(uint32_t devId);
int HB_VIN_DisableDev (uint32_t devId);
int HB_VIN_DestroyDev(uint32_t devId);
int HB_VIN_SetDevBindPipe(uint32_t devId, uint32_t pipeId);
int HB_VIN_GetDevBindPipe(uint32_t devId, uint32_t *pipeId);
int HB_VIN_CreatePipe(uint32_t pipeId, const VIN_PIPE_ATTR_S * stVinPipeAttr);
int HB_VIN_DestroyPipe(uint32_t pipeId);
int HB_VIN_StartPipe(uint32_t pipeId);
int HB_VIN_StopPipe(uint32_t pipeId);
int HB_VIN_EnableChn(uint32_t pipeId, uint32_t chnId);
int HB_VIN_DisableChn(uint32_t pipeId, uint32_t chnId);
int HB_VIN_SetChnLDCAttr(uint32_t pipeId, uint32_t chnId,const VIN_LDC_ATTR_S *stVinLdcAttr);
int HB_VIN_GetChnLDCAttr(uint32_t pipeId, uint32_t chnId, VIN_LDC_ATTR_S*stVinLdcAttr);
int HB_VIN_SetChnDISAttr(uint32_t pipeId, uint32_t chnId, const VIN_DIS_ATTR_S *stVinDisAttr);
int HB_VIN_GetChnDISAttr(uint32_t pipeId, uint32_t chnId, VIN_DIS_ATTR_S *stVinDisAttr);
int HB_VIN_SetChnAttr(uint32_t pipeId, uint32_t chnId);
int HB_VIN_DestroyChn(uint32_t pipeId, uint32_t chnId);
int HB_VIN_GetChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame, int32_t millSec);
int HB_VIN_ReleaseChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame);
int HB_VIN_SendPipeRaw(uint32_t pipeId, void *pstVideoFrame，int32_t millSec);
int HB_VIN_SetPipeAttr(uint32_t pipeId,VIN_PIPE_ATTR_S *stVinPipeAttr);
int HB_VIN_GetPipeAttr(uint32_t pipeId, VIN_PIPE_ATTR_S *stVinPipeAttr);
int HB_VIN_CtrlPipeMirror(uint32_t pipeId, uint8_t on);
int HB_VIN_MotionDetect(uint32_t pipeId);
int HB_VIN_InitLens(uint32_t pipeId, VIN_LENS_FUNC_TYPE_ElensType,const VIN_LENS_CTRL_ATTR_S *lenCtlAttr);
int HB_VIN_DeinitLens(uint32_t pipeId);
int HB_VIN_RegisterDisCallback(uint32_t pipeId,VIN_DIS_CALLBACK_S *pstDISCallback);
int HB_VIN_SetDevVCNumber(uint32_t devId, uint32_t vcNumber);
int HB_VIN_GetDevVCNumber(uint32_t devId, uint32_t *vcNumber);
int HB_VIN_AddDevVCNumber(uint32_t devId, uint32_t vcNumber);
int HB_VIN_SetDevMclk(uint32_t devId, uint32_t devMclk, uint32_t vpuMclk);
int HB_VIN_GetChnFd(uint32_t pipeId, uint32_t chnId);
int HB_VIN_CloseFd(void);
int HB_VIN_EnableDevMd(uint32_t devId);
int HB_VIN_DisableDevMd(uint32_t devId);
int HB_VIN_GetDevFrame(uint32_t devId, uint32_t chnId, void *videoFrame, int32_t millSec);
int HB_VIN_ReleaseDevFrame(uint32_t devId, uint32_t chnId, void *buf);
```

### HB_MIPI_SetBus
【函数声明】
```c
int HB_MIPI_SetBus(MIPI_SENSOR_INFO_S *snsInfo, uint32_t busNum)
```
【功能描述】
> 设置sensor的总线号

【参数描述】

| 参数名称 |       描述       | 输入/输出 |
| :------: | :--------------: | :-------: |
| snsInfo  | sensor的配置信息 |   输入    |
|  busNum  |      bus号       |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_SetPort
【函数声明】
```c
int HB_MIPI_SetPort(MIPI_SENSOR_INFO_S *snsInfo, uint32_t port)
```
【功能描述】
> 设置sensor的port，取值范围 0~7

【参数描述】

| 参数名称 |          描述           | 输入/输出 |
| :------: | :---------------------: | :-------: |
| snsInfo  |    sensor的配置信息     |   输入    |
|   port   | 当前sensor的port号，0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_SensorBindSerdes
【函数声明】
```c
int HB_MIPI_SensorBindSerdes(MIPI_SENSOR_INFO_S *snsInfo, uint32_t serdesIdx, uint32_t serdesPort)
```
【功能描述】
> 设置sensor绑定到哪个serdes上

【参数描述】

|  参数名称  |              描述              | 输入/输出 |
| :--------: | :----------------------------: | :-------: |
|  snsInfo   |        sensor的配置信息        |   输入    |
| serdesIdx  |        serdes的索引0~1         |   输入    |
| serdesPort | serdes的port号954 0~1  960 0~3 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_SensorBindMipi
【函数声明】
```c
int HB_MIPI_SensorBindMipi(MIPI_SENSOR_INFO_S *snsInfo, uint32_t mipiIdx)
```
【功能描述】
> 设置sensor绑定哪一个mipi上

【参数描述】

| 参数名称 |        描述         | 输入/输出 |
| :------: | :-----------------: | :-------: |
| snsInfo  |  sensor的配置信息   |   输入    |
| mipiIdx  | mipi_host的索引 0~3 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor举例

### HB_MIPI_SetExtraMode
【函数声明】
```c
int HB_MIPI_SetExtraMode(MIPI_SENSOR_INFO_S *snsInfo, uint32_t ExtraMode);
```
【功能描述】
> 设置sensor在DOL2或DOL3下的工作模式

【参数描述】

| 参数名称  |        描述        | 输入/输出|
| :-------: | :----------------: | :---------- |
|  snsInfo  |  sensor的配置信息  | 输入 |
| ExtraMode | 选择以何种工作模式 | 1. 单路DOL2,值为0<br /> 2. DOL2分为两路linear,一路值为1，另一路值为2<br /> 3. 单路DOl3,值为0<br /> 4. 一路DOl2(值为1)+一路linear(值为4)<br /> 5. DOL3分为三路linear,一路为2，一路为3，一路为4 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_InitSensor/HB_MIPI_DeinitSensor
【函数声明】
```c
int HB_MIPI_InitSensor (uint32_t DevId, MIPI_SENSOR_INFO_S  *snsInfo);
int HB_MIPI_DeinitSensor (uint32_t  DevId);
```
【功能描述】
> sensor的初始化和释放初始化产生的资源

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |
| snsInfo  |    Sensor 信息    |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
```c
    MIPI_SENSOR_INFO_S  snsInfo;
    MIPI_ATTR_S  mipiAttr;
    int DevId = 0, mipiIdx = 1;
    int bus = 1, port = 0, serdes_index = 0, serdes_port = 0;
    int ExtraMode= 0;

    memset(snsInfo, 0, sizeof(MIPI_SENSOR_INFO_S));
    memset(mipiAttr, 0, sizeof(MIPI_ATTR_S));
    snsInfo.sensorInfo.bus_num = 0;
    snsInfo.sensorInfo.bus_type = 0;
    snsInfo.sensorInfo.entry_num = 0;
    snsInfo.sensorInfo.sensor_name = "imx327";
    snsInfo.sensorInfo.reg_width = 16;
    snsInfo.sensorInfo.sensor_mode = NORMAL_M;
    snsInfo.sensorInfo.sensor_addr = 0x36;

    mipiAttr.dev_enable = 1;
    mipiAttr.mipi_host_cfg.lane = 4;
    mipiAttr.mipi_host_cfg.datatype = 0x2c;
    mipiAttr.mipi_host_cfg.mclk = 24;
    mipiAttr.mipi_host_cfg.mipiclk = 891;
    mipiAttr.mipi_host_cfg.fps = 25;
    mipiAttr.mipi_host_cfg.width = 1952;
    mipiAttr.mipi_host_cfg.height = 1097;
    mipiAttr.mipi_host_cfg->linelenth = 2475;
    mipiAttr.mipi_host_cfg->framelenth = 1200;
    mipiAttr.mipi_host_cfg->settle = 20;

    HB_MIPI_SetBus(snsInfo, bus);
    HB_MIPI_SetPort(snsinfo, port);
    HB_MIPI_SensorBindSerdes(snsinfo, sedres_index, sedres_port);
    HB_MIPI_SensorBindMipi(snsinfo,  mipiIdx);
    HB_MIPI_SetExtraMode (snsinfo,  ExtraMode);
    ret = HB_MIPI_InitSensor(DevId, snsInfo);
    if(ret < 0) {
        printf("HB_MIPI_InitSensor error!\n");
        return ret;
    }
    ret = HB_MIPI_SetMipiAttr(mipiIdx, mipiAttr);
    if(ret < 0) {
        printf("HB_MIPI_SetMipiAttr error! do sensorDeinit\n");
        HB_MIPI_SensorDeinit(DevId);
        return ret;
    }
    ret = HB_MIPI_ResetSensor(DevId);
    if(ret < 0) {
        printf("HB_MIPI_ResetSensor error! do mipi deinit\n");
        HB_MIPI_DeinitSensor(DevId);
        HB_MIPI_Clear(mipiIdx);
        return ret;
    }
    ret = HB_MIPI_ResetMipi(mipiIdx);
    if(ret < 0) {
        printf("HB_MIPI_ResetMipi error!\n");
        HB_MIPI_UnresetSensor(DevId);
        HB_MIPI_DeinitSensor(DevId);
        HB_MIPI_Clear(mipiIdx);
        return ret;
    }
    HB_MIPI_UnresetSensor(DevId);
    HB_MIPI_UnresetMipi(mipiIdx);
    HB_MIPI_DeinitSensor(DevId);
    HB_MIPI_Clear(mipiIdx);
```

### HB_MIPI_ResetSensor/HB_MIPI_UnresetSensor
【函数声明】
```c
int HB_MIPI_ResetSensor(uint32_t DevId);
int HB_MIPI_UnresetSensor(uint32_t DevId);
```
【功能描述】
> sensor数据流的打开和关闭,sensor_start/sensor_stop

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_EnableSensorClock/HB_MIPI_DisableSensorClock
【函数声明】
```c
int HB_MIPI_EnableSensorClock(uint32_t mipiIdx);
int HB_MIPI_DisableSensorClock(uint32_t mipiIdx);
```
【功能描述】
> 打开和关闭,sensor_clk

【参数描述】

| 参数名称 |           描述            | 输入/输出 |
| :------: | :-----------------------: | :-------: |
| mipiIdx  | Mipi host 索引号，范围0~3 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 使用此接口需去掉子板的晶振

【参考代码】
> 暂无

### HB_MIPI_SetSensorClock
【函数声明】
```c
int HB_MIPI_SetSensorClock(uint32_t mipiIdx, uint32_t snsMclk)
```
【功能描述】
> 设置sensor_mclk
> 一共有4个sensor_mclk，目前用到得是sensor0_mclk和sensor1_mclk,
> mipi0连接在sensor_mclk1, mipi1连接在sensor_mclk0,硬件连接关系在dts里面定义。

【参数描述】

| 参数名称 |           描述            |             输入/输出              |
| :------: | :-----------------------: | :--------------------------------: |
| mipiIdx  | Mipi host 索引号，范围0~3 |                输入                |
| snsMclk  |          单位HZ           | 输入，比如24MHZ，snsMclk为24000000 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 使用此接口需去掉子板的晶振

【参考代码】
> 初始化时:
>> 先设置sensor_mclk然后再去使能
>> HB_MIPI_SetSensorClock(mipiIdx, 24000000);
>> HB_MIPI_EnableSensorClock(mipiIdx);

> 退出时：
>> HB_MIPI_Clear(mipiIdx);
>> HB_MIPI_DeinitSensor(devId);
>> HB_MIPI_DisableSensorClock(mipiIdx);

### HB_MIPI_ResetMipi/HB_MIPI_UnresetMipi
【函数声明】
```c
int HB_MIPI_ResetMipi(uint32_t  mipiIdx);
int HB_MIPI_UnresetMipi(uint32_t  mipiIdx)
```
【功能描述】
> mipi的start和stop

【参数描述】

| 参数名称 |           描述            | 输入/输出 |
| :------: | :-----------------------: | :-------: |
| mipiIdx  | Mipi host 索引号，范围0~3 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_SetMipiAttr
【函数声明】
```c
int HB_MIPI_SetMipiAttr(uint32_t  mipiIdx, MIPI_ATTR_S  mipiAttr)
```
【功能描述】
> 设置mipi的属性，host和dev的初始化。

【参数描述】

| 参数名称 |       描述       | 输入/输出 |
| :------: | :--------------: | :-------: |
| mipiIdx  | Mipi host 索引号 |   输入    |
| mipiAttr | Mipi总线属性信息 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_Clear
【函数声明】
```c
int HB_MIPI_Clear(uint32_t  mipiIdx);
```
【功能描述】
> 清除设备相关的配置，mipi host/dev 的deinit，和接口HB_MIPI_SetMipiAttr对应。

【参数描述】

| 参数名称 |           描述            | 输入/输出 |
| :------: | :-----------------------: | :-------: |
| mipiIdx  | Mipi host 索引号，范围0~3 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_MIPI_InitSensor/HB_MIPI_DeinitSensor举例

### HB_MIPI_ReadSensor
【函数声明】
```c
int HB_MIPI_ReadSensor(uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size)
```
【功能描述】
> 通过i2c读取sensor。

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |
| regAddr  |    寄存器地址     |   输入    |
| buffer,  |  存放数据的地址   |   输出    |
|   size   |    读取的长度     |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 必须在HB_MIPI_InitSensor接口调用后才能使用

【参考代码】
> 不同的sensor不一样，以imx327为例：
```c
    int i;
    char buffer[] = {0x34, 0x56};
    char rev_buffer[30] = {0};
    printf("HB_MIPI_InitSensor end\n");
    ret = HB_MIPI_ReadSensor(devId, 0x3018, rev_buffer,  2);
    if(ret < 0) {
        printf("HB_MIPI_ReadSensor error\n");
    }
    for(i = 0; i < strlen(rev_buffer); i++) {
        printf("rev_buffer[%d] 0x%x  \n", i, rev_buffer[i]);
    }
    ret = HB_MIPI_WriteSensor(devId, 0x3018, buffer, 2);
    if(ret < 0) {
        printf("HB_MIPI_WriteSensor error\n");
    }
    ret = HB_MIPI_ReadSensor(devId, 0x3018, rev_buffer, 2);
    if(ret < 0) {
        printf("HB_MIPI_ReadSensor error\n");
    }
    for(i = 0; i < strlen(rev_buffer); i++) {
        printf("rev_buffer[%d] 0x%x  \n", i, rev_buffer[i]);
    }
```

### HB_MIPI_WriteSensor
【函数声明】
```c
int HB_MIPI_WriteSensor (uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size)
```
【功能描述】
> 通过i2c写sensor寄存器

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |
| regAddr  |    寄存器地址     |   输入    |
|  buffer  |  存放数据的地址   |   输入    |
|   size   |     写的长度      |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 必须在HB_MIPI_InitSensor接口调用后才能使用

【参考代码】
> 请参见HB_MIPI_ReadSensor举例

### HB_MIPI_GetSensorInfo
【函数声明】
```c
int HB_MIPI_GetSensorInfo(uint32_t devId, MIPI_SENSOR_INFO_S *snsInfo)
```
【功能描述】
> 获取sensor相关配置信息

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |
| snsInfo  |    sensor信息     |   输出    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 必须在HB_MIPI_InitSensor接口调用后才能使用

【参考代码】
```c
    MIPI_SENSOR_INFO_S *snsinfo = NULL;
    snsinfo = malloc(sizeof(MIPI_SENSOR_INFO_S));
    if(snsinfo == NULL) {
        printf("malloc error\n");
        return -1;
    }
    memset(snsinfo, 0, sizeof(MIPI_SENSOR_INFO_S));
    ret = HB_MIPI_GetSensorInfo(devId, snsinfo);
    if(ret < 0) {
        printf("HB_MIPI_InitSensor error!\n");
        return ret;
    }
```

### HB_MIPI_SwSensorFps
【函数声明】
```c
int HB_MIPI_SwSensorFps(uint32_t devId, uint32_t fps)
```
【功能描述】
> 切换sensor的帧率

【参数描述】

| 参数名称 |       描述        | 输入/输出 |
| :------: | :---------------: | :-------: |
|  devId   | 通路索引，范围0~7 |   输入    |
|   fps    |   sensor的帧率    |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 必须在HB_MIPI_InitSensor接口调用后才能使用

【参考代码】
> 暂无

### HB_VIN_SetMipiBindDev/HB_VIN_GetMipiBindDev
【函数声明】
```c
int HB_VIN_SetMipiBindDev(uint32_t devId, uint32_t mipiIdx)
int HB_VIN_GetMipiBindDev(uint32_t devId, uint32_t *mipiIdx)
```
【功能描述】
> 设置mipi和dev的绑定，dev使用哪一个mipi_host

【参数描述】

| 参数名称 |          描述           | 输入/输出 |
| :------: | :---------------------: | :-------: |
|  devId   | 对应通道索引号，范围0~7 |   输入    |
|mipiIdx|mipi_host的索引| 输入|

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetDevAttr/HB_VIN_GetDevAttr
【函数声明】
```c
int HB_VIN_SetDevAttr(uint32_t devId,  const VIN_DEV_ATTR_S *stVinDevAttr)
int HB_VIN_GetDevAttr(uint32_t devId, VIN_DEV_ATTR_S *stVinDevAttr)
```
【功能描述】
> 设置和获取dev的属性

【参数描述】

|   参数名称   |          描述           |             输入/输出             |
| :----------: | :---------------------: | :-------------------------------: |
|    devId     | 对应通道索引号，范围0~7 |               输入                |
| stVinDevAttr |       dev通道属性       | 输入，调用HB_VIN_GetDevAttr为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> DOL3拆分成多路时，多进程情况：第一个进程要先于第二个进程运行1秒即可。
> 另外目前不支持HB_VIN_DestroyDev之后重新HB_VIN_SetDevAttr。
>
> 出现SIF_IOC_BIND_GROUT ioctl failed报错，一般是前一次pipeid的调用没有退出，又重新调用。

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetDevAttrEx/HB_VIN_GetDevAttrEx
【函数声明】
```c
int HB_VIN_SetDevAttrEx(uint32_t devId,  const VIN_DEV_ATTR_EX_S *stVinDevAttrEx)
int HB_VIN_GetDevAttrEx(uint32_t devId, VIN_DEV_ATTR_EX_S *stVinDevAttrEx)
```
【功能描述】
> 设置何获取dev的扩展属性

【参数描述】

|    参数名称    |          描述           |             输入/输出             |
| :------------: | :---------------------: | :-------------------------------: |
|     devId      | 对应通道索引号，范围0~7 |               输入                |
| stVinDevAttrEx |      dev的扩展属性      | 输入，调用HB_VIN_GetDevAttr为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 该接口暂不支持

【参考代码】
> 暂无

### HB_VIN_EnableDev/HB_VIN_DisableDev
【函数声明】
```c
int HB_VIN_EnableDev(uint32_t devId);
int HB_VIN_DisableDev (uint32_t devId);
```
【功能描述】
> dev模块的使能和关闭

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  devId   | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_DestroyDev
【函数声明】
```c
int HB_VIN_DestroyDev(uint32_t devId)
```
【功能描述】
> dev模块的销毁，资源释放

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  devId   | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetDevBindPipe/HB_VIN_GetDevBindPipe
【函数声明】
```c
int HB_VIN_SetDevBindPipe(uint32_t devId, uint32_t pipeId)
int HB_VIN_GetDevBindPipe(uint32_t devId, uint32_t *pipeId)

```
【功能描述】
> 设置dev的chn输出和pipe的chn输入的绑定
> 设置pipe的chn输入和pipe输出的chn绑定。

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  devId   | 对应每路输入，范围0~7 |   输入    |
|  pipeId  |  对应每路输入，同上   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> HB_VIN_GetDevBindPipe接口暂未实现

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_CreatePipe/HB_VIN_DestroyPipe
【函数声明】
```c
int HB_VIN_CreatePipe(uint32_t pipeId, const VIN_PIPE_ATTR_S * stVinPipeAttr);
int HB_VIN_DestroyPipe(uint32_t pipeId);
```
【功能描述】
> 创建pipe、销毁pipe

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |
|stVinPipeAttr|描述pipe属性的指针|输入|

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
```c
    VIN_DEV_ATTR_S  stVinDevAttr;
    VIN_PIPE_ATTR_S  stVinPipeAttr;
    VIN_DIS_ATTR_S   stVinDisAttr;
    VIN_LDC_ATTR_S  stVinLdcAttr;
    MIPI_SNS_TYPE_E sensorId = 1;
    MIPI_SENSOR_INFO_S  snsInfo;
    MIPI_ATTR_S  mipiAttr;
    MIPI_SNS_TYPE_E sensorId = 1;
    int PipeId = 0;
    int DevId = 0, mipiIdx = 1;
    int ChnId = 1, bus = 1, port = 0, serdes_index = 0, serdes_port = 0;

    memset(snsInfo, 0, sizeof(MIPI_SENSOR_INFO_S));
    memset(mipiAttr, 0, sizeof(MIPI_ATTR_S));
    memset(stVinDevAttr, 0, sizeof(VIN_DEV_ATTR_S));
    memset(stVinPipeAttr, 0, sizeof(VIN_PIPE_ATTR_));
    memset(stVinDisAttr, 0, sizeof(VIN_DIS_ATTR_S));
    memset(stVinLdcAttr, 0, sizeof(VIN_LDC_ATTR_S));
    snsInfo.sensorInfo.bus_num = 0;
    snsInfo.sensorInfo.bus_type = 0;
    snsInfo.sensorInfo.entry_num = 0;
    snsInfo.sensorInfo.sensor_name = "imx327";
    snsInfo.sensorInfo.reg_width = 16;
    snsInfo.sensorInfo.sensor_mode = NORMAL_M;
    snsInfo.sensorInfo.sensor_addr = 0x36;

    mipiAttr.dev_enable = 1;
    mipiAttr.mipi_host_cfg.lane = 4;
    mipiAttr.mipi_host_cfg.datatype = 0x2c;
    mipiAttr.mipi_host_cfg.mclk = 24;
    mipiAttr.mipi_host_cfg.mipiclk = 891;
    mipiAttr.mipi_host_cfg.fps = 25;
    mipiAttr.mipi_host_cfg.width = 1952;
    mipiAttr.mipi_host_cfg.height = 1097;
    mipiAttr.mipi_host_cfg->linelenth = 2475;
    mipiAttr.mipi_host_cfg->framelenth = 1200;
    mipiAttr.mipi_host_cfg->settle = 20;
    stVinDevAttr.stSize.format = 0;
    stVinDevAttr.stSize.width = 1952;
    stVinDevAttr.stSize.height = 1097;
    stVinDevAttr.stSize.pix_length = 2;
    stVinDevAttr.mipiAttr.enable = 1;
    stVinDevAttr.mipiAttr.ipi_channels =  1;
    stVinDevAttr.mipiAttr.enable_frame_id = 1;
    stVinDevAttr.mipiAttr.enable_mux_out = 1;
    stVinDevAttr.DdrIspAttr.enable = 1;
    stVinDevAttr.DdrIspAttr.buf_num = 4;
    stVinDevAttr.DdrIspAttr.raw_feedback_en = 0;
    stVinDevAttr.DdrIspAttr.data.format = 0;
    stVinDevAttr.DdrIspAttr.data.width = 1952;
    stVinDevAttr.DdrIspAttr.data.height = 1907;
    stVinDevAttr.DdrIspAttr.data.pix_length = 2;
    stVinDevAttr.outIspAttr.isp_enable = 1;
    stVinDevAttr.outIspAttr.dol_exp_num = 4;
    stVinDevAttr.outIspAttr.enable_flyby = 0;
    stVinDevAttr.outDdrAttr.enable = 1;
    stVinDevAttr.outDdrAttr.mux_index = 0;
    stVinDevAttr.outDdrAttr.buffer_num = 10;
    stVinDevAttr.outDdrAttr.raw_dump_en = 0;
    stVinDevAttr.outDdrAttr.stride = 2928;
    stVinDevAttr.outIpuAttr.enable_flyby = 0;

    stVinPipeAttr.ddrOutBufNum = 8;
    stVinPipeAttr.pipeDmaEnable = 1;
    stVinPipeAttr.snsMode = 3;
    stVinPipeAttr.stSize.format = 0;
    stVinPipeAttr.stSize.width = 1920;
    stVinPipeAttr.stSize.height = 1080;
    stVinDisAttr.xCrop.rg_dis_start = 0;
    stVinDisAttr.xCrop.rg_dis_end = 1919;
    stVinDisAttr.yCrop.rg_dis_start = 0;
    stVinDisAttr.yCrop.rg_dis_end = 1079
    stVinDisAttr.disHratio = 65536;
    stVinDisAttr.disVratio = 65536;
    stVinDisAttr.disPath.rg_dis_enable = 0;
    stVinDisAttr.disPath.rg_dis_path_sel = 1;
    stVinDisAttr.picSize.pic_w = 1919;
    stVinDisAttr.picSize.pic_h = 1079;
    stVinLdcAttr->ldcEnable = 0;
    stVinLdcAttr->ldcPath.rg_h_blank_cyc = 32;
    stVinLdcAttr->yStartAddr = 524288;
    stVinLdcAttr->cStartAddr = 786432;
    stVinLdcAttr->picSize.pic_w = 1919;
    stVinLdcAttr->picSize.pic_h = 1079;
    stVinLdcAttr->lineBuf = 99;
    stVinLdcAttr->xParam.rg_algo_param_a = 1;
    stVinLdcAttr->xParam.rg_algo_param_b = 1;
    stVinLdcAttr->yParam.rg_algo_param_a = 1;
    stVinLdcAttr->yParam.rg_algo_param_b = 1;
    stVinLdcAttr->xWoi.rg_length = 1919;
    stVinLdcAttr->xWoi.rg_start = 0;
    stVinLdcAttr->yWoi.rg_length = 1079;
    stVinLdcAttr->yWoi.rg_start = 0;

    ret = HB_VIN_CreatePipe(PipeId, pipeInfo);
    if(ret < 0) {
        printf("HB_VIN_CreatePipe t error!\n");
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
    if(ret < 0) {
        printf("HB_VIN_SetMipiBindDev error!\n");
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
    if(ret < 0) {
        printf("HB_VIN_SetDevVCNumber error!\n");
        return ret;
    }
    ret = HB_VIN_SetDevAttr(DevId, devInfo);
    if(ret < 0) {
        printf("HB_VIN_SetDevAttr error!\n");
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetPipeAttr (PipeId, pipeInfo);
    if(ret < 0) {
        printf("HB_VIN_SetPipeAttr error!\n");
        HB_VIN_DestroyDev(DevId);
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetChnDISAttr(PipeId, ChnId, disInfo);
    if(ret < 0) {
        printf("HB_VIN_SetChnDISAttr error!\n");
        HB_VIN_DestroyDev(DevId);
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetChnLDCAttr(PipeId, ChnId, ldcInfo);
    if(ret < 0) {
            printf("HB_VIN_SetChnLDCAttr error!\n");
        HB_VIN_DestroyDev(DevId);
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_VIN_SetChnAttr(PipeId, ChnId );
    if(ret < 0) {
        printf("HB_VIN_SetChnAttr error!\n");
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    HB_VIN_SetDevBindPipe(DevId, PipeId);

    HB_MIPI_SetBus(snsInfo, bus);
    HB_MIPI_SetPort(snsinfo, port);
    HB_MIPI_SensorBindSerdes(snsinfo, sedres_index, sedres_port);
    HB_MIPI_SensorBindMipi(snsinfo,  mipiIdx);
    ret = HB_MIPI_InitSensor(devId, snsInfo);
    if(ret < 0) {
        printf("HB_MIPI_InitSensor error!\n");
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }
    ret = HB_MIPI_SetMipiAttr(mipiIdx, mipiAttr);
    if(ret < 0) {
        printf("HB_MIPI_SetMipiAttr error! do sensorDeinit\n");
        HB_MIPI_SensorDeinit(sensorId);
        HB_VIN_DestroyPipe(PipeId);
        return ret;
    }

    ret = HB_VIN_EnableChn(PipeId, ChnId );
    if(ret < 0) {
        printf("HB_VIN_EnableChn error!\n");
        HB_MIPI_DeinitSensor(DevId );
        HB_MIPI_Clear(mipiIdx);
        HB_VIN_DestroyDev(pipeId);
        HB_VIN_DestroyChn(pipeId, ChnId);
        HB_VIN_DestroyPipe(pipeId);
        return ret;
    }
    ret = HB_VIN_StartPipe(PipeId);
    if(ret < 0) {
        printf("HB_VIN_StartPipe error!\n");
        HB_MIPI_DeinitSensor(DevId );
        HB_MIPI_Clear(mipiIdx);
        HB_VIN_DisableChn(pipeId, ChnId);
        HB_VIN_DestroyDev(pipeId);
        HB_VIN_DestroyChn(pipeId, ChnId);
        HB_VIN_DestroyPipe(pipeId);
        return ret;
    }
    ret = HB_VIN_EnableDev(DevId);
    if(ret < 0) {
        printf("HB_VIN_EnableDev error!\n");
        HB_MIPI_DeinitSensor(DevId );
        HB_MIPI_Clear(mipiIdx);
        HB_VIN_DisableChn(pipeId, ChnId);
        HB_VIN_StopPipe(pipeId);
        HB_VIN_DestroyDev(pipeId);
        HB_VIN_DestroyChn(pipeId, ChnId);
        HB_VIN_DestroyPipe(pipeId);
        return ret;
    }
    ret = HB_MIPI_ResetSensor(DevId );
    if(ret < 0) {
        printf("HB_MIPI_ResetSensor error! do mipi deinit\n");
        HB_MIPI_DeinitSensor(DevId );
        HB_MIPI_Clear(mipiIdx);
        HB_VIN_DisableDev(pipeId);
        HB_VIN_StopPipe(pipeId);
        HB_VIN_DisableChn(pipeId, ChnId);
        HB_VIN_DestroyDev(pipeId);
        HB_VIN_DestroyChn(pipeId, ChnId);
        HB_VIN_DestroyPipe(pipeId);
        return ret;
    }
    ret = HB_MIPI_ResetMipi(mipiIdx);
    if(ret < 0) {
        printf("HB_MIPI_ResetMipi error!\n");
        HB_MIPI_UnresetSensor(DevId );
        HB_MIPI_DeinitSensor(DevId );
        HB_MIPI_Clear(mipiIdx);
        HB_VIN_DisableDev(pipeId);
        HB_VIN_StopPipe(pipeId);
        HB_VIN_DisableChn(pipeId, ChnId);
        HB_VIN_DestroyDev(pipeId);
        HB_VIN_DestroyChn(pipeId, ChnId);
        HB_VIN_DestroyPipe(pipeId);
        return ret;
    }

    HB_MIPI_UnresetSensor(DevId );
    HB_MIPI_UnresetMipi(mipiIdx);
    HB_VIN_DisableDev(PipeId);
    HB_VIN_StopPipe(PipeId);
    HB_VIN_DisableChn(PipeId, ChnId);
    HB_MIPI_DeinitSensor(DevId );
    HB_MIPI_Clear(mipiIdx);
    HB_VIN_DestroyDev(DevId);
    HB_VIN_DestroyChn(PipeId, ChnId);
    HB_VIN_DestroyPipe(PipeId);
```

### HB_VIN_StartPipe/HB_VIN_StopPipe
【函数声明】
```c
int HB_VIN_StartPipe(uint32_t pipeId);
int HB_VIN_StopPipe(uint32_t pipeId);
```
【功能描述】
> 启动和停止pipe

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_EnableChn/HB_VIN_DisableChn
【函数声明】
```c
int HB_VIN_EnableChn(uint32_t pipeId, uint32_t chnId);
int HB_VIN_DisableChn(uint32_t pipeId, uint32_t chnId);
```
【功能描述】
> 对pipe的chn使能和关闭

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |
|  chnId   |       输入1即可       |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetChnLDCAttr/HB_VIN_GetChnLDCAttr
【函数声明】
```c
int HB_VIN_SetChnLDCAttr(uint32_t pipeId, uint32_t chnId,const VIN_LDC_ATTR_S *stVinLdcAttr);
int HB_VIN_GetChnLDCAttr(uint32_t pipeId, uint32_t chnId, VIN_LDC_ATTR_S*stVinLdcAttr);
```
【功能描述】
> 设置和获取LDC的属性

【参数描述】

|   参数名称   |         描述          |         输入/输出          |
| :----------: | :-------------------: | :------------------------: |
|    pipeId    | 对应每路输入，范围0~7 |            输入            |
|    chnId     |       输入1即可       |            输入            |
| stVinLdcAttr |     ldc的属性信息     | 输入，获取属性的时候为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> LDC有调整送往IPU数据时序的功能，在VIN_ISP与VPS模块是在线模式的情况下，必须要通过该接口配置LDC参数，否则VPS会出异常。VIN_ISP与VPS模块是离线模式LDC参数配置与否都不影响。

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetChnDISAttr/HB_VIN_GetChnDISAttr
【函数声明】
```c
int HB_VIN_SetChnDISAttr(uint32_t pipeId, uint32_t chnId, const VIN_DIS_ATTR_S *stVinDisAttr);
int HB_VIN_GetChnDISAttr(uint32_t pipeId, uint32_t chnId, VIN_DIS_ATTR_S *stVinDisAttr);
```
【功能描述】
> 设置和获取DIS的属性

【参数描述】

|   参数名称   |         描述          |         输入/输出          |
| :----------: | :-------------------: | :------------------------: |
|    pipeId    | 对应每路输入，范围0~7 |            输入            |
|    chnId     |       输入1即可       |            输入            |
| stVinDisAttr |     dis的属性信息     | 输入，获取属性的时候为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_SetChnAttr
【函数声明】
```c
int HB_VIN_SetChnAttr(uint32_t pipeId, uint32_t chnId);
```
【功能描述】
> 设置chn的属性

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |
|  chnId   |       输入1即可       |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> LDC和DIS的属性真正设置是在这个接口里面，HB_VIN_SetChnLDCAttr和HB_VIN_SetChnDISAttr只是给属性赋值。这个chn是指isp的其中一个输出chn,值固定为1。

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_DestroyChn
【函数声明】
```c
int HB_VIN_DestroyChn(uint32_t pipeId, uint32_t chnId)
```
【功能描述】
> 销毁chn

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |
|  chnId   |       输入1即可       |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 目前不支持HB_VIN_DestroyChn之后重新HB_VIN_SetChnAttr

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_GetChnFrame/HB_VIN_ReleaseChnFrame
【函数声明】
```c
int HB_VIN_GetChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame, int32_t millSec);
int HB_VIN_ReleaseChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame);
```
【功能描述】
> 获取pipe chn后的数据

【参数描述】

|   参数名称    |                                                                描述                                                                | 输入/输出 |
| :-----------: | :--------------------------------------------------------------------------------------------------------------------------------: | :-------: |
|    pipeId     |                                                       对应每路输入，范围0~7                                                        |   输入    |
|     chnId     |                                                             输入0即可                                                              |   输入    |
| pstVideoFrame |                                                              数据信息                                                              |   输出    |
|    millSec    | 超时参数 millSec<br/>设为-1 时，为阻塞接口；<br/>0 时为 非阻塞接口；<br/>大于 0 时为超时等待时间，<br/>超时时间的 单位为毫秒（ms） |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 此接口是获取ISP处理之后的图像

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_GetDevFrame/HB_VIN_ReleaseDevFrame
【函数声明】
```c
int HB_VIN_GetDevFrame(uint32_t devId, uint32_t chnId, void *videoFrame, int32_t millSec);
int HB_VIN_ReleaseDevFrame(uint32_t devId, uint32_t chnId, void *buf);
```
【功能描述】
> 获取sif chn处理后的数据，chn为0

【参数描述】

| 参数名称  |                                                                描述                                                                | 输入/输出 |
| :-------: | :--------------------------------------------------------------------------------------------------------------------------------: | :-------: |
|   devId   |                                                       对应每路输入，范围0~7                                                        |   输入    |
|   chnId   |                                                             输入0即可                                                              |   输入    |
| videoFram |                                                              数据信息                                                              |   输出    |
|  millSec  | 超时参数 millSec<br/>设为-1 时，为阻塞接口；<br/>0 时为 非阻塞接口；<br/>大于 0 时为超时等待时间，<br/>超时时间的 单位为毫秒（ms） |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 此接口是获取SIF处理之后的图像，sif –offline-isp得时候可以dump raw图，
适用场景：
>>VIN_OFFLINE_VPS_ONLINE
>>VIN_OFFLINE_VPS_OFFINE
>>VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE

另外sif-online-isp 同时sif到ddr也可以dump raw图，适用场景：
>>VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE
>>VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE

【参考代码】
```c
    typedef struct {
        uint32_t frame_id;
        uint32_t plane_count;
        uint32_t xres[MAX_PLANE];
        uint32_t yres[MAX_PLANE];
        char *addr[MAX_PLANE];
        uint32_t size[MAX_PLANE];
    } raw_t;
    typedef struct {
        uint8_t ctx_id;
        raw_t raw;
    } dump_info_t;
    dump_info_t dump_info = {0};
    hb_vio_buffer_t *sif_raw = NULL;
    int pipeId = 0;
    sif_raw = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
    memset(sif_raw, 0, sizeof(hb_vio_buffer_t));

    ret = HB_VIN_GetDevFrame(pipeId, 0, sif_raw, 2000);
    if (ret < 0) {
        printf("HB_VIN_GetDevFrame error!!!\n");
    } else {
        if (sif_raw->img_info.planeCount == 1) {
            dump_info.ctx_id = info->group_id;
            dump_info.raw.frame_id = sif_raw->img_info.frame_id;
            dump_info.raw.plane_count = sif_raw->img_info.planeCount;
            dump_info.raw.xres[0] = sif_raw->img_addr.width;
            dump_info.raw.yres[0] = sif_raw->img_addr.height;
            dump_info.raw.addr[0] = sif_raw->img_addr.addr[0];
            dump_info.raw.size[0] = size;
            printf("pipe(%d)dump normal raw frame id(%d),plane(%d)size(%d)\n",
                dump_info.ctx_id, dump_info.raw.frame_id,
                dump_info.raw.plane_count, size);
        } else if (sif_raw->img_info.planeCount == 2) {
            dump_info.ctx_id = info->group_id;
            dump_info.raw.frame_id = sif_raw->img_info.frame_id;
            dump_info.raw.plane_count = sif_raw->img_info.planeCount;
            for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
                dump_info.raw.xres[i] = sif_raw->img_addr.width;
                dump_info.raw.yres[i] = sif_raw->img_addr.height;
                dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
                dump_info.raw.size[i] = size;
            }
            if(sif_raw->img_info.img_format == 0) {
                printf("pipe(%d)dump dol2 raw frame id(%d),plane(%d)size(%d)\n",
                    dump_info.ctx_id, dump_info.raw.frame_id,
                    dump_info.raw.plane_count, size);
                }
            } else if (sif_raw->img_info.planeCount == 3) {
                dump_info.ctx_id = info->group_id;
                dump_info.raw.frame_id = sif_raw->img_info.frame_id;
                dump_info.raw.plane_count = sif_raw->img_info.planeCount;
                for (int i = 0; i < sif_raw->img_info.planeCount; i ++) {
                    dump_info.raw.xres[i] = sif_raw->img_addr.width;
                    dump_info.raw.yres[i] = sif_raw->img_addr.height;
                    dump_info.raw.addr[i] = sif_raw->img_addr.addr[i];
                    dump_info.raw.size[i] = size;
                }
                printf("pipe(%d)dump dol3 raw frame id(%d),plane(%d)size(%d)\n",
                dump_info.ctx_id, dump_info.raw.frame_id,
                dump_info.raw.plane_count, size);
            } else {
                printf("pipe(%d)raw buf planeCount wrong !!!\n", info->group_id);
            }
            for (int i = 0; i < dump_info.raw.plane_count; i ++) {
                if(sif_raw->img_info.img_format == 0) {
                    sprintf(file_name, "pipe%d_plane%d_%ux%u_frame_%03d.raw",
                            dump_info.ctx_id,
                            i,
                            dump_info.raw.xres[i],
                            dump_info.raw.yres[i],
                            dump_info.raw.frame_id);
                    dumpToFile(file_name,  dump_info.raw.addr[i], dump_info.raw.size[i]);
                }
            }
            if(sif_raw->img_info.img_format == 8) {
                sprintf(file_name, "pipe%d_%ux%u_frame_%03d.yuv",
                        dump_info.ctx_id,
                        dump_info.raw.xres[i],
                        dump_info.raw.yres[i],
                        dump_info.raw.frame_id);
                dumpToFile2plane(file_name, sif_raw->img_addr.addr[0],
                    sif_raw->img_addr.addr[1], size, size/2);
            }
        }
        ret = HB_VIN_ReleaseDevFrame(pipeId, 0, sif_raw);
        if (ret < 0) {
            printf("HB_VIN_ReleaseDevFrame error!!!\n");
        }
        free(sif_raw);
        sif_raw = NULL;
    }

    int dumpToFile(char *filename, char *srcBuf, unsigned int size)
    {
        FILE *yuvFd = NULL;
        char *buffer = NULL;

        yuvFd = fopen(filename, "w+");
        if (yuvFd == NULL) {
            vio_err("ERRopen(%s) fail", filename);
            return -1;
        }
        buffer = (char *)malloc(size);
        if (buffer == NULL) {
            vio_err(":malloc file");
            fclose(yuvFd);
            return -1;
        }
        memcpy(buffer, srcBuf, size);
        fflush(stdout);
        fwrite(buffer, 1, size, yuvFd);
        fflush(yuvFd);
        if (yuvFd)
            fclose(yuvFd);
        if (buffer)
        free(buffer);
        vio_dbg("filedump(%s, size(%d) is successed\n", filename, size);
        return 0;
    }
    int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
                        unsigned int size, unsigned int size1)
    {
        FILE *yuvFd = NULL;
        char *buffer = NULL;

        yuvFd = fopen(filename, "w+");
        if (yuvFd == NULL) {
            vio_err("open(%s) fail", filename);
            return -1;
        }
        buffer = (char *)malloc(size + size1);
        if (buffer == NULL) {
            vio_err("ERR:malloc file");
            fclose(yuvFd);
            return -1;
        }
        memcpy(buffer, srcBuf, size);
        memcpy(buffer + size, srcBuf1, size1);
        fflush(stdout);
        fwrite(buffer, 1, size + size1, yuvFd);
        fflush(yuvFd);
        if (yuvFd)
            fclose(yuvFd);
        if (buffer)
            free(buffer);
        vio_dbg("filedump(%s, size(%d) is successed\n", filename, size);
        return 0;
    }
```

### HB_VIN_SendPipeRaw
【函数声明】
```c
int HB_VIN_SendPipeRaw(uint32_t pipeId, void *pstVideoFrame，int32_t millSec)
```
【功能描述】
> 回灌raw接口，数据给ISP处理

【参数描述】

|   参数名称    |                                                                描述                                                                | 输入/输出 |
| :-----------: | :--------------------------------------------------------------------------------------------------------------------------------: | :-------: |
|    pipeId     |                                                       对应每路输入，范围0~7                                                        |   输入    |
| pstVideoFrame |                                                          回灌raw数据信息                                                           |   输入    |
|    millSec    | 超时参数 millSec<br/>设为-1 时，为阻塞接口；<br/>0 时为 非阻塞接口；<br/>大于 0 时为超时等待时间，<br/>超时时间的 单位为毫秒（ms） |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
```c
    int pipeId = 0;
    hb_vio_buffer_t *feedback_buf;
    hb_vio_buffer_t *isp_yuv = NULL;
    isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
    memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));
    ret = HB_VIN_SendPipeRaw(pipeId, feedback_buf,1000);
    if (ret) {
        printf("HB_VIN_SendFrame error!!!\n");
    }
    ret = HB_VIN_GetChnFrame(pipeId, 0, isp_yuv, -1);
    if (ret < 0) {
        printf("HB_VIN_GetPipeFrame error!!!\n");
    }
    ret = HB_VIN_ReleaseChnFrame(pipeId, 0, isp_yuv);
    if (ret < 0) {
        printf("HB_VPS_ReleaseDevRaw error!!!\n");
    }
```

### HB_VIN_SetPipeAttr/HB_VIN_GetPipeAttr
【函数声明】
```c
int HB_VIN_SetPipeAttr(uint32_t pipeId,VIN_PIPE_ATTR_S *stVinPipeAttr);
int HB_VIN_GetPipeAttr(uint32_t pipeId, VIN_PIPE_ATTR_S *stVinPipeAttr);
```
【功能描述】
> 设置pipe（ISP）属性、获取pipe属性

【参数描述】

|   参数名称    |         描述          |       输入/输出       |
| :-----------: | :-------------------: | :-------------------: |
|    pipeId     | 对应每路输入，范围0~7 |         输入          |
| stVinPipeAttr |  描述pipe属性的指针   | 输入，get的时候为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_CreatePipe/HB_VIN_DestroyPipe举例

### HB_VIN_CtrlPipeMirror
【函数声明】
```c
int HB_VIN_CtrlPipeMirror(uint32_t pipeId, uint8_t on);
```
【功能描述】
> pipe镜像控制。

【参数描述】

| 参数名称 |               描述               | 输入/输出 |
| :------: | :------------------------------: | :-------: |
|  pipeId  |      对应每路输入，范围0~7       |   输入    |
|    on    | 非0打开镜像功能，0关闭镜像功能。 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> Flip功能需要借助GDC实现，如先把镜像打开然后再旋转180度。

### HB_VIN_MotionDetect
【函数声明】
```c
int HB_VIN_MotionDetect(uint32_t pipeId)
```
【功能描述】
> 检测MD是否有中断,有MD中断就返回

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 |                         描述                         |
| :----: | :--------------------------------------------------: |
|   0    | 检测到运行物体，阻塞调用，未检测到运动物体一直阻塞。 |

【注意事项】
> 无

【参考代码】
> 请参见HB_VIN_EnableDevMd举例

### HB_VIN_InitLens
【函数声明】
```c
int HB_VIN_InitLens(uint32_t pipeId, VIN_LENS_FUNC_TYPE_E lensType, const VIN_LENS_CTRL_ATTR_S *lenCtlAttr)
```
【功能描述】
> 马达驱动初始化。

【参数描述】

|  参数名称  |             描述             | 输入/输出 |
| :--------: | :--------------------------: | :-------: |
|   pipeId   |    对应每路输入，范围0~7     |   输入    |
|  lensType  | 马达的功能类型，AF、Zoom功能 |   输入    |
| lenCtlAttr |           控制属性           |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 如果使用AF调用一次接口，如果同时使用AF和Zoom功能，调用两次初始化。使用就去调用，不使用建议不调用。

【参考代码】
> 暂无

### HB_VIN_DeinitLens
【函数声明】
```c
int HB_VIN_DeinitLens(uint32_t pipeId)
```
【功能描述】
> 马达退出

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 暂无

### HB_VIN_RegisterDisCallback
【函数声明】
```c
int HB_VIN_RegisterDisCallback(uint32_t pipeId, VIN_DIS_CALLBACK_S *pstDISCallback)
```
【功能描述】
> 注册dis回调

【参数描述】

|    参数名称    |         描述          | 输入/输出 |
| :------------: | :-------------------: | :-------: |
|     pipeId     | 对应每路输入，范围0~7 |   输入    |
| pstDISCallback |       回调接口        |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 暂无

### HB_VIN_SetDevVCNumber/HB_VIN_GetDevVCNumber
【函数声明】
```c
int HB_VIN_SetDevVCNumber(uint32_t devId, uint32_t vcNumber);
int HB_VIN_GetDevVCNumber(uint32_t devId, uint32_t *vcNumber);
```
【功能描述】
> 设置和获取dev的vc_index，使用MIPI的哪个vc.

【参数描述】

| 参数名称 |         描述          |       输入/输出        |
| :------: | :-------------------: | :--------------------: |
|  devId   | 对应每路输入，范围0~7 |          输入          |
| vcNumber | 对应mipi的vc,范围0~3  | 输入，获取的时候为输出 |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 无

### HB_VIN_AddDevVCNumber
【函数声明】
```c
int HB_VIN_AddDevVCNumber(uint32_t devId, uint32_t vcNumber)
```
【功能描述】
> 设置dev的vc_index,使用MIPI的哪个vc.

【参数描述】

| 参数名称 |          描述           | 输入/输出 |
| :------: | :---------------------: | :-------: |
|  devId   | 对应每路输vc入，范围0~7 |   输入    |
| vcNumber |  对应mipi的vc,范围0~3   |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 当使用linear模式时，这个接口不用使用，当使用DOL2模式时，此接口vcNumber设置为1，当使用DOL3模式时，调用两次HB_VIN_AddDevVCNumber，vcNumber分别传0和1.

【参考代码】
> 一路DOL2
> 初始化顺序：
> 1)  把dev0绑到mipi0
> HB_VIN_SetMipiBindDev(0, 0)
> 2)  把mipi0的虚通道0绑到dev0
> HB_VIN_SetDevVCNumber(0, 0)
> 3)  把mipi0的虚通道1绑到dev0
> HB_VIN_AddDevVCNumber(0, 1);
> 4)  把dev0分别绑到ISP pipe0,
> HB_VIN_SetDevBindPipe(0, 0)
```c
    ret = HB_SYS_SetVINVPSMode(pipeId, vin_vps_mode);
    if(ret < 0) {
        printf("HB_SYS_SetVINVPSMode%d error!\n", vin_vps_mode);
        return ret;
    }
    ret = HB_VIN_CreatePipe(pipeId, pipeinfo);   // isp init
    if(ret < 0) {
        printf("HB_MIPI_InitSensor error!\n");
        return ret;
    }
    ret = HB_VIN_SetMipiBindDev(pipeId, mipiIdx);
    if(ret < 0) {
        printf("HB_VIN_SetMipiBindDev error!\n");
        return ret;
    }
    ret = HB_VIN_SetDevVCNumber(pipeId, deseri_port);
    if(ret < 0) {
        printf("HB_VIN_SetDevVCNumber error!\n");
        return ret;
    }
    ret = HB_VIN_AddDevVCNumber(pipeId, vc_num);
    if(ret < 0) {
        printf("HB_VIN_AddDevVCNumber error!\n");
        return ret;
    }
    ret = HB_VIN_SetDevAttr(pipeId, devinfo);
    if(ret < 0) {
        printf("HB_MIPI_InitSensor error!\n");
        return ret;
    }
    ret = HB_VIN_SetPipeAttr(pipeId, pipeinfo);
    if(ret < 0) {
        printf("HB_VIN_SetPipeAttr error!\n");
        goto pipe_err;
    }
    ret = HB_VIN_SetChnDISAttr(pipeId, 1, disinfo);
    if(ret < 0) {
        printf("HB_VIN_SetChnDISAttr error!\n");
        goto pipe_err;
    }
    ret = HB_VIN_SetChnLDCAttr(pipeId, 1, ldcinfo);
    if(ret < 0) {
        printf("HB_VIN_SetChnLDCAttr error!\n");
        goto pipe_err;
    }
    ret = HB_VIN_SetChnAttr(pipeId, 1);
    if(ret < 0) {
        printf("HB_VIN_SetChnAttr error!\n");
        goto chn_err;
    }
    HB_VIN_SetDevBindPipe(pipeId, pipeId);
```

### HB_VIN_SetDevMclk
【函数声明】
```c
int HB_VIN_SetDevMclk(uint32_t devId, uint32_t devMclk, uint32_t vpuMclk);
```
【功能描述】
> 设置sif mclk和vpu clk.

【参数描述】

| 参数名称 |             描述             |   输入/输出   |
| :------: | :--------------------------: | :-----------: |
|  devId   |    对应每路输入，范围0~7     |     输入      |
| devMclk  | Sif mclk设置，请参见SIF MCLK | 输入，单位KHz |
| vpuMclk  |  vpu clk设置, 请参见VPU CLK  | 输入，单位KHz |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 暂无

### HB_VIN_GetChnFd
【函数声明】
```c
int HB_VIN_GetChnFd(uint32_t pipeId, uint32_t chnId)
```
【功能描述】
> 获取通道的fd

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  pipeId  | 对应每路输入，范围0~7 |   输入    |
|  chnId   |      通道号，为0      |   输入    |

【返回值】

| 返回值 | 描述  |
| :----: | :---: |
|  正值  | 成功  |
|  负值  | 失败  |

【注意事项】
> 无

【参考代码】
> 暂无

### HB_VIN_CloseFd
【函数声明】
```c
int HB_VIN_CloseFd(void)
```
【功能描述】
> 关闭通道的fd

【参数描述】

| 参数名称 | 描述  | 输入/输出 |
| :------: | :---: | :-------: |
|   void   |  无   |  无输入   |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 无

【参考代码】
> 暂无

### HB_VIN_EnableDevMd
【函数声明】
```c
int HB_VIN_EnableDevMd(uint32_t devId)
```
【功能描述】
> 打开motiondetect功能

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  devId   | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 调用得在HB_VIN_SetDevAttrEx之后，HB_VIN_SetDevAttrEx接口是设置MD的一些属性值

【参考代码】
```c
    VIN_DEV_ATTR_EX_S devAttr;
    devAttr. path_sel = 0;
    devAttr. roi_top = 0;
    devAttr. roi_left = 0;
    devAttr. roi_width = 1280;
    devAttr. roi_height = 640;
    devAttr. grid_step = 128;
    devAttr. grid_tolerance =10;
    devAttr. threshold = 10;
    devAttr. weight_decay = 128;
    devAttr. precision = 0;
    ret = HB_VIN_SetDevAttrEx(pipeId, devexinfo);
    if(ret < 0) {
        printf("HB_VIN_SetDevAttrEx error!\n");
        return ret;
    }
    ret = HB_VIN_EnableDevMd(pipeId);
    if(ret < 0) {
        printf("HB_VIN_EnableDevMd error!\n");
        return ret;
    }
```
下面起一个线程调用HB_VIN_MotionDetect检测收到MD中断后将MD功能关闭HB_VIN_DisableDevMd。
```c
    int md_func(work_info_t * info)
    {
        int ret = 0;
        int pipeId = info->group_id;
        ret =  HB_VIN_MotionDetect(pipeId);
        if (ret < 0) {
            printf("HB_VIN_MotionDetect error!!! ret %d \n", ret);
        } else {
            HB_VIN_DisableDevMd(pipeId);
            printf("HB_VIN_DisableDevMd success!!! ret %d \n", ret);
        }
        return ret;
    }
```

### HB_VIN_DisableDevMd
【函数声明】
```c
int HB_VIN_DisableDevMd(uint32_t devId)
```
【功能描述】
> 关闭motiondetect功能

【参数描述】

| 参数名称 |         描述          | 输入/输出 |
| :------: | :-------------------: | :-------: |
|  devId   | 对应每路输入，范围0~7 |   输入    |

【返回值】

| 返回值 | 描述 |
|:------:|:----:|
|    0   | 成功 |
|   非0  | 失败 |

【注意事项】
> 用户收到md中断后关闭md功能

【参考代码】
> 请参见HB_VIN_EnableDevMd举例

## 数据结构

### MIPI_INPUT_MODE_E
【结构定义】
```c
typedef enum HB_MIPI_INPUT_MODE_E
{
    INPUT_MODE_MIPI         = 0x0,              /* mipi */
    INPUT_MODE_DVP         = 0x1,              /* DVP*/
    INPUT_MODE_BUTT
} MIPI_INPUT_MODE_E;
```
【功能描述】
> sensor接入方式

【成员说明】
- MIPI接入
- DVP接入

### MIPI_SENSOR_MODE_E
【结构定义】
```c
typedef enum HB_MIPI_SENSOR_MODE_E
{
    NORMAL_M             = 0x0,
    DOL2_M               = 0x1,
    DOL3_M               = 0x2,
    PWL_M                = 0x3,
} MIPI_SENSOR_MODE_E;
```
【功能描述】
> sensor工作模式

【成员说明】
> linear模式、DOL2模式、DOL3模式、PWL模式

### MIPI_DESERIAL_INFO_T
【结构定义】
```c
typedef struct HB_MIPI_DESERIAL_INFO_T {
    int bus_type;
    int bus_num;
    int deserial_addr;
    int physical_entry;
    char *deserial_name;
} MIPI_DESERIAL_INFO_T;
```
【功能描述】
> 定义serdes初始化的属性信息

【成员说明】

|      成员      | 含义                                         |
| :------------: | :------------------------------------------- |
|    bus_type    | 总线类型，0是i2c,1是spi                      |
|    bus_num     | 总线号,根据具体板子硬件原理图确定，目前用的5 |
| deserial_addr  | serdes地址                                   |
| physical_entry | 保留                                         |
| deserial_name  | serdes名字                                   |

### MIPI_SNS_INFO_S
【结构定义】
```c
typedef struct HB_MIPI_SNS_INFO_S {
    int port;
    int dev_port;
    int bus_type;
    int bus_num;
    int fps;
    int resolution;
    int sensor_addr;
    int serial_addr;
    int entry_index;
    MIPI_SENSOR_MODE_E sensor_mode;
    int reg_width;
    char *sensor_name;
    int extra_mode;
    int deserial_index;
    int deserial_port;
    int gpio_num;
    int gpio_pin[GPIO_NUM];
    int gpio_level[GPIO_NUM];
    MIPI_SPI_DATA_S spi_info;
} MIPI_SNS_INFO_S;
```
【功能描述】
> 定义sensor初始化的属性信息

【成员说明】

|      成员      | 含义                                                                       |
| :------------: | :------------------------------------------------------------------------- |
|      port      | 当前sensor的一个逻辑编号，必须从0开始                                      |
|    dev_port    | 每路sensor操作的驱动节点，一个驱动支持多个节点。                           |
|    bus_type    | 总线类型，0是i2c,1是spi                                                    |
|    bus_num     | 总线号，根据具体板子硬件原理图确定,现在默认i2c5                            |
|      fps       | 帧率                                                                       |
|   resolution   | Sensor的分辨率                                                             |
|  sensor_addr   | sensor地址                                                                 |
|  serial_addr   | sensor内部serdes地址                                                       |
|  entry_index   | sensor使用的mipi索引                                                       |
|  sensor_mode   | sensor工作模式，1是normal,2是dol2,3是dol3                                  |
|   reg_width    | 寄存器地址宽度                                                             |
|  sensor_name   | sensor名字                                                                 |
|   extra_mode   | 区分sensor的特性，具体sensor驱动实现                                       |
| deserial_index | 当前属于哪一个serdes                                                       |
| deserial_port  | 当前属于serdes哪一个port                                                   |
|    gpio_num    | 有的sensor需要gpio上下电，此sensor用到的相关GPIO管脚                       |
|    gpio_pin    | 操作的GPIO管脚，GPIO_NUM是用到的GPIO管脚的个数                             |
|   gpio_level   | 初始有效值，比如该管脚需要先拉低再拉高，此值为0，如果先拉高在拉低，此值为1 |
|    spi_info    | sensor spi信息，有的sensor通过spi总线访问                                  |

### MIPI_SENSOR_INFO_S
【结构定义】
```c
typedef struct HB_MIPI_SENSOR_INFO_S {
    int    deseEnable;
    MIPI_INPUT_MODE_E  inputMode;
    MIPI_DESERIAL_INFO_T deserialInfo;
    MIPI_SNS_INFO_S  sensorInfo;
} MIPI_SENSOR_INFO_S;
```
【功能描述】
> 定义dev初始化的属性信息

【成员说明】

|     成员     | 含义                 |
| :----------: | :------------------- |
|  deseEnable  | 该sensor是否有serdes |
|  inputMode   | sensor接入方式       |
| deserialInfo | serdes信息           |
|  sensorInfo  | sensor信息           |

### MIPI_HOST_CFG_S
【结构定义】
```c
typedef struct HB_MIPI_HOST_CFG_S {
    uint16_t  lane;
    uint16_t  datatype;
    uint16_t  mclk;
    uint16_t  mipiclk;
    uint16_t  fps;
    uint16_t  width;
    uint16_t  height;
    uint16_t  linelenth;
    uint16_t  framelenth;
    uint16_t  settle;
    uint16_t  channel_num;
    uint16_t  channel_sel[4];
} MIPI_HOST_CFG_S;
```
【功能描述】
> 定义mipi初始化参数信息

【成员说明】

|      成员      | 含义                                                   |
| :------------: | :----------------------------------------------------- |
|      lane      | lane个数，0~4                                          |
|    datatype    | 数据格式,参见DATA TYPE                                 |
|      mclk      | mipi模块主时钟，目前固定是24MHZ                        |
|    mipiclk     | sensor 输出 总的mipi bit rate, 单位 Mbits/每秒         |
|      fps       | sensor输出实际帧率                                     |
|     width      | sensor输出实际宽度                                     |
|     height     | sensor输出实际高度                                     |
|   linelenth    | sensor输出带blanking的总行长                           |
|   framelenth   | sensor输出带blanking的总行数                           |
|     settle     | sensor输出实际 Ttx-zero + Ttx-prepare时间（clk为单位） |
|  channel_num   | 使用虚通道的个数                                       |
| channel_sel[4] | 保存每个虚通道的值                                     |

### MIPI_ATTR_S
【结构定义】
```c
typedef struct HB_MIPI_ATTR_S {
    MIPI_HOST_CFG_S mipi_host_cfg;
    uint32_t  dev_enable;
} MIPI_ATTR_S;
```
【功能描述】
> 定义mipi初始化参数信息

【成员说明】

|     成员      | 含义                               |
| :-----------: | :--------------------------------- |
| mipi_host_cfg | mipi host属性结构体                |
|  dev_enable   | mipi dev是否使能，1是使能，0是关闭 |

### MIPI_SPI_DATA_S
【结构定义】
```c
typedef struct HB_MIPI_SPI_DATA_S {
    int spi_mode;
    int spi_cs;
    uint32_t spi_speed;
} MIPI_SPI_DATA_S;
```
【功能描述】
> 定义sensor相关spi信息

【成员说明】

|   成员    | 含义          |
| :-------: | :------------ |
| spi_mode  | spi的工作模式 |
|  spi_cs   | spi的片选     |
| spi_speed | spi的传输速率 |

### VIN_DEV_SIZE_S
【结构定义】
```c
typedef struct HB_VIN_DEV_SIZE_S {
    uint32_t  format;
    uint32_t  width;
    uint32_t  height;
    uint32_t  pix_length;
} VIN_DEV_SIZE_S;
```
【功能描述】
> 定义dev初始化的属性信息

【成员说明】

|    成员    | 含义                                                                            |
| :--------: | :------------------------------------------------------------------------------ |
|   format   | 像素格式，format为0代表是raw8~raw16,根据pixel_lenght来表示究竟是raw8还是raw16。 |
|   width    | 数据宽                                                                          |
|   height   | 数据高                                                                          |
| pix_length | 每个像素点长度                                                                  |

### VIN_MIPI_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_MIPI_ATTR_S {
    uint32_t  enable;
    uint32_t  ipi_channels;
    uint32_t  ipi_mode;
    uint32_t  enable_mux_out;
    uint32_t  enable_frame_id;
    uint32_t  enable_bypass;
    uint32_t  enable_line_shift;
    uint32_t  enable_id_decoder;
    uint32_t  set_init_frame_id;
    uint32_t  set_line_shift_count;
    uint32_t  set_bypass_channels;
    uint32_t  enable_pattern;
} VIN_MIPI_ATTR_S;
```
【功能描述】
> 定义dev mipi初始化的信息

【成员说明】

|         成员         | 含义                                                                                        |
| :------------------: | :------------------------------------------------------------------------------------------ |
|        enable        | mipi使能,0是关闭，1是使能                                                                   |
|     ipi_channels     | ipi_channels表示用了几个channel，默认是0开始，如果设置是2，是用了0，1                       |
|       ipi_mode       | 当DOL2分成两路linear或者DOL3分成一路DOl2和一路linear或者三路linear的时候，此值就赋值为2或3. |
|    enable_mux_out    | 使能mux选择输出                                                                             |
|   enable_frame_id    | 是否使能frameid                                                                             |
|    enable_bypass     | 是否使能bypass                                                                              |
|  enable_line_shift   | 未用                                                                                        |
|  enable_id_decoder   | 未用                                                                                        |
|  set_init_frame_id   | 初始frame id值一般为1                                                                       |
| set_line_shift_count | 未用                                                                                        |
| set_bypass_channels  | 未用                                                                                        |
|    enable_pattern    | 是否使能testpartern                                                                         |

### VIN_DEV_INPUT_DDR_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_DEV_INPUT_DDR_ATTR_S {
    uint32_t stride;
    uint32_t buf_num;
    uint32_t raw_feedback_en;
    VIN_DEV_SIZE_S data;
} VIN_DEV_INPUT_DDR_ATTR_S;
```
【功能描述】
> 定义dev输入信息，offline和回灌场景用

【成员说明】

|      成员       | 含义                                                         |
| :-------------: | :----------------------------------------------------------- |
|     stride      | 硬件stride 跟格式匹配，如果是12bit那么stride = widthx1.5，如果是10bit，stride = widthx1.25,如此类推 |
|     buf_num     | 回灌的存储数据的 buf 数目                                    |
| raw_feedback_en | 使能回灌模式，不能和offline 模式同时开启，独立使用           |
|      data       | 数据格式，见 VIN_DEV_SIZE_S                                  |

### VIN_DEV_OUTPUT_DDR_S
【结构定义】
```c
typedef struct HB_VIN_DEV_OUTPUT_DDR_S {
    uint32_t stride;
    uint32_t buffer_num;
    uint32_t frameDepth
} VIN_DEV_OUTPUT_DDR_S;
```
【功能描述】
> 定义dev 输出到ddr初始化的信息

【成员说明】

|    成员    | 含义                                                                            |
| :--------: | :------------------------------------------------------------------------------ |
|   stride   | 硬件stride 跟格式匹配，目前12bit  1952x1.5                                      |
| buffer_num | dev 输出到ddr 的buf 个数                                                        |
| frameDepth | 最多get的帧数, buffer_num是总buff数量，建议frameDepth值最大是ddrOutBufNum – 4。 |

### VIN_DEV_OUTPUT_ISP_S
【结构定义】
```c
typedef struct HB_VIN_DEV_OUTPUT_ISP_S {
    uint32_t dol_exp_num;
    uint32_t enable_dgain;
    uint32_t set_dgain_short;
    uint32_t set_dgain_medium;
    uint32_t set_dgain_long;
    uint32_t short_maxexp_lines;
    uint32_t medium_maxexp_lines;
    uint32_t vc_short_seq;
    uint32_t vc_medium_seq;
    uint32_t vc_long_seq;
} VIN_DEV_OUTPUT_ISP_S;
```
【功能描述】
> 定义dev 输出到pipe初始化的信息

【成员说明】

|        成员         | 含义                                                                                |
| :-----------------: | :---------------------------------------------------------------------------------- |
|     dol_exp_num     | 曝光模式，1 为普通模式，dol 2 或者 3 设置对应数目                                   |
|    enable_dgain     | ISP内部调试参数，暂可忽略                                                           |
|   set_dgain_short   | ISP 内部调试参数，暂可忽略                                                          |
|  set_dgain_medium   | ISP 内部调试参数，暂可忽略                                                          |
|   set_dgain_long    | ISP 内部调试参数，暂可忽略                                                          |
| short_maxexp_lines  | 最短帧的最大曝光行数，一般是sensor mode寄存器表中找，DOL2/3需要填，用来分配IRAM大小 |
| medium_maxexp_lines | 普通帧的最大曝光行数，一般是sensor mode寄存器表中找，DOL3需要填，用来分配IRAM大小   |
|    vc_short_seq     | 用来描述DOL2/3模式下，短帧的顺序                                                    |
|    vc_medium_seq    | 用来描述DOL2/3模式下，普通帧的顺序                                                  |
|     vc_long_seq     | 用来描述DOL2/3模式下，长帧的顺序                                                    |

### VIN_DEV_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_DEV_ATTR_S {
    VIN_DEV_SIZE_S        stSize;
    union
    {
        VIN_MIPI_ATTR_S  mipiAttr;
        VIN_DVP_ATTR_S   dvpAttr;
    };
    VIN_DEV_INPUT_DDR_ATTR_S DdrIspAttr;
    VIN_DEV_OUTPUT_DDR_S outDdrAttr;
    VIN_DEV_OUTPUT_ISP_S outIspAttr;
    }VIN_DEV_ATTR_S;
```
【功能描述】
> 定义dev初始化的属性信息

【成员说明】

|        成员         | 含义                                                        |
| :-----------------: | :---------------------------------------------------------- |
|   VIN_DEV_SIZE_S    | stSize 输入的数据                                           |
| VIN_DEV_INTF_MODE_E | enIntfMode sif(dev)输入的接口模式，mipi or dvp,目前都是mipi |
|     DdrIspAttr      | isp(pipe)的输入属性配置，offline或者是回灌                  |
|     outDdrAttr      | sif(dev)的输出到ddr配置                                     |
|     outIspAttr      | sif到isp一些属性设置                                        |

### VIN_DEV_ATTR_EX_S
【结构定义】
```c
typedef struct HB_VIN_DEV_ATTR_EX_S {
    uint32_t path_sel;
    uint32_t roi_top;
    uint32_t roi_left;
    uint32_t roi_width;
    uint32_t roi_height;
    uint32_t grid_step;
    uint32_t grid_tolerance;
    uint32_t threshold;
    uint32_t weight_decay;
    uint32_t precision;
}VIN_DEV_ATTR_EX_S;
```
【功能描述】
> 定义md相关信息

【成员说明】

|      成员      | 含义                                                                                                                                                   |
| :------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------- |
|    path_sel    | 0：sif-isp通路；1：sif-ipu通路                                                                                                                         |
|    roi_top     | ROI的y坐标                                                                                                                                             |
|    roi_left    | ROI的x坐标                                                                                                                                             |
|   roi_width    | ROI的长，必须是step的整数s倍                                                                                                                           |
|   roi_height   | ROI的宽， 必须是step的整数倍                                                                                                                           |
|   grid_step    | 对应motion detect的区域中划分的每块的宽和高。为2的整数次幂，有效范围为4~128。                                                                          |
| grid_tolerance | 每个块前后两帧进行比较的阈值。当前后两帧中相同块进行相减，插值超过这个阈值时，判断为不同。                                                             |
|   threshold    | 动态检测选取的ROI区域中划分的块比较不同的个数超过这个阈值,发出mot_det中断。                                                                            |
|  weight_decay  | 新的一帧更新ref buffer时不是完全替代上一帧的数据，而是前后两帧加权平均的结果。Mot_det_wgt_decay为当前帧的权重，前一帧的权重为(256-mot_det_wgt_decay)。 |
|   precision    | 为进行每个块计算时保留的小数点后的精度的位数，有效范围为1~4.                                                                                           |

### VIN_PIPE_SENSOR_MODE_E
【结构定义】
```c
typedef enum HB_VIN_PIPE_SENSOR_MODE_E {
    SENSOR_NORMAL_MODE = 1,
    SENSOR_DOL2_MODE,
    SENSOR_DOL3_MODE,
    SENSOR_DOL4_MODE,
    SENSOR_PWL_MODE,
    SENSOR_INVAILD_MODE
} VIN_PIPE_SENSOR_MODE_E;
```
【功能描述】
> sensor工作模式

【成员说明】
> normal模式、DOL2模式、DOL3模式、PWL模式（压缩模式）

### VIN_PIPE_CFA_PATTERN_E
【结构定义】
```c
typedef enum HB_VIN_PIPE_CFA_PATTERN_E {
    PIPE_BAYER_RGGB = 0,
    PIPE_BAYER_GRBG,
    PIPE_BAYER_GBRG,
    PIPE_BAYER_BGGR,
    PIPE_MONOCHROME,
} VIN_PIPE_CFA_PATTERN_E;
```
【功能描述】
> 数据格式布局

【成员说明】
>不同的数据存储格式

### VIN_PIPE_SIZE_S
【结构定义】
```c
typedef struct HB_VIN_PIPE_SIZE_S {
    uint32_t  format;
    uint32_t  width;
    uint32_t  height;
} VIN_PIPE_SIZE_S;
```
【功能描述】
> 定义pipe size 数据信息

【成员说明】

|  成员  | 含义     |
| :----: | :------- |
| format | 数据格式 |
| width  | 数据宽   |
| height | 数据高   |

### VIN_PIPE_CALIB_S
【结构定义】
```c
typedef struct HB_VIN_PIPE_CALIB_S {
    uint32_t mode;
    unsigned char *lname;
} VIN_PIPE_CALIB_S;
```
【功能描述】
> sensor矫正数据加载

【成员说明】

| 成员  | 含义                       |
| :---: | :------------------------- |
| mode  | 是否开启sensor矫正数据加载 |
| lname | 对应使用的校准库           |

### VIN_PIPE_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_PIPE_ATTR_S {
    uint32_t  ddrOutBufNum;
    uint32_t  frameDepth;
    VIN_PIPE_SENSOR_MODE_E snsMode;
    VIN_PIPE_SIZE_S stSize;
    VIN_PIPE_CFA_PATTERN_E cfaPattern;
    uint32_t   temperMode;
    uint32_t   ispBypassEn;
    uint32_t   ispAlgoState;
    uint32_t   ispAfEn;s
    uint32_t   bitwidth;
    uint32_t   startX;
    uint32_t   startY;
    VIN_PIPE_CALIB_S calib;
} VIN_PIPE_ATTR_S;
```
【功能描述】
> 定义pipe属性信息

【成员说明】

|     成员     | 含义                                                         |
| :----------: | :----------------------------------------------------------- |
| ddrOutBufNum | 数据的位宽，8 \10\12\14\16                                   |
|  frameDepth  | 最多get的帧数, ddrOutBufNum是总buff数量，建议frameDepth值最大是ddrOutBufNum – 3。 |
|   snsMode    | sensor工作模式                                               |
|    stSize    | sensor的数据信息，见17                                       |
|  cfaPattern  | 数据格式布局，和sensor保持一致                               |
|  temperMode  | temper模式，0关闭，2打开                                     |
| BypassEnable | 是否使能isp的bypass                                          |
| ispAlgoState | 是否启动3a算法库,1是启动，0是关闭                            |
|   bitwidth   | 位宽，有效值 8、10、12、14、16、20                           |
|    startX    | 相对于原点的X偏移                                            |
|    startY    | 相对于原点的Y偏移                                            |
|    calib     | 是否开启sensor矫正数据加载，1是开启，0是关闭。               |

### VIN_LDC_PATH_SEL_S
【结构定义】
```c
typedef struct HB_VIN_LDC_PATH_SEL_S {
    uint32_t rg_y_only:1;
    uint32_t rg_uv_mode:1;
    uint32_t rg_uv_interpo:1;
    uint32_t reserved1:5;
    uint32_t rg_h_blank_cyc:8;
    uint32_t reserved0:16;
} VIN_LDC_PATH_SEL_S;
```
【功能描述】
> 定义LDC属性信息

【成员说明】

|      成员      | 含义      |
| :------------: | :-------- |
|   rg_y_only    | 输出类型  |
|   rg_uv_mode   | 输出类型  |
| rg_uv_interpo  | turning用 |
| rg_h_blank_cyc | turning用 |

### VIN_LDC_PICSIZE_S
【结构定义】
```c
typedef struct HB_VIN_LDC_PICSIZE_S {
    uint16_t pic_w;
    uint16_t pic_h;
} VIN_LDC_PICSIZE_S;
```
【功能描述】
> 定义LDC宽高输入信息

【成员说明】

| 成员  | 含义                                                               |
| :---: | :----------------------------------------------------------------- |
| pic_w | 需要设置比接入尺寸  -1 的size, 如果ISP 输出 1920 , 则这里设置 1919 |
| pic_h | 除了size, ldc以及dis 部分其他设置不要更改                          |

### VIN_LDC_ALGOPARAM_S
【结构定义】
```c
typedef struct HB_VIN_LDC_ALGOPARAM_S {
    uint16_t rg_algo_param_b;
    uint16_t rg_algo_param_a;
} VIN_LDC_ALGOPARAM_S;
```
【功能描述】
> 定义LDC属性信息

【成员说明】

|      成员       | 含义           |
| :-------------: | :------------- |
| rg_algo_param_b | 参数需要tuning |
| rg_algo_param_a | 参数需要tuning |

### VIN_LDC_OFF_SHIFT_S
【结构定义】
```c
typedef struct HB_VIN_LDC_OFF_SHIFT_S {
    uint32_t rg_center_xoff:8;
    uint32_t rg_center_yoff:8;
    uint32_t reserved0:16;
} VIN_LDC_OFF_SHIFT_S;
```
【功能描述】
> 定义LDC属性信息

【成员说明】

|      成员      | 含义         |
| :------------: | :----------- |
| rg_center_xoff | 处理区域修正 |
| rg_center_yoff | 处理区域修正 |

### VIN_LDC_WOI_S
【结构定义】
```c
typedef struct HB_VIN_LDC_WOI_S {
    uint32_t rg_start:12;
    uint32_t reserved1:4;
    uint32_t rg_length:12;
    uint32_t reserved0:4;
}VIN_LDC_WOI_S;
```
【功能描述】
> 定义LDC属性信息

【成员说明】

|   成员    | 含义         |
| :-------: | :----------- |
| rg_start  | 处理区域修正 |
| rg_length | 处理区域修正 |

### VIN_LDC_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_LDC_ATTR_S {
    uint32_t         ldcEnable;
    VIN_LDC_PATH_SEL_S  ldcPath;
    uint32_t yStartAddr;
    uint32_t cStartAddr;
    VIN_LDC_PICSIZE_S  picSize;
    uint32_t lineBuf;
    VIN_LDC_ALGOPARAM_S xParam;
    VIN_LDC_ALGOPARAM_S yParam;
    VIN_LDC_OFF_SHIFT_S offShift;
    VIN_LDC_WOI_S   xWoi;
    VIN_LDC_WOI_S   yWoi;
} VIN_LDC_ATTR_S;
```
【功能描述】
> 定义LDC属性信息

【成员说明】

|    成员    | 含义           |
| :--------: | :------------- |
| ldcEnable  | LDC是否使能    |
|  ldcPath   | 输出类型       |
| yStartAddr | Iram使用地址   |
| cStartAddr | Iram使用地址   |
|  picSize   | 接入的尺寸     |
|  lineBuf   | 值设置99       |
|   xParam   | 参数需要tuning |
|   yParam   | 参数需要tuning |
|  offShift  | 处理区域修正   |
|    xWoi    | 处理区域修正   |
|    yWoi    | 处理区域修正   |

### VIN_DIS_PICSIZE_S
【结构定义】
```c
typedef struct HB_VIN_DIS_PICSIZE_S {
    uint16_t pic_w;
    uint16_t pic_h;
} VIN_DIS_PICSIZE_S;
```
【功能描述】
> 定义DIS属性信息

【成员说明】

| 成员  | 含义                                                              |
| :---: | :---------------------------------------------------------------- |
| pic_w | 需要设置比接入尺寸  -1 的size, 如果ISP输出 1920 , 则这里设置 1919 |
| pic_h | 需要设置比接入尺寸  -1 的size                                     |

### VIN_DIS_PATH_SEL_S
【结构定义】
```c
typedef struct HB_VIN_DIS_PATH_SEL_S {
    uint32_t rg_dis_enable:1;
    uint32_t rg_dis_path_sel:1;
    uint32_t reserved0:30;
} VIN_DIS_PATH_SEL_S;
```
【功能描述】
> 定义DIS属性信息

【成员说明】

|      成员       | 含义     |
| :-------------: | :------- |
|  rg_dis_enable  | 输出类型 |
| rg_dis_path_sel | 输出类型 |

### VIN_DIS_CROP_S
【结构定义】
```c
typedef struct HB_VIN_DIS_CROP_S {
    uint16_t rg_dis_start;
    uint16_t rg_dis_end;
} VIN_DIS_CROP_S;
```
【功能描述】
> 定义DIS属性信息

【成员说明】

|     成员     | 含义         |
| :----------: | :----------- |
| rg_dis_start | 处理区域修正 |
|  rg_dis_end  | 处理区域修正 |

### VIN_DIS_CALLBACK_S
【结构定义】
```c
typedef struct HB_VIN_DIS_CALLBACK_S {
    void (*VIN_DIS_DATA_CB) (uint32_t pipeId, uint32_t event,
    VIN_DIS_MV_INFO_S *disData, void *userData);
} VIN_DIS_CALLBACK_S;
```
【功能描述】
> 定义dis回调接口

【成员说明】

|      成员       | 含义                           |
| :-------------: | :----------------------------- |
| VIN_DIS_DATA_CB | 回调函数，收到数据后返回给用户 |

### VIN_DIS_MV_INFO_S
【结构定义】
```c
typedef struct HB_VIN_DIS_MV_INFO_S {
    int  gmvX;
    int  gmvY;
    int  xUpdate;
    int  yUpdate;
} VIN_DIS_MV_INFO_S;
```

【功能描述】
> 定义坐标移动的信息

【成员说明】

|  成员   | 含义                                                                                                                                                         |
| :-----: | :----------------------------------------------------------------------------------------------------------------------------------------------------------- |
|  gmvX   | 绝对坐标,相对于相机中心的x移动量, 如果相机锁好固定住，gmv就是相对于固定锁好位置的移动。                                                                      |
|  gmvY   | 绝对坐标,相对于相机中心的y移动量                                                                                                                             |
| xUpdate | 相对量，相对于前一帧的x移动量, Update则是不管锁在那,只看前一帧相机晃动的位置的移动.(如果前一帧是锁好的位置,则update与gmv相同,但这只会在连续晃动的第一帧发生) |
| yUpdate | 相对量，相对于前一帧的y移动量                                                                                                                                |

### VIN_DIS_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_DIS_ATTR_S {
    VIN_DIS_PICSIZE_S picSize;
    VIN_DIS_PATH_SEL_S disPath;
    uint32_t disHratio;
    uint32_t disVratio;
    VIN_DIS_CROP_S xCrop;
    VIN_DIS_CROP_S yCrop;
} VIN_DIS_ATTR_S;
```
【功能描述】
> 定义DIS属性信息

【成员说明】

|   成员    | 含义         |
| :-------: | :----------- |
|  picSize  | 输入数据宽高 |
|  disPath  | 输出类型     |
| disHratio | 设置为65536  |
| disVrati  | 设置为65536  |
|   xCrop   | 处理区域修正 |
|   yCrop   | 处理区域修正 |

### VIN_LENS_FUNC_TYPE_E
【结构定义】
```c
typedef enum HB_VIN_LENS_FUNC_TYPE_E {
    VIN_LENS_AF_TYPE = 1,
    VIN_LENS_ZOOM_TYPE,
    VIN_LENS_INVALID,
} VIN_LENS_FUNC_TYPE_E;
```
【功能描述】
> 马达功能

【成员说明】
- AF自动对焦，改变像距
- ZOOM变焦，改变焦距

### VIN_LENS_CTRL_ATTR_S
【结构定义】
```c
typedef struct HB_VIN_LENS_CTRL_ATTR_S {
    uint16_t port;
    VIN_LENS_MOTOR_TYPE_E motorType;
    uint32_t maxStep;
    uint32_t initPos;
    uint32_t minPos;
    uint32_t maxPos;
    union {
        struct {
            uint16_t pwmNum;
            uint32_t pwmDuty;
            uint32_t pwmPeriod;
        } pwmParam;
        struct {
            uint16_t pulseForwardNum;
            uint16_t pulseBackNum;
            uint32_t pulseDuty;
            uint32_t pulsePeriod;
        } pulseParam;
        struct {
            uint16_t i2cNum;
            uint32_t i2cAddr;
        } i2cParam;
        struct {
            uint16_t gpioA1;
            uint16_t gpioA2;
            uint16_t gpioB1;
            uint16_t gpioB2;
        } gpioParam;
    };
} VIN_LENS_CTRL_ATTR_S;
```
【功能描述】
> 定义pipe属性信息

【成员说明】

|      成员       | 含义                                    |
| :-------------: | :-------------------------------------- |
|      port       | 每一路输入，和pipeId对应                |
|    motorType    | 电机驱动类型，详见VIN_LENS_MOTOR_TYPE_E |
|     maxStep     | 电机最大步数                            |
|     initPos     | 电机初始位置                            |
|     minPos      | 电机最小位置                            |
|     maxPos      | 电机最大位置                            |
|     pwmNum      | 马达控制pwm  设备号                     |
|     pwmDuty     | 马达控制pwm 占空比                      |
|    pwmPeriod    | 马达控制pwm 频率                        |
| pulseForwardNum | 马达控制 前向控制 pulse 设备号          |
|  pulseBackNum   | 马达控制 后向控制 pulse 设备号          |
|    pulseDuty    | 马达控制 脉冲占空比                     |
|   pulsePeriod   | 马达控制 脉冲 频率                      |
|     i2cNum      | 马达控制I2C 设备号                      |
|     i2cAddr     | 马达控制I2C 地址                        |
|     gpioA1      | 马达控制a+ gpio 号                      |
|     gpioA2      | 马达控制a- gpio 号                      |
|     gpioB1      | 马达控制b+ gpio 号                      |
|     gpioB2      | 马达控制b- gpio 号                      |

### VIN_LENS_MOTOR_TYPE_E
【结构定义】
```c
typedef enum HB_VIN_LENS_MOTOR_TYPE_E {
    VIN_LENS_PWM_TYPE = 0,
    VIN_LENS_PULSE_TYPE,
    VIN_LENS_I2C_TYPE,
    VIN_LENSSPI_TYPE,
    VIN_LENS_GPIO_TYPE
} VIN_LENS_MOTOR_TYPE_E;
```
【功能描述】
> 电机驱动类型，由以上几种。

【成员说明】
- PWM 驱动、脉冲个数驱动、I2C 通信方式控制、spi 通信方式控制、GPIP 引脚时序控制。
由于硬件环境因素，只调试验证过GPIO方式。

### DATA TYPE

| Data | Type Description                               |
| :--: | :--------------------------------------------- |
| 0x28 | RAW6                                           |
| 0x29 | RAW7                                           |
| 0x2A | RAW8                                           |
| 0x2B | RAW10                                          |
| 0x2C | RAW12                                          |
| 0x2D | RAW14                                          |
| 0x2E | Reserved                                       |
| 0x18 | YUV 420 8-bit                                  |
| 0x19 | YUV 420 10-bit                                 |
| 0x1A | Legacy YUV420 8-bit                            |
| 0x1B | Reserved                                       |
| 0x1C | YUV 420 8-bit(Chroma Shifted Pixel Sampling)   |
| 0x1D | YUV 420 10-bit(Chroma Shifted Pixel Sampling)) |
| 0x1E | YUV 422 8-bit                                  |
| 0x1F | YUV 422 10-bit                                 |

### SIF MCLK

| ISP应用场景          | SIF_MCLK(MHz) |
| :------------------- | :-----------: |
| 8M 30fps输入         |     326.4     |
| 2M 30fps 2路分时多工 |    148.36     |
| 2M 30fps 1路输入     |    102.00     |
| 8M DOL2 30fps        |    544.00     |
| 2M 15fps 4路分时多工 |    148.36     |

### VPU CLK

| VPU应用场景 | 编码  | VPU_BCLK/VPU_CCLK(MHz) |
| :---------- | :---: | :--------------------: |
| 8M@30fps    |  AVC  |         326.4          |
|             | HEVC  |          408           |
| 2M*4@30fps  |  AVC  |          544           |
|             | HEVC  |          544           |
| 2M @30fps   |  AVC  |          204           |
|             | HEVC  |          204           |

## 错误码

VIN错误码如下表：

|   错误码   | 宏定义                           | 描述                         |
| :--------: | :------------------------------- | :--------------------------- |
| -268565505 | HB_ERR_VIN_CREATE_PIPE_FAIL      | 创建PIPE失败                 |
| -268565506 | HB_ERR_VIN_SIF_INIT_FAIL         | DEV(Sif)初始化失败           |
| -268565507 | HB_ERR_VIN_DEV_START_FAIL        | DEV(Sif) start失败           |
| -268565508 | HB_ERR_VIN_PIPE_START_FAIL       | ISP start失败                |
| -268565509 | HB_ERR_VIN_CHN_UNEXIST           | Chn不存在                    |
| -268565510 | HB_ERR_VIN_INVALID_PARAM         | 接口参数错误                 |
| -268565511 | HB_ERR_VIN_ISP_INIT_FAIL         | ISP初始化错误                |
| -268565512 | HB_ERR_VIN_ISP_FRAME_CORRUPTED   | ISP破帧，isp驱动应该会有drop |
| -268565513 | HB_ERR_VIN_CHANNEL_INIT_FAIL     | ISP初始化两个chn通道时失败   |
| -268565514 | HB_ERR_VIN_DWE_INIT_FAIL         | DWE初始化失败                |
| -268565515 | HB_ERR_VIN_SET_DEV_ATTREX_FAIL   | SIF扩展属性初始化失败        |
| -268565516 | HB_ERR_VIN_LENS_INIT_FAIL        | 马达初始化失败               |
| -268565517 | HB_ERR_VIN_SEND_PIPERAW_FAIL     | SIF回灌raw失败               |
| -268565518 | HB_ERR_VIN_NULL_POINT            | VIN模块有空指针              |
| -268565519 | HB_ERR_VIN_GET_CHNFRAME_FAIL     | 获取ISP出来的数据失败        |
| -268565520 | HB_ERR_VIN_GET_DEVFRAME_FAIL     | 获取SIF出来的数据失败        |
| -268565521 | HB_ERR_VIN_MD_ENABLE_FAIL        | 使能MotionDetect失败         |
| -268565522 | HB_ERR_VIN_MD_DISABLE_FAIL       | 关闭MotionDetect失败         |
| -268565523 | HB_ERR_VIN_SWITCH_SNS_TABLE_FAIL | ISP模式linear\DOL切换失败    |

## 参考代码
VIN部分示例代码可以参考，[get_sif_data](./multimedia_samples#get_sif_data)和[get_isp_data](./multimedia_samples#get_isp_data)。