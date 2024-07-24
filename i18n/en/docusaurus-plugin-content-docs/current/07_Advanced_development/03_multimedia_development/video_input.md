---
sidebar_position: 4
---

# 7.3.4 Video Input
## Overview
The Video Input Module (VIN) handles video data reception via the MIPI Rx interface and processes this data for further use. It captures raw video images, stores them in designated memory regions, and applies optional preprocessing to the data.

### Terminology

- **Video Input Device**: Primarily refers to SIF (Sensor Image Format), an interface that receives image data from camera modules and sends it directly to the ISP (Image Signal Processor) for processing without or with real-time adjustments.

- **Video Input PIPE (ISP)**: Bound to the device backend, responsible for image processing, core function configuration, and supports multi-context. It can handle up to 8 sensor inputs.

- **Lens Distortion Correction (LDC)**: Corrects image distortion caused by lens curvature, especially in low-end lenses, by applying distortion correction based on the degree of distortion.

- **Digital Image Stabilization (DIS)**: Compares current frames with previous ones to calculate displacement vectors along each axis using different degrees of freedom in anti-shake algorithms. This helps stabilize the image.

- **DWE**: Integrates LDC and DIS functionality, combining both distortion correction and stabilization results.

## Function Description

VIN is divided into four software components as shown below:

![image-20220329195124946](./image/video_input/image-20220329195124946.png)

### Video Input Device

- SIF primarily receives image data from camera modules and sends it to the ISP for processing in RAW8/RAW10/RAW12/RAW14/RAW16 formats, or YUV422 8-bit/10-bit. It supports up to 8 sensor connections.

### Video Input PIPE

- The ISP performs image processing, core function configuration, and supports multi-context with a maximum of 8 inputs. It pipelines image data and outputs in YUV format to channels. It also includes LDC and DIS functionalities.

### Video Physical Channels

- VIN has two physical channels: Channel 0 connects the ISP output to DDR or forwards it to the next module VPS, while Channel 1 routes the ISP output online to VPS. The relationship between VIN and VPS is detailed in the "System Control" section.

### Binding Relationships

- The binding relationship between VIN and VPS is described in the "System Control" chapter: HB_SYS_SetVINVPSMode.

## API Reference

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
int HB_VIN_SetPipeAttr(uint32_t pipeId, VIN_PIPE_ATTR_S *stVinPipeAttr);
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

These functions provide control over the MIPI interface, sensor configuration, video input pipeline management, and various video processing operations.



### HB_MIPI_SetBus
**Function Declaration**
```c
int HB_MIPI_SetBus(MIPI_SENSOR_INFO_S *snsInfo, uint32_t busNum)
```
**Function Description**
> Configures the bus number for the sensor

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| snsInfo        | Sensor configuration information | Input      |
| busNum         | Bus number to be set | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Note**
> None

**Reference Code**
> Refer to examples in HB_MIPI_InitSensor and HB_MIPI_DeinitSensor

### HB_MIPI_SetPort
**Function Declaration**
```c
int HB_MIPI_SetPort(MIPI_SENSOR_INFO_S *snsInfo, uint32_t port)
```
**Function Description**
> Sets the port for the sensor, valid range is 0~7

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| snsInfo        | Sensor configuration information | Input      |
| port           | Current sensor's port number (0~7) | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Note**
> None

**Reference Code**
> Refer to examples in HB_MIPI_InitSensor and HB_MIPI_DeinitSensor

### HB_MIPI_SensorBindSerdes
**Function Declaration**
```c
int HB_MIPI_SensorBindSerdes(MIPI_SENSOR_INFO_S *snsInfo, uint32_t serdesIdx, uint32_t serdesPort)
```
**Function Description**
> Assigns the sensor to a specific Serdes interface

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| snsInfo        | Sensor configuration information | Input      |
| serdesIdx      | Index of the Serdes, 0~1 | Input      |
| serdesPort     | Serdes port number (954:0~1, 960:0~3) | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Note**
> None

**Reference Code**
> Refer to examples in HB_MIPI_InitSensor and HB_MIPI_DeinitSensor

### HB_MIPI_SensorBindMipi
**Function Declaration**
```c
int HB_MIPI_SensorBindMipi(MIPI_SENSOR_INFO_S *snsInfo, uint32_t mipiIdx)
```
**Function Description**
> Binds the sensor to a specific MIPI host

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| snsInfo        | Sensor configuration information | Input      |
| mipiIdx        | Index of the MIPI host, 0~3 | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Note**
> None

**Reference Code**
> Refer to example in HB_MIPI_InitSensor

### HB_MIPI_SetExtraMode
**Function Declaration**
```c
int HB_MIPI_SetExtraMode(MIPI_SENSOR_INFO_S *snsInfo, uint32_t ExtraMode)
```
**Function Description**
> Sets the sensor's working mode in DOL2 or DOL3

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| snsInfo        | Sensor configuration information | Input      |
| ExtraMode      | Mode selection (see notes below) | 1. Single DOL2 mode, value 0<br />2. DOL2 split into two linear lanes, one with value 1, the other with value 2<br />3. Single DOL3 mode, value 0<br />4. One DOL2 lane (value 1) + one linear lane (value 4)<br />5. DOL3 split into three linear lanes, lane 2, 3, and 4<br />      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |



**Reference Code**
> Refer to examples in HB_MIPI_InitSensor and HB_MIPI_DeinitSensor

### HB_MIPI_InitSensor / HB_MIPI_DeinitSensor
**Function Declaration**
```c
int HB_MIPI_InitSensor (uint32_t DevId, MIPI_SENSOR_INFO_S  *snsInfo);
int HB_MIPI_DeinitSensor (uint32_t  DevId);
```
**Function Description**
> Initializes and releases resources for the sensor

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| DevId          | Channel index, 0~7 | Input      |
| snsInfo        | Sensor information | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Note**
> None

**Reference Code**
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
**Function Declaration**
```c
int HB_MIPI_ResetSensor(uint32_t DevId);
int HB_MIPI_UnresetSensor(uint32_t DevId);
```
**Function Description**
> Control the opening and closing of the sensor data stream, equivalent to sensor_start/sensor_stop.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
| devId         | Sensor channel index, range 0~7 | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> None

**Reference Code**
> Refer to examples for HB_MIPI_InitSensor/HB_MIPI_DeinitSensor.

### HB_MIPI_EnableSensorClock/HB_MIPI_DisableSensorClock
**Function Declaration**
```c
int HB_MIPI_EnableSensorClock(uint32_t mipiIdx);
int HB_MIPI_DisableSensorClock(uint32_t mipiIdx);
```
**Function Description**
> Enable or disable the sensor_clk.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
| mipiIdx       | Mipi host index, range 0~3 | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> Use this interface with the subboard crystal removed.

**Reference Code**
> No example provided.

### HB_MIPI_SetSensorClock
**Function Declaration**
```c
int HB_MIPI_SetSensorClock(uint32_t mipiIdx, uint32_t snsMclk)
```
**Function Description**
> Set the sensor_mclk. There are four sensor_mclk available, currently used for sensor0_mclk and sensor1_mclk. Mipi0 connects to sensor_mclk1, and mipi1 connects to sensor_mclk0; the hardware connection details are defined in the DTS.

**Parameter Descriptions**

| Parameter Name | Description                  | Input/Output |
| :------------: | :---------------------------: | :---------: |
| mipiIdx       | Mipi host index, range 0~3   | Input      |
| snsMclk       | Frequency in Hz, e.g., 24MHz | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> Use this interface with the subboard crystal removed.

**Reference Code**
- Initialization:
  > Set sensor_mclk first, then enable.
  > HB_MIPI_SetSensorClock(mipiIdx, 24000000);
  > HB_MIPI_EnableSensorClock(mipiIdx);

- Shutdown:
  > HB_MIPI_Clear(mipiIdx);
  > HB_MIPI_DeinitSensor(devId);
  > HB_MIPI_DisableSensorClock(mipiIdx);

### HB_MIPI_ResetMipi/HB_MIPI_UnresetMipi
**Function Declaration**
```c
int HB_MIPI_ResetMipi(uint32_t  mipiIdx);
int HB_MIPI_UnresetMipi(uint32_t  mipiIdx)
```
**Function Description**
> Control the start and stop of the Mipi functionality.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
| mipiIdx       | Mipi host index, range 0~3 | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> None

**Reference Code**
> Refer to examples for HB_MIPI_InitSensor/HB_MIPI_DeinitSensor.

### HB_MIPI_SetMipiAttr
**Function Declaration**
```c
int HB_MIPI_SetMipiAttr(uint32_t  mipiIdx, MIPI_ATTR_S  mipiAttr)
```
**Function Description**
> Set the Mipi attributes, including host and device initialization.

**Parameter Descriptions**

| Parameter Name | Description       | Input/Output |
| :------------: | :----------------: | :---------: |
| mipiIdx       | Mipi host index   | Input      |
| mipiAttr      | Mipi bus attribute information | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> None

**Reference Code**
> Refer to examples for HB_MIPI_InitSensor/HB_MIPI_DeinitSensor.

### HB_MIPI_Clear
**Function Declaration**
```c
int HB_MIPI_Clear(uint32_t  mipiIdx);
```
**Function Description**
> Clear device-related configurations, equivalent to deinitializing the Mipi host/device and corresponds to the interface HB_MIPI_SetMipiAttr.

**Parameter Descriptions**

| Parameter Name | Description            | Input/Output |
| :------------: | :---------------------: | :---------: |
| mipiIdx       | Mipi host index, range 0~3 | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|    0         | Success |
| Non-zero     | Failure |

**Notes**
> None

**Reference Code**
> Refer to examples for HB_MIPI_InitSensor/HB_MIPI_DeinitSensor.



### HB_MIPI_ReadSensor
**Function Declaration**
```c
int HB_MIPI_ReadSensor(uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size)
```
**Function Description**
> Reads data from a sensor via I2C.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| devId          | Sensor channel index, range 0-7 | Input      |
| regAddr        | Register address to read | Input      |
| buffer         | Address to store the read data | Output     |
| size            | Length of data to read | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
| 0            | Success |
| Non-zero     | Failure |

**Note**
> Must call HB_MIPI_InitSensor before using this function.

**Example Code**
> (Example with imx327)
```c
    int i;
    char buffer[] = {0x34, 0x56};
    char rev_buffer[30] = {0};
    printf("HB_MIPI_InitSensor end\n");
    ret = HB_MIPI_ReadSensor(devId, 0x3018, rev_buffer, 2);
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
**Function Declaration**
```c
int HB_MIPI_WriteSensor (uint32_t devId, uint32_t regAddr, char *buffer, uint32_t size)
```
**Function Description**
> Writes data to a sensor's register via I2C.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| devId          | Sensor channel index, range 0-7 | Input      |
| regAddr        | Register address to write | Input      |
| buffer         | Address containing data to write | Input      |
| size            | Length of data to write | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
| 0            | Success |
| Non-zero     | Failure |

**Note**
> Must call HB_MIPI_InitSensor before using this function.

**Example Code**
> Refer to the HB_MIPI_ReadSensor example.

### HB_MIPI_GetSensorInfo
**Function Declaration**
```c
int HB_MIPI_GetSensorInfo(uint32_t devId, MIPI_SENSOR_INFO_S *snsInfo)
```
**Function Description**
> Retrieves sensor configuration information.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| devId          | Sensor channel index, range 0-7 | Input      |
| snsInfo        | Structure containing sensor info | Output     |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
| 0            | Success |
| Non-zero     | Failure |

**Note**
> Must call HB_MIPI_InitSensor before using this function.

**Example Code**
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
**Function Declaration**
```c
int HB_MIPI_SwSensorFps(uint32_t devId, uint32_t fps)
```
**Function Description**
> Switches the sensor's frame rate.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| devId          | Sensor channel index, range 0-7 | Input      |
| fps             | Desired sensor frame rate | Input      |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
| 0            | Success |
| Non-zero     | Failure |

**Note**
> Must call HB_MIPI_InitSensor before using this function.

**Example Code**
> No code provided as there is no specific example given.



### HB_VIN_SetMipiBindDev/HB_VIN_GetMipiBindDev
**Function Declaration:**
```c
int HB_VIN_SetMipiBindDev(uint32_t devId, uint32_t mipiIdx)
int HB_VIN_GetMipiBindDev(uint32_t devId, uint32_t *mipiIdx)
```
**Function Description:**
> Sets the binding between a device and a Mipi_host, specifying which Mipi_host the device uses.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|   devId        | Device index, range 0~7 | Input      |
|  mipiIdx      | Index of the Mipi_host | Input      |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Note:**
> No additional notes.

**Reference Code:**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetDevAttr/HB_VIN_GetDevAttr
**Function Declaration:**
```c
int HB_VIN_SetDevAttr(uint32_t devId, const VIN_DEV_ATTR_S *stVinDevAttr)
int HB_VIN_GetDevAttr(uint32_t devId, VIN_DEV_ATTR_S *stVinDevAttr)
```
**Function Description:**
> Sets or retrieves the attributes of a device.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    devId       | Device index, range 0~7 | Input      |
| stVinDevAttr  | Device attribute structure | Input/Output |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Notes:**
> For multi-process scenarios with DOL3 split, the first process should run one second before the second. Also, setting attributes after destroying a device is currently not supported.
> If encountering an error like SIF_IOC_BIND_GROUT ioctl failed, it's usually due to a previous pipeid call that didn't exit before being called again.

**Reference Code:**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetDevAttrEx/HB_VIN_GetDevAttrEx
**Function Declaration:**
```c
int HB_VIN_SetDevAttrEx(uint32_t devId, const VIN_DEV_ATTR_EX_S *stVinDevAttrEx)
int HB_VIN_GetDevAttrEx(uint32_t devId, VIN_DEV_ATTR_EX_S *stVinDevAttrEx)
```
**Function Description:**
> Sets or retrieves the extended attributes of a device.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    devId       | Device index, range 0~7 | Input      |
| stVinDevAttrEx | Extended device attribute structure | Input/Output |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Notes:**
> This interface is not yet supported.

**Reference Code:**
> No code example provided.

### HB_VIN_EnableDev/HB_VIN_DisableDev
**Function Declaration:**
```c
int HB_VIN_EnableDev(uint32_t devId);
int HB_VIN_DisableDev(uint32_t devId);
```
**Function Description:**
> Enables or disables a device module.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    devId       | Device index, range 0~7 | Input      |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Notes:**
> No additional notes.

**Reference Code:**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_DestroyDev
**Function Declaration:**
```c
int HB_VIN_DestroyDev(uint32_t devId)
```
**Function Description:**
> Destroys a device module, releasing its resources.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    devId       | Device index, range 0~7 | Input      |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Notes:**
> No additional notes.

**Reference Code:**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetDevBindPipe/HB_VIN_GetDevBindPipe
**Function Declaration:**
```c
int HB_VIN_SetDevBindPipe(uint32_t devId, uint32_t pipeId)
int HB_VIN_GetDevBindPipe(uint32_t devId, uint32_t *pipeId)
```
**Function Description:**
> Sets the binding between a device's channel output and a pipe's channel input, as well as between a pipe's channel input and output.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :---------: |
|    devId       | Device index, range 0~7 | Input      |
|    pipeId      | Pipe index, same range | Input      |

**Return Values:**

| Return Value | Description |
|:------------:|:-----------:|
|    0         | Success    |
| Non-zero     | Failure    |

**Note:**
> The HB_VIN_GetDevBindPipe interface is not yet implemented.

**Reference Code:**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.



### HB_VIN_CreatePipe/HB_VIN_DestroyPipe
**Function Declaration**
```c
int HB_VIN_CreatePipe(uint32_t pipeId, const VIN_PIPE_ATTR_S *stVinPipeAttr);
int HB_VIN_DestroyPipe(uint32_t pipeId);
```
**Function Description**
> Create a pipe and destroy a pipe

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|  pipeId       | Unique identifier for each input channel, range 0~7 | Input |
| stVinPipeAttr | Pointer to the structure describing pipe attributes | Input |

**Return Values**

| Return Value | Meaning |
|:------------:|:-------:|
|     0       | Success |
| Non-zero    | Failure |

**Note**
> None

**Reference Code**
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
**Function Declaration**
```c
int HB_VIN_StartPipe(uint32_t pipeId);
int HB_VIN_StopPipe(uint32_t pipeId);
```
**Function Description**
> Start and stop the pipe.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> None

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_EnableChn/HB_VIN_DisableChn
**Function Declaration**
```c
int HB_VIN_EnableChn(uint32_t pipeId, uint32_t chnId);
int HB_VIN_DisableChn(uint32_t pipeId, uint32_t chnId);
```
**Function Description**
> Enable or disable a channel on the pipe.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input |
| chnId | Input 1 is sufficient | Input |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> None

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetChnLDCAttr/HB_VIN_GetChnLDCAttr
**Function Declaration**
```c
int HB_VIN_SetChnLDCAttr(uint32_t pipeId, uint32_t chnId, const VIN_LDC_ATTR_S *stVinLdcAttr);
int HB_VIN_GetChnLDCAttr(uint32_t pipeId, uint32_t chnId, VIN_LDC_ATTR_S*stVinLdcAttr);
```
**Function Description**
> Set or get the LDC attribute for a channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input (Output for getting attribute) |
| chnId | Input 1 is sufficient | Input (Output for getting attribute) |
| stVinLdcAttr | LDC attribute information | Input (Output for getting attribute) |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> When VIN_ISP and VPS are in online mode, LDC parameters must be configured through this interface. Otherwise, VPS may encounter an exception. LDC parameter configuration is not required when VIN_ISP and VPS are in offline mode.

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetChnDISAttr/HB_VIN_GetChnDISAttr
**Function Declaration**
```c
int HB_VIN_SetChnDISAttr(uint32_t pipeId, uint32_t chnId, const VIN_DIS_ATTR_S *stVinDisAttr);
int HB_VIN_GetChnDISAttr(uint32_t pipeId, uint32_t chnId, VIN_DIS_ATTR_S *stVinDisAttr);
```
**Function Description**
> Set or get the DIS attribute for a channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input (Output for getting attribute) |
| chnId | Input 1 is sufficient | Input (Output for getting attribute) |
| stVinDisAttr | DIS attribute information | Input (Output for getting attribute) |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> None

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_SetChnAttr
**Function Declaration**
```c
int HB_VIN_SetChnAttr(uint32_t pipeId, uint32_t chnId);
```
**Function Description**
> Set the attributes for a channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input |
| chnId | Input 1 is sufficient | Input |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> The LDC and DIS attributes are actually set in this interface. HB_VIN_SetChnLDCAttr and HB_VIN_SetChnDISAttr only assign values to the attributes. This 'chn' refers to one of ISP's output channels, with a fixed value of 1.

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_DestroyChn
**Function Declaration**
```c
int HB_VIN_DestroyChn(uint32_t pipeId, uint32_t chnId)
```
**Function Description**
> Destroy a channel.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :-------------: | :----------: | :---------: |
| pipeId | Corresponding input channel ID, range 0~7 | Input |
| chnId | Input 1 is sufficient | Input |

**Return Values**

| Return Value | Description |
|:------------:|:-----------:|
| 0            | Success    |
| Non-zero     | Failure    |

**Note**
> Currently, it is not supported to re-configure HB_VIN_SetChnAttr after HB_VIN_DestroyChn.

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.



### HB_VIN_GetChnFrame/HB_VIN_ReleaseChnFrame
**Function Declaration**
```c
int HB_VIN_GetChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame, int32_t millSec);
int HB_VIN_ReleaseChnFrame(uint32_t pipeId, uint32_t chnId, void *pstVideoFrame);
```
**Function Description**
> Retrieve data after ISP processing on the pipe channel

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :---------- | :---------- |
|    pipeId      | Corresponding input pipe, range 0~7 | Input       |
|     chnId      | Can be set to 0 | Input       |
| pstVideoFrame | Data information | Output      |
|    millSec     | Timeout parameter<br/>-1 for blocking interface;<br/>0 for non-blocking interface;<br/>positive value for timeout in milliseconds (ms) | Input       |

**Return Values**

| Return Value | Description |
|:------------:|:------------|
|     0        | Success     |
| Non-zero     | Failure     |

**Notes**
> This interface retrieves the image processed by the ISP

**Reference Code**
> Refer to HB_VIN_CreatePipe/HB_VIN_DestroyPipe examples

### HB_VIN_GetDevFrame/HB_VIN_ReleaseDevFrame
**Function Declaration**
```c
int HB_VIN_GetDevFrame(uint32_t devId, uint32_t chnId, void *videoFrame, int32_t millSec);
int HB_VIN_ReleaseDevFrame(uint32_t devId, uint32_t chnId, void *buf);
```
**Function Description**
> Retrieve data after SIF processing, where chnId is 0

**Parameter Descriptions**

| Parameter Name  | Description | Input/Output |
| :-------------: | :---------- | :---------- |
|    devId        | Corresponding input pipe, range 0~7 | Input       |
|     chnId       | Can be set to 0 | Input       |
| videoFrame     | Data information | Output      |
|    millSec      | Timeout parameter<br/>-1 for blocking interface;<br/>0 for non-blocking interface;<br/>positive value for timeout in milliseconds (ms) | Input       |

**Return Values**

| Return Value | Description |
|:------------:|:------------|
|     0        | Success     |
| Non-zero     | Failure     |

**Notes**
> This interface retrieves the image processed by SIF, useful when SIF is offline-ISP and can dump raw images.
> Scenarios:
>> VIN_OFFLINE_VPS_ONLINE
>> VIN_OFFLINE_VPS_OFFINE
>> VIN_SIF_OFFLINE_ISP_OFFLINE_VPS_ONLINE

Additionally, when SIF is online-ISP with DDR, it can also dump raw images. Scenarios:
>> VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE
>> VIN_SIF_ONLINE_DDR_ISP_ONLINE_VPS_ONLINE

**Example Code**
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
**Function Declaration**
```c
int HB_VIN_SendPipeRaw(uint32_t pipeId, void *pstVideoFrame, int32_t millSec)
```
**Function Description**
> Sends raw data to the ISP for processing.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|    pipeId      | ID of each input, range 0~7 | Input        |
| pstVideoFrame | Pointer to the raw data information | Input        |
|    millSec    | Timeout parameter<br/>-1 for blocking interface<br/>0 for non-blocking interface<br/>Positive value for timeout in milliseconds (ms) | Input        |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero    | Failure     |

**Note**
> None

**Reference Code**
```c
int pipeId = 0;
hb_vio_buffer_t *feedback_buf;
hb_vio_buffer_t *isp_yuv = NULL;
isp_yuv = (hb_vio_buffer_t *) malloc(sizeof(hb_vio_buffer_t));
memset(isp_yuv, 0, sizeof(hb_vio_buffer_t));
ret = HB_VIN_SendPipeRaw(pipeId, feedback_buf, 1000);
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
**Function Declaration**
```c
int HB_VIN_SetPipeAttr(uint32_t pipeId, VIN_PIPE_ATTR_S *stVinPipeAttr);
int HB_VIN_GetPipeAttr(uint32_t pipeId, VIN_PIPE_ATTR_S *stVinPipeAttr);
```
**Function Description**
> Sets or retrieves pipe (ISP) attributes.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|    pipeId      | ID of each input, range 0~7 | Input (set), Output (get) |
| stVinPipeAttr | Pointer to the structure describing pipe attributes | Input (set), Output (get) |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero    | Failure     |

**Note**
> None

**Reference Code**
> Refer to examples for HB_VIN_CreatePipe/HB_VIN_DestroyPipe.

### HB_VIN_CtrlPipeMirror
**Function Declaration**
```c
int HB_VIN_CtrlPipeMirror(uint32_t pipeId, uint8_t on);
```
**Function Description**
> Controls mirroring of the pipe.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|  pipeId       | ID of each input, range 0~7 | Input        |
|     on       | 0 to disable, non-zero to enable mirror | Input        |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Success     |
| Non-zero    | Failure     |

**Note**
> Flip functionality requires GDC support; first enable mirroring and then rotate by 180 degrees if needed.

### HB_VIN_MotionDetect
**Function Declaration**
```c
int HB_VIN_MotionDetect(uint32_t pipeId)
```
**Function Description**
> Detects motion in MD (Motion Detection) and returns if an interrupt is detected.

**Parameter Descriptions**

| Parameter Name | Description | Input/Output |
| :------------: | :----------: | :----------: |
|  pipeId       | ID of each input, range 0~7 | Input        |

**Return Values**

| Return Value | Description |
|:------------:|:------------:|
|     0       | Motion detected, blocking call until motion detected. |
|   Non-zero  | No motion detected. |

**Note**
> None

**Reference Code**
> Refer to examples for HB_VIN_EnableDevMd.



### HB_VIN_InitLens
**Function Declaration:**
```c
int HB_VIN_InitLens(uint32_t pipeId, VIN_LENS_FUNC_TYPE_E lensType, const VIN_LENS_CTRL_ATTR_S *lenCtlAttr)
```
**Function Description:**
> Motor driver initialization.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :-------------: | :---------: | :----------: |
|   pipeId       | Corresponding input for each path, range 0-7 | Input       |
|  lensType      | Lens function type (AF, Zoom) | Input       |
| lenCtlAttr     | Control attributes | Input       |

**Return Values:**

| Return Value | Description |
|:-----------:|:-----------:|
|     0       | Success    |
| Non-zero    | Failure    |

**Note:**
> If using AF, call this interface once. For both AF and Zoom functions, call initialization twice. Call when needed; not recommended if not used.

**Reference Code:**
> None provided

### HB_VIN_DeinitLens
**Function Declaration:**
```c
int HB_VIN_DeinitLens(uint32_t pipeId)
```
**Function Description:**
> Motor deinitialization.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :-------------: | :---------: | :----------: |
|  pipeId        | Corresponding input for each path, range 0-7 | Input       |

**Return Values:**

| Return Value | Description |
|:-----------:|:-----------:|
|     0       | Success    |
| Non-zero    | Failure    |

**Note:**
> No additional information.

**Reference Code:**
> None provided

### HB_VIN_RegisterDisCallback
**Function Declaration:**
```c
int HB_VIN_RegisterDisCallback(uint32_t pipeId, VIN_DIS_CALLBACK_S *pstDISCallback)
```
**Function Description:**
> Register dis callback function.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :-------------: | :---------: | :----------: |
|     pipeId     | Corresponding input for each path, range 0-7 | Input       |
| pstDISCallback | Callback interface pointer | Input       |

**Return Values:**

| Return Value | Description |
|:-----------:|:-----------:|
|     0       | Success    |
| Non-zero    | Failure    |

**Note:**
> No additional information.

**Reference Code:**
> None provided

### HB_VIN_SetDevVCNumber / HB_VIN_GetDevVCNumber
**Function Declarations:**
```c
int HB_VIN_SetDevVCNumber(uint32_t devId, uint32_t vcNumber);
int HB_VIN_GetDevVCNumber(uint32_t devId, uint32_t *vcNumber);
```
**Function Description:**
> Set and get the device's vc_index, which MIPI VC to use.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :-------------: | :---------: | :----------: |
|   devId        | Corresponding input for each path, range 0-7 | Input/Output (Output for Get) |
|  vcNumber      | Corresponding MIPI VC, range 0-3 | Input (Input for Set, Output for Get) |

**Return Values:**

| Return Value | Description |
|:-----------:|:-----------:|
|     0       | Success    |
| Non-zero    | Failure    |

**Note:**
> No additional information.

**Reference Code:**
> None provided

### HB_VIN_AddDevVCNumber
**Function Declaration:**
```c
int HB_VIN_AddDevVCNumber(uint32_t devId, uint32_t vcNumber)
```
**Function Description:**
> Set the device's vc_index, which MIPI VC to use.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :-------------: | :---------: | :----------: |
|  devId         | Corresponding input for each path, range 0-7 | Input       |
|  vcNumber      | Corresponding MIPI VC, range 0-3 | Input       |

**Return Values:**

| Return Value | Description |
|:-----------:|:-----------:|
|     0       | Success    |
| Non-zero    | Failure    |

**Note:**
> In linear mode, this interface is not used. In DOL2 mode, call with `vcNumber` set to 1. In DOL3 mode, call twice HB_VIN_AddDevVCNumber with `vcNumber` set to 0 and 1, respectively.

**Reference Code Snippet (for DOL2 mode):**

1) Bind dev0 to mipi0
HB_VIN_SetMipiBindDev(0, 0)

2) Bind mipi0's virtual channel 0 to dev0
HB_VIN_SetDevVCNumber(0, 0)

3) Bind mipi0's virtual channel 1 to dev0
HB_VIN_AddDevVCNumber(0, 1);

4) Bind dev0 to ISP pipe0
HB_VIN_SetDevBindPipe(0, 0)
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
**Function Declaration:**
```c
int HB_VIN_SetDevMclk(uint32_t devId, uint32_t devMclk, uint32_t vpuMclk);
```
**Function Description:**
> Sets the SIF mclk and VPU clk for a given input channel.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|  devId         | Input ID (0-7) for each channel | Input        |
| devMclk       | SIF mclk value to set, refer to SIF MCLK | Input, kHz  |
| vpuMclk       | VPU clk value to set, refer to VPU CLK | Input, kHz  |

**Return Values:**

| Return Value | Meaning |
|:------------:| :------:|
|    0         | Success |
| Non-zero     | Failure |

**Note:**
> None

**Reference Code:**
> Not provided

### HB_VIN_GetChnFd
**Function Declaration:**
```c
int HB_VIN_GetChnFd(uint32_t pipeId, uint32_t chnId)
```
**Function Description:**
> Retrieves the file descriptor (FD) for a specified channel.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|  pipeId        | Input ID (0-7) for each channel | Input        |
|  chnId         | Channel number, set to 0 | Input        |

**Return Values:**

| Return Value | Meaning  |
|:------------:| :-------:|
| Positive    | Success  |
| Negative    | Failure  |

**Note:**
> None

**Reference Code:**
> Not provided

### HB_VIN_CloseFd
**Function Declaration:**
```c
int HB_VIN_CloseFd(void)
```
**Function Description:**
> Closes the file descriptor associated with a channel.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|   void        | No input   | No output   |

**Return Values:**

| Return Value | Meaning |
|:------------:| :------:|
|    0         | Success |
| Non-zero     | Failure |

**Note:**
> None

**Reference Code:**
> Not provided

### HB_VIN_EnableDevMd
**Function Declaration:**
```c
int HB_VIN_EnableDevMd(uint32_t devId)
```
**Function Description:**
> Enables motion detect functionality for a given input channel.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|  devId         | Input ID (0-7) for each channel | Input        |

**Return Values:**

| Return Value | Meaning |
|:------------:| :------:|
|    0         | Success |
| Non-zero     | Failure |

**Note:**
> This function should be called after HB_VIN_SetDevAttrEx, as HB_VIN_SetDevAttrEx sets MD-related attributes. An example of using it is shown below:

```c
VIN_DEV_ATTR_EX_S devAttr;
devAttr.path_sel = 0;
devAttr.roi_top = 0;
devAttr.roi_left = 0;
devAttr.roi_width = 1280;
devAttr.roi_height = 640;
devAttr.grid_step = 128;
devAttr.grid_tolerance =10;
devAttr.threshold = 10;
devAttr.weight_decay = 128;
devAttr.precision = 0;
ret = HB_VIN_SetDevAttrEx(pipeId, &devAttr);
if (ret < 0) {
    printf("HB_VIN_SetDevAttrEx error!\n");
    return ret;
}
ret = HB_VIN_EnableDevMd(pipeId);
if (ret < 0) {
    printf("HB_VIN_EnableDevMd error!\n");
    return ret;
}

```
A separate thread can be created to call HB_VIN_MotionDetect upon receiving an MD interrupt, followed by disabling MD with HB_VIN_DisableDevMd.

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
**Function Declaration:**
```c
int HB_VIN_DisableDevMd(uint32_t devId)
```
**Function Description:**
> Disables motion detect functionality for a given input channel.

**Parameter Descriptions:**

| Parameter Name | Description | Input/Output |
| :------------: | :---------: | :----------: |
|  devId         | Input ID (0-7) for each channel | Input        |

**Return Values:**

| Return Value | Meaning |
|:------------:| :------:|
|    0         | Success |
| Non-zero     | Failure |

**Note:**
> This function is used by the user after receiving an MD interrupt to disable the MD feature.

**Reference Code:**
> Please refer to the example provided for HB_VIN_EnableDevMd.

## Data Structures

### MIPI_INPUT_MODE_E
```c
typedef enum HB_MIPI_INPUT_MODE_E
{
    INPUT_MODE_MIPI         = 0x0,              /* mipi */
    INPUT_MODE_DVP         = 0x1,              /* Digital Video Port (DVP) */
    INPUT_MODE_BUTT
} MIPI_INPUT_MODE_E;
```
**Function Description:**
> Sensor connection method

**Member Descriptions:**
- MII (M-PHY Interface) connection
- DVP (Digital Video Port) connection


### MIPI_SENSOR_MODE_E
```c
typedef enum HB_MIPI_SENSOR_MODE_E
{
    NORMAL_M             = 0x0,
    DOL2_M               = 0x1,
    DOL3_M               = 0x2,
    PWL_M                = 0x3,
} MIPI_SENSOR_MODE_E;
```
**Function Description:** Sensor working mode

**Member Descriptions:**
- linear mode
- DOL2 mode
- DOL3 mode
- PWM mode

### MIPI_DESERIAL_INFO_T
```c
typedef struct HB_MIPI_DESERIAL_INFO_T {
    int bus_type;
    int bus_num;
    int deserial_addr;
    int physical_entry;
    char *deserial_name;
} MIPI_DESERIAL_INFO_T;
```
**Function Description:** Defines the attributes for SerDes initialization

**Member Descriptions:**
- bus_type: Bus type, 0 for I2C, 1 for SPI
- bus_num: Bus number, determined by the hardware schematic, currently set to 5
- deserial_addr: SerDes address
- physical_entry: Reserved
- deserial_name: SerDes name

### MIPI_SNS_INFO_S
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
**Function Description:** Defines the attributes for initializing a sensor

**Member Descriptions:**
- port: Logical sensor identifier, must start from 0
- dev_port: Driver node for each sensor path, one driver supports multiple nodes
- bus_type: Bus type, 0 for I2C, 1 for SPI
- bus_num: Bus number (default I2C5 based on hardware schematic)
- fps: Frame rate
- resolution: Sensor resolution
- sensor_addr: Sensor address
- serial_addr: Internal SerDes address of the sensor
- entry_index: MIPI index used by the sensor
- sensor_mode: Sensor working mode, 1 is normal, 2 is DOL2, 3 is DOL3
- reg_width: Register address width
- sensor_name: Sensor name
- extra_mode: To distinguish sensor characteristics, specific to the sensor driver implementation
- deserial_index: Index of the current SerDes
- deserial_port: Port within the SerDes the sensor belongs to
- gpio_num: Number of GPIO pins required for the sensor
- gpio_pin: Array of GPIO pins used
- gpio_level: Initial values for GPIO pins (0 for low-high, 1 for high-low)
- spi_info: Sensor SPI information, some sensors access via SPI bus



### MIPI_SENSOR_INFO_S
**Structure Definition**
```c
typedef struct HB_MIPI_SENSOR_INFO_S {
    int    deseEnable;
    MIPI_INPUT_MODE_E  inputMode;
    MIPI_DESERIAL_INFO_T deserialInfo;
    MIPI_SNS_INFO_S  sensorInfo;
} MIPI_SENSOR_INFO_S;
```
**Function Description**
> Defines the property information for device initialization

**Member Descriptions**

|   Member   | Meaning                         |
| :--------: | :------------------------------ |
| deseEnable | Whether the sensor has serdes |
| inputMode  | Sensor connection mode         |
| deserialInfo | Serdes information             |
| sensorInfo  | Sensor information              |

### MIPI_HOST_CFG_S
**Structure Definition**
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
**Function Description**
> Defines the initialization parameters for the MIPI host

**Member Descriptions**

|   Member   | Meaning                                             |
| :--------: | :-------------------------------------------------- |
|   lane     | Number of lanes, 0~4                                |
| datatype   | Data format, see DATA TYPE                            |
|   mclk     | Main clock of the MIPI module, currently fixed at 24MHz |
|  mipiclk   | Sensor's total MIPI bitrate, units Mbits per second    |
|    fps     | Actual frame rate of the sensor output                |
|   width    | Actual width of the sensor output                    |
|   height   | Actual height of the sensor output                   |
| linelenth  | Total line length with blanking                     |
| framelenth | Total frame length with blanking                    |
|   settle   | Actual Ttx-zero + Ttx-prepare time (in clock units)   |
| channel_num | Number of virtual channels used                      |
| channel_sel[4] | Array storing values for each virtual channel        |

### MIPI_ATTR_S
**Structure Definition**
```c
typedef struct HB_MIPI_ATTR_S {
    MIPI_HOST_CFG_S mipi_host_cfg;
    uint32_t  dev_enable;
} MIPI_ATTR_S;
```
**Function Description**
> Defines the initialization parameters for the MIPI host and device

**Member Descriptions**

|   Member   | Meaning                             |
| :--------: | :---------------------------------- |
| mipi_host_cfg | MIPI host configuration structure |
| dev_enable   | MIPI device enable, 1 for enabled, 0 for disabled |

### MIPI_SPI_DATA_S
**Structure Definition**
```c
typedef struct HB_MIPI_SPI_DATA_S {
    int spi_mode;
    int spi_cs;
    uint32_t spi_speed;
} MIPI_SPI_DATA_S;
```
**Function Description**
> Defines sensor-related SPI information

**Member Descriptions**

|   Member    | Meaning          |
| :-------: | :------------ |
| spi_mode  | SPI operating mode |
|  spi_cs   | SPI Chip Select   |
| spi_speed | SPI data transfer rate |

### VIN_DEV_SIZE_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_SIZE_S {
    uint32_t  format;
    uint32_t  width;
    uint32_t  height;
    uint32_t  pix_length;
} VIN_DEV_SIZE_S;
```
**Function Description**
> Defines the attribute information for device initialization

**Member Descriptions**

|    Member    | Meaning                                                                                   |
| :--------: | :----------------------------------------------------------------------------------------- |
|   format   | Pixel format, format 0 represents raw8~raw16, indicating whether it's raw8 or raw16 based on pix_length. |
|   width    | Data width                                                                                |
|   height   | Data height                                                                               |
| pix_length | Length of each pixel point                                                               |



### VIN_MIPI_ATTR_S
**Structure Definition**
```c
typedef struct HB_VIN_MIPI_ATTR_S {
    uint32_t  enable;                /* Mipi enable, 0 is off, 1 is on */
    uint32_t  ipi_channels;          /* Ipi channels used, starts from 0, e.g., 2 means using channels 0 and 1 */
    uint32_t  ipi_mode;              /* 2 for DOL2 split into two linear, 3 for DOL3 split into DOL2 and one linear, or three linear */
    uint32_t  enable_mux_out;        /* Enable Mux output selection */
    uint32_t  enable_frame_id;       /* Enable Frame ID */
    uint32_t  enable_bypass;         /* Enable bypass functionality */
    uint32_t  enable_line_shift;     /* Unused */
    uint32_t  enable_id_decoder;     /* Unused */
    uint32_t  set_init_frame_id;     /* Initial Frame ID value, usually 1 */
    uint32_t  set_line_shift_count;  /* Unused */
    uint32_t  set_bypass_channels;   /* Unused */
    uint32_t  enable_pattern;        /* Enable test pattern */
} VIN_MIPI_ATTR_S;
```
**Function Description**
> Defines the initialization information for dev Mipi

**Member Descriptions**

| Member         | Meaning                                                                                   |
| :-------------: | :----------------------------------------------------------------------------------------- |
| enable         | Mipi enable, 0 for disabled, 1 for enabled                                                      |
| ipi_channels   | Number of IPI channels used, starting from 0, e.g., 2 indicates channels 0 and 1 are in use       |
| ipi_mode       | Value for DOL2 split into two linear (2), DOL3 split into DOL2 and one linear (3), or three linear |
| enable_mux_out | Enables Mux output selection                                                                       |
| enable_frame_id | Enables Frame ID                                                                               |
| enable_bypass  | Enables bypass functionality                                                                      |
| enable_line_shift | Unused                                                                                      |
| enable_id_decoder | Unused                                                                                      |
| set_init_frame_id | Initial Frame ID value, commonly set to 1                                                            |
| set_line_shift_count | Unused                                                                                     |
| set_bypass_channels | Unused                                                                                      |
| enable_pattern | Enables test pattern                                                                          |

### VIN_DEV_INPUT_DDR_ATTR_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_INPUT_DDR_ATTR_S {
    uint32_t stride;                  /* Hardware stride matches format, e.g., 12-bit: stride = width * 1.5, 10-bit: stride = width * 1.25 */
    uint32_t buf_num;                 /* Number of buffers for offline or feedback */
    uint32_t raw_feedback_en;         /* Enables feedback mode, exclusive with offline mode, can be used independently */
    VIN_DEV_SIZE_S data;             /* Data format, see VIN_DEV_SIZE_S */
} VIN_DEV_INPUT_DDR_ATTR_S;
```
**Function Description**
> Defines the input information for dev, applicable for offline and feedback scenarios

**Member Descriptions**

| Member     | Meaning                                                                                   |
| :---------: | :----------------------------------------------------------------------------------------- |
| stride     | Hardware stride matching the format, e.g., for 12-bit: 1952x1.5                                   |
| buf_num    | Number of buffers for offline or feedback operations                                          |
| raw_feedback_en | Enables raw feedback mode, mutually exclusive with offline mode, can be used separately      |
| data       | Data format details, as described in VIN_DEV_SIZE_S                                             |

### VIN_DEV_OUTPUT_DDR_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_OUTPUT_DDR_S {
    uint32_t stride;                    /* Hardware stride matches format, currently set for 12-bit at 1952x1.5 */
    uint32_t buffer_num;               /* Number of buffers for DDR output */
    uint32_t frameDepth;                /* Maximum frames to get, recommended max is ddrOutBufNum - 4 */
} VIN_DEV_OUTPUT_DDR_S;
```
**Function Description**
> Defines the DDR output initialization information for dev

**Member Descriptions**

| Member     | Meaning                                                                                     |
| :---------: | :------------------------------------------------------------------------------------------- |
| stride     | Hardware stride matching the format, currently for 12-bit at 1952x1.5                                |
| buffer_num | Number of buffers for DDR output to the device                                                 |
| frameDepth | The maximum number of frames that can be fetched, typically set to ddrOutBufNum minus 4 for optimal use |



### VIN_DEV_OUTPUT_ISP_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_OUTPUT_ISP_S {
    uint32_t dol_exp_num;       // Exposure mode, 1 for normal, dol 2 or 3 with corresponding count
    uint32_t enable_dgain;      // ISP internal debug parameter, currently ignored
    uint32_t set_dgain_short;   // ISP internal debug parameter, currently ignored
    uint32_t set_dgain_medium;  // ISP internal debug parameter, currently ignored
    uint32_t set_dgain_long;    // ISP internal debug parameter, currently ignored
    uint32_t short_maxexp_lines; // Maximum exposure lines for shortest frame, usually found in sensor mode registers, DOL2/3 values needed for IRAM allocation
    uint32_t medium_maxexp_lines; // Maximum exposure lines for normal frame, usually found in sensor mode registers, DOL3 value needed for IRAM allocation
    uint32_t vc_short_seq;      // Sequence for DOL2/3 mode's shortest frames
    uint32_t vc_medium_seq;     // Sequence for DOL2/3 mode's normal frames
    uint32_t vc_long_seq;       // Sequence for DOL2/3 mode's longest frames
} VIN_DEV_OUTPUT_ISP_S;
```
**Function Description**
> Defines the information for initializing dev output to the pipe.

**Member Descriptions**

| Member          | Meaning                                                                                     |
| :--------------: | :------------------------------------------------------------------------------------------- |
| dol_exp_num      | Exposure mode, 1 for standard, dol 2 or 3 set to respective counts.                            |
| enable_dgain     | ISP internal debugging parameter, temporarily ignored.                                          |
| set_dgain_short  | ISP internal debugging parameter, temporarily ignored.                                          |
| set_dgain_medium | ISP internal debugging parameter, temporarily ignored.                                          |
| set_dgain_long   | ISP internal debugging parameter, temporarily ignored.                                          |
| short_maxexp_lines | Maximum exposure lines for the shortest frame, typically found in the sensor mode register table, DOL2/3 values required for IRAM allocation. |
| medium_maxexp_lines | Maximum exposure lines for a normal frame, typically found in the sensor mode register table, DOL3 value required for IRAM allocation. |
| vc_short_seq     | Describes the sequence for DOL2/3 mode's shortest frames.                                       |
| vc_medium_seq    | Describes the sequence for DOL2/3 mode's normal frames.                                      |
| vc_long_seq      | Describes the sequence for DOL2/3 mode's longest frames.                                       |

### VIN_DEV_ATTR_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_ATTR_S {
    VIN_DEV_SIZE_S        stSize; // Input data size
    union {
        VIN_MIPI_ATTR_S  mipiAttr; // Mipi attributes
        VIN_DVP_ATTR_S   dvpAttr;  // Dvp attributes
    };
    VIN_DEV_INPUT_DDR_ATTR_S DdrIspAttr; // DDR input attribute configuration for ISP (offline or refill)
    VIN_DEV_OUTPUT_DDR_S outDdrAttr; // DDR output configuration for sif(dev)
    VIN_DEV_OUTPUT_ISP_S outIspAttr; // Output attributes for sif to ISP
} VIN_DEV_ATTR_S;
```
**Function Description**
> Defines the initialization attributes for dev.

**Member Descriptions**

| Member           | Meaning                                                                                         |
| :---------------: | :----------------------------------------------------------------------------------------------- |
| stSize            | Size of input data                                                                               |
| enIntfMode        | Interface mode for sif(dev), either mipi or dvp, currently mipi.                                  |
| DdrIspAttr        | Input DDR attribute configuration for ISP, offline or refill.                                   |
| outDdrAttr        | DDR output configuration for sif(dev) connection.                                                   |
| outIspAttr        | Attributes for sif to ISP connections.                                                              |

### VIN_DEV_ATTR_EX_S
**Structure Definition**
```c
typedef struct HB_VIN_DEV_ATTR_EX_S {
    uint32_t path_sel; // 0: sif-isp path; 1: sif-ipu path
    uint32_t roi_top;
    uint32_t roi_left;
    uint32_t roi_width;
    uint32_t roi_height;
    uint32_t grid_step; // Grid step size, a power of 2 between 4 and 128
    uint32_t grid_tolerance;
    uint32_t threshold;
    uint32_t weight_decay;
    uint32_t precision;
} VIN_DEV_ATTR_EX_S;
```
**Function Description**
> Defines metadata-related information.

**Member Descriptions**

| Member         | Meaning                                                                                      |
| :-------------: | :--------------------------------------------------------------------------------------------- |
| path_sel       | 0: SIF-ISP path; 1: SIF-IPU path                                                                 |
| roi_top        | Y-coordinate of ROI                                                                                   |
| roi_left       | X-coordinate of ROI                                                                                   |
| roi_width      | ROI width, must be a multiple of `grid_step`                                                            |
| roi_height     | ROI height, must be a multiple of `grid_step`                                                           |
| grid_step      | Width and height of each region in the motion detection grid, a power of 2 between 4 and 128.         |
| grid_tolerance | Threshold for comparing adjacent regions in a motion detection. A difference exceeds this to trigger a mot_det interrupt. |
| threshold      | Number of differing regions in the ROI exceeding this threshold triggers a mot_det interrupt.         |
| weight_decay   | Weighting factor for the current frame when updating the reference buffer, not a full replacement.     |
| precision      | Number of decimal places to retain during block calculations, valid range is 1 to 4.                  |



### VIN_PIPE_SENSOR_MODE_E
【Structure Definition】
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
【Function Description】
> Sensor operation mode

【Member Descriptions】
> Normal mode, DOL2 mode, DOL3 mode, PWL mode (compressed mode)

### VIN_PIPE_CFA_PATTERN_E
【Structure Definition】
```c
typedef enum HB_VIN_PIPE_CFA_PATTERN_E {
    PIPE_BAYER_RGGB = 0,
    PIPE_BAYER_GRBG,
    PIPE_BAYER_GBRG,
    PIPE_BAYER_BGGR,
    PIPE_MONOCHROME,
} VIN_PIPE_CFA_PATTERN_E;
```
【Function Description】
> Data format layout

【Member Descriptions】
> Different data storage formats

### VIN_PIPE_SIZE_S
【Structure Definition】
```c
typedef struct HB_VIN_PIPE_SIZE_S {
    uint32_t  format;
    uint32_t  width;
    uint32_t  height;
} VIN_PIPE_SIZE_S;
```
【Function Description】
> Defines pipe size data information

【Member Descriptions】

|  Member  | Meaning         |
| :------: | :------------- |
| format  | Data format    |
| width   | Data width     |
| height  | Data height    |

### VIN_PIPE_CALIB_S
【Structure Definition】
```c
typedef struct HB_VIN_PIPE_CALIB_S {
    uint32_t mode;
    unsigned char *lname;
} VIN_PIPE_CALIB_S;
```
【Function Description】
> Sensor calibration data loading

【Member Descriptions】

| Member   | Meaning                          |
| :------- | :------------------------------- |
| mode     | Enables sensor calibration loading (1 enabled, 0 disabled) |
| lname    | Calibration library name pointer |

### VIN_PIPE_ATTR_S
【Structure Definition】
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
    uint32_t   ispAfEn;
    uint32_t   bitwidth;
    uint32_t   startX;
    uint32_t   startY;
    VIN_PIPE_CALIB_S calib;
} VIN_PIPE_ATTR_S;
```
【Function Description】
> Defines pipe attribute information

【Member Descriptions】

|       Member       | Meaning                                                      |
| :----------------: | :----------------------------------------------------------- |
| ddrOutBufNum      | Data bit width (8, 10, 12, 14, 16, 20)                        |
| frameDepth        | Maximum number of frames to be fetched, frameDepth is the maximum when ddrOutBufNum is frameDepth - 3 |
| snsMode           | Sensor operating mode                                         |
| stSize            | Sensor data information, see previous definition                |
| cfaPattern        | Data format layout, consistent with the sensor                   |
| temperMode        | Temper mode, 0 off, 2 on                                      |
| BypassEnable      | Enables ISP bypass                                             |
| ispAlgoState      | Enables 3A algorithm library, 1 enabled, 0 disabled             |
| bitwidth          | Bit depth (8, 10, 12, 14, 16, 20)                               |
| startX            | X offset relative to the origin                                |
| startY            | Y offset relative to the origin                                |
| calib             | Calibration data loading status, 1 enabled, 0 disabled         |

### VIN_LDC_PATH_SEL_S
【Structure Definition】
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
【Function Description】
> Defines LDC attribute information

【Member Descriptions】

| Member   | Meaning                         |
| :------- | :------------------------------ |
| rg_y_only | Output type                     |
| rg_uv_mode | Output type                     |
| rg_uv_interpo | Turning related                 |
| rg_h_blank_cyc | Turning related                  |



### VIN_LDC_PICSIZE_S
**Structure Definition**
```c
typedef struct HB_VIN_LDC_PICSIZE_S {
    uint16_t pic_w;
    uint16_t pic_h;
} VIN_LDC_PICSIZE_S;
```
**Function Description**
> Defines the LDC width and height input information.

**Member Descriptions**

| Member | Meaning                                                                                   |
| :-----: | :----------------------------------------------------------------------------------------- |
| pic_w  | Set the size minus 1, e.g., if ISP output is 1920, set it to 1919.                            |
| pic_h  | Do not change this value except for size, ldc, and dis settings.                            |

### VIN_LDC_ALGOPARAM_S
**Structure Definition**
```c
typedef struct HB_VIN_LDC_ALGOPARAM_S {
    uint16_t rg_algo_param_b;
    uint16_t rg_algo_param_a;
} VIN_LDC_ALGOPARAM_S;
```
**Function Description**
> Defines LDC property information.

**Member Descriptions**

| Member        | Meaning                                                                                     |
| :------------ | :--------------------------------------------------------------------------------------------- |
| rg_algo_param_b | Parameters that require tuning.                                                            |
| rg_algo_param_a | Parameters that require tuning.                                                            |

### VIN_LDC_OFF_SHIFT_S
**Structure Definition**
```c
typedef struct HB_VIN_LDC_OFF_SHIFT_S {
    uint32_t rg_center_xoff:8;
    uint32_t rg_center_yoff:8;
    uint32_t reserved0:16;
} VIN_LDC_OFF_SHIFT_S;
```
**Function Description**
> Defines LDC property information.

**Member Descriptions**

| Member      | Meaning                                                                                     |
| :---------- | :--------------------------------------------------------------------------------------------- |
| rg_center_xoff | Correction for processing area.                                                            |
| rg_center_yoff | Correction for processing area.                                                            |

### VIN_LDC_WOI_S
**Structure Definition**
```c
typedef struct HB_VIN_LDC_WOI_S {
    uint32_t rg_start:12;
    uint32_t reserved1:4;
    uint32_t rg_length:12;
    uint32_t reserved0:4;
} VIN_LDC_WOI_S;
```
**Function Description**
> Defines LDC property information.

**Member Descriptions**

| Member    | Meaning                                                                                     |
| :-------: | :--------------------------------------------------------------------------------------------- |
| rg_start  | Correction for processing area.                                                            |
| rg_length | Correction for processing area.                                                            |

### VIN_LDC_ATTR_S
**Structure Definition**
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
**Function Description**
> Defines LDC property information.

**Member Descriptions**

| Member    | Meaning                                                                                           |
| :--------: | :-------------------------------------------------------------------------------------------------- |
| ldcEnable | Enables or disables the LDC.                                                                       |
| ldcPath   | Output type selection.                                                                              |
| yStartAddr | Address in Iram for usage.                                                                          |
| cStartAddr | Address in Iram for usage.                                                                          |
| picSize   | Input dimension size (excluding padding).                                                             |
| lineBuf   | Value set to 99.                                                                                   |
| xParam    | Parameters that require tuning.                                                                      |
| yParam    | Parameters that require tuning.                                                                      |
| offShift  | Correction for the processing area.                                                                |
| xWoi      | Correction for the processing area in the X direction.                                                 |
| yWoi      | Correction for the processing area in the Y direction.                                                 |

### VIN_DIS_PICSIZE_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_PICSIZE_S {
    uint16_t pic_w;
    uint16_t pic_h;
} VIN_DIS_PICSIZE_S;
```
**Function Description**
> Defines DIS property information.

**Member Descriptions**

| Member  | Meaning                                                                                   |
| :---: | :----------------------------------------------------------------------------------------- |
| pic_w  | Set the size minus 1, e.g., if ISP output is 1920, set it to 1919.                            |
| pic_h  | Set the size minus 1.                                                                               |

### VIN_DIS_PATH_SEL_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_PATH_SEL_S {
    uint32_t rg_dis_enable:1;
    uint32_t rg_dis_path_sel:1;
    uint32_t reserved0:30;
} VIN_DIS_PATH_SEL_S;
```
**Function Description**
> Defines DIS property information.

**Member Descriptions**

| Member       | Meaning                                                                                     |
| :-------------: | :--------------------------------------------------------------------------------------------- |
| rg_dis_enable | Dis output type selection.                                                                      |
| rg_dis_path_sel | Dis output type selection.                                                                      |



### VIN_DIS_CROP_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_CROP_S {
    uint16_t rg_dis_start;
    uint16_t rg_dis_end;
};
```
**Function Description**
> Defines the DIS cropping information.

**Member Descriptions**

| Member       | Meaning                                            |
| :----------- | :-------------------------------------------------- |
| rg_dis_start | Start of the cropping area for processing.          |
| rg_dis_end   | End of the cropping area for processing.             |

### VIN_DIS_CALLBACK_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_CALLBACK_S {
    void (*VIN_DIS_DATA_CB)(uint32_t pipeId, uint32_t event,
                           VIN_DIS_MV_INFO_S *disData, void *userData);
};
```
**Function Description**
> Defines the callback interface for DIS data.

**Member Descriptions**

| Member           | Meaning                                             |
| :--------------- | :-------------------------------------------------- |
| VIN_DIS_DATA_CB  | Callback function that returns data to the user after receiving it. |

### VIN_DIS_MV_INFO_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_MV_INFO_S {
    int  gmvX;
    int  gmvY;
    int  xUpdate;
    int  yUpdate;
};
```
**Function Description**
> Defines information about coordinate movement.

**Member Descriptions**

| Member   | Meaning                                                                                   |
| :------- | :----------------------------------------------------------------------------------------- |
| gmvX     | Absolute position relative to the camera center, with gmv being the movement relative to the fixed lock position. |
| gmvY     | Absolute position relative to the camera center.                                                  |
| xUpdate  | Relative change in x position compared to the previous frame, regardless of whether the camera was locked or not. |
| yUpdate  | Relative change in y position compared to the previous frame.                                   |

### VIN_DIS_ATTR_S
**Structure Definition**
```c
typedef struct HB_VIN_DIS_ATTR_S {
    VIN_DIS_PICSIZE_S picSize;
    VIN_DIS_PATH_SEL_S disPath;
    uint32_t disHratio;
    uint32_t disVratio;
    VIN_DIS_CROP_S xCrop;
    VIN_DIS_CROP_S yCrop;
};
```
**Function Description**
> Defines DIS attribute information.

**Member Descriptions**

| Member    | Meaning                                      |
| :--------: | :------------------------------------------- |
| picSize   | Input image size                              |
| disPath   | Output type                                  |
| disHratio | Set to 65536                                  |
| disVratio | Set to 65536                                  |
| xCrop     | Cropping area adjustment for X axis           |
| yCrop     | Cropping area adjustment for Y axis           |

### VIN_LENS_FUNC_TYPE_E
**Structure Definition**
```c
typedef enum HB_VIN_LENS_FUNC_TYPE_E {
    VIN_LENS_AF_TYPE = 1,
    VIN_LENS_ZOOM_TYPE,
    VIN_LENS_INVALID,
};
```
**Function Description**
> Lens motor function types.

**Member Descriptions**
- AF: Automatic focus, changing focus distance.
- ZOOM: Zoom, changing focal length.

### VIN_LENS_CTRL_ATTR_S
**Structure Definition**
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
**Function Description**
> Defines attributes for lens control.

**Member Descriptions**

| Member       | Meaning                                                                                       |
| :----------- | :--------------------------------------------------------------------------------------------- |
| port         | Corresponds to each input, with pipeId.                                                       |
| motorType    | Motor driver type, see VIN_LENS_MOTOR_TYPE_E.                                                 |
| maxStep      | Maximum motor steps.                                                                             |
| initPos      | Initial motor position.                                                                         |
| minPos       | Minimum motor position.                                                                         |
| maxPos       | Maximum motor position.                                                                         |
| pwmNum       | PWM device number for motor control.                                                           |
| pwmDuty      | PWM duty cycle for motor control.                                                               |
| pwmPeriod    | PWM frequency for motor control.                                                                |
| pulseForwardNum | Pulse device number for forward motor control.                                               |
| pulseBackNum  | Pulse device number for backward motor control.                                              |
| pulseDuty    | Pulse duty cycle for motor control.                                                            |
| pulsePeriod  | Pulse frequency for motor control.                                                             |
| i2cNum       | I2C device number for motor control.                                                           |
| i2cAddr      | I2C address for motor control.                                                                  |
| gpioA1       | GPIO number for motor control A+.                                                              |
| gpioA2       | GPIO number for motor control A-.                                                              |
| gpioB1       | GPIO number for motor control B+.                                                              |
| gpioB2       | GPIO number for motor control B-.                                                              |



### VIN_LENS_MOTOR_TYPE_E
**Structure Definition**
```c
typedef enum HB_VIN_LENS_MOTOR_TYPE_E {
    VIN_LENS_PWM_TYPE = 0,
    VIN_LENS_PULSE_TYPE,
    VIN_LENS_I2C_TYPE,
    VIN_LENSSPI_TYPE,
    VIN_LENS_GPIO_TYPE
} VIN_LENS_MOTOR_TYPE_E;
```
**Function Description**
> Motor drive type, available options include PWM, pulse count, I2C control, SPI communication, and GPIO timing control. GPIO method has been tested and validated due to hardware environment factors.

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
| 0x1C | YUV 420 8-bit (Chroma Shifted Pixel Sampling)   |
| 0x1D | YUV 420 10-bit (Chroma Shifted Pixel Sampling) |
| 0x1E | YUV 422 8-bit                                  |
| 0x1F | YUV 422 10-bit                                 |

### SIF MCLK

| ISP Application Scenario | SIF_MCLK(MHz) |
| :----------------------- | :-----------: |
| 8M 30fps Input           |     326.4     |
| 2M 30fps 2-Stream TDM   |    148.36     |
| 2M 30fps Single Stream   |    102.00     |
| 8M DOL2 30fps            |    544.00     |
| 2M 15fps 4-Stream TDM   |    148.36     |

### VPU CLK

| VPU Application Scenario | Codec | VPU_BCLK/VPU_CCLK(MHz) |
| :----------------------- | :----: | :--------------------: |
| 8M@30fps                 |  AVC  |         326.4          |
|                          | HEVC  |          408           |
| 2M*4@30fps               |  AVC  |          544           |
|                          | HEVC  |          544           |
| 2M @30fps                |  AVC  |          204           |
|                          | HEVC  |          204           |

## Error Codes

VIN error codes are listed below:

| Error Code   | Macro Definition                            | Description                              |
| :----------- | :----------------------------------------- | :---------------------------------------- |
| -268565505   | HB_ERR_VIN_CREATE_PIPE_FAIL                 | Failed to create PIPE                     |
| -268565506   | HB_ERR_VIN_SIF_INIT_FAIL                    | DEV(Sif) initialization failure           |
| -268565507   | HB_ERR_VIN_DEV_START_FAIL                   | DEV(Sif) start failure                    |
| -268565508   | HB_ERR_VIN_PIPE_START_FAIL                  | ISP start failure                        |
| -268565509   | HB_ERR_VIN_CHN_UNEXIST                     | Channel does not exist                   |
| -268565510   | HB_ERR_VIN_INVALID_PARAM                   | Invalid interface parameter               |
| -268565511   | HB_ERR_VIN_ISP_INIT_FAIL                    | ISP initialization failure                |
| -268565512   | HB_ERR_VIN_ISP_FRAME_CORRUPTED              | ISP frame corrupted; driver may drop frames |
| -268565513   | HB_ERR_VIN_CHANNEL_INIT_FAIL                | Failure initializing two ISP channels     |
| -268565514   | HB_ERR_VIN_DWE_INIT_FAIL                    | DWE initialization failure                |
| -268565515   | HB_ERR_VIN_SET_DEV_ATTREX_FAIL              | SIF extended attribute initialization fail |
| -268565516   | HB_ERR_VIN_LENS_INIT_FAIL                   | Lens motor initialization failure          |
| -268565517   | HB_ERR_VIN_SEND_PIPERAW_FAIL                | Failed to send SIF raw data               |
| -268565518   | HB_ERR_VIN_NULL_POINT                      | VIN module has a null pointer              |
| -268565519   | HB_ERR_VIN_GET_CHNFRAME_FAIL                | Failed to get ISP output data              |
| -268565520   | HB_ERR_VIN_GET_DEVFRAME_FAIL                | Failed to get SIF output data              |
| -268565521   | HB_ERR_VIN_MD_ENABLE_FAIL                   | MotionDetect enable failed                |
| -268565522   | HB_ERR_VIN_MD_DISABLE_FAIL                  | MotionDetect disable failed               |
| -268565523   | HB_ERR_VIN_SWITCH_SNS_TABLE_FAIL            | Failed to switch ISP mode (linear/DOL)     |

## Reference Code
Example code for the VIN part can be referred to in the [get_sif_data](./multimedia_samples/get_sif_data) and [get_isp_data](./multimedia_samples/get_isp_data) functions.

