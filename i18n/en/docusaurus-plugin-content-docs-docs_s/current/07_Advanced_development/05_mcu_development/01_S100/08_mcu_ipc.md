---
sidebar_position: 8
---

# IPC Usage Guide

This section focuses on usage instructions related to the MCU side. For more details on IPC principles and usage, please refer to the [IPC Module Introduction](../../../07_Advanced_development/02_linux_development/04_driver_development_s100/06_driver_ipc.md) section.

## IPC Configuration

An IPC has 8 channels but shares a single interrupt; therefore, an IPC can only be enabled on either MCU0 or MCU1, but not both simultaneously.

When using IPC on MCU1, two parts need to be configured:

1. Configure callback functions, which are triggered when IPC receives/sends data. Customers may also customize callback functions for handling data transmission errors. The following example uses IPC0 for illustration.

```c
static Ipc_ChannelConfigType Ipc_ShmInstance0CfgChannel[8] = {
{
    .ChannelId = 0,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_0BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 1,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_1BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 2,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_2BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 3,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_3BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 4,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_4BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 5,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_5BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 6,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_6BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
{
    .ChannelId = 7,
    .ChannelData   = {
        .NumPools = 1,
        .PoolCfg        = Ipc_ShmIpcInstance_0CfgIpcChannel_7BufPool,
        .RxCallback     = IpcTp_InsCan_RxCallback,
        .RxCallbackArg  = (NULL_PTR),
        .TxErrCallback    = DefaultTxErrCallback,
        .TxErrCallbackArg = (NULL_PTR),
    },
},
};
```

2. Set `receive_coreid`. If IPC is used on MCU1, set `receive_coreid=Ipc_Receive_Core1`. Ensure that the IPC configuration on MCU0 matches accordingly.

- MCU0 file path: `/mcu/Config/McalCdd/gen_s100_sip_B/Ipc/src/Ipc_Cfg.c`  
- MCU1 file path: `/mcu/Config/McalCdd/gen_s100_sip_B_mcu1/Ipc/src/Ipc_Cfg.c`

```c
Ipc_InstanceConfigType Ipc_ShmCfgInstances0 = {
    .Ipc_InstanceId       = 0U,
    .Ipc_ChannelNum       = 8U,
    .LocalCtlAddr         = 0xcdd9e00,
    .RemoteCtlAddr        = 0xcdd9400,
    .CtlShmSize           = 0xa00,
    .LocalDataAddr        = 0xb4080000,
    .RemoteDataAddr       = 0xb4000000,
    .DataShmSize          = 0x80000,
    .SendDmaChanIdx       = 0xffU,
    .Async                = (TRUE),
    .HwInfo               = {
        .Ipc_HwId         = CPU_IPC0,/**< the id of the Hardware */
        .RecvIrqUsed      = (TRUE),/**< Whether to use Recv interrupt */
        .SendMboxId       = 0,/**< the mailbox id */
        .RecvMboxId       = 16,/**< the mailbox id */
        .RemoteIrq        = 16,
        .LocalIrq         = 0,
        .UseMDMA          = (TRUE),
    },
    .Ipc_ChannelConfigPtr = Ipc_ShmInstance0CfgChannel,
    .receive_coreid = Ipc_Receive_Core1,
};
```

## IPC Usage Overview

#### instance0
| Instance   | Channel | receive core id    | Description       |
|------------|---------|--------------------|-------------------|
| instance0  | 0       | Ipc_Receive_Core1  | CAN Reserve       |
| instance0  | 1       | Ipc_Receive_Core1  | CAN Reserve       |
| instance0  | 2       | Ipc_Receive_Core1  | CAN8              |
| instance0  | 3       | Ipc_Receive_Core1  | CAN9              |
| instance0  | 4       | Ipc_Receive_Core1  | CAN5              |
| instance0  | 5       | Ipc_Receive_Core1  | CAN Reserve       |
| instance0  | 6       | Ipc_Receive_Core1  | CAN6              |
| instance0  | 7       | Ipc_Receive_Core1  | CAN7              |

#### instance1

| Instance   | Channel | receive core id    | Description         |
|------------|---------|--------------------|---------------------|
| instance1  | 0       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 1       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 2       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 3       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 4       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 5       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 6       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance1  | 7       | Ipc_Receive_Core0  | Regulatory Reserve  |

#### instance2

| Instance   | Channel | receive core id    | Description         |
|------------|---------|--------------------|---------------------|
| instance2  | 0       | Ipc_Receive_Core0  | Regulatory Reserve  |
| instance2  | 1       | Ipc_Receive_Core0  | Regulatory Reserve  |

#### instance3

| Instance   | Channel | receive core id    | Description       |
|------------|---------|--------------------|-------------------|
| instance3  | 0       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 1       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 2       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 3       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 4       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 5       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 6       | Ipc_Receive_Core0  | Crypto Reserve    |
| instance3  | 7       | Ipc_Receive_Core0  | Crypto Reserve    |

#### instance4

| Instance   | Channel | receive core id    | Description              |
|------------|---------|--------------------|--------------------------|
| instance4  | 0       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 1       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 2       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 3       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 4       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 5       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 6       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance4  | 7       | Ipc_Receive_Core0  | Customer Reserved: Reserve |

#### instance5

| Instance   | Channel | receive core id    | Description              |
|------------|---------|--------------------|--------------------------|
| instance5  | 0       | Ipc_Receive_Core0  | Customer Reserved: Reserve |
| instance5  | 1       | Ipc_Receive_Core0  | Customer Reserved: Reserve || instance5| 2       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance5| 3       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance5| 4       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance5| 5       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance5| 6       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance5| 7       | Ipc_Receive_Core0| Customer Reserved: Reserve   |

#### instance6

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance6| 0       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 1       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 2       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 3       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 4       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 5       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 6       | Ipc_Receive_Core0| Customer Reserved: Reserve   |
| instance6| 7       | Ipc_Receive_Core0| Customer Reserved: Reserve   |

#### instance7

| Instance | Channel | receive core id  | Description |
|----------|---------|------------------|-------------|
| instance7| 0       | Ipc_Receive_Core1| runcmd      |
| instance7| 1       | Ipc_Receive_Core1| runcmd      |
| instance7| 2       | Ipc_Receive_Core1| SPI         |
| instance7| 3       | Ipc_Receive_Core1| SPI         |
| instance7| 4       | Ipc_Receive_Core1| UART        |
| instance7| 5       | Ipc_Receive_Core1| UART        |
| instance7| 6       | Ipc_Receive_Core1| I2C         |
| instance7| 7       | Ipc_Receive_Core1| I2C         |

#### instance8

| Instance  | Channel | Function          | Description |
|-----------|---------|-------------------|-------------|
| instance8 | 0       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 1       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 2       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 3       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 4       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 5       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 6       | Ipc_Receive_Core0 | Reserve        |
| instance8 | 7       | Ipc_Receive_Core0 | Crypto Reserve |


## Application Sample

:::tip
All application samples run on the Acore side and communicate with MCU1. Therefore, the MCU1 system must be running before use.

**How to run:** [MCU1 Startup Steps](./01_basic_information.md#start_mcu1)
:::

### IpcBox Feature Overview

IpcBox is an IPC application extension that uses instance7 to transparently transmit peripheral data and execute CMD applications on the MCU side, referred to as the RunCmd application.

For usage instructions on the Acore side, refer to the [IPC Module Introduction](../../../07_Advanced_development/02_linux_development/04_driver_development_s100/06_driver_ipc.md) section.

#### Transparent Transmission of Peripheral Data

**Implementation Status:**

| Item          | Implementation Status | Remarks           |
|---------------|------------------------|-------------------|
| SPI Passthrough | Not Implemented      | Not Implemented   |
| I2C Passthrough | Not Implemented      | Not Implemented   |
| UART Passthrough| Implemented          | Fixed to UART5    |

:::tip
Continuously being updated
:::

#### RUNCMD Application

The implementation principle consists mainly of the following two processes:

1. **Receiving Process**  
   When Acore sends data to the MCU, it triggers an interrupt on the MCU. Within the interrupt callback, the data is stored into a queue.

2. **Execution Process**  
   - The MCU runs a persistent thread that continuously checks whether the queue contains data. If data is present, it validates and parses the data, identifies the CMD command, and executes it.
   - The CMD application in FreeRTOS is similar to U-Boot’s CMD framework. This approach allows users to easily customize their own applications. In this scenario, the executed CMD reads the ADC value and returns it to Acore via IPC.

![Architecture Diagram of CAN Data Transparent Transmission between Acore and MCU](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu-runcmd.jpg)

### Application Programming Interface

This section describes the IPC interfaces on the MCU side.

#### void Ipc_MDMA_Init(Ipc_InstanceConfigType* pConfigPtr, uint32 InstanceId)

```shell
Description: Ipc MDMA Init.

Sync/Async: Synchronous
Parameters (in)
    pConfigPtr: pointer to the device configuration parameters
    InstanceId: Instance ID
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```


#### void Ipc_MDMA_DeInit(uint32 InstanceId)

```shell
Description: Subsystem driver deinitialization function.

Sync/Async: Synchronous
Parameters (in)
    InstanceId: Instance ID
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```

#### void Ipc_GetVersionInfo(Std_VersionInfoType * versioninfo)

```shell
Description: Get driver version.

Sync/Async: Synchronous
Parameters (in)
    None
Parameters (inout)
    versioninfo: pointer to Version Info
Parameters (out)
    None
Return value: None
```


#### Std_ReturnType Ipc_MDMA_CheckRemoteCoreReady(uint32 InstanceId)

```shell
Description: Check whether the remote core is ready.

Sync/Async: Synchronous
Parameters (in)
    InstanceId: Instance ID
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: remote core is ready
    IPC_E_PARAM_ERROR: illegal parameter
    IPC_E_DRIVER_NOT_INIT: Driver is not initialized
    IPC_E_INSTANCE_NOT_READY_ERROR: remote core is not ready
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open
```

#### Std_ReturnType Ipc_MDMA_SendMsg(uint32 InstanceId, uint32 ChanId, uint32 Size, uint8* Buf, uint32 Timeout)

```shell
Description: Send message.

Sync/Async: Synchronous
Parameters (in)
    InstanceId: Instance ID
    ChanId: Channel ID
    Size: size of the buffer to be sent
    Buf: pointer to the memory containing the data to be sent
    Timeout: timeout (in microseconds)
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: success
    IPC_E_PARAM_ERROR: illegal parameter
    IPC_E_DRIVER_NOT_INIT: Driver is not initialized
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open
    IPC_E_TIMEOUT_ERROR: send timeout
    IPC_E_NO_MEMORY_ERROR: insufficient memory to send buffer
    IPC_E_CHECKRESERROR: resource check error
```

:::tip
DMA hardware requires 16-byte alignment for transfer addresses. The buffer should be defined as follows, ensuring both the starting address and size are 16-byte aligned:  
static uint8 __attribute__((aligned(16))) Ipc_Send_Buf[8192];
:::

#### Std_ReturnType Ipc_MDMA_PollMsg(uint32 InstanceId)

```shell
Description: Poll for messages if the instance does not use interrupts to receive data.

Sync/Async: Synchronous
Parameters (in)
    InstanceId: Instance ID
Parameters (inout)
    None
Parameters (out)
    None
```Return value: Std_ReturnType  
    E_OK: success  
    IPC_E_PARAM_ERROR: param is illegal  
    IPC_E_DRIVER_NOT_INIT: Driver is not init  
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open  
    IPC_E_NO_DATA_TO_RECEIVE_ERROR: No data to be received  

```

#### Std_ReturnType Ipc_MDMA_OpenInstance(uint32 InstanceId)

```shell
Description: Open an instance pointed to by ID.

Sync/Async: Synchronous  
Parameters(in)  
    InstanceId: Instance id  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: Std_ReturnType  
    E_OK: success  
    IPC_E_DRIVER_NOT_INIT: Driver is not init  
    IPC_E_CHANNEL_NOT_CLOSE: Instance has been opened  
    IPC_E_PARAM_ERROR: param is illegal  
```

### Std_ReturnType Ipc_MDMA_CloseInstance(uint32 InstanceId)

```shell
Description: Close an instance pointed to by ID.

Sync/Async: Synchronous  
Parameters(in)  
    InstanceId: Instance id  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: Std_ReturnType  
    E_OK: success  
    IPC_E_DRIVER_NOT_INIT: Driver is not init  
    IPC_E_CHANNEL_NOT_CLOSE: Instance has been opened  
    IPC_E_PARAM_ERROR: param is illegal  
```


### Std_ReturnType Ipc_MDMA_TryGetHwResource(uint32 InstanceId, uint32 ChanId, uint32 BufSize)

```shell
Description: Try to get hardware resource.

Sync/Async: Synchronous  
Parameters(in)  
    InstanceId ChanId BufSize: Instance id Channel Id buf size  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: Std_ReturnType  
    E_OK: success  
    IPC_E_DRIVER_NOT_INIT: Driver is not init  
    IPC_E_DEVICE_BUSY: Instance is busy.  
    IPC_E_MDMA_BUSY: Send MDMA is busy.  
    IPC_E_NO_BUF_ERROR: no buffer  
    IPC_E_CHANNEL_NOT_OPEN: Instance has been closed  
```