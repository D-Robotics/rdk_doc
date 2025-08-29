---
sidebar_position: 8
---

# IPC使用指南

此章节着重说明MCU侧的相关使用说明，更多的IPC的原理和使用可以查阅 [IPC模块介绍](../../../07_Advanced_development/02_linux_development/04_driver_development_s100/06_driver_ipc.md) 章节。

## IPC配置相关

一个IPC有8个channels，但是共享一个中断，因此一个IPC只能在MCU0或MCU1其中一个系统上使能。

当MCU1使用IPC时，需要配置两部分内容。
1. 需要配置回调函数，作用是当IPC收到/发送数据时产生回调。当然客户可自行定制当传输数据错误时的回调函数。下文以IPC0为例介绍。
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
2. 设置receive_coreid。如果是在MCU1上，则需要"receive_coreid=Ipc_Receive_Core1"。同时需要保障MCU0关于IPC设置相同。
- MCU0文件地址：/mcu/Config/McalCdd/gen_s100_sip_B/Ipc/src/Ipc_Cfg.c
- MCU1文件地址：/mcu/Config/McalCdd/gen_s100_sip_B_mcu1/Ipc/src/Ipc_Cfg.c
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
## IPC 使用情况

#### instance0
| Instance | Channel | receive core id  | Description       |
|----------|---------|------------------|-------------------|
| instance0| 0       | Ipc_Receive_Core1| CAN Reserve       |
| instance0| 1       | Ipc_Receive_Core1| CAN Reserve       |
| instance0| 2       | Ipc_Receive_Core1| CAN8              |
| instance0| 3       | Ipc_Receive_Core1| CAN9              |
| instance0| 4       | Ipc_Receive_Core1| CAN5              |
| instance0| 5       | Ipc_Receive_Core1| CAN Reserve       |
| instance0| 6       | Ipc_Receive_Core1| CAN6              |
| instance0| 7       | Ipc_Receive_Core1| CAN7              |

#### instance1

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance1| 0       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 1       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 2       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 3       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 4       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 5       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 6       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance1| 7       | Ipc_Receive_Core0| Regulatory Reserve  |

#### instance2

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance2| 0       | Ipc_Receive_Core0| Regulatory Reserve  |
| instance2| 1       | Ipc_Receive_Core0| Regulatory Reserve  |

#### instance3

| Instance | Channel | receive core id  | Description       |
|----------|---------|------------------|-------------------|
| instance3| 0       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 1       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 2       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 3       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 4       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 5       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 6       | Ipc_Receive_Core0| Crypto Reserve    |
| instance3| 7       | Ipc_Receive_Core0| Crypto Reserve    |

#### instance4

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance4| 0       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 1       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 2       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 3       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 4       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 5       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 6       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance4| 7       | Ipc_Receive_Core0| 客户预留：Reserve   |

#### instance5

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance5| 0       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 1       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 2       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 3       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 4       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 5       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 6       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance5| 7       | Ipc_Receive_Core0| 客户预留：Reserve   |

#### instance6

| Instance | Channel | receive core id  | Description         |
|----------|---------|------------------|---------------------|
| instance6| 0       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 1       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 2       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 3       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 4       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 5       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 6       | Ipc_Receive_Core0| 客户预留：Reserve   |
| instance6| 7       | Ipc_Receive_Core0| 客户预留：Reserve   |

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


## 应用sample

:::tip
所运行的应用程序sample均运行于Acore侧，并与MCU1进行通信，因此在使用前需运行MCU1的系统.

**运行方式:** [MCU1启动步骤](./01_basic_information.md#start_mcu1)
:::

### IpcBox功能介绍

IpcBox是IPC的应用扩展，使用instance7用于透传外设数据和执行MCU侧的CMD应用，简称RunCmd应用。

对应的Acore侧应用使用方式见[IPC模块介绍](../../../07_Advanced_development/02_linux_development/04_driver_development_s100/06_driver_ipc.md) 章节。

#### 透传外设数据

**实现情况：**

| 项目      | 实现情况 |        备注     |
|---------- |---------|----------------|
| 透传SPI   | 未实现   | 未实现          |
| 透传I2C   | 未实现   | 未实现          |
| 透传Uart  | 已实现   | 固定使用uart5   |

:::tip
持续更新中
:::

#### RUNCMD应用

实现原理，主要分为以下两个过程：
1. 接收过程
Acore向MCU发送数据时触发mcu的中断，在中断的callback中将数据存储到队列中
2. 运行过程
- mcu存在一个常驻线程，不断的在去读队列中的数据是否为空，若不为空，则校验并解析数据，识别出cmd命令并运行
- freertos的cmd的应用类似于uboot的cmd的命令，通过此方式用户可以很方便的定制化自己的应用，在此场景中，运行的cmd将adc的值读出，再通过ipc返回给Acore

![Acore与MCU之间透传Can数据架构图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu-runcmd.jpg)

### 应用程序接口

此部分为MCU侧的IPC接口。

#### void Ipc_MDMA_Init(Ipc_InstanceConfigType* pConfigPtr, uint32 InstanceId)

```shell
Description：Ipc MDMA Init.

Sync/Async: Synchronous
Parameters(in)
    pConfigPtr：the pointer to the device configuration parameter
    InstanceId：InstanceId id
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```


#### void Ipc_MDMA_DeInit(uint32 InstanceId)

```shell
Description：Subsystem driver deinitialization function.

Sync/Async: Synchronous
Parameters(in)
    InstanceId：InstanceId id
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Ipc_GetVersionInfo(Std_VersionInfoType * versioninfo)

```shell
Description：get driver version.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    versioninfo: the pointer to Version Info
Parameters(out)
    None
Return value：None
```


#### Std_ReturnType Ipc_MDMA_CheckRemoteCoreReady(uint32 InstanceId)

```shell
Description：check whether remote core is ready.

Sync/Async: Synchronous
Parameters(in)
    InstanceId：InstanceId id
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: remote core is ready
    IPC_E_PARAM_ERROR: param illegal
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_INSTANCE_NOT_READY_ERROR : remote core is Not ready
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open
```

#### void Std_ReturnType Ipc_MDMA_SendMsg(uint32 InstanceId, uint32 ChanId, uint32 Size, uint8* Buf, uint32 Timeout)

```shell
Description：send message.

Sync/Async: Synchronous
Parameters(in)
    InstanceId: Instance id
    ChanId: channel id
    Size: the size of buf to be sent
    Buf: the pointer to the memory that contains the buf to be sent
    Timeout: timeout(us)
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    IPC_E_PARAM_ERROR: param is illegal
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open
    IPC_E_TIMEOUT_ERROR: send timeout
    IPC_E_NO_MEMORY_ERROR: no memory to send buf
    PC_E_CHECKRESERROR: check resource error
```

:::tip
dma硬件要求传输地址16字节对齐，buffer应该如下定义，首地址和size16字节对齐: static uint8 __attribute__((aligned(16))) Ipc_Send_Buf[8192];
:::

#### Std_ReturnType Ipc_MDMA_PollMsg(uint32 InstanceId)

```shell
Description：poll message If the Instance does not receive data using interrupts.

Sync/Async: Synchronous
Parameters(in)
    InstanceId: Instance id
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    IPC_E_PARAM_ERROR: param is illegal
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_CHANNEL_NOT_OPEN: Instance is not open
    IPC_E_NO_DATA_TO_RECEIVE_ER ROR: No data to be recvived
```

#### Std_ReturnType Ipc_MDMA_OpenInstance(uint32 InstanceId)

```shell
Description：Open a Instance pointed to by ID.

Sync/Async: Synchronous
Parameters(in)
    InstanceId: Instance id
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_CHANNEL_NOT_CLOSE: Instance has been opened
    IPC_E_PARAM_ERROR param is illegal
```

### Std_ReturnType Ipc_MDMA_CloseInstance(uint32 InstanceId)

```shell
Description：close a Instance pointed to by ID.

Sync/Async: Synchronous
Parameters(in)
    InstanceId: Instance id
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_CHANNEL_NOT_CLOSE: Instance has been opened
    IPC_E_PARAM_ERROR param is illegal
```


### Std_ReturnType Ipc_MDMA_TryGetHwResource(uint32 InstanceId, uint32 ChanId, uint32 BufSize)

```shell
Description：try get Hardware resource.

Sync/Async: Synchronous
Parameters(in)
    InstanceId ChanId BufSize: Instance id Chanel Id buf size
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    IPC_E_DRIVER_NOT_INIT: Driver is not init
    IPC_E_DEVICE_BUSY: Instance is busy.
    IPC_E_MDMA_BUSY: Send MDMA is busy.
    IPC_E_NO_BUF_ERROR: no buffer
    IPC_E_CHANNEL_NOT_OPEN: Instance has been closed
```
