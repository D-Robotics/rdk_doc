---
sidebar_position: 6
---

# SPI使用指南

## 硬件支持

- 如果SPI使用DMA传输，必须保证以下限制:
    - 传输长度需要保证8字节对齐，否则会出现数据越界的问题;
    - 发送和接收数据的buffer地址必须对齐为64字节;
    - Channel单次传输数据量不能超过4096字节；
- 支持SPI作为Slave功能;
- Mcu侧SPI做Slave时，速率大于9M，只能采用DMA模式;
- Mcu侧SPI在8位模式下工作时，具有以下限制:
    - 1个SPI并行收发最大速度: 50M;
    - 2个SPI并行收发最大速度: 33.3M;
    - 3个SPI并行收发最大速度: 25M;
    - 4个SPI并行收发最大速度: 20M;
    - 5个SPI并行收发最大速度: 15.6M;
    - 6个SPI并行收发最大速度: 12.5M;
- SPI外设只支持MSB;
- SPI SpiCsPolarity和spitimeclk2c都不支持配置，SpiCsPolarity默认为低电平有效。
- SPI SpiCsToggleEnable配置使能后，Spi传输完一个frame后，会拉高CS引脚，仅支持TRAILING模式，LEADING模式不支持。
- SPI DMA模式相对于非DMA模式，消耗CPU负载更低。
- 当SPI采用非DMA方式发送数据时，若系统负载比较大，会因为SPI中断响应不及时导致SPI FIFO未能及时写入，从而导致CS引脚短暂拉高，此时推荐采用DMA模式。
- 如果使用场景是多个Sequence使用同一个SPI IP进行异步传输，且Sequence之间并没有传输完成先后顺序，则会存在Sequence传输排队的现象，此时需要在SchM_Spi.c文件中实现关中断临界区保护。


## 代码路径

- McalCdd/Spi/inc/Spi.h - SPI驱动程序的头文件
- McalCdd/Spi/inc/Spi_Lld.h - SPI底层驱动程序的头文件
- McalCdd/Spi/src/Spi.c - SPI驱动程序的源文件
- McalCdd/Spi/src/Spi_Lld.c - SPI底层驱动程序的源文件
- McalCdd/Common/Register/inc/Spi_Register.h - SPI寄存器定义文件
- Platform/Schm/SchM_Spi.h - SPI模块的调度管理头文件
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/inc/Spi_Cfg.h - SPI配置头文件
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/inc/Spi_PBcfg.h - SPI PB配置头文件
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/src/Spi_PBcfg.c - SPI PB配置源文件
- samples/Spi/SPI_sample/Spi_sample.c - SPI sample 代码

## 应用sample

- Sample中使用了DMA，但是没有进行初始化，是因为在Target/Target-hobot-lite-freertos-mcu1/target/HorizonTask.c文件中已经进行了初始化。


S100 有SPI2-SPI7六个SPI硬件，SPI通道和外部设备以及物理单元抽象关键信息对应如下:

| **SPI通道** | **External device**     | **Spi Physical Unit**      |
|-------------|-------------------------| ---------------------------|
| SPI2        | SPI_SpiExternalDevice_0 | Spi_PhyConfig_SpiPhyUnit_0 |
| SPI3        | SPI_SpiExternalDevice_1 | Spi_PhyConfig_SpiPhyUnit_1 |
| SPI4        | SPI_SpiExternalDevice_2 | Spi_PhyConfig_SpiPhyUnit_2 |
| SPI5        | SPI_SpiExternalDevice_3 | Spi_PhyConfig_SpiPhyUnit_3 |
| SPI6        | SPI_SpiExternalDevice_4 | Spi_PhyConfig_SpiPhyUnit_4 |
| SPI7        | SPI_SpiExternalDevice_5 | Spi_PhyConfig_SpiPhyUnit_5 |

### 配置说明

以SPI4为例,配置如下：

```c
//Config/McalCdd/gen_s100_sip_B_mcu1/Spi/src/Spi_PBcfg.c
static const Spi_PhyCfgType Spi_PhyConfig_SpiPhyUnit_2 =
{
    (uint8)2U, /* Instance */
    SPI_SPURIOUS_CORE_ID, /* SpiCoreUse */
    /* Spi Slave mode*/
    (boolean)FALSE,  //FALSE为master模式，true为slave模式
    /* Spi Test Mode Operation Enable*/
    (boolean)FALSE, // 回环模式，不走线
    /* Spi Sample Point*/
    (uint32)0U,
#if (SPI_DMA_ENABLE == STD_ON)
    (boolean)FALSE, //DMA配置
#endif
    SPI_IP_POLLING, /* Transfer mode 可选中断模式和轮询模式*/
    (uint8)2U, /* State structure element from the array */
    //SPI_PHYUNIT_SYNC /* IsSync */
    SPI_PHYUNIT_ASYNC //同步模式或异步模式
};

static const Spi_ChannelCfg SpiChannel_2 =
{
    /* SpiChannel_2*/
    EB, /* BufferType IB or EB */ //使用外部内存或者内部内存
    8U, /* Frame size */
    (uint32)1U, /* In the case SpiDefaultData is disabled. Set this value is 1U */
    Spi_TxDefaultBufferSpiChannel_2,
    Spi_RxDefaultBufferSpiChannel_2,
    256U, /* Length (configured in SpiEbMaxLength) */
    &Spi_BufferSpiChannel_2,
    SPI_SPURIOUS_CORE_ID,
    &Spi_ChannelState[2U] /* Spi_ChannelState pointer */
};

```


### 使用示例

spi_test 命令用于测试SPI（Serial Peripheral Interface，串行外设接口）功能。该命令支持初始化和参数设置、获取参数以及执行SPI测试。

- spitest 命令支持以下三种操作模式，通过第一个参数指定：
    - 0：初始化和参数设置
    - 1：获取参数
    - 2：执行SPI测试

- 参数解析
    - 操作模式 (argv[1]): 指定要执行的操作，取值范围为0、1或2。
    - 设备ID (argv[2]): 指定SPI设备的ID，与使用哪一个SPI相关，对应情况可参考本文首节中SPI关键信息对应表。
    - 通道号 (argv[3]): 指定SPI通道号，对应上文的Spi_ChannelCfg配置，通常只在切换内部内存和外部内存时需要更改。
    - 同步模式 (argv[4]): 仅在操作模式为0时使用，指定SPI通信的同步模式。
        - 0: 异步模式
        - 1: 同步模式
    - 传输模式 (argv[5]): 仅在操作模式为0时使用，指定SPI数据的传输模式。
        - 0: 轮询模式 (Polling)
        - 1: 中断模式 (Interrupt)

:::tip
应用层配置与底层配置应保持一致，否则会出现错误。
:::

将SPI4的MISO和MOSI短接，运行以下命令
```shell
D-Robotics:/$ spi_test 2
[0433.628723 0]set interrupt
[0433.628914 0]interrupt,2
[0433.639657 0]interrupt,2
[0433.649684 0]
RxChBuf0 (256 bytes):
0CC20DC0: 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 00  | 123456789:;<=>?.
0CC20DD0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20DE0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20DF0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E10: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E20: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E80: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20E90: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20EA0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CC20EB0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
[0433.679963 0]
TxChBuf0 (256 bytes):
0CBC3040: 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 00  | 123456789:;<=>?.
0CBC3050: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3060: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3070: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3080: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3090: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30A0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30B0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30C0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30D0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30E0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC30F0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3100: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3110: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3120: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
0CBC3130: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  | ................
[0433.700110 0]=====SPI ASYNC TEST SUCCESS=====

```


### 应用程序接口

#### void Spi_Init(const Spi_ConfigType* ConfigPtr)

```shell
Description：Service for SPI initialization.

Parameters(in)
    ConfigPtr: Pointer to configuration set
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```


#### Std_ReturnType Spi_WriteIB(Spi_ChannelType Channel, const Spi_DataBufferType* DataBufferPtr)

```shell
Description：Service for SPI de-initialization.

Parameters(in)
    Channel: Channel ID.
    DataBufferPtr: Source data buffer pointer
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: Spi write IB buffer success.
    E_NOT_OK: Spi write IB buffer failed.
```


#### Std_ReturnType Spi_AsyncTransmit(Spi_SequenceType Sequence)

```shell
Description：Service to transmit data on the SPI bus.

Sync/Async:Asynchronous
Parameters(in)
    Sequence: Sequence ID.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Spi_ReadIB(Spi_ChannelType Channel, Spi_DataBufferType* DataBufferPtr)

```shell
Description：Service for reading synchronously one or more data from an IB SPI
             Handler/Driver Channel specified by parameter.

Sync/Async:Synchronous
Parameters(in)
    Channel: Channel ID.
Parameters(inout)
    None
Parameters(out)
    DataBufferPtr: Pointer to destination data buffer in RAM
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Spi_SetupEB(Spi_ChannelType Channel, const Spi_DataBufferType* SrcDataBufferPtr, Spi_DataBufferType* DesDataBufferPtr, Spi_NumberOfDataType Length)

```shell
Description：Service to setup the buffers and the length of data for the EB SPI
             Handler/Driver Channel specified.

Sync/Async:Synchronous
Parameters(in)
    Channel: Channel ID.
    SrcDataBufferPtr: Pointer to the memory location that will hold the transmitted data
    Length: Length (number of data elements) of the data to be transmitted
Parameters(inout)
    None
Parameters(out)
    DesDataBufferPtr: Pointer to the memory location that will hold the received data
Return value：Std_ReturnType
	E_OK: Spi Setup EB buffer success.
    E_NOT_OK: Spi Setup EB buffer failed.
```

#### Spi_StatusType Spi_GetStatus(const Spi_ConfigType* ConfigPtr)

```shell
Description：Service returns the SPI Handler/Driver software module status.

Sync/Async:Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：Spi_StatusType
    Spi_StatusType
```

#### Spi_JobResultType Spi_GetJobResult(Spi_JobType Job)

```shell
Description：This service returns the last transmission result of the specified Job.

Sync/Async:Synchronous
Parameters(in)
    Job: Job ID. An invalid job ID will return an undefined result.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Spi_JobResultType
    Spi_JobResultType
```

#### Spi_SeqResultType Spi_GetSequenceResult(Spi_SequenceType Sequence)

```shell
Description：This service returns the last transmission result of the specified Sequence.

Sync/Async:Synchronous
Parameters(in)
    Sequence: Sequence ID. An invalid sequence ID will return an undefined result.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Spi_JobResultType
    Spi_JobResultType
```

#### void Spi_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description：This service returns the version information of this module.

Sync/Async:Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    versioninfo: Pointer to where to store the version information of this module.
Return value：None
```

#### Std_ReturnType Spi_SyncTransmit(Spi_SequenceType Sequence)

```shell
Description：Service to transmit data on the SPI bus.

Sync/Async:Synchronous
Parameters(in)
    Sequence: Sequence ID.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: Transmission command has been accepted
    E_NOT_OK: Transmission command has not been accepted
```

#### Spi_StatusType Spi_GetHWUnitStatus(Spi_HWUnitType HWUnit)

```shell
Description：This service returns the status of the specified SPI Hardware
             microcontroller peripheral.

Sync/Async:Synchronous
Parameters(in)
    HWUnit: SPI Hardware microcontroller peripheral (unit) ID.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Spi_StatusType
	Spi_StatusType
```

#### void Spi_Cancel(Spi_SequenceType Sequence)

```shell
Description：Service cancels the specified on-going sequence transmission.

Sync/Async:Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Spi_SetAsyncMode(Spi_AsyncModeType Mode)

```shell
Description：Service to set the asynchronous mechanism mode for SPI
             busses handled asyn-chronously.

Sync/Async:Synchronous
Parameters(in)
    Mode: New mode required.
Parameters(inout)
    None
Parameters(out)
    None
Return value：    Std_ReturnType:
    E_OK: Setting command has been accepted
    E_NOT_OK: Setting command has not been accepted
```

#### void Spi_MainFunction_Handling(void)

```shell
Description：This function shall polls the SPI interrupts linked to HW Units
             allocated to the transmission of SPI sequences to enable the evolution
             of transmission state machine.

Sync/Async:Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```
