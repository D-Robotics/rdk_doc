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

### 软件操作流程

一般的软件操作流程如下:

1. 初始化
    - 修改 `ChannelCfgArrayPtr` , `JobCfgArrayPtr` , `SeqCfgArrayPtr` , `HwCfgPtr` , `PhyCfgPtr` 等配置结构体，将SPI对应instance配置为自己需要的波特率和传输模式，可参考SPI配置说明[Spi 配置说明](./06_mcu_spi.md#spi_config)
    - 调用 `Spi_Init(&Spi_Config)` 或者 `Spi_Init(NULL)` ，当传入为NULL时，使用 `Spi_PBcfg.c` 中的默认配置
    - 驱动程序遍历 ，根据配置信息初始化内部数据结构（如 `Spi_ChannelState` , `Spi_JobState` , `Spi_HwQueueArray` 等）和物理SPI硬件单元（根据 `Spi_PhyCfgType` 和 `Spi_IpCfg` 设置寄存器）

2. 设置传输模式:
    - 应用程序通过 `Spi_SetAsyncMode()` 选择使用中断模式还是轮询模式。

3. 数据准备:
    - 应用程序通过 `Spi_WriteIB()` 或 `Spi_SetupEB()` 函数将要发送的数据放入通道关联的缓冲区中。

4. 传输请求：
    - 应用程序调用 `Spi_AsyncTransmit(sequence)` 或 `Spi_SyncTransmit(sequence)` 来启动一个序列的传输。

5. 驱动处理:
    - 驱动程序根据 `sequence ID` 找到对应的 `Spi_SeqCfg`
    - 根据 `Spi_SeqCfg.JobIndexList`  获取作业列表。
    - 对于序列中的每个作业：
    - 根据 `Spi_JobCfg.HWUnit` 确定要使用的物理SPI硬件单元。
    - 根据 `Spi_JobCfg.HwCfgPtr` 获取该作业对应的外部设备通信参数。
    - 根据 `Spi_JobCfg.ChannelIndexList` 获取通道列表。
    - 遍历通道列表，从 `Spi_ChannelCfg.BufferDescriptor` 指向的缓冲区中获取发送数据。
    - 配置物理SPI硬件单元。
    - 启动硬件传输。

6. 传输完成:
    - 对于异步传输，硬件完成传输后（通过中断或轮询检测），驱动程序会调用相应的通知函数
    - 对于同步传输，函数会等待传输完成。

7. 状态查询:
    - 应用程序可以通过 `Spi_GetJobResult()` , `Spi_GetSequenceResult()` 等函数查询传输状态。


:::tip
- SPI只需要在INIT之后配置一次`set_AsyncMode()`，不需要反复调用，反复调用会导致传输异常。
- 当传输发生异常的时候，SPI的状态机不会自动复位，需要手动调用 `Spi_Cancel()` 函数复位传输状态机。
- Sample里用了DMA，但没进行初始化，这是因为在 `Target/Target-hobot-lite-freertos-mcu1/target/HorizonTask.c` 文件中已经完成了初始化。
:::

### 单片选使用示例

spi_test 命令用于测试SPI（Serial Peripheral Interface，串行外设接口）功能。该命令支持初始化和参数设置、显示当前参数以及执行SPI数据传输测试。

**命令语法**
```bash
spi_test <operation> [bus_id] [sync_mode] [trans_mode]
```

**参数说明**
- operation: 指定要执行的操作。
    - 0: 初始化和参数设置。
    - 1: 显示当前设置的参数。
    - 2: 执行SPI测试（异步或同步）。
- bus_id (仅当 operation 为 0 时需要): 指定要使用的SPI总线。
    - 取值范围: 2 到 6，分别对应 SPI2 到 SPI6。
- sync_mode (仅当 operation 为 0 时需要): 指定SPI通信的同步模式。
    - 0: 异步模式 (async)
    - 1: 同步模式 (sync)
- trans_mode (仅当 operation 为 0 时需要): 指定SPI数据传输的底层机制。
    - 0: 轮询模式 (polling)
    - 1: 中断模式 (interrupt)

:::tip
应用层配置与底层配置应保持一致，否则会出现错误。
:::

以SPI3 为例，将SPI3的MISO和MOSI短接，运行以下命令，设置传输参数
```shell
D-Robotics:/$ spi_test 0 3 0 0
[055.578172 0]Init&&Parameter setting
[055.578428 0]Show Spi parameter
[055.578792 0]spi_bus = 3
[055.579085 0]sync_mode = async
[055.579443 0]trans_mode = polling
```
运行以下命令，测试
```shell
D-Robotics:/$ spi_test 2
[059.643996 0]Sequence: 1, transfer_length = 128 spi_framesize = 16
[059.673978 0]
RxChBuf0 (256 bytes):
0CBD2A80: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F  | ................
0CBD2A90: 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F  | ................
0CBD2AA0: 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F  |  !"#$%&'()*+,-./
0CBD2AB0: 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F  | 0123456789:;<=>?
0CBD2AC0: 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F  | @ABCDEFGHIJKLMNO
0CBD2AD0: 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F  | PQRSTUVWXYZ[\]^_
0CBD2AE0: 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F  | `abcdefghijklmno
0CBD2AF0: 70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F  | pqrstuvwxyz{|}~.
0CBD2B00: 80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F  | ................
0CBD2B10: 90 91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9E 9F  | ................
0CBD2B20: A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 AA AB AC AD AE AF  | ................
0CBD2B30: B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 BA BB BC BD BE BF  | ................
0CBD2B40: C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CE CF  | ................
0CBD2B50: D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA DB DC DD DE DF  | ................
0CBD2B60: E0 E1 E2 E3 E4 E5 E6 E7 E8 E9 EA EB EC ED EE EF  | ................
0CBD2B70: F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 FA FB FC FD FE FF  | ................
[059.687954 0]
TxChBuf0 (256 bytes):
0CBD2B80: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F  | ................
0CBD2B90: 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F  | ................
0CBD2BA0: 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F  |  !"#$%&'()*+,-./
0CBD2BB0: 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F  | 0123456789:;<=>?
0CBD2BC0: 40 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F  | @ABCDEFGHIJKLMNO
0CBD2BD0: 50 51 52 53 54 55 56 57 58 59 5A 5B 5C 5D 5E 5F  | PQRSTUVWXYZ[\]^_
0CBD2BE0: 60 61 62 63 64 65 66 67 68 69 6A 6B 6C 6D 6E 6F  | `abcdefghijklmno
0CBD2BF0: 70 71 72 73 74 75 76 77 78 79 7A 7B 7C 7D 7E 7F  | pqrstuvwxyz{|}~.
0CBD2C00: 80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F  | ................
0CBD2C10: 90 91 92 93 94 95 96 97 98 99 9A 9B 9C 9D 9E 9F  | ................
0CBD2C20: A0 A1 A2 A3 A4 A5 A6 A7 A8 A9 AA AB AC AD AE AF  | ................
0CBD2C30: B0 B1 B2 B3 B4 B5 B6 B7 B8 B9 BA BB BC BD BE BF  | ................
0CBD2C40: C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CE CF  | ................
0CBD2C50: D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 DA DB DC DD DE DF  | ................
0CBD2C60: E0 E1 E2 E3 E4 E5 E6 E7 E8 E9 EA EB EC ED EE EF  | ................
0CBD2C70: F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 FA FB FC FD FE FF  | ................
[059.702104 0]=====SPI ASYNC TEST SUCCESS=====
```
### 双片选使用示例

SpiTest_Mul_cs 命令用于测试SPI（Serial Peripheral Interface，串行外设接口）功能。

使用方式和参数解析如下：
```shell
########################## support test case: ##########################
usag: SpiTest_Mul_cs Case_num Sequences Cs DataWidth DataLen Loop_times
[1]: SpiTest_Mul_cs 1 1 0 8 1 1 -- get versioninfo
[2]: SpiTest_Mul_cs 2 5 0 8 10 1 -- InterruptMode Sync Transfer
[3]: SpiTest_Mul_cs 3 5 0 8 10 1 -- PollingMode Sync Transfer
[4]: SpiTest_Mul_cs 4 5 0 8 10 1 -- InterruptMode Async Transfer
[5]: SpiTest_Mul_cs 5 5 0 8 10 1 -- PollingMode Async Transfer
other: SpiTest 110  -- help
```
- 参数解析：
    - Case_num (argv[1]): 指定要执行的操作，支持5种模式，如使用中断异步传输， Case_num设置为4
    - Sequences (argv[2]): 指定使用哪路spi， 如使用spi4，Sequences参数设置为4
    - Cs (argv[3]): 表示使用哪路cs， 如使用cs0， Cs参数设置为0
    - DataWidth (argv[4]): 表示传输数据位宽， 如位宽为8bit，DataWidth设置为8
    - Datalen (argv[5]): 表示传输的数据量，如传输10组数据， Datalen设置为10
    - Loop_times (argv[6]): 表示测试几次， 如测试5次， Loop_times设置为5
- 举例说明：
    - spi4，async interrupt传输，cs1，8bit数据宽度， 10组数据， 测试1次： SpiTest_Mul_cs 4 4 1 8 10 1

将SPI4的MISO和MOSI短接，运行以下命令
```shell
Robotics:/$ SpiTest_Mul_cs 4 4 1 8 10 1
[get_spi_status 98] [INFO]: SPI status: SPI_IDLE
[SpiTest_Mul_cs 450] [INFO]: ####################### test_case_num: 4 #######################
############################# Loop Times: 1 #############################
[Spi_Trans_Test 231] [INFO]: data_tx: 0xcbccc40, data_rx: 0xcbcca40
[Spi_Trans_Test 238] [INFO]: len = 10, check_data_len = 10
[get_spi_sequence_result 122] [INFO]: SPI result: SPI_SEQ_PENDING
TX | 00 01 02 03 04 05 06 07 08 09 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
RX | 00 01 02 03 04 05 06 07 08 09 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
[check_data 81] [INFO]: check data success.
[Spi_Interrupt_Async_Transfer_Test 353] [INFO]: Transfer success.
[SpiTest_Mul_cs 474] [INFO]: Test case pass.
[SpiTest_Mul_cs 479] [INFO]: #####################################################################

```

### 配置文件说明{#spi_config}

配置文件`Spi_PBcfg.c` 存放着SPI的驱动配置，其中`Spi_ConfigType`为顶层容器，它将所有配置信息（通道、作业、序列、硬件单元等）组织在一起，形成一个完整的、用于初始化 SPI 驱动程序的配置集。

```c
/**
 * @struct Spi_ConfigType
 * @NO{S01E17C01}
 * @brief This is the top level structure containing all the
 *         needed parameters for the SPI Handler Driver.
 */
typedef struct
{
    uint32 SpiCoreUse;                                  /**< CoreID used*/
    const Spi_ChannelCfgArray *ChannelCfgArrayPtr;      /**< Pointer to Array of channels defined in the configuration.*/
    const Spi_JobCfgArray *JobCfgArrayPtr;              /**< Pointer to Array of jobs defined in the configuration.*/
    const Spi_SeqCfgArray *SeqCfgArrayPtr;              /**< Pointer to Array of sequences defined in the configuration.*/
    const Spi_HwCfgArray *HwCfgPtr;                     /**< External device unit attributes.*/
    const Spi_PhyCfgArray *PhyCfgPtr;                   /**< Pointer to Array of device instances.*/
    Spi_ChannelType SpiMaxChannel;                      /**< Number of channels defined in the configuration.*/
    Spi_JobType SpiMaxJob;                              /**< Number of jobs defined in the configuration.*/
    Spi_SequenceType SpiMaxSequence;                    /**< Number of sequences defined in the configuration.*/
    Spi_HwNumType SpiMaxHwNum;                          /**< Number of Spi HW defined in the configuration.*/
#if (SPI_DISABLE_DEM_REPORT_ERROR_STATUS == STD_OFF)
    const Mcal_DemErrorType SpiErrorHwCfg;        /**< SPI Driver DEM Error: SPI_E_HARDWARE_ERROR.*/
#endif
} Spi_ConfigType;
```
**成员解释**

1. **SpiCoreUse**：指定哪个CPU核心可以使用这个SPI配置，单核系统保持原状即可
2. **ChannelCfgArrayPtr**：指向一个包含所有Spi_ChannelCfg配置的数组。这个数组定义了系统中所有可用的SPI通道（Channel）的属性，如缓冲区类型（IB/EB）、帧大小、默认值、缓冲区指针等
3. **JobCfgArrayPtr**：指向一个包含所有Spi_JobCfg配置的数组。这个数组定义了系统中所有可用的SPI作业（Job）的属性，如包含的通道列表、通知函数、优先级、关联的物理单元（HWUnit）等
4. **SeqCfgArrayPtr**：指向一个包含所有Spi_SeqCfg配置的数组。这个数组定义了系统中所有可用的SPI序列（Sequence）的属性，如包含的作业列表、通知函数、是否可中断等
5. **HwCfgPtr**：指向一个包含所有Spi_HwCfg配置的数组。这个数组定义了系统中所有外部设备（External Device）的硬件属性，主要是与通信相关的参数（如波特率、时钟极性、片选等）
6. **PhyCfgPtr**：指向一个包含所有Spi_PhyCfgType配置的数组。这个数组定义了系统中所有物理SPI硬件单元（Physical SPI Unit, PhyUnit）的基本工作模式和特性（如主/从模式、DMA 使用、传输模式等）。
7. **SpiMaxChannel**: 配置中定义的通道总数。用于驱动程序内部数组的大小分配和边界检查
8. **SpiMaxJob**：配置中定义的作业总数。用于驱动程序内部数组的大小分配和边界检查
9. **SpiMaxSequence**：配置中定义的序列总数。用于驱动程序内部数组的大小分配和边界检查
10. **SpiMaxHwNum**：配置中涉及的物理SPI硬件单元（PhyUnit）的数量。用于驱动程序内部数组的大小分配和边界检查
11. **SpiErrorHwCfg**：用于配置SPI硬件错误的报告参数

**成员之间的关联关系**

应用程序通常通过调用 `Spi_AsyncTransmit()` 或 `Spi_SyncTransmit()` 并传入一个序列ID来启动传输，因此序列是用户直接交互的入口点，它与通道、作业、硬件配置和物理单元配置的简略实体关系如下：

 ```mermaid
erDiagram
    SPI_SEQUENCE ||--o{ SPI_JOB : contains
    SPI_JOB ||--|| SPI_HWUNIT : uses
    SPI_JOB ||--|| SPI_EXTERNAL_DEVICE : configures
    SPI_JOB ||--o{ SPI_CHANNEL : includes

    SPI_SEQUENCE {
        uint8 NumJobs
        array JobIndexList
    }
    SPI_JOB {
        uint8 NumChannels
        array ChannelIndexList
        enum HWUnit
        ptr HwCfgPtr
    }
    SPI_CHANNEL {
        uint16 FrameSize
        uint16 Length
        ptr BufferDescriptor
    }
    SPI_HWUNIT {
        uint8 Instance "0~5"
        bool SlaveMode
        string TransferMode
    }
    SPI_EXTERNAL_DEVICE {
        uint8 Instance
        ptr IpCfg
    }
```


`SPI_HWUNIT` 和 `SPI_EXTERNAL_DEVICE` 都需要关联一个硬件实例（Instance），该实例决定了使用哪一个SPI接口进行通信。由于RDKS100平台共提供8个SPI 控制器，其中MAIN域包含2个（SPI0、SPI1），MCU域包含6个（SPI2 至 SPI7），因此MCU域中的 SPI 实际从 SPI2 开始编号。下表<font color='green'>**绿色**</font>字体部分展示了 SPI 序列配置（Spi SeqCfg）与对应硬件资源（Spi BusId、HWUnit、Instance）之间的映射关系。



| **SPI SeqCfg** | <font color='green'>**Spi BusId**</font>  | <font color='green'>**HWUnit**</font>  | <font color='green'>**Instance**</font>  | **Spi Baudrate** | **Spi Cs** | **Frame size**|
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| SpiSequence_0 | <font color='green'>SPI2</font> | <font color='green'>CSIB0</font> | <font color='green'>0</font> | 2000000 | CS0 | 16 bit |
| SpiSequence_1 | <font color='green'>SPI3</font> | <font color='green'>CSIB1</font> | <font color='green'>1</font> | 2000000 | CS0 | 16 bit |
| SpiSequence_2 | <font color='green'>SPI4</font> | <font color='green'>CSIB2</font> | <font color='green'>2</font> | 1000000 | CS0 | 8 bit |
| SpiSequence_3 | <font color='green'>SPI5</font> | <font color='green'>CSIB3</font> | <font color='green'>3</font> | 1000000 | CS0 | 8 bit |
| SpiSequence_4 | <font color='green'>SPI5</font> | <font color='green'>CSIB3</font> | <font color='green'>3</font> | 1000000 | CS0 | 16 bit |
| SpiSequence_5 | <font color='green'>SPI6</font> | <font color='green'>CSIB4</font> | <font color='green'>4</font> | 1000000 | CS0 | 8 bit |
| SpiSequence_6 | <font color='green'>SPI7</font> | <font color='green'>CSIB5</font> | <font color='green'>5</font> | 1000000 | CS0 | 8 bit |
| SpiSequence_7 | <font color='green'>SPI2</font> | <font color='green'>CSIB0</font> | <font color='green'>0</font> | 2000000 | CS1 | 16 bit |
| SpiSequence_8 | <font color='green'>SPI3</font> | <font color='green'>CSIB1</font> | <font color='green'>1</font> | 2000000 | CS1 | 16 bit |
| SpiSequence_9 | <font color='green'>SPI4</font> | <font color='green'>CSIB2</font> | <font color='green'>2</font> | 1000000 | CS1 | 8 bit |
| SpiSequence_10 | <font color='green'>SPI5</font> | <font color='green'>CSIB3</font> | <font color='green'>3</font> | 1000000 | CS1 | 8 bit |
| SpiSequence_11 | <font color='green'>SPI5</font> | <font color='green'>CSIB3</font> | <font color='green'>3</font> | 1000000 | CS1 | 16 bit |
| SpiSequence_12 | <font color='green'>SPI6</font> | <font color='green'>CSIB4</font> | <font color='green'>4</font> | 1000000 | CS1 | 8 bit |
| SpiSequence_13 | <font color='green'>SPI7</font> | <font color='green'>CSIB5</font> | <font color='green'>5</font> | 1000000 | CS1 | 8 bit |

:::tip
MCU1 的 SPI 出厂默认参数基于 Spi_PBcfg.c 配置文件，具有一定的时效性，但通常不会发生变更。
:::



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
