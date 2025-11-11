---
sidebar_position: 6
---

# SPI Usage Guide

## Hardware Support

- If SPI uses DMA transmission, the following constraints must be met:
    - The transfer length must be 8-byte aligned; otherwise, data boundary overflow issues may occur;
    - The buffer addresses for transmit and receive data must be 64-byte aligned;
    - The data amount per single DMA channel transfer must not exceed 4096 bytes;
- SPI Slave mode is supported;
- When the MCU-side SPI operates as a Slave with a speed greater than 9 MHz, only DMA mode is allowed;
- When the MCU-side SPI operates in 8-bit mode, the following limitations apply:
    - Maximum speed for 1 parallel SPI Tx/Rx: 50 MHz;
    - Maximum speed for 2 parallel SPI Tx/Rx: 33.3 MHz;
    - Maximum speed for 3 parallel SPI Tx/Rx: 25 MHz;
    - Maximum speed for 4 parallel SPI Tx/Rx: 20 MHz;
    - Maximum speed for 5 parallel SPI Tx/Rx: 15.6 MHz;
    - Maximum speed for 6 parallel SPI Tx/Rx: 12.5 MHz;
- The SPI peripheral supports only MSB (Most Significant Bit first);
- SPI `SpiCsPolarity` and `spitimeclk2c` are not configurable; `SpiCsPolarity` defaults to active-low;
- When `SpiCsToggleEnable` is enabled, the CS pin will be pulled high after one SPI frame transmission completes. Only TRAILING mode is supported; LEADING mode is not supported;
- SPI DMA mode consumes less CPU load compared to non-DMA mode;
- When SPI transmits data in non-DMA mode and the system load is high, delayed SPI interrupt response may cause the SPI FIFO to not be written in time, resulting in a brief high pulse on the CS pin. In such cases, DMA mode is recommended;
- If multiple Sequences use the same SPI IP for asynchronous transmission without strict ordering requirements, Sequence transmission queuing may occur. In this scenario, interrupt-disabling critical section protection must be implemented in the `SchM_Spi.c` file.

## Code Paths

- McalCdd/Spi/inc/Spi.h – Header file for the SPI driver  
- McalCdd/Spi/inc/Spi_Lld.h – Header file for the SPI low-level driver  
- McalCdd/Spi/src/Spi.c – Source file for the SPI driver  
- McalCdd/Spi/src/Spi_Lld.c – Source file for the SPI low-level driver  
- McalCdd/Common/Register/inc/Spi_Register.h – SPI register definition file  
- Platform/Schm/SchM_Spi.h – SPI module scheduling management header file  
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/inc/Spi_Cfg.h – SPI configuration header file  
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/inc/Spi_PBcfg.h – SPI PB configuration header file  
- Config/McalCdd/gen_s100_sip_B_mcu1/Spi/src/Spi_PBcfg.c – SPI PB configuration source file  
- samples/Spi/SPI_sample/Spi_sample.c – SPI sample code

## Application Sample

- The sample uses DMA but does not explicitly initialize it because initialization is already performed in the file `Target/Target-hobot-lite-freertos-mcu1/target/HorizonTask.c`.

The S100 chip has six SPI hardware instances: SPI2 through SPI7. The mapping between SPI channels, external devices, and physical unit abstraction is as follows:

| **SPI Channel** | **External device**     | **Spi Physical Unit**      |
|-----------------|-------------------------|----------------------------|
| SPI2            | SPI_SpiExternalDevice_0 | Spi_PhyConfig_SpiPhyUnit_0 |
| SPI3            | SPI_SpiExternalDevice_1 | Spi_PhyConfig_SpiPhyUnit_1 |
| SPI4            | SPI_SpiExternalDevice_2 | Spi_PhyConfig_SpiPhyUnit_2 |
| SPI5            | SPI_SpiExternalDevice_3 | Spi_PhyConfig_SpiPhyUnit_3 |
| SPI6            | SPI_SpiExternalDevice_4 | Spi_PhyConfig_SpiPhyUnit_4 |
| SPI7            | SPI_SpiExternalDevice_5 | Spi_PhyConfig_SpiPhyUnit_5 |

### Configuration Example

Taking SPI4 as an example, the configuration is as follows:

```c
//Config/McalCdd/gen_s100_sip_B_mcu1/Spi/src/Spi_PBcfg.c
static const Spi_PhyCfgType Spi_PhyConfig_SpiPhyUnit_2 =
{
    (uint8)2U, /* Instance */
    SPI_SPURIOUS_CORE_ID, /* SpiCoreUse */
    /* Spi Slave mode*/
    (boolean)FALSE,  // FALSE for master mode, TRUE for slave mode
    /* Spi Test Mode Operation Enable*/
    (boolean)FALSE, // Loopback mode (no external wiring)
    /* Spi Sample Point*/
    (uint32)0U,
#if (SPI_DMA_ENABLE == STD_ON)
    (boolean)FALSE, // DMA configuration
#endif
    SPI_IP_POLLING, /* Transfer mode: interrupt or polling mode selectable */
    (uint8)2U, /* State structure element from the array */
    //SPI_PHYUNIT_SYNC /* IsSync */
    SPI_PHYUNIT_ASYNC // Synchronous or asynchronous mode
};

static const Spi_ChannelCfg SpiChannel_2 =
{
    /* SpiChannel_2*/
    EB, /* BufferType IB or EB */ // Use external or internal memory
    8U, /* Frame size */
    (uint32)1U, /* In the case SpiDefaultData is disabled. Set this value to 1U */
    Spi_TxDefaultBufferSpiChannel_2,
    Spi_RxDefaultBufferSpiChannel_2,
    256U, /* Length (configured in SpiEbMaxLength) */
    &Spi_BufferSpiChannel_2,
    SPI_SPURIOUS_CORE_ID,
    &Spi_ChannelState[2U] /* Spi_ChannelState pointer */
};

```


### Single Chip Select Usage Example

The `spi_test` command is used to test SPI (Serial Peripheral Interface) functionality. It supports initialization and parameter setup, parameter retrieval, and SPI testing.

- The `spitest` command supports three operation modes, specified by the first argument:
    - 0: Initialization and parameter setup
    - 1: Retrieve parameters
    - 2: Execute SPI test

- Parameter explanation:
    - Operation mode (`argv[1]`): Specifies the operation to perform; valid values are 0, 1, or 2.
    - Device ID (`argv[2]`): Specifies the SPI device ID, which determines which SPI instance is used. Refer to the SPI mapping table in the first section of this document.
    - Channel number (`argv[3]`): Specifies the SPI channel number, corresponding to the `Spi_ChannelCfg` configuration above. This typically needs to be changed only when switching between internal and external memory buffers.
    - Synchronization mode (`argv[4]`): Used only when operation mode is 0, specifies the SPI communication synchronization mode.
        - 0: Asynchronous mode
        - 1: Synchronous mode
    - Transfer mode (`argv[5]`): Used only when operation mode is 0, specifies the SPI data transfer mode.
        - 0: Polling mode
        - 1: Interrupt mode

:::tip
Application-layer configuration must be consistent with low-level configuration; otherwise, errors will occur.
:::

Short the MISO and MOSI pins of SPI4 and run the following command:
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

### Dual Chip Select Usage Example

The `SpiTest_Mul_cs` command is used to test SPI (Serial Peripheral Interface) functionality.

Usage and parameter explanation are as follows:
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

- Parameter explanation:
    - `Case_num` (`argv[1]`): Specifies the test case to run; 5 modes are supported. For example, to use interrupt-driven asynchronous transfer, set `Case_num` to 4.
    - `Sequences` (`argv[2]`): Specifies which SPI instance to use. For SPI4, set `Sequences` to 4.
    - `Cs` (`argv[3]`): Specifies which chip select (CS) line to use. For CS0, set `Cs` to 0.
    - `DataWidth` (`argv[4]`): Specifies the data width in bits. For 8-bit width, set `DataWidth` to 8.
    - `DataLen` (`argv[5]`): Specifies the number of data frames to transfer. For 10 frames, set `DataLen` to 10.
    - `Loop_times` (`argv[6]`): Specifies how many times to repeat the test. For 5 iterations, set `Loop_times` to 5.

- Example:
    - SPI4, asynchronous interrupt-driven transfer, CS1, 8-bit data width, 10 data frames, 1 test iteration:  
      `SpiTest_Mul_cs 4 4 1 8 10 1`

Short the MISO and MOSI pins of SPI4 and run the following command:
```shell
Robotics:/$ SpiTest_Mul_cs 4 4 1 8 10 1
[get_spi_status 98] [INFO]: SPI status: SPI_IDLE
[SpiTest_Mul_cs 450] [INFO]: ####################### test_case_num: 4 #######################
############################# Loop Times: 1 #############################
```[Spi_Trans_Test 231] [INFO]: data_tx: 0xcbccc40, data_rx: 0xcbcca40  
[Spi_Trans_Test 238] [INFO]: len = 10, check_data_len = 10  
[get_spi_sequence_result 122] [INFO]: SPI result: SPI_SEQ_PENDING  
TX | 00 01 02 03 04 05 06 07 08 09 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __   
RX | 00 01 02 03 04 05 06 07 08 09 __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __   
[check_data 81] [INFO]: check data success.  
[Spi_Interrupt_Async_Transfer_Test 353] [INFO]: Transfer success.  
[SpiTest_Mul_cs 474] [INFO]: Test case pass.  
[SpiTest_Mul_cs 479] [INFO]: #####################################################################

```
### Application Programming Interface

#### void Spi_Init(const Spi_ConfigType* ConfigPtr)

```shell
Description: Service for SPI initialization.

Parameters (in)
    ConfigPtr: Pointer to configuration set
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```


#### Std_ReturnType Spi_WriteIB(Spi_ChannelType Channel, const Spi_DataBufferType* DataBufferPtr)

```shell
Description: Service for SPI de-initialization.

Parameters (in)
    Channel: Channel ID.
    DataBufferPtr: Source data buffer pointer
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: Spi write IB buffer success.
    E_NOT_OK: Spi write IB buffer failed.
```


#### Std_ReturnType Spi_AsyncTransmit(Spi_SequenceType Sequence)

```shell
Description: Service to transmit data on the SPI bus.

Sync/Async: Asynchronous
Parameters (in)
    Sequence: Sequence ID.
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Spi_ReadIB(Spi_ChannelType Channel, Spi_DataBufferType* DataBufferPtr)

```shell
Description: Service for reading synchronously one or more data from an IB SPI
             Handler/Driver Channel specified by parameter.

Sync/Async: Synchronous
Parameters (in)
    Channel: Channel ID.
Parameters (inout)
    None
Parameters (out)
    DataBufferPtr: Pointer to destination data buffer in RAM
Return value: Std_ReturnType
    E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Spi_SetupEB(Spi_ChannelType Channel, const Spi_DataBufferType* SrcDataBufferPtr, Spi_DataBufferType* DesDataBufferPtr, Spi_NumberOfDataType Length)

```shell
Description: Service to setup the buffers and the length of data for the EB SPI
             Handler/Driver Channel specified.

Sync/Async: Synchronous
Parameters (in)
    Channel: Channel ID.
    SrcDataBufferPtr: Pointer to the memory location that will hold the transmitted data
    Length: Length (number of data elements) of the data to be transmitted
Parameters (inout)
    None
Parameters (out)
    DesDataBufferPtr: Pointer to the memory location that will hold the received data
Return value: Std_ReturnType
    E_OK: Spi Setup EB buffer success.
    E_NOT_OK: Spi Setup EB buffer failed.
```

#### Spi_StatusType Spi_GetStatus(const Spi_ConfigType* ConfigPtr)

```shell
Description: Service returns the SPI Handler/Driver software module status.

Sync/Async: Synchronous
Parameters (in)
    None
Parameters (inout)
    None
Parameters (out)
    None
Return value: Spi_StatusType
    Spi_StatusType
```

#### Spi_JobResultType Spi_GetJobResult(Spi_JobType Job)

```shell
Description: This service returns the last transmission result of the specified Job.

Sync/Async: Synchronous
Parameters (in)
    Job: Job ID. An invalid job ID will return an undefined result.
Parameters (inout)
    None
Parameters (out)
    None
Return value: Spi_JobResultType
    Spi_JobResultType
```

#### Spi_SeqResultType Spi_GetSequenceResult(Spi_SequenceType Sequence)

```shell
Description: This service returns the last transmission result of the specified Sequence.

Sync/Async: Synchronous
Parameters (in)
    Sequence: Sequence ID. An invalid sequence ID will return an undefined result.
Parameters (inout)
    None
Parameters (out)
    None
Return value: Spi_JobResultType
    Spi_JobResultType
```

#### void Spi_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description: This service returns the version information of this module.

Sync/Async: Synchronous
Parameters (in)
    None
Parameters (inout)
    None
Parameters (out)
    versioninfo: Pointer to where to store the version information of this module.
Return value: None
```

#### Std_ReturnType Spi_SyncTransmit(Spi_SequenceType Sequence)

```shell
Description: Service to transmit data on the SPI bus.

Sync/Async: Synchronous
Parameters (in)
    Sequence: Sequence ID.
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: Transmission command has been accepted
    E_NOT_OK: Transmission command has not been accepted
```

#### Spi_StatusType Spi_GetHWUnitStatus(Spi_HWUnitType HWUnit)

```shell
Description: This service returns the status of the specified SPI Hardware
             microcontroller peripheral.

Sync/Async: Synchronous
Parameters (in)
    HWUnit: SPI Hardware microcontroller peripheral (unit) ID.
Parameters (inout)
    None
Parameters (out)
    None
Return value: Spi_StatusType
    Spi_StatusType
```

#### void Spi_Cancel(Spi_SequenceType Sequence)
``````shell
Description: Service cancels the specified on-going sequence transmission.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Spi_SetAsyncMode(Spi_AsyncModeType Mode)

```shell
Description: Service to set the asynchronous mechanism mode for SPI
             busses handled asynchronously.

Sync/Async: Synchronous
Parameters(in)
    Mode: New mode required.
Parameters(inout)
    None
Parameters(out)
    None
Return value:    Std_ReturnType:
    E_OK: Setting command has been accepted
    E_NOT_OK: Setting command has not been accepted
```

#### void Spi_MainFunction_Handling(void)

```shell
Description: This function shall poll the SPI interrupts linked to HW Units
             allocated to the transmission of SPI sequences to enable the evolution
             of transmission state machine.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```