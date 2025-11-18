---
sidebar_position: 4
---

# UART Usage Guide

The S100 MCU chip has a total of 3 UART interfaces, namely UART4 to UART6. Among them, UART4 is used as the debug console (shared by MCU0 and MCU1), with DMA disabled by default. The default configuration is as follows:

| **Configuration Item** | **UART4** | **UART5** | **UART6** |
|------------------------|-----------|-----------|-----------|
| Channel Identifier     | Uart_Channel0 | Uart_Channel1 | Uart_Channel2 |
| Baud Rate              | 921600    | 921600    | 921600    |
| Parity Bit             | None      | None      | None      |
| Stop Bits              | 1 bit     | 1 bit     | 1 bit     |
| Data Bits              | 8 bits    | 8 bits    | 8 bits    |

## Hardware Support

- Maximum number of UARTs available on the MCU: 3
- UART FIFO depth: 8 bytes × 16
- Supports common baud rates such as 4800, 9600, 38400, 115200, and 921600
- Supports data bit configuration from 5 to 8 bits
- Supports parity configuration (odd/even/none)
- Supports stop bit configuration of 1, 1.5, or 2 bits
- Supports DMA mode; in DMA mode, the transmit and receive buffer addresses provided by the application layer must be 64-byte aligned


## Software Architecture

- **UART APP**: Application-layer code for UART.
- **UART Interface**: Interface-layer code providing standardized UART operation APIs.
- **UART LLD**: Low-level driver code that directly manipulates hardware registers to implement core functionalities such as asynchronous/synchronous transmission, interrupt handling, and FIFO management.
- **UART PBcfg**: Peripheral configuration (PB) files containing UART peripheral configuration parameters.
- **Hardware**: UART hardware.


![MCU Software Architecture Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_uart.png)



## Code Paths


- McalCdd/Common/Register/inc/Uart_Register.h # Register-related definitions
- McalCdd/Uart/src/Uart.c # Driver implementation
- McalCdd/Uart/src/Uart_Lld.c # Low-level driver implementation
- McalCdd/Uart/inc/Uart.h # Driver header file
- McalCdd/Uart/inc/Uart_Lld.h # Low-level driver header file
- Config/McalCdd/gen_s100_sip_B_mcu1/Uart/src/Uart_PBcfg.c # MCU PB configuration file
- Config/McalCdd/gen_s100_sip_B_mcu1/Uart/inc/Uart_PBcfg.h # PB configuration header file
- samples/Uart/Uart_sample/Uart_Test.c  # Sample test code


## Application Sample

### Usage Example

On the S100 development board, UART5 is exposed for user development and learning purposes. The corresponding pins are located on the MCU Expansion Header of the Main Board.

//TODO Add board image

In the mainline code, UART5 is by default passed through to Acore. Therefore, before testing, you need to enter the following command in the MCU console to release UART5:

```shell
D-Robotics:/$ ipcbox_set_mode uart 0
[058.203488 0]uart processing disabled
[058.203749 0]uart2ipc_release
Return: 0, 0x00000000
```

- `uarttest 1`: Loopback test (note: connect the RX pin to the TX pin)

```shell
D-Robotics:/$ uarttest 1
[073.515926 0]Async receive ret: 0
Tx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38                                                                                                                                 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
Rx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38                                                                                                                                 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
[073.523782 0]SyncSend & AsyncReceive test pass!
```


- `uarttest 2`: Receive data  
  A serial assistant (configured at 921600 baud, 8-N-1) is used to send data to the MCU.
```shell
D-Robotics:/$ uarttest 2
Rx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
[0359.598869 0]AsyncSend & ASyncReceive test pass!
```

- `uarttest 3`: Transmit data
```shell

D-Robotics:/$ uarttest 3
Tx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
```

- `uarttest 4`: Retrieve GPS data (internal RDK test case).

- `uarttest 5`: Set baud rate to 9600
```
D-Robotics:/$ uarttest 5
[042602.833314 0]Channel 1 Baud: 9600

```

- `uarttest 6`: Set baud rate to 115200

```
D-Robotics:/$ uarttest 6
[042617.473286 0]Channel 1 Baud: 115200
```

### Application Programming Interface (API)

#### void Uart_Init(void)

```shell
Description: Subsystem driver initialization function.

Parameters (in)
    None
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```


#### void Uart_Deinit(void)

```shell
Description: Subsystem driver deinitialization function.

Parameters (in)
    None
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```


#### Std_ReturnType Uart_BaudSet(uint8 Channel, Uart_BaudrateType Baudrate)

```shell
Description: Set the baud rate for a UART channel.

Parameters (in)
    Channel: UART channel
    Baudrate: Desired baud rate
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: Set successfully
    E_NOT_OK: Set failed
```

#### Std_ReturnType Uart_BaudGet(uint8 Channel, uint32* Baudrate)

```shell
Description: Get the current baud rate of a UART channel.

Parameters (in)
    Channel: UART channel
Parameters (inout)
    None
Parameters (out)
    Baudrate: Current baud rate
Return value: Std_ReturnType
    E_OK: Get successfully
    E_NOT_OK: Get failed
```

#### Std_ReturnType Uart_SetDatabits(uint8 Channel, uint8 Databits)

```shell
Description: Set the number of data bits for a UART channel.

Parameters (in)
    Channel: UART channel
    Databits: Desired number of data bits
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: Set successfully
    E_NOT_OK: Set failed
```

#### Std_ReturnType Uart_SetStopbit(uint8 Channel, uint8 Stopbit)

```shell
Description: Set the number of stop bits for a UART channel.

Parameters (in)
    Channel: Uart Channel
    Stopbit: Desired Stopbit
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Uart_SetParity(uint8 Channel, Uart_ParityType CurParity)

```shell
Description: Set Parity type for an Uart channel.

Parameters(in)
    Channel: Uart Channel
    CurParity: Desired Parity
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Uart_StatusType Uart_StatusGet(uint8 Channel, uint32* BytesTransfered, Uart_DataDirectionType TransferType)

```shell
Description: Gets the status of an Uart channel.

Sync/Async: Synchronous
Parameters(in)
    Channel: Uart Channel
    BytesTransfered: Byte has transfered
    TransferType: Send or Receive
Parameters(inout)
    None
Parameters(out)
    None
Return value: Uart_StatusType
    Uart current state
```

#### Std_ReturnType Uart_SyncDataTrans(uint8 Channel, const uint8* Buffer, uint32 BufferSize, uint32 Timeout)

```shell
Description: Sends an Uart message blocking.

Sync/Async: Synchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    BufferSize: number bytes buffer
    Timeout: Timeout value
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_SyncDataReceive(uint8 Channel, const uint8* Buffer, uint32 BufferSize, uint32 Timeout)

```shell
Description: Receive an Uart message blocking.

Sync/Async: Synchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    BufferSize: number bytes buffer
    Timeout: Timeout value
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_AsyncDataTrans(uint8 Channel, const uint8* Buffer, uint32 BufferSize)

```shell
Description: Sends an Uart message async.

Sync/Async: Asynchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    BufferSize: number bytes buffer
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_AsyncDataReceive(uint8 Channel, const uint8* Buffer, uint32 BufferSize)

```shell
Description: Receive an Uart message async.

Sync/Async: Asynchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    BufferSize: number bytes buffer
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### void Uart_GetVersionInfo(Std_VersionInfoType* VersionInfo)

```shell
Description: This function Gets the version information of this module

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    VersionInfo: version information of this module
Return value: None
```