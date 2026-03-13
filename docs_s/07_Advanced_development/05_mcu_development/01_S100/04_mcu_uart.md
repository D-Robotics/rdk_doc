---
sidebar_position: 4
---

# UART使用指南

S100 MCU芯片共有3路uart，即uart4~uart6。其中uart4作为调试控制台使用（MCU0、MCU1共用调试串口），默认不开启DMA，默认配置如下：

| **配置项**         | **uart4** | **uart5** | **uart6** |
|--------------------|-----------|-----------|-----------|
| 通道标识符         | Uart_Channel0 | Uart_Channel1 | Uart_Channel2 |
| 波特率             | 921600    | 921600    | 921600    |
| 校验位             | 无        | 无        | 无        |
| 停止位             | 1位       | 1位       | 1位       |
| 数据位             | 8位       | 8位       | 8位       |

## 硬件支持

- MCU最大可用UART数量: 3个
- UART FIFO 深度：8byte x 16
- 支持4800、9600、38400 115200、921600等常用波特率
- 支持5-8位数据位配置
- 支持奇偶校验配置
- 支持1、1.5、2位停止位配置
- 支持DMA模式，DMA模式下应用层提供的发送和接收buffer地址必须是64字节对齐的


## 软件架构

- UART APP: Uart的应用层代码。
- UART Interface: Uart的接口层代码，提供标准化的UART操作接口。
- UART LLD: Uart的底层驱动代码，直接操作硬件寄存器，实现异步/同步传输、中断处理、FIFO管理能核心功能。
- UART PBcfg: Uart的PB配置文件，用于外设的配置参数。
- Hardware: UART硬件。


![MCU 软件架构图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_uart.png)



## 代码路径


- McalCdd/Common/Register/inc/Uart_Register.h # 寄存器相关内容
- McalCdd/Uart/src/Uart.c # 驱动代码
- McalCdd/Uart/src/Uart_Lld.c # 底层驱动代码
- McalCdd/Uart/inc/Uart.h # 驱动头文件
- McalCdd/Uart/inc/Uart_Lld.h # 底层驱动头文件
- Config/McalCdd/gen_s100_sip_B_mcu1/Uart/src/Uart_PBcfg.c # mcu PB配置文件
- Config/McalCdd/gen_s100_sip_B_mcu1/Uart/inc/Uart_PBcfg.h # PB配置头文件
- samples/Uart/Uart_sample/Uart_Test.c  # 测试代码


## 应用sample

### 使用示例

S100 开发板将UART5引出供用户开发学习使用，PIN脚位于Main Board板上的MCU Expansion Header。

//TODO 增加板子图片

由于主线的代码，uart5默认透传给acore，所以在测试前需要在mcu的控制终端输入以下cmd命令,释放占用的uart5：

```shell
D-Robotics:/$ ipcbox_set_mode uart 0
[058.203488 0]uart processing disabled
[058.203749 0]uart2ipc_release
Return: 0, 0x00000000
```

- uarttest  1 自环测试，注意将RX引脚接TX引脚

```shell
D-Robotics:/$ uarttest 1
[073.515926 0]Async receive ret: 0
Tx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38                                                                                                                                 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
Rx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38                                                                                                                                 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
[073.523782 0]SyncSend & AsyncReceive test pass!
```


- uarttest  2 接收数据
这里使用了串口助手(配置：921600波特率8-N-1)向mcu发送数据
```shell
D-Robotics:/$ uarttest 2
Rx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
[0359.598869 0]AsyncSend & ASyncReceive test pass!
```

- uarttest  3 发送数据
```shell

D-Robotics:/$ uarttest 3
Tx: 0 1 2 3 4 5 6 7 8 9 a b c d e f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 41 42 43 44 45
```

- uarttest 4 获取GPS数据，RDK内部测试用例。

- uarttest  5 设置波特率为9600
```
D-Robotics:/$ uarttest 5
[042602.833314 0]Channel 1 Baud: 9600

```

- uarttest  6 设置波特率为115200

```
D-Robotics:/$ uarttest 6
[042617.473286 0]Channel 1 Baud: 115200
```

### 应用程序接口

#### void Uart_Init(void)

```shell
Description：Subsystem driver initialization function.

Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```


#### void Uart_Deinit(void)

```shell
Description：Subsystem driver deinitialization function.

Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```


#### Std_ReturnType Uart_BaudSet(uint8 Channel, Uart_BaudrateType Baudrate)

```shell
Description：Set baud for an Uart channel.

Parameters(in)
    Channel: Uart Channel
    Baudrate: Desired baud
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Uart_BaudGet(uint8 Channel, uint32* Baudrate)

```shell
Description：Get baud for an Uart channel.

Parameters(in)
    Channel: Uart Channel
Parameters(inout)
    None
Parameters(out)
    Baudrate: current baud
Return value：Std_ReturnType
	E_OK: get success
    E_NOT_OK: get failed
```

#### Std_ReturnType Uart_SetDatabits(uint8 Channel, uint8 Databits)

```shell
Description：Set Databits for an Uart channel.

Parameters(in)
    Channel: Uart Channel
    Databits：Desired Databits
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Uart_SetStopbit(uint8 Channel, uint8 Stopbit)

```shell
Description：Set Stopbit for an Uart channel.

Parameters(in)
    Channel: Uart Channel
    Stopbit: Desired Stopbit
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Std_ReturnType Uart_SetParity(uint8 Channel, Uart_ParityType CurParity)

```shell
Description：Set Parity type for an Uart channel.

Parameters(in)
    Channel: Uart Channel
    CurParity:Desired Parity
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: set success
    E_NOT_OK: set failed
```

#### Uart_StatusType Uart_StatusGet(uint8 Channel, uint32* BytesTransfered, Uart_DataDirectionType TransferType)

```shell
Description：Gets the status of an Uart channel.

Sync/Async: Synchronous
Parameters(in)
    Channel: Uart Channel
    BytesTransfered: Byte has transfered
    TransferType: Send or Receive
Parameters(inout)
    None
Parameters(out)
    None
Return value：Uart_StatusType
    Uart current state
```

#### Std_ReturnType Uart_SyncDataTrans(uint8 Channel, const uint8* Buffer, uint32 BufferSize, uint32 Timeout)

```shell
Description：Sends an Uart message blocking.

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
Return value：Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_SyncDataReceive(uint8 Channel, const uint8* Buffer, uint32 BufferSize, uint32 Timeout)

```shell
Description：Receive an Uart message blocking.

Sync/Async: Synchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    ufferSize: number bytes buffer
    Timeout: Timeout value
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_AsyncDataTrans(uint8 Channel, const uint8* Buffer, uint32 BufferSize)

```shell
Description：Sends an Uart message async.

Sync/Async:Asynchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    ufferSize: number bytes buffer
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### Std_ReturnType Uart_AsyncDataReceive(uint8 Channel, const uint8* Buffer, uint32 BufferSize)

```shell
Description：Receive an Uart message async.

Sync/Async:Asynchronous
Parameters(in)
    Channel: Uart Channel
    Buffer: pointer to Data buffer
    ufferSize: number bytes buffer
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

#### void Uart_GetVersionInfo(Std_VersionInfoType* VersionInfo)

```shell
Description：This function Gets the version information of this module

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    VersionInfo: version information of this module
Return value: None
```

