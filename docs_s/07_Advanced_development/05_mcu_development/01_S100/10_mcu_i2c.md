---
sidebar_position: 10
---

# I2C使用指南

S100 MCU芯片提供了标准的I2C总线，I2C总线控制器通过串行数据线（SDA）和串行时钟（SCL）线在连接到总线的器件间传递信息。每个器件都有一个唯一的地址。I2C子系统的主要功能是实现单片机与外围设备之间的串行通信。它可以驱动mipi子卡、pmic芯片和其他常用的外围设备。

## I2C控制器

I2C控制器支持以下功能：
- 三种速率模式选择（目前驱动不支持HIGH SPEED模式）
  - **Standard Mode**：0–100 Kb/s
  - **Fast Mode & Fast Mode Plus**：
    - Fast Mode：0–400 Kb/s
    - Fast Mode Plus：0–1000 Kb/s
  - **High-Speed Mode**：0–3.4 Mb/s
- 支持主从模式配置
- 支持 7 位和 10 位寻址模式

S100 MCU芯片总共提供4个I2C控制器(I2C6-9)，默认速率为Fast Mode Plus。

## 代码路径

- Mcalcdd/Common/Register/I2c_Register.h # 寄存器相关内容
- McalCdd/I2c/src/I2c.c # 驱动代码
- McalCdd/I2c/src/I2c_Lld.c # 底层驱动代码
- McalCdd/I2c/inc/I2c.h # 驱动头文件
- McalCdd/I2c/inc/I2c_Lld.h # 底层驱动头文件
- Config/McalCdd/gen_s100_sip_B_mcu1/I2c/src/I2c_PBcfg.c # PB配置文件
- Config/McalCdd/gen_s100_sip_B_mcu1/I2c/inc/I2c_PBcfg.h # PB配置头文件

### 初始化和调度

通用I/O引脚的初始化不在I2c驱动程序的范围内，应由PORT驱动先完成IO初始化(Port_Init(NULL);)，然后使用I2c驱动。 I2c驱动初始化函数为I2c_Init(NULL)。

## I2C使用

S100 为MCU侧实现了一套类似i2c-tools开源工具的命令，来支持用户调试使用.

代码路径
```sh
samples/I2c/src/I2c_Cmd.c
```

目前支持i2cdetect, i2cget, i2cset。
- i2cdetect — 用来列举I2C bus及该bus上的所有设备
- i2cget — 读取I2C设备某个register的值
- i2cset — 写入I2C设备某个register的值

测试示例如下：
```sh
D-Robotics:/$ i2cdetect 7
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:    -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- 12 -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

## 应用程序接口

### void I2c_Init ( const I2c_ConfigType * Config )

```shell
Description：Subsystem driver initialization function.

Sync/Async：Synchronous
Parameters(in)
    Config： Pointer to configuration structure
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

### void I2c_DeInit ( void )

```shell
Description：De-initialize of i2c system to reset values.

Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

### Std_ReturnType I2c_SyncDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr )

```shell
Description：Sends or receives an I2C message blocking.

Sync/Async：Synchronous
Parameters(in)
    Channel: I2C Channel.
    RequestPtr: I2C transmit request type
Parameters(inout)
    None
Parameters(out)
    None
Return value:
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

### Std_ReturnType I2c_SyncMultiDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr uint8 I2cRequestCnt)

```shell
Description：Sends or receives an I2C message blocking.

Sync/Async：Synchronous
Parameters(in)
    Channel: I2C Channel.
    RequestPtr: I2C transmit request type
    I2cRequestCnt: I2C transmit num
Parameters(inout)
    None
Parameters(out)
    None
Return value:
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

### Std_ReturnType I2c_AsyncDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr )

```shell
Description：This function performs Sends or receives an I2C message non-blocking

Sync/Async：Asynchronous
Parameters(in)
    Channel: I2C Channel.
    RequestPtr: I2C transmit request type
Parameters(inout)
    None
Parameters(out)
    None
Return value:
    E_OK: command has been accepted
    E_NOT_OK: command has not been accepted
```

### I2c_StatusType I2c_StatusGet ( uint8 Channel )

```shell
Description：Gets the status of an I2C channel.

Sync/Async：Synchronous
Parameters(in)
    Channel: I2C Channel.
Parameters(inout)
    None
Parameters(out)
    None
Return value: status of an I2C channel
```

### void I2c_GetVersionInfo ( Std_VersionInfoType* VersionInfo )

```shell
Description：This function Gets the version information of this module

Sync/Async：Synchronous
Parameters(in)
    VersionInfo: version information of this module
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```
