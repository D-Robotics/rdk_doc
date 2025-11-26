---
sidebar_position: 10
---

# I2C Usage Guide

The S100 MCU chip provides a standard I2C bus. The I2C bus controller transfers information between devices connected to the bus via a Serial Data Line (SDA) and a Serial Clock Line (SCL). Each device has a unique address. The primary function of the I2C subsystem is to enable serial communication between the microcontroller and peripheral devices. It can drive MIPI daughter cards, PMIC chips, and other commonly used peripherals.

## I2C Controller

The I2C controller supports the following features:
- Three speed mode selections (currently, the driver does not support HIGH SPEED mode):
  - **Standard Mode**: 0–100 Kb/s
  - **Fast Mode & Fast Mode Plus**:
    - Fast Mode: 0–400 Kb/s
    - Fast Mode Plus: 0–1000 Kb/s
  - **High-Speed Mode**: 0–3.4 Mb/s
- Supports master and slave mode configuration
- Supports both 7-bit and 10-bit addressing modes

The S100 MCU chip provides a total of four I2C controllers (I2C6–I2C9), with a default speed setting of Fast Mode Plus.

## Code Paths

- McalCdd/Common/Register/inc/I2c_Register.h # Register-related content  
- McalCdd/I2c/src/I2c.c # Driver source code  
- McalCdd/I2c/src/I2c_Lld.c # Low-level driver source code  
- McalCdd/I2c/inc/I2c.h # Driver header file  
- McalCdd/I2c/inc/I2c_Lld.h # Low-level driver header file  
- Config/McalCdd/gen_s100_sip_B_mcu1/I2c/src/I2c_PBcfg.c # PB configuration file  
- Config/McalCdd/gen_s100_sip_B_mcu1/I2c/inc/I2c_PBcfg.h # PB configuration header file  

### Initialization and Scheduling

Initialization of general-purpose I/O pins is outside the scope of the I2C driver and should be handled first by the PORT driver (via `Port_Init(NULL);`) before using the I2C driver. The I2C driver initialization function is `I2c_Init(NULL)`.

## I2C Usage

S100 implements a set of commands on the MCU side similar to the open-source i2c-tools utilities to support user debugging.

Code path:
```sh
samples/I2c/src/I2c_Cmd.c
```

Currently supported commands include `i2cdetect`, `i2cget`, and `i2cset`:
- `i2cdetect` — Lists I2C buses and all devices present on a specific bus  
- `i2cget` — Reads the value of a specific register from an I2C device  
- `i2cset` — Writes a value to a specific register of an I2C device  

Test example:
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

## Application Programming Interface (API)

### void I2c_Init ( const I2c_ConfigType * Config )

```shell
Description: Subsystem driver initialization function.

Sync/Async: Synchronous
Parameters (in):
    Config: Pointer to configuration structure
Parameters (inout):
    None
Parameters (out):
    None
Return value: None
```

### void I2c_DeInit ( void )

```shell
Description: De-initializes the I2C system to reset values.

Sync/Async: Synchronous
Parameters (in):
    None
Parameters (inout):
    None
Parameters (out):
    None
Return value: None
```

### Std_ReturnType I2c_SyncDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr )

```shell
Description: Sends or receives an I2C message in blocking mode.

Sync/Async: Synchronous
Parameters (in):
    Channel: I2C channel.
    RequestPtr: I2C transmit request type
Parameters (inout):
    None
Parameters (out):
    None
Return value:
    E_OK: Command has been accepted
    E_NOT_OK: Command has not been accepted
```

### Std_ReturnType I2c_SyncMultiDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr, uint8 I2cRequestCnt )

```shell
Description: Sends or receives multiple I2C messages in blocking mode.

Sync/Async: Synchronous
Parameters (in):
    Channel: I2C channel.
    RequestPtr: I2C transmit request type
    I2cRequestCnt: Number of I2C transmit requests
Parameters (inout):
    None
Parameters (out):
    None
Return value:
    E_OK: Command has been accepted
    E_NOT_OK: Command has not been accepted
```

### Std_ReturnType I2c_AsyncDataTrans ( uint8 Channel, const I2c_RequestType* RequestPtr )

```shell
Description: Sends or receives an I2C message in non-blocking mode.

Sync/Async: Asynchronous
Parameters (in):
    Channel: I2C channel.
    RequestPtr: I2C transmit request type
Parameters (inout):
    None
Parameters (out):
    None
Return value:
    E_OK: Command has been accepted
    E_NOT_OK: Command has not been accepted
```

### I2c_StatusType I2c_StatusGet ( uint8 Channel )

```shell
Description: Gets the status of an I2C channel.

Sync/Async: Synchronous
Parameters (in):
    Channel: I2C channel.
Parameters (inout):
    None
Parameters (out):
    None
Return value: Status of the I2C channel
```

### void I2c_GetVersionInfo ( Std_VersionInfoType* VersionInfo )

```shell
Description: Retrieves the version information of this module.

Sync/Async: Synchronous
Parameters (in):
    VersionInfo: Version information of this module
Parameters (inout):
    None
Parameters (out):
    None
Return value: None
```