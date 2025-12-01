---
sidebar_position: 1
---
# Port Development Guide
## Basic Overview
The Port module is generally divided into two major parts: the external APIs and the "Low Level Driver (LLD)." This document only covers the user API development part.

## Port_Func Module
The SweetPotato MCU system provides a Port_Func module for overall functional configuration.

In the McalCdd directory path, the external interfaces of the Port_Func module are defined as follows:
```bash
# Driver source code:
McalCdd/Port/inc/Port_Func.h
McalCdd/Port/src/Port_Func.c
```
Under the Config directory, specific PinCtrl configurations for the MCU peripherals' pins are defined:
```bash
# PIN definition source code:
Config/McalCdd/gen_s100_sip_B_mcu1/Port/inc/Port_FuncCfg.h
Config/McalCdd/gen_s100_sip_B_mcu1/Port/src/Port_FuncCfg.c
```

### PinCtrl Configuration Description
In `Config/McalCdd/gen_s100_sip_B_mcu1/Port/inc/Port_FuncCfg.h`, macros are provided to facilitate defining pin attributes:
```C
#define PORT_FUNC_MCU_PIN(Pin_Idx, Pin_Name, Pin_Func, Schmitt, Input_En, SlewRate, Pull_Type, Drive_Strength) { \
	(uint8)(Pin_Idx), /**< Pin Id */ \
	(Pin_Name), /**< Pin name */ \
	(boolean)(TRUE), /**< IsUsed */ \
	{ \
		(boolean)(TRUE), /**< ModeChang */ \
		(boolean)(Schmitt), /**< Schmitt Trigger */ \
		(boolean)(Input_En), /**< Input Enable*/ \
		(boolean)(FALSE), /**< Is Used GPIO */ \
		(boolean)(FALSE), /**< Direciton change allowed */ \
		(Pin_Func), /**< Pin Function */ \
		(SlewRate), /**< Slew Rate */ \
		(Pull_Type), /**< Pin Pull Type */ \
		(Drive_Strength), /**< Pin Drive Strength */ \
		(PORT_PIN_DIR_IN), /**< GPIO Direction */ \
		(PORT_PIN_LEVEL_LOW) /**< GPIO Output Value */ \
	} \
}

...


#define PORT_FUNC_AON_PIN(Pin_Idx, Pin_Name, Pin_Func, Schmitt, Input_En, SlewRate, Pull_Type, Drive_Strength) { \
	(uint8)((Pin_Idx) + (S100_PORT_MCU_PIN_NUM)), /**< Pin Id Converted */ \
	(Pin_Name), /**< Pin name */ \
	(boolean)(TRUE), /**< IsUsed */ \
	{ \
		(boolean)(TRUE), /**< ModeChang */ \
		(boolean)(Schmitt), /**< Schmitt Trigger */ \
		(boolean)(Input_En), /**< Input Enable*/ \
		(boolean)(FALSE), /**< Is Used GPIO */ \
		(boolean)(FALSE), /**< Direciton change allowed */ \
		(Pin_Func), /**< Pin Function */ \
		(SlewRate), /**< Slew Rate */ \
		(Pull_Type), /**< Pin Pull Type */ \
		(Drive_Strength), /**< Pin Drive Strength */ \
		(PORT_PIN_DIR_IN), /**< GPIO Direction */ \
		(PORT_PIN_LEVEL_LOW) /**< GPIO Output Value */ \
	} \
}

...
```
Based on actual functional requirements, SweetPotato provides a default set of pin attribute definitions in the file `Config/McalCdd/gen_s100_sip_B_mcu1/Port/src/Port_FuncCfg.c`. Customers can modify these according to their specific needs.

## Port Module
The Port module is SweetPotato MCU's low-level PinCtrl module. Typically, users only need to use the Port_Func module to initialize and configure peripherals they intend to use. The following section covers advanced Port development topics; some features are only available in the commercial version of the code.

### Pin Initial State Configuration
During startup, MCU0 calls the `Port_Init()` interface to configure the initial state of all MCU pins. By default, MCU1 does not call the `Port_Init()` interface again during its startup.
:::warning
It is not recommended for users to invoke `Port_Init()` a second time on MCU1, as re-initializing pin states may cause basic functionalities such as power supply in the S100 SIP to malfunction.
:::

The initial states of each pin are defined in `Config/McalCdd/gen_s100_sip_B/Port/src/Port_PBCfg.c`, as shown in the following example:
```c
...
    /* Pin Id,  Pin name,      IsUsed,        ModeChang,    SchmittTriger,  InputEnable,     IsUsedGpio,     DirChang,   PinMode,    Config Type,        Pull Type,       Drive Strength,         GpioDir,           GpioData*/
    {(uint8)51, "EMAC2_TX_CLK", (boolean)TRUE, {(boolean)TRUE, (boolean)FALSE, (boolean)TRUE, (boolean)TRUE, (boolean)TRUE, GPIO, PORT_PIN_CONFIG_TYPE0, PORT_PULL_NONE, PORT_DRIVE_STRENGTH_5, PORT_PIN_DIR_IN, PORT_PIN_LEVEL_LOW}},
...
```
The code is split into two lines:
- The first line is a **comment line** indicating the meaning of each parameter.
- The second line is the **code line** defining the state of each individual pin. Key configuration items are explained below:
  - `PinMode`: Specifies the specific function of the pin. Refer to `Config/McalCdd/gen_s100_sip_B/Port/src/Port_PBCfg.h` for available function definitions.
  - `Config Type`: Defines the pin's SlewRate control. For detailed definitions, commercial version users can refer to `Port/Inc/Port_Lld.h`, while community version users can refer to `Include/Port_Lld.h`.
  - `IsUsed`: Indicates whether the Port module will configure this pin during initialization.
  - `ModeChange`: Specifies whether the pin's function can be modified at runtime when calling Port module APIs (excluding Port_Func module APIs).
  - `IsUsedGpio`: Indicates whether the Port module will configure this pin's GPIO direction and output value during initialization.

Customers can define the initial state of pins according to their requirements.