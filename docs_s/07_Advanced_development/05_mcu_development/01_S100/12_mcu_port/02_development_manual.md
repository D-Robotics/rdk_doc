---
sidebar_position: 1
---
# Port开发指南
## 基本概述
Port整体分为对外接口和"Low Level Driver(LLD)"两大部分，这里只介绍用户接口开发部分。

## Port_Func模块
地瓜MCU系统提供针对功能整体配置的Port_Func模块。

在McalCdd的路径中，定义了Port_Func模块的对外接口：
``` bash
# Driver source code:
McalCdd/Port/inc/Port_Func.h
McalCdd/Port/src/Port_Func.c
```
在Config目录下，定义了MCU提供的外设的PIN的具体PinCtrl配置。
```bash
# PIN definition source code:
Config/McalCdd/gen_s100_sip_B_mcu1/Port/inc/Port_FuncCfg.h
Config/McalCdd/gen_s100_sip_B_mcu1/Port/src/Port_FuncCfg.c
```
### PinCtrl配置说明
在`Config/McalCdd/gen_s100_sip_B_mcu1/Port/inc/Port_FuncCfg.h`中，提供了方便定义PIN属性的宏：
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
地瓜根据各个功能的实际情况，提供了一套默认的PIN属性定义，在`Config/McalCdd/gen_s100_sip_B_mcu1/Port/src/Port_FuncCfg.c`文件内，客户可以根据自己的实际需求进行修改。

## Port模块
Port模块是地瓜MCU底层的PinCtrl模块，一般用户使用Port_Func模块对需要使用的外设进行功能的初始化配置即可。以下部分属于进阶Port开发的内容，部分功能仅在商业版代码开放。

### Pin初始状态配置
MCU0在启动时，会调度`Port_Init()`接口对MCU的所有PIN进行初始状态配置，MCU1在启动时默认不会再次`Port_Init()`接口。
:::warning
不建议用户在MCU1二次调度`Port_Init()`接口，二次初始化PIN状态可能会导致S100 SIP的供电等基础功能异常。
:::

在`Config/McalCdd/gen_s100_sip_B/Port/src/Port_PBCfg.c`中定义了各个pin的初始状态，示例如下：
```c
...
    /* Pin Id,  Pin name,      IsUsed,        ModeChang,    SchmittTriger,  InputEnable,     IsUsedGpio,     DirChang,   PinMode,    Config Type,        Pull Type,       Drive Strength,         GpioDir,           GpioData*/
    {(uint8)51, "EMAC2_TX_CLK", (boolean)TRUE, {(boolean)TRUE, (boolean)FALSE, (boolean)TRUE, (boolean)TRUE, (boolean)TRUE, GPIO, PORT_PIN_CONFIG_TYPE0, PORT_PULL_NONE, PORT_DRIVE_STRENGTH_5, PORT_PIN_DIR_IN, PORT_PIN_LEVEL_LOW}},
...
```
代码分为两行：
- 第一行“注释行”标注了代码行每一个参数所代表的含义；
- 第二行“代码行”定义了每一根PIN的状态，下面介绍几个重点配置：
  - `PinMode`：定义了PIN的具体功能，功能定义可以参考`Config/McalCdd/gen_s100_sip_B/Port/src/Port_PBCfg.h`
  - `Config Type`：定义了PIN的SlewRate控制，具体定义，商业版可以参考`Port/Inc/Port_Lld.h`，社区版可以参考：`Include/Port_Lld.h`
  - `IsUsed`：定义Port模块是否会在初始化时对该PIN进行配置；
  - `ModeChange`：定义了调度Port模块（不包括Port_Func模块）API时，是否允许该PIN的功能在Runtime被修改；
  - `IsUsedGpio`：定义Port模块是否会在初始化时对该PIN进行GPIO的方向及输出配置；

客户可以根据需要自行定义PIN的初始状态。
