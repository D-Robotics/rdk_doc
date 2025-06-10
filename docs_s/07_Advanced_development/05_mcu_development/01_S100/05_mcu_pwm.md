---
sidebar_position: 5
---

# PWM 使用指南

## 硬件支持

S100 有1个PWM，每个PWM IP 有12个通道。

- 每个通道都是独立的，支持irq requset和dma requset
- 可以支持硬件触发脉冲输出
- 每个通道有自己独立的时钟分频寄存器
- 每个IP所有通道共享一个中断
- 当目标边沿或者脉冲类型到来的时候，会触发中断或者dma req
- 支持DMA更新period和duty
- 支持周期边沿对齐方式设置，可以设置为边沿对齐或者中心对齐
- 支持pwm同步输出模式，6通道同步或者12通道同步。

## 软件驱动

- 支持CPU更新PWM通道的周期和占空比
- 支持DMA更新PWM通道的周期和占空比
- 支持设置开启和关闭PWM中断，设置PWM通道的中断函数，支持中断类型：上升沿，下降沿，双边沿
- 读取PWM输出信号的内部状态并将其返回
- 支持多通道同步输出


## 代码路径

# PWM 模块相关文件说明

- Config/McalCdd/gen_s100_sip_B_mcu1/Pwm/src/Pwm_PBCfg.c: 正常模式下 PWM 的预编译配置源文件，包含通道和实例的具体配置参数（如周期、占空比、极性等）。
- Config/McalCdd/gen_s100_sip_B_mcu1/Pwm/inc: 预编译配置头文件，定义宏开关。
- McalCdd/Pwm/src/Pwm_Lld.c: 底层驱动实现文件，直接操作硬件寄存器，提供底层接口。
- McalCdd/Pwm/src/Pwm.c: 上层驱动逻辑实现，封装API 接口，并处理错误检测、状态管理等控制逻辑。
- McalCdd/Pwm/inc/Pwm_Lld.h: 底层驱动头文件，声明底层函数原型、结构体和枚举类型。
- McalCdd/Pwm/inc/Pwm_Types.h: 定义通用数据类型、结构体和回调函数指针类型，供上下层共享使用。
- McalCdd/Pwm/inc/Pwm.h: 主头文件，声明高层 API和核心结构体。
- samples/Pwm/Pwm_sample/Pwm_test.c: Pwm测试sample。

## 平台特殊配置

PWM驱动中的配置源文件是Pwm_PBCfg.c，S100芯片默认是支持12个Channel的配置，但是由于Pcb的差异，部分PWM通道的引脚被占用，例如RDKS100的PWM2和PWM3被占用，为避免冲突，在配置上需要做如下更改：

1. 修改Pwm_Channels_PB数组中结构体成员LldChannelCfg的值为NULL，表示这个PWM通道无效
```c
static Pwm_ChannelConfigType Pwm_Channels_PB[PWM_CONF_CHANNELS_PB] = {
    ......
    {
        /* @brief Pwm Channel id */
        .ChannelId = 1,
        /* @brief Pwm Channel Class */
        .PwmChannelClass = PWM_VARIABLE_PERIOD,
        /** @brief Pointer to channel pwm hw channel: pwm 0 - ch 1 */
        .LldChannelCfg = &Pwm_HwChannelConfig_PB[0][1],
    },
    {
        /* @brief Pwm Channel id */
        .ChannelId = 2,
        /* @brief Pwm Channel Class */
        .PwmChannelClass = PWM_VARIABLE_PERIOD,
        /** @brief Pointer to channel pwm hw channel: pwm 0 - ch 2 */
        .LldChannelCfg = NULL,
    },
    ......
}
```

2. 修改Pwm_HwChannelConfig_PB数组中的最后一个结构体成员HwChannelId的值为PWM_HW_CHANNEL_INVALID_ID，表示从这个配置开始，后面的配置无效
```c
static Pwm_Lld_ChannelConfigType Pwm_HwChannelConfig_PB[PWM_HW_CONF_MODS_PB][12] = {
    {
        ......
        {
            /**< pwm hardware channel PwmHwChId11 */
            .HwChannelId = PwmHwChId11,
            /**< pwm hardware ip id 0 */
            .HwIpId = 0,
            /**< pwm clear mode */
            .ClearMode = FALSE,
            /**< pwm channel clock ratio*/
            .ClockRatio = 0,
            /**< pwm period*/
            .Period = 3125000,
            /**< pwm polarity*/
            .Polarity = PWM_HIGH,
            /**< pwm duty cycle*/
            .DutyCycle = 1562500,
            /**< pwm edge align mode */
            .EdgeAlign = PWM_LLD_GEN_ALIGN_EDGE,
            /**< pwm edge mode */
            .EdgeMode = PWM_LLD_EDGEMODE_BOTH,
            /**< hardware triger mask */
            .HwTrigMask = TRUE,
            /**< pwm hardware triger width*/
            .HwTrigWidth = 0,
            /**< the switch of isr notification*/
            .NotificationEnable = TRUE,
            /**< the callback of isr notification */
            .Notification = Pwm_Channel_0_6_ISR,
            /**< the switch of dma complete notification*/
            .DmaCpltCallbackEnable = TRUE,
            /**< the callback of dma complete notification */
            .DmaCpltCallback = NULL_PTR,
        },
        {
            /**< Add termination structure members */
            .HwChannelId = PWM_HW_CHANNEL_INVALID_ID,
        }
    }
}
```


:::tip
- Pwm_Channels_PB 结构体数组定义了逻辑PWM通道与底层硬件通道之间的映射关系。
- Pwm_HwChannelConfig_PB 结构体数组定义了每个PWM通道的默认硬件参数，如周期、占空比、极性、中断使能等。
:::



## 应用sample


### 使用示例
S100 开发板将PWM引出供用户开发学习使用，已引出PWM Channel的PIN脚位置以及状态如下：

| PWM通道 | 所属板子                  | 引脚状态/复用              |
|--------|---------------------------|---------------------------|
| pwm0   | MCU扩展板                 |  与I2C9 SCL复用            |
| pwm1   | MCU扩展板                 |  NONE                     |
| pwm6   | Mainboad板的MCU expansion Header | NONE               |
| pwm7   | Mainboad板的MCU expansion Header | NONE              |
| pwm10  | MCU扩展板                 | 与I2C8 SCL复用            |
| pwm11  | MCU扩展板                 | 与I2C8 SDA复用            |


`pwmtest`命令用于配置和控制PWM（脉冲宽度调制）通道。下面是`pwmtest`命令的使用说明和示例。

- 使用方法

设置PWM占空比：
```sh
pwmtest <pwm通道> <周期> <占空比>
```

停止PWM输出：
```sh
pwmtest stop <pwm通道>
```

例如设置PWM通道0的周期为1000us，占空比为50%：
```
pwmtest 0 0x30d40 0x4000
```

- 参数说明

```sh
<pwm通道>: 要配置或停止的PWM通道号。
<周期>: PWM信号的周期。
<占空比>: PWM信号的占空比，必须在0x0000（0%）到0x8000（100%）的范围内。
```

- 周期计算

PWM周期=时钟源频率/周期寄存器值​

例：输出周期为1000us的波，pwm时钟源默认为200Mhz，则需要在寄存器中写入 200000000/1000=200000(0x30d40)。


### Debug Sample

- 使用方法

```bash
pwmdumpregs <pwm通道>
```

例如dump pwm channel0的寄存器
```bash
D-Robotics:/$ pwmdumpregs 0
[06915.231597 0]INFO: Pwm_RegDump pwm channel:0
[06915.231967 0]INFO: Pwm_RegDump channel 0
ch[0]                              PERIOD 22370000 60000 # 周期
ch[0]                              PERIOD 22370000 48000 # 占空比
ch[0]                            CAP_TIME 22370004 0
ch[0]                            CAP_TIME 22370004 0
ch[0]               PWM_CTRL_REG.MODE_SEL          0
ch[0]          PWM_CTRL_REG.GEN_ALIGN_SEL          0
ch[0]           PWM_CTRL_REG.GEN_POLARITY          0
ch[0]               PWM_CTRL_REG.INT_MASK          1
ch[0]               PWM_CTRL_REG.DMA_MASK          0
ch[0]           PWM_CTRL_REG.HW_TRIG_MASK          1
ch[0]       PWM_CTRL_REG.GLITCH_FILTER_EN          0
ch[0]       PWM_CTRL_REG.GLITCH_THRESHOLD          0
ch[0]          PWM_CTRL_REG.HW_TRIG_WIDTH          0
ch[0]          PWM_CTRL_REG.CLK_DIV_RATIO          0
ch[0]               PWM_CTRL_REG.EDGE_SEL          0
ch[0]        PWM_CTRL_REG.CPU_HALT_ENABLE          0
ch[0]       PWM_CTRL_REG.TIMER_CLEAR_MODE          0
ch[0]           PWM_CTRL_REG.RESERVED_BIT          0
ch[0]                                  EN 2237000c 1
ch[0]                       COMPARE_STATE 22370010 0
ch[0]                           PIN_STATE 22370014 3
ch[0]                   CAP_CNT_THRESHOLD 22370018 1
ch[0]                             CAP_CNT 2237001c 1539
ch[0]                                 EOI 22370024 0
ch[0]                              INT_ST 22370028 0
ch[0]                                 CNT 2237002c 28991
pwm                                  EOIS 22370300 0
pwm                                INT_ST 22370304 0
pwm                            RAW_INT_ST 22370308 1
pwm                              RESERVED 2237030c 0
pwm                              CAP_MISS 22370310 0
pwm                            CLEAR_MODE 22370314 33c
pwm                             SYNC_MODE 22370318 0
pwm                         CFG_LOCK_MODE 2237031c 0
[06915.263883 0]INFO: Pwm_RegDump end

```


### 应用程序接口

#### void Pwm_Init(const Pwm_ConfigType* ConfigPtr)

```shell
Description：Service for PWM initialization

Sync/Async：Synchronous
Parameters(in)
    ConfigPtr： Pointer to configuration set
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```



#### void Pwm_DeInit(void)

```shell
Description：Service for PWM De-Initialization.

Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Pwm_SetDutyCycle(Pwm_ChannelType ChannelNumber, uint16 DutyCycle)

```shell
Description：Service sets the duty cycle of the PWM channel.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    DutyCycle: Min=0x0000 Max=0x8000
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Pwm_SetPeriodAndDuty(Pwm_ChannelType ChannelNumber, Pwm_PeriodType Period, uint16 DutyCycle)

```shell
Description：Service sets the duty cycle of the PWM channel.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    Period: Period of the PWM signal
    DutyCycle: Min=0x0000 Max=0x8000
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Pwm_SetOutputToIdle(Pwm_ChannelType ChannelNumber)

```shell
Description：Service sets the PWM output to the configured Idle state.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Pwm_OutputStateType Pwm_GetOutputState(Pwm_ChannelType ChannelNumber)

```shell
Description：Service to read the internal state of the PWM output signal.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None

Return value：Pwm_OutputStateType
    PWM_HIGH: The PWM output state is high
    PWM_LOW: The PWM output state is low
```

#### void Pwm_DisableNotification(Pwm_ChannelType ChannelNumber)

```shell
Description：Service to disable the PWM signal edge notification.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Pwm_EnableNotification(Pwm_ChannelType ChannelNumber, Pwm_EdgeNotificationType Notification)

```shell
Description：Service to enable the PWM signal edge notification.
Sync/Async：Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    Notification: Notification type to be enabled
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Pwm_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description：Service returns the version information of this module.
Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    versioninfo: Pointer to where to store the version information of this module.
Return value：None
```
