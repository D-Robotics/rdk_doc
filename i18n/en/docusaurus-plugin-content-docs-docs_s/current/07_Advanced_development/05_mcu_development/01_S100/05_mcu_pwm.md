---
sidebar_position: 5
---

# PWM Usage Guide

## Hardware Support

The S100 has 1 PWM module, and each PWM IP contains 12 channels.

- Each channel is independent and supports IRQ request and DMA request.
- Hardware-triggered pulse output is supported.
- Each channel has its own independent clock divider register.
- All channels within a single IP share one interrupt line.
- An interrupt or DMA request is triggered when the target edge or pulse type arrives.
- DMA-based updates of period and duty cycle are supported.
- Period edge alignment mode can be configured as either edge-aligned or center-aligned.
- PWM synchronous output mode is supported, allowing either 6-channel or 12-channel synchronization.

## Software Driver

- CPU-based updates of PWM channel period and duty cycle are supported.
- DMA-based updates of PWM channel period and duty cycle are supported.
- Enabling/disabling PWM interrupts is supported. PWM channel interrupt callback functions can be configured, with interrupt types including rising edge, falling edge, and both edges.
- Internal status of the PWM output signal can be read and returned.
- Multi-channel synchronous output is supported.

## Code Paths

# Description of PWM Module Related Files

- `Config/McalCdd/gen_s100_sip_B_mcu1/Pwm/src/Pwm_PBCfg.c`: Pre-compiled configuration source file for PWM in normal mode, containing specific configuration parameters (e.g., period, duty cycle, polarity) for channels and instances.
- `Config/McalCdd/gen_s100_sip_B_mcu1/Pwm/inc`: Pre-compiled configuration header files defining macro switches.
- `McalCdd/Pwm/src/Pwm_Lld.c`: Low-level driver implementation file that directly manipulates hardware registers and provides low-level interfaces.
- `McalCdd/Pwm/src/Pwm.c`: High-level driver logic implementation that encapsulates API interfaces and handles error detection, state management, and other control logic.
- `McalCdd/Pwm/inc/Pwm_Lld.h`: Low-level driver header file declaring low-level function prototypes, structures, and enumerations.
- `McalCdd/Pwm/inc/Pwm_Types.h`: Defines common data types, structures, and callback function pointer types shared between upper and lower layers.
- `McalCdd/Pwm/inc/Pwm.h`: Main header file declaring high-level APIs and core structures.
- `samples/Pwm/Pwm_sample/Pwm_test.c`: PWM test sample.

## Platform-Specific Configuration

The configuration source file in the PWM driver is `Pwm_PBCfg.c`. By default, the S100 chip supports configurations for 12 channels. However, due to PCB differences, some PWM channel pins are occupied. For example, on the RDKS100 board, PWM2 and PWM3 pins are occupied. To avoid conflicts, the following changes are required in the configuration:

1. Set the `LldChannelCfg` member of the corresponding structure in the `Pwm_Channels_PB` array to `NULL` to mark the PWM channel as invalid:
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

2. Set the `HwChannelId` member of the last valid structure in the `Pwm_HwChannelConfig_PB` array to `PWM_HW_CHANNEL_INVALID_ID`, indicating that subsequent configurations are invalid:
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
- The `Pwm_Channels_PB` structure array defines the mapping relationship between logical PWM channels and underlying hardware channels.
- The `Pwm_HwChannelConfig_PB` structure array defines default hardware parameters for each PWM channel, such as period, duty cycle, polarity, interrupt enable status, etc.
:::



## Application Sample


### Usage Example

The S100 development board exposes PWM pins for user development and learning. The pin locations and statuses of the exposed PWM channels are as follows:

| PWM Channel | Board                     | Pin Status / Multiplexing        |
|-------------|---------------------------|----------------------------------|
| pwm0        | MCU Expansion Board       | Multiplexed with I2C9 SCL        |
| pwm1        | MCU Expansion Board       | NONE                             |
| pwm6        | MCU Expansion Header on Mainboard | NONE                    |
| pwm7        | MCU Expansion Header on Mainboard | NONE                   |
| pwm10       | MCU Expansion Board       | Multiplexed with I2C8 SCL        |
| pwm11       | MCU Expansion Board       | Multiplexed with I2C8 SDA        |


The `pwmtest` command is used to configure and control PWM (Pulse Width Modulation) channels. Below is the usage guide and examples for the `pwmtest` command.

- Usage

Set PWM duty cycle:
```sh
pwmtest <pwm_channel> <period> <duty_cycle>
```

Stop PWM output:
```sh
pwmtest stop <pwm_channel>
```

For example, to set PWM channel 0 with a period of 1000 µs and a 50% duty cycle:
```
pwmtest 0 0x30d40 0x4000
```

- Parameter Description

```sh
<pwm_channel>: PWM channel number to configure or stop.
<period>: Period of the PWM signal.
<duty_cycle>: Duty cycle of the PWM signal, which must be within the range 0x0000 (0%) to 0x8000 (100%).
```

- Period Calculation

PWM Period = Clock Source Frequency / Period Register Value

Example: To generate a waveform with a period of 1000 µs, assuming the PWM clock source is 200 MHz by default, the register value should be 200000000 / 1000 = 200000 (0x30d40).


### Debug Sample

- Usage

```bash
pwmdumpregs <pwm_channel>
```

For example, to dump registers of PWM channel 0:
```bash
D-Robotics:/$ pwmdumpregs 0
[06915.231597 0]INFO: Pwm_RegDump pwm channel:0
[06915.231967 0]INFO: Pwm_RegDump channel 0
ch[0]                              PERIOD 22370000 60000 # Period
ch[0]                              PERIOD 22370000 48000 # Duty Cycle
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
```ch[0]        PWM_CTRL_REG.CPU_HALT_ENABLE          0  
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


### Application Programming Interface

#### void Pwm_Init(const Pwm_ConfigType* ConfigPtr)

```shell
Description: Service for PWM initialization

Sync/Async: Synchronous
Parameters(in)
    ConfigPtr: Pointer to configuration set
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```



#### void Pwm_DeInit(void)

```shell
Description: Service for PWM De-Initialization.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Pwm_SetDutyCycle(Pwm_ChannelType ChannelNumber, uint16 DutyCycle)

```shell
Description: Service sets the duty cycle of the PWM channel.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    DutyCycle: Min=0x0000 Max=0x8000
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Pwm_SetPeriodAndDuty(Pwm_ChannelType ChannelNumber, Pwm_PeriodType Period, uint16 DutyCycle)

```shell
Description: Service sets the duty cycle of the PWM channel.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    Period: Period of the PWM signal
    DutyCycle: Min=0x0000 Max=0x8000
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Pwm_SetOutputToIdle(Pwm_ChannelType ChannelNumber)

```shell
Description: Service sets the PWM output to the configured Idle state.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Pwm_OutputStateType Pwm_GetOutputState(Pwm_ChannelType ChannelNumber)

```shell
Description: Service to read the internal state of the PWM output signal.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None

Return value: Pwm_OutputStateType
    PWM_HIGH: The PWM output state is high
    PWM_LOW: The PWM output state is low
```

#### void Pwm_DisableNotification(Pwm_ChannelType ChannelNumber)

```shell
Description: Service to disable the PWM signal edge notification.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Pwm_EnableNotification(Pwm_ChannelType ChannelNumber, Pwm_EdgeNotificationType Notification)

```shell
Description: Service to enable the PWM signal edge notification.
Sync/Async: Asynchronous
Parameters(in)
    ChannelNumber: Numeric identifier of PWM
    Notification: Notification type to be enabled
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Pwm_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description: Service returns the version information of this module.
Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    versioninfo: Pointer to where to store the version information of this module.
Return value: None
```