---
sidebar_position: 10
---

# Low Power Mode Debugging Guide

## S100 Chip Power Domains
The S100 chip contains three power domains: AON, MCU, and Main. The AON domain must remain powered at all times (never powered off). The MCU domain supplies power to the HSM, MCU, and their internal IPs. The Main domain powers all other components.

## S100 Power States List
The S100 currently implements five power states: Off, MCU only, Working, Deep sleep, and Light sleep. Details are as follows:

| Power State | Description                                      | AON  | MCU  | Main | DDR Chips |
|-------------|--------------------------------------------------|------|------|------|-----------|
| Off         | Chip completely powered off                      | Off  | Off  | Off  | Off       |
| MCU only    | MCU Rcore operates normally                      | On   | On   | Off  | Off       |
| Deep sleep  | Only AON is active; can be woken up              | On   | Off  | Off  | Self-refresh |
| Working     | Normal operating mode                            | On   | On   | On   | On        |
| Light sleep | MCU operates normally; only DDR chips in Main domain are powered to maintain self-refresh | On   | On   | Off  | Self-refresh |

## Sleep and Wake-up

**Note: The wake-up source APIs and commands described below, including those specific to Light sleep mode, can only be invoked from MCU0.**

### Setting Sleep Mode
The default sleep mode is Deep sleep.

#### Display currently supported sleep modes
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/suspend_mode
light deep
```

Currently supported sleep modes are `deep` and `light`, with `deep` as the default.

#### Display the current sleep mode
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
deep
```

#### Switch sleep mode
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
deep
root@ubuntu:~# echo light > /sys/devices/platform/suspend-mode/suspend_mode/suspend_mode
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
light
```

### Setting Wake-up Sources

The default wake-up source is RTC with a timeout of 15 seconds.

#### Setting wake-up source via MCU command

```Shell
D-Robotics:/$ wakeupsource
[01504.149120 0]WakeupSource Usage:
[01504.149349 0]    WakeupSource rtc <time(seconds)>
[01504.149935 0]    WakeupSource can
[01504.150347 0]    WakeupSource gpio <gpio_index> <type> <level>
[01504.151093 0]        type:  0:level other:edge
[01504.151628 0]        level: 0:low   other:high
```
Currently, only RTC and GPIO are supported as wake-up sources. Examples are shown below:

- Set RTC as wake-up source with a 60-second timeout:
```Shell
D-Robotics:/$ wakeupsource rtc 60
[01620.452562 0]rtc init alarm time 60.
Return: 0, 0x00000000
```

- Set GPIO as wake-up source: AON GPIO 11, active-low:
```Shell
D-Robotics:/$ wakeupsource gpio 11 0 0
[01680.254419 0]set gpio wakeup source with index 11 type 0 level 0.
Return: 0, 0x00000000
```

#### Setting wake-up source via MCU API

##### RTC Wake-up Time Configuration

**SysPower_RtcWakeupSet**

| Attribute        | Description                                      |
|------------------|--------------------------------------------------|
| Name             | SysPower_RtcWakeupSet                            |
| Syntax           | Std_ReturnType SysPower_RtcWakeupSet (uint32 WakeupTime) |
| Sync/Async       | Synchronous                                      |
| Reentrancy       | Non-reentrant                                    |
| Parameters (in)  | WakeupTime: RTC alarm time in seconds            |
| Parameters (inout)| None                                            |
| Parameters (out) | None                                             |
| Return value     | Std_ReturnType                                   |
| Description      | Set RTC wake-up time                             |
| Available via    | as extern function                               |

**Note: After setting the RTC wake-up time, the countdown starts only after the MCU actually initiates the sleep procedure; i.e., counting begins after `SysPower_Suspend` is called and core0 enters sleep.**

##### GPIO Wake-up Source Configuration

**SysPower_GpioWakeupSet**

| Attribute        | Description                                      |
|------------------|--------------------------------------------------|
| Name             | SysPower_GpioWakeupSet                           |
| Syntax           | Std_ReturnType SysPower_GpioWakeupSet (uint32 GpioIdx, uint32 Type, uint32 Polarity) |
| Sync/Async       | Synchronous                                      |
| Reentrancy       | Non-reentrant                                    |
| Parameters (in)  | GpioIdx: index of AON GPIO; Type: 0 for level-triggered, others for edge-triggered; Polarity: 0 for low, others for high |
| Parameters (inout)| None                                            |
| Parameters (out) | None                                             |
| Return value     | Std_ReturnType                                   |
| Description      | Set GPIO wake-up source                          |
| Available via    | as extern function                               |

##### GPIO Index Definitions
```Shell
0: CAN0_RX/(GPIO0)
1: CAN1_RX/(GPIO1)
2: CAN2_RX/(GPIO2)
3: CAN3_RX/(GPIO3)
4: CAN4_RX/(GPIO4)
5: CAN5_RX/(GPIO5)
6: CAN6_RX/(GPIO6)
7: CAN7_RX/(GPIO7)
8: CAN8_RX/(GPIO8)
9: CAN9_RX/(GPIO9)
10: LIN1_RXD/GPIO(10)
11: LIN2_RXD/GPIO(11)
```

**RTC can be configured as the sole wake-up source, but GPIO cannot be used alone. When configuring GPIO as a wake-up source, RTC must also be configured as a wake-up source simultaneously.**

### Sleep Commands

- Sleep via button press (SLEEP button, short press)
- Enter sleep from Acore by running `systemctl suspend`

### Wake-up Commands

- In Deep sleep mode, the system wakes up automatically when RTC is configured as the wake-up source.
- In Deep sleep mode, if GPIO is configured as a wake-up source, pressing the SLEEP button (short press) after entering sleep will wake the system.
- In Light sleep mode, wake-up can only be triggered by executing a command or calling an API from the MCU.

#### Light sleep mode wake-up command
```Shell
D-Robotics:/$ wakefromll
D-Robotics:/$ [0544.238961 0] main on start
...
```

#### Light sleep mode wake-up API

**Pmu_PerformMainDomainOn**

| Attribute        | Description                                      |
|------------------|--------------------------------------------------|
| Name             | Pmu_PerformMainDomainOn                          |
| Syntax           | Std_ReturnType Pmu_PerformMainDomainOn (void)    |
| Sync/Async       | Synchronous                                      |
| Reentrancy       | Non-reentrant                                    |
| Parameters (in)  | None                                             |
| Parameters (inout)| None                                            |
| Parameters (out) | None                                             |
| Return value     | Std_ReturnType                                   |
| Description      | Power on or resume the Main domain and its peripherals |
| Available via    | Pmu.h                                            |

Sample code for the above sleep/wake-up APIs can be found in  
```Service/Cmd_Utility/power_sample_cmd/src/PowerControl.c```