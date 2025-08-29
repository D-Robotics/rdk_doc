---
sidebar_position: 10
---

# 低功耗模式调试指南

## S100芯片电源域
S100内部有AON、MCU和Main域三个电源域。其中AON为非下电状态需要一直供电的电源域，MCU电源域用于给Hsm和MCU及其内部IP供电，Main域给其他部分供电。

## S100电源状态列表
S100目前实现了Off，MCU only，Working，Deep sleep和Light sleep五种电源状态，详细说明如下：

| 电源状态    |       描述                             |  AON  |  MCU  |  Main  |  DDR颗粒  |
| --------   | ---------------------------------------| ----  | ----- | -----  | ---------- |
| Off        | 芯片完全下电                            | Off   |   Off |   Off  |   Off      |
| MCU only   | MCU Rcore正常工作                       | On   |   On   |   Off  |   Off      |
| Deep sleep | 只有AON工作，可被唤醒                    | On   |   Off  |   Off  |   自刷新    |
| Working    | 正常工作模式                            | On    |   On  |   On   |   On    |
| Light sleep | MCU正常工作，Main仅DDR颗粒供电维持自刷新  | On   |   On   |   Off  |   自刷新    |


## 休眠唤醒

**注意：以下介绍的唤醒源的API及命令，Light sleep模式下的唤醒API及命令，只能在MCU0内调用**

### 设置休眠模式
休眠模式默认为Deep sleep模式

#### 显示当前支持的休眠模式
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/suspend_mode
light deep
```

当前支持的休眠模式有deep和light，默认是deep

#### 显示当前的休眠模式
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
deep
```

#### 切换休眠模式
```Shell
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
deep
root@ubuntu:~# echo light > /sys/devices/platform/suspend-mode/suspend_mode/suspend_mode
root@ubuntu:~# cat /sys/devices/platform/suspend-mode/suspend_mode/current_suspend_mode
light
```

### 设置唤醒源

唤醒源默认是RTC，时间为15秒

#### 通过MCU命令设置唤醒源

```Shell
D-Robotics:/$ wakeupsource
[01504.149120 0]WakeupSource Usage:
[01504.149349 0]    WakeupSource rtc <time(seconds)>
[01504.149935 0]    WakeupSource can
[01504.150347 0]    WakeupSource gpio <gpio_index> <type> <level>
[01504.151093 0]        type:  0:level other:edge
[01504.151628 0]        level: 0:low   other:high
```
目前仅支持设置RTC和GPIO作为唤醒源，以下是使用示例：

- 设置RTC作为唤醒源，时间是60秒
```Shell
D-Robotics:/$ wakeupsource rtc 60
[01620.452562 0]rtc init alarm time 60.
Return: 0, 0x00000000
```

- 设置GPIO为唤醒源，AON GPIO 11，低电平有效
```Shell
D-Robotics:/$ wakeupsource gpio 11 0 0
[01680.254419 0]set gpio wakeup source with index 11 type 0 level 0.
Return: 0, 0x00000000
```

#### 通过MCU API设置唤醒源

##### RTC唤醒时间设置

**SysPower_RtcWakeupSet**

| 属性 | 描述 |
| ----- | ----- |
| Name | SysPower_RtcWakeupSet |
| Syntax | Std_ReturnType SysPower_RtcWakeupSet (uint32 WakeupTime) |
| Sync/Async | Synchronous |
| Reentrancy | No Reentrant |
| Parameters (in) | WakeupTime: time of rtc alarm (s) |
| Parameters (inout) | None |
| Parameters (out) | None |
| Return value | Std_ReturnType |
| Description | Set rtc wakeup time |
| Available via | as extern function |

**RTC 唤醒时间设置后，从MCU实际开始做休眠动作才会计时；即SysPower_Suspend调用后，core0进入休眠再开始计时**

##### GPIO唤醒源设置

**SysPower_GpioWakeupSet**

| 属性 | 描述 |
| ----- | ----- |
| Name | SysPower_GpioWakeupSet |
| Syntax | Std_ReturnType SysPower_GpioWakeupSet (uint32 GpioIdx, uint32 Type, uint32 Polarity) |
| Sync/Async | Synchronous |
| Reentrancy | No Reentrant |
| Parameters (in) | GpioIdx: index of aon gpio; Type: 0:level other:edge; Polarity: 0:low other:high |
| Parameters (inout) | None |
| Parameters (out) | None |
| Return value | Std_ReturnType |
| Description | Set gpio wakeup source |
| Available via | as extern function |

##### GPIO Index定义
```Shell
0：CAN0_RX/(GPIO0)
1：CAN1_RX/(GPIO1)
2：CAN2_RX/(GPIO2)
3：CAN3_RX/(GPIO3)
4：CAN4_RX/(GPIO4)
5：CAN5_RX/(GPIO5)
6：CAN6_RX/(GPIO6)
7：CAN7_RX/(GPIO7)
8：CAN8_RX/(GPIO8)
9：CAN9_RX/(GPIO9)
10：LIN1_RXD/GPIO(10)
11：LIN2_RXD/GPIO(11)
```

**可以单独设置RTC为唤醒源，不能单独设置GPIO为唤醒源。设置GPIO为唤醒源时必须同时设置RTC作为唤醒源**

### 休眠命令

- 通过按键（SLEEP按键）休眠（短按）
- Acore输入`systemctl suspend`

### 唤醒命令

- Deep sleep模式下，RTC作为唤醒源时可以自动唤醒

- Deep sleep模式下，设置GPIO作为唤醒源，休眠之后短按按键（SLEEP按键）可以唤醒

- Light sleep模式下，只能通过在MCU执行命令或者调用API接口唤醒

#### Light sleep模式唤醒命令
```Shell
D-Robotics:/$ wakefromll
D-Robotics:/$ [0544.238961 0] main on start
...
```

#### Light sleep模式唤醒接口

**Pmu_PerformMainDomainOn**

| 属性 | 描述 |
| ----- | ----- |
| Name | Pmu_PerformMainDomainOn |
| Syntax | Std_ReturnType Pmu_PerformMainDomainOn (void) |
| Sync/Async | Synchronous |
| Reentrancy | No Reentrant |
| Parameters (in) | None |
| Parameters (inout) | None |
| Parameters (out) | None |
| Return value | Std_ReturnType |
| Description | main domain power on or resume, and main domain periperals |
| Available via | Pmu.h |

以上休眠唤醒API调用sample在```Service/Cmd_Utility/power_sample_cmd/src/PowerControl.c```
