---
sidebar_position: 7
---

# ADC 使用指南

## 硬件支持

S100 Adc 有一个Adc硬件，包含chennel0-channel13和channel15共15个通道。

- Adc可测试电压范围：100mv - 1700mv。
- 硬件触发、inject模式下只允许有一个组。
- Adc在温度变化超过20°时，要进行校准操作。
- Adc软件触发连续转换模式应当配合DMA功能使用，不推荐其他使用方式。
- Adc用户接口的使用需要确保上电自检结束后进行。


## 代码路径

| **文件路径**                                  | **作用**                                                                 |
|-----------------------------------------------|--------------------------------------------------------------------------|
| `McalCdd/Adc/inc/Adc.h`                       | 公共API接口，供上层调用。                                                 |
| `McalCdd/Adc/inc/Adc_Lld.h`                   | 声明底层硬件操作函数。                                                    |
| `McalCdd/Adc/inc/Adc_Private.h`               | 定义私有结构、宏和函数声明。                                               |
| `McalCdd/Adc/src/Adc.c`                       | 实现公共API，调用底层函数。                                                |
| `McalCdd/Adc/src/Adc_Lld.c`                   | 实现底层硬件操作，直接配置寄存器。                                          |
| `McalCdd/Adc/src/Adc_Private.c`               | 实现私有函数，辅助驱动内部逻辑。                                            |
| `McalCdd/Common/Register/inc/Adc_Register.h`      | 定义ADC外设寄存器地址和位域。                                              |
| `Platform/Schm/SchM_Adc.h`                    | 管理ADC的访问权限和资源保护（如中断安全）。                                 |
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/inc/Adc_PBcfg.h`          | 定义板级外设配置参数（如通道、采样率等）。               |
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/inc/Adc_Cfg.h`            | 提供通用配置宏或默认配置参数（如最大通道数、中断优先级）。|
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/src/Adc_PBcfg.c`          | 实现板级配置数据（如通道映射、硬件参数）。               |
| `samples/Adc/src/Adc_Cmd.c`                   | ADC sample 代码                                                       |


## 应用sample

`AdcTest` 命令用于对设备执行 ADC（模数转换器）采样测试。它可以读取特定通道或多个通道的的ADC值，取值并显示结果（以原始值和毫伏 (mv) 格式）。



### 使用示例


- 语法

```bash
AdcTest [ADC 通道]
```

ADC 通道 (可选): 要读取的特定 ADC 通道。如果未提供，该命令将读取多个通道。

- 示例

读取通道 1 的 ADC 值：
```bash
D-Robotics:/$ Adc_Test 1
[052.860562 0]--------------Adc_PrivateApiTest start-----------!
[052.876472 0]AdcCurrentValue [1]: 2404 -> 1056 mv
[052.869226 0]--------------Adc_PrivateApiTest end!-----------
```


读取所有通道的 ADC 值：

```bash
D-Robotics:/$ Adc_Test
[038.836359 0]--------------Adc_PrivateApiTest start-----------!
[038.852268 0]AdcCurrentValue [0]: 1117 -> 490 mv
[038.852648 0]BoradIdMsb code: 6!
[038.853028 0]
[038.853212 0]AdcCurrentValue [1]: 2393 -> 1051 mv
[038.854451 0]BoradIdLsb code: 10!
[038.854842 0]
[038.855026 0]AdcCurrentValue [2]: 1754 -> 770 mv
[038.856320 0]DDR TYPE code: 8!
[038.856634 0]
[038.856819 0]AdcCurrentValue [3]: 1725 -> 758 mv
[038.858210 0]
[038.858306 0]AdcCurrentValue [4]: 630 -> 276 mv
[038.858760 0]
[038.858945 0]AdcCurrentValue [5]: 2515 -> 1105 mv
[038.860273 0]
[038.860391 0]AdcCurrentValue [6]: 2492 -> 1095 mv
[038.860925 0]
[038.861109 0]AdcCurrentValue [7]: 2156 -> 947 mv
[038.862341 0]
[038.862525 0]AdcCurrentValue [8]: 2163 -> 950 mv
[038.863067 0]
[038.863252 0]AdcCurrentValue [9]: 2161 -> 949 mv
[038.864483 0]
[038.864667 0]AdcCurrentValue [10]: 2169 -> 953 mv
[038.865220 0]
[038.866262 0]AdcCurrentValue [11]: 2223 -> 977 mv
[038.866665 0]
[038.866850 0]AdcCurrentValue [12]: 1837 -> 807 mv
[038.868234 0]
[038.868331 0]AdcCurrentValue [13]: 2101 -> 923 mv
[038.868825 0]
[038.868999 0]PASS.
[038.869226 0]--------------Adc_PrivateApiTest end!-----------

```


### 应用程序接口

#### void Adc_Init(const Adc_ConfigType* ConfigPtr)

```shell
Description：Initializes the ADC hardware units and driver.

Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```



#### Std_ReturnType Adc_SetupResultBuffer(Adc_GroupType Group, const Adc_ValueGroupType* DataBufferPtr)

```shell
Description：Initializes the group specific ADC result buffer pointer as configured
             to point to the pDataBufferPtr address which is passed as parameter.

Sync/Async：Synchronous
Parameters(in)
    Group: Numeric ID of requested ADC channel group.
    DataBufferPtr: pointer to result data buffer.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: result buffer pointer initialized correctly
    E_NOT_OK: operation failed or development error occurred
```

#### void Adc_DeInit(void)

```shell
Description：Returns all ADC HW Units to a state comparable to their power on reset state.

Sync/Async：Synchronous
Parameters(in)
    ConfigPtr: Pointer to configuration set in Variant PB
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Adc_StartGroupConversion(Adc_GroupType Group)

```shell
Description：Starts the conversion of all channels of the requested ADC Channel group.

Sync/Async：Synchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Adc_ReadGroup(Adc_GroupType Group, Adc_ValueGroupType* DataBufferPt)

```shell
Description：Reads the group conversion result of the last completed conversion round of the requested group
             and stores the channel values starting at the DataBufferPtr address.
             The group channel values are stored in ascending channel number order
             (in contrast to the storage layout of the result buffer if streaming access is configured).

Sync/Async：Synchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    DataBufferPtr: ADC results of all channels of the selected group are stored in the data buffer
                   addressed with the pointer.
Return value：Std_ReturnType
    E_OK: Aresults are available and written to the data buffer
    E_NOT_OK: no results are available or development error occurred
```

#### void Adc_EnableHardwareTrigger(Adc_GroupType Group)

```shell
Description：Enables the hardware trigger for the requested ADC Channel group.

Sync/Async：Asynchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    DataBufferPtr: ADC results of all channels of the selected group are stored
                   in the data buffer addressed with the pointer.
Return value：None
```

#### void Adc_DisableHardwareTrigger(Adc_GroupType Group)

```shell
Description：Disables the hardware trigger for the requested ADC Channel group.

Sync/Async：Asynchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Adc_EnableGroupNotification(Adc_GroupType Group)

```shell
Description：Enables the notification mechanism for the requested ADC Channel group.

Sync/Async：Asynchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Adc_DisableGroupNotification(Adc_GroupType Group)

```shell
Description：Disables the notification mechanism for the requested ADC Channel group.

Sync/Async：Asynchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Adc_StatusType Adc_GetGroupStatus(Adc_GroupType Group)

```shell
Description：Returns the conversion status of the requested ADC Channel group.

Sync/Async：Asynchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Adc_StatusType
	Conversion status for the requested group.
```

#### Adc_StreamNumSampleType Adc_GetStreamLastPointer(Adc_GroupType Group, Adc_ValueGroupType** PtrToSamplePtr)

```shell
Description：Returns the number of valid samples per channel, stored in the result buffer.
             Reads a pointer, pointing to a position in the group result buffer.
             With the pointer position, the results of all group channels of the last
             completed conversion round can be accessed.
             With the pointer and the return value, all valid group conversion results can
             be accessed.

Sync/Async：Synchronous
Parameters(in)
    Group: Numeric ID of requested ADC Channel group.
Parameters(inout)
    None
Parameters(out)
    PtrToSamplePtr: Pointer to result buffer pointer.
Return value：Adc_StreamNum SampleType
	Number of valid samples per channel.
```

#### void Adc_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description：Returns the version information of this module.

Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    versioninfo: Pointer to where to store the version information of this module.
Return value：None

```

#### Std_ReturnType Adc_SetPowerState(Adc_PowerStateRequestResultType* Result)

```shell
Description：This API configures the Adc module so that it enters the already prepared
             power state, chosen between a predefined set of configured ones.

Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    Result: If the API returns E_OK:
        ADC_SERVICE_ACCEPTED: Power state change executed.
    If the API returns E_NOT_OK:
        ADC_NOT_INIT: ADC Module not initialized.
        ADC_SEQUENCE_ERROR: wrong API call sequence.
        ADC_HW_FAILURE: the HW module has a failure which prevents it to enter the required power state.
Return value：Std_ReturnType
    E_OK: Power Mode changed
    E_NOT_OK: request rejected
```

#### Std_ReturnType Adc_GetCurrentPowerState(Adc_PowerStateType* CurrentPowerState, Adc_PowerStateRequestResultType* Result)

```shell
Description：This API returns the current power state of the ADC HW unit.


Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    CurrentPowerState: The current power mode of the ADC HW Unit is returned in this parameter
    Result: If the API returns E_OK: ADC_SERVICE_ACCEPTED: Current power mode was returned
            If the API returns E_NOT_OK: ADC_NOT_INIT: ADC Module not initialized.
Return value：Std_ReturnType
    E_OK: Mode could be read
    E_NOT_OK: request rejected
```

#### Std_ReturnType Adc_GetTargetPowerState(Adc_PowerStateType* TargetPowerState, Adc_PowerStateRequestResultType* Result)

```shell
Description：This API returns the Target power state of the ADC HW unit.


Sync/Async：Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    CurrentPowerState: The current power mode of the ADC HW Unit is returned in this parameter
    Result: If the API returns E_OK: ADC_SERVICE_ACCEPTED: Current power mode was returned
            If the API returns E_NOT_OK: ADC_NOT_INIT: ADC Module not initialized.
Return value：Std_ReturnType
    E_OK: Mode could be read
    E_NOT_OK: request rejected
```

#### Std_ReturnType Adc_PreparePowerState(Adc_PowerStateType PowerState, Adc_PowerStateRequestResultType* Result)

```shell
Description：This API returns the Target power state of the ADC HW unit.

Sync/Async：Synchronous
Parameters(in)
    PowerState: The target power state intended to be attained
Parameters(inout)
    None
Parameters(out)
    Result:
    If the API returns E_OK:
        ADC_SERVICE_ACCEPTED: ADC Module power state preparation was started.
    If the API returns E_NOT_OK:
        ADC_NOT_INIT: ADC Module not initialized.
        ADC_SEQUENCE_ERROR: wrong API call sequence (Current Power State = Target Power State).
        ADC_POWER_STATE_NOT_SUPP: ADC Module does not support the requested power state.
        ADC_TRANS_NOT_POSSIBLE: ADC Module cannot transition directly from the current power
                                state to the requested power state or the HW peripheral is still busy.
Return value：Std_ReturnType
    E_OK: Mode could be read
    E_NOT_OK: request rejected
```

#### Std_ReturnType Adc_PreparePowerState(Adc_PowerStateType PowerState, Adc_PowerStateRequestResultType* Result)

```shell
Description：This API starts the needed process to allow the ADC HW module to
             enter the requested power state.

Sync/Async：Synchronous
Parameters(in)
    PowerState: The target power state intended to be attained
Parameters(inout)
    None
Parameters(out)
    Result:
    If the API returns E_OK:
        ADC_SERVICE_ACCEPTED: ADC Module power state preparation was started.
    If the API returns E_NOT_OK:
        ADC_NOT_INIT: ADC Module not initialized.
        ADC_SEQUENCE_ERROR: wrong API call sequence (Current Power State = Target Power State).
        ADC_POWER_STATE_NOT_SUPP: ADC Module does not support the requested power state.
        ADC_TRANS_NOT_POSSIBLE: ADC Module cannot transition directly from the current power
                                state to the requested power state or the HW peripheral is still busy.
Return value：Std_ReturnType
    E_OK: Mode could be read
    E_NOT_OK: request rejected
```

#### void Adc_EnableWdgNotification(Adc_ChannelType ChannelId)

```shell
Description：Enable notification of a channel that has watchdog functionality
             configured at initialization

Parameters(in)
    Adc_ChannelType: Symbolic name of channel
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Adc_DisableWdgNotification(Adc_ChannelType ChannelId)

```shell
Description：Disable notification of a channel that has watchdog functionality
             configured at initialization

Parameters(in)
    Adc_ChannelType: Symbolic name of channel
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```
