---
sidebar_position: 7
---

# ADC Usage Guide

## Hardware Support

The S100 ADC has one ADC hardware unit, containing a total of 15 channels: channel0 to channel13 and channel15.

- ADC measurable voltage range: 100 mV – 1700 mV.
- Only one group is allowed under hardware-triggered or inject mode.
- ADC calibration must be performed when the temperature changes by more than 20°C.
- ADC user interface usage must ensure that the power-on self-test has completed before proceeding.


## Software Driver

There are actually two sets of ADC drivers in the code, with the following differences:

- **Standard ADC Driver (Main ADC Driver)**
    - Located in the `McalCdd/Adc` directory, includes a complete ADC module implementation with files such as `Adc.h`, `Adc.c`, `Adc_Lld.h`, and `Adc_Lld.c`.
    - Provides full ADC functionality.

- **Private ADC Driver**
    - Located in the `McalCdd/Adc` directory, includes `Adc_Private.h` and `Adc_Private.c`.
    - Provides simplified interfaces specific for internal use.

### Usage Flow

- Typical usage flow for the Standard ADC Driver:
    ```c
    // 1. Initialize ADC module
    Adc_Init(&Adc_Config);
    // 2. Set up result buffer
    Adc_SetupResultBuffer(AdcGroup_0, dataBuffer);
    // 3. Start conversion
    Adc_StartGroupConversion(AdcGroup_0);
    // 4. Read results
    Adc_ReadGroup(AdcGroup_0, dataBuffer);
    // 5. Stop conversion
    Adc_StopGroupConversion(AdcGroup_0);
    // 6. De-initialize
    Adc_DeInit();
    ```

- Usage flow for the Private ADC Driver:
    ```c
    // 1. Initialize ADC hardware
    Adc_Private_Init(0);
    // 2. Read result from a specific channel
    Adc_Private_ReadChannelResult(0, channel, &result);
    // 3. De-initialize
    Adc_Private_DeInit(0);
    ```

### Key Differences

| Feature               | Standard ADC Driver                                      | Private ADC Driver                          |
|-----------------------|----------------------------------------------------------|---------------------------------------------|
| Complexity            | Full-featured ADC driver implementation                  | Simplified interface with limited features  |
| Configuration Method  | Uses complete configuration structures (e.g., Adc_GroupsCfg) | Direct hardware register manipulation     |
| API Richness          | Provides comprehensive ADC APIs                          | Offers only basic initialization and read functions |
| Interrupt Support     | Full interrupt and callback mechanism                    | No interrupt usage                          |
| Conversion Mode       | Single and continuous conversions                        | Supports only single conversion             |
| Trigger Mode          | Hardware and software triggers                           | Supports only software trigger              |
| Threshold Checking    | Software and hardware threshold checking                 | No threshold checking support               |
| Injected Conversion   | Supports both injected and normal conversions            | Supports only normal conversion             |
| Error Handling        | Comprehensive error detection and reporting              | Basic error handling                        |



## Code Paths

| **File Path**                                                | **Purpose**                                                                 |
|--------------------------------------------------------------|-----------------------------------------------------------------------------|
| `McalCdd/Adc/inc/Adc.h`                                      | Public API interface for upper-layer calls.                                 |
| `McalCdd/Adc/inc/Adc_Lld.h`                                  | Declares low-level hardware operation functions.                            |
| `McalCdd/Adc/inc/Adc_Private.h`                              | Defines private structures, macros, and function declarations.              |
| `McalCdd/Adc/src/Adc.c`                                      | Implements public APIs, calling low-level functions.                        |
| `McalCdd/Adc/src/Adc_Lld.c`                                  | Implements low-level hardware operations by directly configuring registers. |
| `McalCdd/Adc/src/Adc_Private.c`                              | Implements private functions supporting internal driver logic.              |
| `McalCdd/Common/Register/inc/Adc_Register.h`                 | Defines ADC peripheral register addresses and bit fields.                   |
| `Platform/Schm/SchM_Adc.h`                                   | Manages ADC access permissions and resource protection (e.g., interrupt safety). |
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/inc/Adc_PBcfg.h`     | Defines board-level peripheral configuration parameters (e.g., channels, sampling rate). |
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/inc/Adc_Cfg.h`       | Provides general configuration macros or default parameters (e.g., max channel count, interrupt priority). |
| `Config/McalCdd/gen_s100_sip_B_mcu1/Adc/src/Adc_PBcfg.c`     | Implements board-level configuration data (e.g., channel mapping, hardware parameters). |
| `samples/Adc/src/Adc_Cmd.c`                                  | Sample code for software-triggered single-shot ADC sampling using Adc_Private; suitable for simple scenarios. |
| `samples/Adc/src/Adc_SoftTrigerContinuous.c`                 | Sample code for software-triggered continuous ADC sampling using standard APIs; suitable for complex scenarios. |



## Application Examples

### ADC Software-Triggered Single Conversion Application

The `AdcTest` application performs a single-shot ADC sampling test on the device. Using the `Adc_Private` implementation, it reads ADC values from a specific channel or multiple channels and displays the results in both raw and millivolt (mV) formats.


#### Usage Example


- Syntax

```bash
AdcTest [ADC Channel]
```

ADC Channel (optional): The specific ADC channel to read. If omitted, the command reads multiple channels.

- Example

Read ADC value from channel 1:
```bash
D-Robotics:/$ Adc_Test 1
[052.860562 0]--------------Adc_PrivateApiTest start-----------!
[052.876472 0]AdcCurrentValue [1]: 2404 -> 1056 mv
[052.869226 0]--------------Adc_PrivateApiTest end!-----------
```


Read ADC values from all channels:

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

### ADC Software-Triggered Continuous Conversion Application

The ADC software-triggered continuous sampling application is implemented based on the standard ADC driver. It features automatic repeated conversions—immediately starting the next conversion upon completion of the current one without requiring additional triggers. This is suitable for scenarios requiring continuous signal monitoring, though it incurs relatively higher power consumption due to continuous operation.


#### Key Configuration
```c
// McalCdd/gen_s100_sip_B_mcu1/Adc/src/Adc_PBcfg.c
static const Adc_GroupCfg Adc_GroupsCfg[] =
{
    /**< @brief Group0 -- Logical Unit Id 0 -- Hardware Unit ADC0 */
    {
        /**< @brief Index of group */
        0U, /* GroupId */
        /**< @brief ADC Logical Unit Id that the group belongs to */
        (uint8)0, /* UnitId */
        /**< @brief Access mode */
        ADC_ACCESS_MODE_SINGLE, /* AccessMode */
        /**< @brief Conversion mode */
        ADC_CONV_MODE_CONTINUOUS, /* Mode */  // Use continuous conversion mode
        /**< @brief Conversion type */
        ADC_NORMAL_CONV, /* Type */ // Can select normal or injected conversion
#if (ADC_PRIORITY_IMPLEMENTATION != ADC_PRIORITY_NONE)
        /**< @brief Priority configured */
        (Adc_GroupPriorityType)ADC_GROUP_PRIORITY(0), /* Priority */
#endif /* ADC_PRIORITY_IMPLEMENTATION != ADC_PRIORITY_NONE */
        /**< @brief Trigger source configured */
        ADC_TRIGG_SRC_SW, /* TriggerSource */  // Software trigger
#if (STD_ON == ADC_HW_TRIGGER_API)
        /**< @brief Hardware trigger source for the group */
        0U, /* HwTriggerSource */
#endif /* (STD_ON == ADC_HW_TRIGGER_API) */
#if (STD_ON == ADC_GRP_NOTIF_CAPABILITY)
        /**< @brief Notification function */
        Adc_TestNormal_Notification_0, /* Notification */ // Notification function to inform upper layer that conversion is complete
#endif /* (STD_ON == ADC_GRP_NOTIF_CAPABILITY) */
    ............
```/**< @brief Enables or Disables the ADC and DMA interrupts */
        (uint8)(STD_ON), /* AdcWithoutInterrupt */  // STD_ON: non-interrupt mode; STD_OFF: interrupt mode; S100 uses non-interrupt mode by default
#if (ADC_ENABLE_LIMIT_CHECK == STD_ON)
        /**< @brief Enables or disables the usage of limit checking for an ADC group. */
        (boolean)FALSE, /* AdcGroupLimitcheck */
#endif /* (STD_ON == ADC_ENABLE_LIMIT_CHECK) */
        { { 0x3FFFU } }, /* AssignedChannelMask */
#if (ADC_SET_ADC_CONV_TIME_ONCE == STD_OFF)
        &AdcLldGroupConfig_0 /* AdcLldGroupConfigPtr */
#endif /* (ADC_SET_ADC_CONV_TIME_ONCE == STD_OFF) */
    }
};
```

#### Usage Example

- Syntax

```
Adc_TestNormal [Action]
```

##### Non-interrupt Mode
:::tip
Note: Set the `AdcWithoutInterrupt` field in `Adc_GroupsCfg` to `STD_ON`.
:::

Step 1: Start continuous ADC sampling
```
D-Robotics:/$ Adc_TestNormal start
[0129.823385 0]Adc test running...
```

Step 2: Read sampling results
```
D-Robotics:/$ Adc_TestNormal read noirq
[0112.002331 0]not use irq
[0112.002480 0]##############################
[0112.002970 0] ResultBuffer0[0]: 1103 : 484 mv
[0112.003502 0] ResultBuffer0[1]: 2346 : 1031 mv
[0112.004044 0] ResultBuffer0[2]: 1728 : 759 mv
[0112.004576 0] ResultBuffer0[3]: 1704 : 749 mv
[0112.005108 0] ResultBuffer0[4]: 828 : 363 mv
[0112.005629 0] ResultBuffer0[5]: 3411 : 1499 mv
[0112.006171 0] ResultBuffer0[6]: 3180 : 1397 mv
[0112.006713 0] ResultBuffer0[7]: 3051 : 1341 mv
[0112.007256 0] ResultBuffer0[8]: 2935 : 1290 mv
[0112.007798 0] ResultBuffer0[9]: 2820 : 1239 mv
[0112.008341 0] ResultBuffer0[10]: 2731 : 1200 mv
[0112.008894 0] ResultBuffer0[11]: 2645 : 1162 mv
[0112.009448 0] ResultBuffer0[12]: 1854 : 814 mv
[0112.009990 0] ResultBuffer0[13]: 1798 : 790 mv
[0112.010533 0]==============================

```

Step 3: Stop continuous ADC sampling
```
D-Robotics:/$ Adc_TestNormal stop
[0268.403214 0]Adc test exit.
```


##### Interrupt Mode
:::tip
Note: Set the `AdcWithoutInterrupt` field in `Adc_GroupsCfg` to `STD_OFF`.
:::

Step 1: Start continuous ADC sampling
```
D-Robotics:/$ Adc_TestNormal start
[0129.823385 0]Adc test running...
```

Step 2: Read sampling results
```
D-Robotics:/$ Adc_TestNormal read irq
[0195.226347 0]##############################
[0195.228233 0] ResultBuffer0[0]: 1103 : 484 mv
[0195.230181 0] ResultBuffer0[1]: 2346 : 1031 mv
[0195.232191 0] ResultBuffer0[2]: 1727 : 759 mv
[0195.234261 0] ResultBuffer0[3]: 1705 : 749 mv
[0195.236271 0] ResultBuffer0[4]: 827 : 363 mv
[0195.238261 0] ResultBuffer0[5]: 3396 : 1492 mv
[0195.240231 0] ResultBuffer0[6]: 3229 : 1419 mv
[0195.242241 0] ResultBuffer0[7]: 3076 : 1352 mv
[0195.244292 0] ResultBuffer0[8]: 2953 : 1298 mv
[0195.246263 0] ResultBuffer0[9]: 2841 : 1248 mv
[0195.248331 0] ResultBuffer0[10]: 2735 : 1202 mv
[0195.250341 0] ResultBuffer0[11]: 2657 : 1167 mv
[0195.252392 0] ResultBuffer0[12]: 1856 : 815 mv
[0195.254442 0] ResultBuffer0[13]: 1799 : 790 mv
[0195.256431 0]==============================
```

Step 3: Stop continuous ADC sampling
```
D-Robotics:/$ Adc_TestNormal stop
[0268.403214 0]Adc test exit.
```

### Application Programming Interface

#### void Adc_Init(const Adc_ConfigType* ConfigPtr)

```shell
Description: Initializes the ADC hardware units and driver.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```



#### Std_ReturnType Adc_SetupResultBuffer(Adc_GroupType Group, const Adc_ValueGroupType* DataBufferPtr)

```shell
Description: Initializes the group-specific ADC result buffer pointer as configured
             to point to the address provided by the pDataBufferPtr parameter.

Sync/Async: Synchronous
Parameters(in)
    Group: Numeric ID of the requested ADC channel group.
    DataBufferPtr: Pointer to the result data buffer.
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: Result buffer pointer initialized correctly
    E_NOT_OK: Operation failed or a development error occurred
```

#### void Adc_DeInit(void)

```shell
Description: Returns all ADC hardware units to a state equivalent to their power-on reset state.

Sync/Async: Synchronous
Parameters(in)
    ConfigPtr: Pointer to the configuration set in Variant PB
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Adc_StartGroupConversion(Adc_GroupType Group)

```shell
Description: Starts the conversion of all channels in the requested ADC channel group.

Sync/Async: Synchronous
Parameters(in)
    Group: Numeric ID of the requested ADC channel group.
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Adc_ReadGroup(Adc_GroupType Group, Adc_ValueGroupType* DataBufferPtr)

```shell
Description: Reads the group conversion results from the last completed conversion round of the requested group
             and stores the channel values starting at the address pointed to by DataBufferPtr.
             The group channel values are stored in ascending channel number order
             (in contrast to the storage layout of the result buffer if streaming access is configured).

Sync/Async: Synchronous
Parameters(in)
    Group: Numeric ID of the requested ADC channel group.
Parameters(inout)
    None
Parameters(out)
    DataBufferPtr: ADC results of all channels in the selected group are stored in the data buffer
                   addressed by this pointer.
Return value: Std_ReturnType
    E_OK: Results are available and written to the data buffer
    E_NOT_OK: No results are available or a development error occurred
```

#### void Adc_EnableHardwareTrigger(Adc_GroupType Group)

```shell
Description: Enables the hardware trigger for the requested ADC channel group.

Sync/Async: Asynchronous
Parameters(in)
    Group: Numeric ID of the requested ADC channel group.
Parameters(inout)
    None
```Parameters(out)  
    DataBufferPtr: ADC results of all channels of the selected group are stored  
                   in the data buffer addressed with the pointer.  
Return value: None  
```  

#### void Adc_DisableHardwareTrigger(Adc_GroupType Group)  

```shell  
Description: Disables the hardware trigger for the requested ADC Channel group.  

Sync/Async: Asynchronous  
Parameters(in)  
    Group: Numeric ID of requested ADC Channel group.  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: None  
```  

#### void Adc_EnableGroupNotification(Adc_GroupType Group)  

```shell  
Description: Enables the notification mechanism for the requested ADC Channel group.  

Sync/Async: Asynchronous  
Parameters(in)  
    Group: Numeric ID of requested ADC Channel group.  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: None  
```  

#### void Adc_DisableGroupNotification(Adc_GroupType Group)  

```shell  
Description: Disables the notification mechanism for the requested ADC Channel group.  

Sync/Async: Asynchronous  
Parameters(in)  
    Group: Numeric ID of requested ADC Channel group.  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: None  
```  

#### Adc_StatusType Adc_GetGroupStatus(Adc_GroupType Group)  

```shell  
Description: Returns the conversion status of the requested ADC Channel group.  

Sync/Async: Asynchronous  
Parameters(in)  
    Group: Numeric ID of requested ADC Channel group.  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: Adc_StatusType  
	Conversion status for the requested group.  
```  

#### Adc_StreamNumSampleType Adc_GetStreamLastPointer(Adc_GroupType Group, Adc_ValueGroupType** PtrToSamplePtr)  

```shell  
Description: Returns the number of valid samples per channel, stored in the result buffer.  
             Reads a pointer, pointing to a position in the group result buffer.  
             With the pointer position, the results of all group channels of the last  
             completed conversion round can be accessed.  
             With the pointer and the return value, all valid group conversion results can  
             be accessed.  

Sync/Async: Synchronous  
Parameters(in)  
    Group: Numeric ID of requested ADC Channel group.  
Parameters(inout)  
    None  
Parameters(out)  
    PtrToSamplePtr: Pointer to result buffer pointer.  
Return value: Adc_StreamNumSampleType  
	Number of valid samples per channel.  
```  

#### void Adc_GetVersionInfo(Std_VersionInfoType* versioninfo)  

```shell  
Description: Returns the version information of this module.  

Sync/Async: Synchronous  
Parameters(in)  
    None  
Parameters(inout)  
    None  
Parameters(out)  
    versioninfo: Pointer to where to store the version information of this module.  
Return value: None  

```  

#### Std_ReturnType Adc_SetPowerState(Adc_PowerStateRequestResultType* Result)  

```shell  
Description: This API configures the Adc module so that it enters the already prepared  
             power state, chosen between a predefined set of configured ones.  

Sync/Async: Synchronous  
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
Return value: Std_ReturnType  
    E_OK: Power Mode changed  
    E_NOT_OK: request rejected  
```  

#### Std_ReturnType Adc_GetCurrentPowerState(Adc_PowerStateType* CurrentPowerState, Adc_PowerStateRequestResultType* Result)  

```shell  
Description: This API returns the current power state of the ADC HW unit.  

Sync/Async: Synchronous  
Parameters(in)  
    None  
Parameters(inout)  
    None  
Parameters(out)  
    CurrentPowerState: The current power mode of the ADC HW Unit is returned in this parameter  
    Result: If the API returns E_OK: ADC_SERVICE_ACCEPTED: Current power mode was returned  
            If the API returns E_NOT_OK: ADC_NOT_INIT: ADC Module not initialized.  
Return value: Std_ReturnType  
    E_OK: Mode could be read  
    E_NOT_OK: request rejected  
```  

#### Std_ReturnType Adc_GetTargetPowerState(Adc_PowerStateType* TargetPowerState, Adc_PowerStateRequestResultType* Result)  

```shell  
Description: This API returns the Target power state of the ADC HW unit.  

Sync/Async: Synchronous  
Parameters(in)  
    None  
Parameters(inout)  
    None  
Parameters(out)  
    TargetPowerState: The target power mode of the ADC HW Unit is returned in this parameter  
    Result: If the API returns E_OK: ADC_SERVICE_ACCEPTED: Target power mode was returned  
            If the API returns E_NOT_OK: ADC_NOT_INIT: ADC Module not initialized.  
Return value: Std_ReturnType  
    E_OK: Mode could be read  
    E_NOT_OK: request rejected  
```  

#### Std_ReturnType Adc_PreparePowerState(Adc_PowerStateType PowerState, Adc_PowerStateRequestResultType* Result)  

```shell  
Description: This API starts the needed process to allow the ADC HW module to  
             enter the requested power state.  

Sync/Async: Synchronous  
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
Return value: Std_ReturnType  
    E_OK: Preparation initiated successfully  
    E_NOT_OK: request rejected  
```  

#### Std_ReturnType Adc_PreparePowerState(Adc_PowerStateType PowerState, Adc_PowerStateRequestResultType* Result)  

```shell  
Description: This API starts the needed process to allow the ADC HW module to  
             enter the requested power state.  

Sync/Async: Synchronous  
Parameters(in)PowerState: The target power state intended to be attained
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
Return value: Std_ReturnType
    E_OK: Mode could be read
    E_NOT_OK: request rejected
```

#### void Adc_EnableWdgNotification(Adc_ChannelType ChannelId)

```shell
Description: Enable notification of a channel that has watchdog functionality
             configured at initialization

Parameters(in)
    Adc_ChannelType: Symbolic name of channel
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Adc_DisableWdgNotification(Adc_ChannelType ChannelId)

```shell
Description: Disable notification of a channel that has watchdog functionality
             configured at initialization

Parameters(in)
    Adc_ChannelType: Symbolic name of channel
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```