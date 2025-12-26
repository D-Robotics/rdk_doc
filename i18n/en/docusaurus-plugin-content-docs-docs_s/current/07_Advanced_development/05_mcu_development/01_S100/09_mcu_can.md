---
sidebar_position: 9
---

# CAN Usage Guide

## Basic Overview

- Maximum number of usable CAN controllers: 10.
- Maximum CAN transmission rate: 8 Mbps. (Limited by transceiver baud rate constraints; currently, the lab has only tested and verified up to 5 Mbps.)
- Number of Blocks allocated in RAM per controller:
    - CAN0~CAN3: 4 Blocks (variable payload);
    - CAN4~CAN9: 4 Blocks (variable payload) + 4 Blocks (fixed payload).
- Maximum number of Mailboxes supported per controller: 128.
- Each controller supports one RxFIFO with the following FIFO depth:
    - CAN0~CAN3: 8 × 64 bytes;
    - CAN4~CAN9: 32 × 64 bytes.
- TTController is not supported, i.e., TTCAN (a higher-layer protocol based on CAN bus) is unsupported.
- CAN supports multi-core usage: different CAN controllers can be bound to different cores, but multiple cores cannot simultaneously access the same CAN controller.

## Software Architecture

The CAN controllers of the S100 chip reside in the MCU domain and handle CAN data transmission and reception. Since perception-related applications run on the Acore, some CAN data must be forwarded to the Acore via an IPC (Inter-Processor Communication) mechanism. The architecture ensures transmission reliability, implementing mechanisms for data integrity verification, packet loss detection, and transmission timeout detection. Additionally, it addresses performance issues such as excessive CPU utilization on the MCU side caused by frequent forwarding of small data blocks, which could degrade MCU real-time performance.

The core workflow of the S100 CAN forwarding solution is as follows:
- First, the CAN2IPC module on the MCU side maps CAN channels to corresponding IPC channels. Then, the CANHAL module on the Acore side remaps these IPC channels to virtual CAN device channels. Finally, users retrieve data from the virtual CAN devices via APIs provided by CANHAL. Here, the CAN2IPC module is an MCU-side service, while the CANHAL module is a dynamic library provided to applications on the Acore side.
- The CAN2IPC module periodically collects CAN data from the MCU side, packages it according to a specified transmission protocol, and forwards it to the Acore via IPC. By default, channels 0–7 of IPC instance 0 are allocated for CAN data forwarding.
- The CANHAL module receives IPC data from the MCU side, parses it according to the specified transmission protocol, and enables application software to retrieve raw CAN frames via APIs.

![Architecture Diagram of CAN Data Transparent Transmission Between Acore and MCU](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_can.png)

As shown in the diagram above:
- Peripheral data is received by the MCU-domain CAN driver through the CAN PHY and controller hardware. The CAN driver then reports and caches the data in the hobot CANIF module.
- The CAN2IPC Service periodically fetches CAN frames from the CANIF module, packages them according to a reliable transmission protocol, and forwards them to the Acore via IPC.
- The CANHAL module receives IPC data from the MCU side, parses it according to the specified transmission protocol, and allows Acore applications to retrieve CAN frames via APIs provided by the CANHAL library.

Key features of this solution:
- Supports data integrity verification for transparent transmission.
- Supports packet loss detection during transparent transmission.
- Supports transmission timeout detection. When the MCU-side CAN2IPC forwards data, it timestamps each packet with the MCU’s local time. Upon receiving the data, the Acore-side CANHAL reads the Acore’s timestamp and triggers an alert if the transmission exceeds the timeout threshold. Note: MCU RTC time and Acore NIC phc0 time must be synchronized beforehand.
- Supports parallel transmission across multiple CAN channels. Data from multiple CAN controllers on the MCU side can be simultaneously forwarded to the Acore, and Acore applications can read CAN data from different channel IDs via CANHAL.
- Since CANHAL relies on IPC for inter-core communication, and IPC currently does not support multiple processes or threads reading from/writing to the same channel, CANHAL does not support this feature either.

## Hardware Connection Instructions

- CAN physical layer topologies are primarily divided into closed-loop (bus) and open-loop networks—closed-loop is suitable for high-speed communication, while open-loop is better for long-distance communication. **By default, the S100 sample uses a closed-loop bus network architecture**.

- CAN bus pins are located on the S100 MCU expansion board, providing 5 CAN interfaces via green screw-type 3-pin connectors. Pin 1 (marked with a triangle) is GND, the middle pin is CAN_L, and the remaining pin is CAN_H.

![MCU CAN Physical Diagram](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_can_phy-en.jpg)

- The MCU daughterboard uses 2-pin jumpers to optionally connect a 120-ohm resistor between CAN_H and CAN_L. Inserting the jumper connects the resistor, providing the termination impedance required for closed-loop networks; removing the jumper disconnects the termination resistor, suitable for open-loop networks or repeater node scenarios.

![MCU CAN Schematic Diagram](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_can_sche-en.jpg)

Using two 120-ohm resistors in a CAN closed-loop network is the standard CAN bus configuration. Below are examples using the S100 to illustrate correct resistor placement:

:::info Note
In general, open-loop network configurations do not require 120-ohm resistors, whereas closed-loop configurations require exactly **two** 120-ohm resistors in total.
:::

- When using an open-loop network, ensure CAN_H and CAN_L lines are correctly connected, and **do not** install jumpers on the used CAN channels (i.e., do not connect 120-ohm resistors in the network).
- If connecting S100’s CAN5 and CAN6 to form an internal two-node closed-loop network, ensure CAN_H and CAN_L are correctly wired, and install jumpers on the pins behind the CAN5 and CAN6 terminal blocks (i.e., insert two 120-ohm resistors into the network).
- If connecting S100’s CAN5~CAN9 to form an internal multi-node closed-loop network, ensure proper CAN_H/CAN_L wiring and install jumpers on **any two** of the terminal blocks—**never install more than two**, as this may cause unpredictable issues.
- If connecting any one of S100’s CAN5~CAN9 controllers to external CAN devices to form an external closed-loop network, ensure correct CAN_H/CAN_L wiring, install a jumper on the pins behind the RDK’s CAN controller terminal block, and connect one 120-ohm resistor at the other end of the external device in the network.

## CAN Filter Configuration

Up to 128 filters can be configured for standard frames, and up to 64 for extended frames. Supported filter types include:
- **ONE_ID_FILTER**: Specifies an ID and uses a MASK to ignore certain bits during filtering.
- **RANGE_ID_FILTER**: Filters based on an ID range.
- **TWO_ID_FILTER**: Specifies two IDs for filtering.

### Filter Identification

The filter type is determined by the two most significant bits of `u32HwFilterCode`:
- `0b00`: ONE_ID_FILTER  
- `0b01`: RANGE_ID_FILTER  
- `0b10`: TWO_ID_FILTER  

```c
/**
 * @struct Can_HwFilterType
 * @brief CAN Hardware Filter
 * @NO{S01E03C01}
 */
typedef struct Can_HwFilterType
{
    const uint32 u32HwFilterCode;   /**< @brief Specifies (together with the filter mask) the identifiers range that passes the hardware filter. */
    const uint32 u32HwFilterMask;   /**< @brief Describes a mask for hardware-based filtering of CAN identifiers. */
} Can_HwFilterType;
```

- Configuration example:
    - This is the filter configuration for CAN7, containing two filters.
    - The high 2 bits of the first element of Filter 0 are `01`, indicating range-based filtering.
    - Filtering for extended frames and standard frames is independent and does not interfere with each other.
    - All standard-frame filters (e.g., Filter 0 and Filter 1 in the example below) operate in an "OR" relationship: if at least one filter element matches, the CAN message is transferred to the enhanced RX FIFO memory.
    - Similarly, all extended-frame filters (e.g., Filter 2 and Filter 3 below) also use an "OR" relationship: if any filter element matches, the message is transferred to the enhanced RX FIFO.

```c
// Config/McalCdd/gen_s100_sip_B_mcu1/Can/src/Can_PBcfg.c
static const Can_HwFilterType Can_aHwFilter_Object7[4U] =
{
    { /* Standard frame configuration */
        (uint32)0x400007ffU,    // Standard frame: accepts messages with ID 0x0 ~ 0x7ff
        (uint32)0x00000000U
    },
    { /* Standard frame configuration */
        (uint32)0x400007ffU,   // Standard frame: accepts messages with ID 0x600 ~ 0x7ff
        (uint32)0x00000600U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts messages with ID 0x0 ~ 0x1fffffff
        (uint32)0x00000000U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts messages with ID 0x600 ~ 0x1fffffff
        (uint32)0x00000600U
    }
};
```

### ONE_ID_FILTER (Single-ID Filtering)

This is the most common filter type, using a filter code and mask. Pseudocode:

```c
if ((Received_ID & Filter_Mask) == (Filter_Code & Filter_Mask))
    // Accept the message
else
    // Discard the message
```

Example using Standard Frame Filter 0:

```c
// Config/McalCdd/gen_s100_sip_B_mcu1/Can/src/Can_PBcfg.c
static const Can_HwFilterType Can_aHwFilter_Object7[4U] =
{
    { /* Standard frame configuration */
        (uint32)0x00000400U,  // Accept only ID = 0x400 & 0x7ff = 0x400
        (uint32)0x000007ffU
    },
    {  /* Standard frame configuration */
        (uint32)0x400007ffU,  // Range filtering mode (mixed usage supported)
        (uint32)0x00000600U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts ID 0x0 ~ 0x1fffffff
        (uint32)0x00000000U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts ID 0x600 ~ 0x1fffffff
        (uint32)0x00000600U
    }
};
```

### RANGE_ID_FILTER (Range-Based Filtering)

In this mode, filtering uses range logic:

```c
if (id1 <= Received_ID <= id2)
    // Accept the message
else
    // Discard the message
```

This is the default and most commonly used filtering method on the S100 MCU. Example code:

```c
// Config/McalCdd/gen_s100_sip_B_mcu1/Can/src/Can_PBcfg.c
static const Can_HwFilterType Can_aHwFilter_Object7[4U] =
{
    { /* Standard frame configuration */
        (uint32)0x00000400U,  // Accept only ID = 0x400 & 0x7ff = 0x400
        (uint32)0x000007ffU
    },
    {  /* Standard frame configuration */
        (uint32)0x400007ffU,  // Range filtering mode (mixed usage supported)
        (uint32)0x00000600U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts ID 0x0 ~ 0x1fffffff
        (uint32)0x00000000U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame: accepts ID 0x600 ~ 0x1fffffff
        (uint32)0x00000600U
    }
};
```

### TWO_ID_FILTER (Dual-ID Filtering)

This type allows matching against two independent IDs:
- `id1`: First matching ID  
- `id2`: Second matching ID  
```c
if (Received_ID == id1 || Received_ID == id2)
    // Receive this message
else
    // Discard this message
```

Taking Standard Frame Filter 0 configuration as an example, the code is as follows:
```c
// Config/McalCdd/gen_s100_sip_B_mcu1/Can/src/Can_PBcfg.c
static const Can_HwFilterType Can_aHwFilter_Object7[4U]=
{
    { /* Standard frame configuration */
        (uint32)0x80000404U, // Only receive messages with ID 0x404
        (uint32)0x00000303U  // Only receive messages with ID 0x303
    },
    { /* Standard frame configuration */
        (uint32)0x400007ffU,  // Range-based filtering; mixed usage supported
        (uint32)0x00000600U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame configuration: receive messages with ID from 0x0 to 0x1fffffff
        (uint32)0x00000000U
    },
    { /* Extended frame configuration */
        (uint32)0x5fffffffU,   // Extended frame configuration: receive messages with ID from 0x600 to 0x1fffffff
        (uint32)0x00000600U
    }
};
```

:::tip
1. RDK S100 hardware and software support both extended and standard frames without requiring configuration changes.
2. RDK S100 hardware and software support independent filtering for extended and standard frames.
3. Pay attention to the ID length configuration. IDs exceeding the specified length will be truncated: extended frame IDs can be up to 29 bits (maximum value 0x1FFFFFFF), while standard frame IDs can be up to 11 bits (maximum value 0x7FF).
:::

## Bit Rate Configuration

The nominal bit timing of CAN can be divided into four segments:
1. **Synchronization Segment (sync_seg)**: Used for clock synchronization among nodes. All nodes detect signal edges within this segment. Its length is fixed at 1 time quantum (TQ).
2. **Propagation Segment (prop_seg)**: Compensates for signal propagation delay on the physical bus. Its length is adjustable to ensure proper signal transmission timing over the physical medium.
3. **Phase Buffer Segment 1 (phase_seg1)**: Adjusts phase errors to ensure accurate sampling point placement and supports resynchronization extension.
4. **Phase Buffer Segment 2 (phase_seg2)**: Also adjusts phase errors but can be shortened.

The sum of these segments determines the total CAN bit time. Different bit rates can be configured by adjusting the lengths of these segments.

Additionally, several important concepts exist:
1. **Synchronization Jump Width (SJW)**: The maximum time amount allowed for adjusting phase buffer segments during CAN bus synchronization. It compensates for clock deviations among nodes during hard synchronization and resynchronization, ensuring aligned sampling points.
2. **Transceiver Delay Compensation Offset**: Supported only in CAN FD, this fixed correction value compensates for transceiver loop delay and signal propagation time under high-speed data segment transmission in CAN FD mode.
3. **Sampling Point**: The precise moment within a bit time when the CAN controller samples the bus voltage level to determine the bit’s logical value (dominant 0 or recessive 1).

### Value Ranges and Formula Calculations
1. Sampling Point Calculation:  
   (sync_seg + prop_seg + phase_seg1) / (sync_seg + prop_seg + phase_seg1 + phase_seg2) × 100%
2. The synchronization segment is fixed at 1 TQ.
3. prop_seg + phase_seg1 > phase_seg2
4. SJW ≤ min(Phase_Seg1, Phase_Seg2)
5. When configuring bit rates of 5 Mbps or higher, delay compensation parameters must be set. The compensation offset is calculated as:  
   - TDC offset = (PropSeg + Seg1 + 1) × Fd Prescaler

### Example Configuration: Arbitration Segment at 1 Mbps, Data Segment at 5 Mbps
#### 1. Confirm Basic Parameters
- CAN Clock Frequency (CAN_CLK): 40 MHz  
- Target Bit Rate: 5 Mbps  
- Prescaler: 1 (no division)  
- Total TQ per Bit: TQ = CAN_CLK / (Bit Rate × Prescaler) = 40 MHz / (5 Mbps × 1) = 8 TQ  
- Time Quantum (Tq time): 1 / (40 MHz / prescaler) = 1/40 MHz = 25 ns

#### 2. Time Quantum (TQ) Allocation
Sync_Seg (fixed segment): 1 TQ (cannot be modified)

Remaining TQ allocation: Prop_Seg + Phase_Seg1 + Phase_Seg2 = 8 − 1 = 7 TQ

Here, we choose Prop_Seg = 1, Phase_Seg1 = 4, Phase_Seg2 = 2.  
Sampling Point = (Sync_Seg + Prop_Seg + Phase_Seg1) / Total TQ × 100% = (1 + 1 + 4) / 8 × 100% = 75%

#### 3. Transceiver Delay Compensation Offset

Offset = (Prop_Seg + Phase_Seg1 + 1) × Prescaler = (1 + 4 + 1) × 1 = 6 TQ

**Synchronization Jump Width (SJW) Setting**:

SJW must satisfy: SJW ≤ min(Phase_Seg1, Phase_Seg2) = min(4, 2) = 2

Thus, configure SJW = 2 TQ.

#### 4. Final Configuration Parameters

Using the same method, parameters for the 1 Mbps case can also be calculated. Note that some register values are automatically incremented by hardware, so the actual values written to registers must be decremented by 1. See the table below for details.

- **5 Mbps Data Segment Configuration (75% Sampling Point)**

| Parameter Name               | Value (TQ or Time) | Value to Write to Register |
|------------------------------|--------------------|----------------------------|
| Sync_Seg                     | 1 TQ               | Not written (fixed at 1)   |
| Prop_Seg                     | 1 TQ               | 1                          |
| Phase_Seg1                   | 4 TQ               | 3                          |
| Phase_Seg2                   | 2 TQ               | 1                          |
| Prescaler                    | 1                  | 0                          |
| SJW                          | 2 TQ               | 1                          |
| Transceiver Delay Compensation Offset | 6 TQ      | 6                          |

- **1 Mbps Arbitration Segment Configuration (80% Sampling Point)**

| Parameter Name        | Value (TQ or Time) | Value to Write to Register |
|-----------------------|--------------------|----------------------------|
| Sync_Seg              | 1 TQ               | Not written (fixed at 1)   |
| Prop_Seg              | 7 TQ               | 6                          |
| Phase_Seg1            | 8 TQ               | 7                          |
| Phase_Seg2            | 4 TQ               | 3                          |
| Prescaler             | 2                  | 1                          |
| SJW                   | 2 TQ               | 1                          |

#### 5. 8 Mbps Configuration (Special Case with 60% Sampling Point)

| Parameter Name          | Value (TQ or Time) | Value to Write to Register |
|-------------------------|--------------------|----------------------------|
| Sync_Seg                | 1 TQ               | Not written (fixed at 1)   |
| Prop_Seg                | 1 TQ               | 1                          |
| Phase_Seg1              | 1 TQ               | 0                          |
| Phase_Seg2              | 2 TQ               | 1                          |
| Prescaler               | 1                  | 0                          |
| SJW                     | 1 TQ               | 0                          |
| Transceiver Delay Compensation Offset | 3 TQ | 3                          |

#### 6. Update Results to Configuration File
Configuration file path:
```
${mcu_sdk}/Config/McalCdd/gen_s100_sip_B_mcu1/Can/src/Can_PBcfg.c
```

Two important baud-rate-related structures exist in the configuration file. Taking CAN5 as an example:

- **Can_aControllerConfig**: Configures the CAN controller. Each controller has a corresponding configuration entry.
```c
static const Can_ControllerConfigType Can_aControllerConfig[CAN_CONTROLLER_CONFIG_COUNT]=
{
    ...
 {
        /* Controller ID configured */
        (uint8)5U,
        /* Hw controller Offset */
        (uint8)5U,
        /* Base Address */
        FLEXCAN_5_BASE,
        /* Activation or not */
        (boolean)TRUE,
        /* Bus Off uses polling or not */
        (boolean)TRUE,
        /* Global mask of Legacy FIFO (not used) */
        (uint32)0xFFFFFFFFU,
        /* Acceptance Mode of Legacy FIFO (not used)*/
        CAN_LEGACY_FIFO_FORMAT_A,
        /* Legacy FIFO Warning Notification */
        NULL_PTR,
        /* Legacy FIFO Overflow Notification */
        NULL_PTR,
        /* Enhanced FIFO Overflow Notification */
        NULL_PTR,
        /* Error interrupt enable or not */
        (boolean)TRUE,
        /* Can Error Notification */
        Can_ErrorNotif,
        /* CanFd Error Notification */
        CanFd_ErrorNotif,
        /* Default Baudrate ID, 4--1M+5M 5--1M+8M */
        (uint16)4U,
         /* Baudrate config Count*/
        (uint16)6U,
        /* Pointer to baudrate config Structure */
        Can_aBaudrateConfig_Ctrl5,
        /* Pointer to LLD structure to IP config */
        &Flexcan_aCtrlConfigPB[5U],
        /* HwObject count */
        (uint8)9U,
        /* Point to group of HwObject that referenced to the current Controller */
        Can_apHwObject_Ctrl5
    },
    ...
}
```
- **Can_aBaudrateConfig_Ctrl5**: Defines the baud rate configuration for a specific controller. This is a large array where all parameters calculated above are written.
```c
static const Can_BaudrateConfigType Can_aBaudrateConfig_Ctrl5[6U]=
{
    {
        /*Enhance CBT support */
        (boolean)TRUE,
        /* Tx Bit Rate Switch or not */
        (boolean)TRUE,
        /* CanFd support */
        (boolean)TRUE,
        /* Nominal bit rate */ // Arbitration segment configuration
        {
            (uint8)6U, // Propagation Segment (prop_seg)
            (uint8)7U, // Phase Buffer Segment 1 (phase_seg1)
            (uint8)3U, // Phase Buffer Segment 2 (phase_seg2)
            (uint16)3U, // Prescaler
            (uint8)1U // Synchronization Jump Width (SJW)
        },
        /* Data bit rate */ // Data segment configuration
    {
            (uint8)3U,
            (uint8)3U,
            (uint8)1U,
            (uint16)3U,
            (uint8)1U
        },

        /* Tx Arbitration Start delay */
        (uint8)12U, // Transceiver Delay Compensation Offset
        /* Tranceiver Delay Disable */
        (boolean)FALSE,
        (uint8)0U
    },
    ...
```

The RDK S100 defaults to six sets of parameters. Users can select the baud rate by modifying the `u16DefaultBaudrateID` member value in `Can_aControllerConfig`. The table below shows the baud rate parameters corresponding to each index:

| u16DefaultBaudrateID | Arbitration Segment Frequency | Data Segment Frequency |
|----------------------|-------------------------------|------------------------|
| 0                    | 500K                          | 1M                     |
| 1                    | 500K                          | 2M                     |
| 2                    | 1M                            | 2M                     |
| 3                    | 1M                            | 5M (Short distance: \<50m) |
| 4                    | 1M                            | 5M (Long distance: \>50m)  |
| 5                    | 1M                            | 8M                     |



## Application Sample

### Usage Guide


:::info NOTE

Before executing the sample, you need to first power on MCU1. For the procedure to power on MCU1, refer to [Powering on MCU](../01_S100/01_basic_information.md#mcu1-startupshutdown-process).

:::


For reference on using Acore CANHAL, consult the sample source code located at: `source/hobot-io-samples/debian/app/Can`. You can directly compile and use it on S100 under the `/app/Can` directory.

Taking multi-channel transparent transmission as an example, the directory structure is as follows:

```bash
$ tree /app/Can/can_multi_ch
.
├── Makefile // Main build script
├── config  // Configuration file directory
│   ├── channels.json // Channel mapping configuration file
│   ├── ipcf_channel.json // IPCF channel mapping configuration file
│   └── nodes.json // CAN virtual device mapping configuration file
├── main.cpp // Main program
├── readme.md // Documentation file
├── can_multich_log.h // Logging header file
└── run.sh // Execution script

```

JSON configuration primarily involves three files: `nodes.json`, `ipcf_channel.json`, and `channels.json`. Currently, to support multi-process operation, each process searches for these three configuration files in the `config` subdirectory of its current working path.

`nodes.json` is responsible for creating virtual CAN device nodes accessible by the CANHAL API. Key configuration options include:
- The `channel_id` field specifies which node in the IPC configuration file `ipcf_channel.json` provides data for this virtual CAN device.
- The `target` field indicates the name of this virtual CAN device node, which the CANHAL API uses to access the specified node.
- The `enable` field indicates whether this node is enabled.

```json
{
  "nodes" : [
    {
      "id" : 0,
      "enable" : true,
      "mode_comment" : "value_table: R, W, RW",
      "mode" : "RW",
      "target" : "can6_ins0ch6",
      "clk_source" : "/dev/hrtc0",
      "io_channel" : {
        "device_type_comment" : "value_table: can, eth, ipcf, spi",
        "device_type" : "ipcf",
        "channel_id" : 0
      },
      "raw_protocol" : "built_1.0"
    },
    .....
    {
      "id" : 4,
      "enable" : true,
      "mode_comment" : "value_table: R, W, RW",
      "mode" : "RW",
      "target" : "can5_ins0ch4",
      "clk_source" : "/dev/hrtc0",
      "io_channel" : {
        "device_type_comment" : "value_table: can, eth, ipcf, spi",
        "device_type" : "ipcf",
        "channel_id" : 4
      },
      "raw_protocol" : "built_1.0"
    }
  ]
}
```

`ipcf_channel.json` maps the IPC nodes referenced in `nodes.json` to specific instances and channels.

```json
{
  "enable" : true,
  "libipcf_path" : "/usr/hobot/lib/libhbipcfhal.so.1",
  "channels" : [
    {
      "id" : 0,
      "channel" : {
        "name" : "can6_ins0ch6",
        "instance": 0,
        "channel": 6,
        "fifo_size": 64000,
        "fifo_type": 0,
        "pkg_size_max": 4096,
        "dev_path":"/dev/ipcdrv",
        "dev_name":"ipcdrv",
        "recv_timeout" : 4000
      }
    },
    ......
    {
      "id" : 4,
      "channel" : {
        "name" : "can5_ins0ch4",
        "instance": 0,
        "channel": 4,
        "fifo_size": 64000,
        "fifo_type": 0,
        "pkg_size_max": 4096,
        "dev_path":"/dev/ipcdrv",
        "dev_name":"ipcdrv",
        "recv_timeout" : 4000
      }
    }
  ]
}
```

`channels.json` specifies the IPC configuration file; users typically do not need to modify it.

```json
{
  "io_channels" : {
   "ipcf" : "./config/ipcf_channel.json"
 }
}
```

Acore cannot directly operate CAN peripherals and must rely on the IPC module to relay data. For the mapping relationship between Acore and peripheral channels, please refer to the "IPC Usage" section in the [MCU IPC Usage Guide](../../../07_Advanced_development/05_mcu_development/01_S100/08_mcu_ipc.md).

The pseudocode below illustrates how an Acore application receives CAN frames from the MCU side via CANHAL:

```c
void send_frame_data(void *arg)
{
    ... do
    {
        ret = canSendMsgFrame(test_params->target, &frame[0], &pack);
        if (ret > 0) { /* Send succeeded */
            send_count++;
            PRINT_DEBUG("Send length %d\n", ret);
            break;
        } else if (ret == -CAN_TRY_AGAIN && /* Retry due to insufficient IPC or other resources */
                   retry++ < MAX_RETRY) {
            usleep(2);
            continue;
        } else { /* Send failed */
            PRINT_ERR("Send failed after retries, exiting...\n");
            break;
        }
    }
    while (1)
        ;
}

void *recv_frame_data(void *arg)
{
    while (!exit_flag) {
        ret = canRecvMsgFrame(target, frame, &pack); // non blocking
        if (ret < 0) {
            if (ret == -CAN_TRY_AGAIN) { /* Retry due to insufficient IPC or other resources */
                PRINT_DEBUG("canRecvMsgFrame try again\n");
                continue;
            } else {
                PRINT_ERR("canRecvMsgFrame failed ret: %d\n", ret);
                gettimeofday(&current_time, NULL);
                double elapsed =
                        difftime(current_time.tv_sec, last_recv_time.tv_sec);
                if (elapsed > RECV_TIMEOUT) { /* Exit on timeout */
                    PRINT_INFO(
                            "No data received for %d seconds. Exiting thread.\n",
                            RECV_TIMEOUT);
                    goto can_recv_exit;
                }
                continue;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ret = canInit();
    pthread_create(&send_thread, NULL, send_frame_data, &tx_params);
    pthread_create(&rx_threads[i], NULL, recv_frame_data, &rx_params[i]);

    pthread_join(send_thread, NULL);
    pthread_join(rx_threads[i], NULL);
    canDeInit();
}
```

- First, execute `canInit()` to complete initialization, then create separate sending and receiving threads.
- The sending thread calls `canSendMsgFrame()` to send data packets, while the receiving thread calls `canRecvMsgFrame()` to receive data packets. The `target` parameter corresponds to the channel configured in the JSON file.
- The `pack` information includes metadata about this data packet, such as the number of CAN frames, the MCU-side timestamp, and the ACORE-side monotonic timestamp.
- CANHAL parses CAN frames from this IPC data packet, allowing users to read all CAN frames via the `frame` pointer.
- Finally, execute `canDeInit()` to release resources.

:::tip
The CAN send/receive functions depend on IPC resources. When the transmission rate is too high, resource exhaustion may occur; in such cases, reduce the transmission rate and implement retransmission.
:::

### ACORE-Side Example Description

#### Simple CAN Send/Receive Sample

##### Directory Structure
```
// /app/Can/can_send
.
├── Makefile // Main build script
├── canhal_send.c // Sends one standard CAN frame
└── config // Configuration directory
    ├── channels.json  // Channel mapping configuration file
    ├── ipcf_channel.json  // IPCF channel mapping configuration file
    └── nodes.json // Node mapping configuration file

// /app/Can/can_get
.
├── Makefile // Main build script
├── canhal_get.c // Continuously receives data in a while(1) loop
└── config // Configuration directory
    ├── channels.json   // Channel mapping configuration file
    ├── ipcf_channel.json  // IPCF channel mapping configuration file
    └── nodes.json  // Node mapping configuration file
```

##### Prerequisites

This is only a simple sample; actual applications require modifications based on specific requirements.

Prerequisites:
- MCU1 must be running normally.
- Hardware connection: Use a CAN closed-loop bus network. Connect CAN5 to CAN6, and place a 120Ω resistor between CAN_H and CAN_L (short the two pins behind the CAN5 and CAN6 terminal blocks using jumper caps or Dupont wires).
- Since both `can_send` and `can_get` use instance channel 4 (which by default maps data from CAN5), **and since a single IPC channel can only be used by one thread**, you must modify `ipcf_channel.json` in `can_send` to use instance channel 6 instead. The modification is as follows:

```json
{
  "enable" : true,
  "libipcf_path" : "/usr/hobot/lib/libhbipcfhal.so.1",
  "channels" : [
    {
      "id" : 0,
      "channel" : {
        "name" : "bypass",
        "instance": 0,
        "channel": 6, // Changed from 4 to 6 here
        "fifo_size": 64000,
        "fifo_type": 0,
        "pkg_size_max": 4096,
        "dev_path":"/dev/ipcdrv",
        "dev_name":"ipcdrv",
        "recv_timeout" : 4000
      }
    }
  ]
}
```

##### Usage Instructions
1. Compile both `can_send` and `can_get` samples separately.
2. Navigate to the `can_get` directory and run:
```bash
./canhal_get bypass &
```
3. Navigate to the `can_send` directory and run:
```bash
root@ubuntu:/app/Can/can_send# ./canhal_send bypass 6
[CANHAL][INFO][ipcf_dev.cpp:32][2025-2-20 21:43:47.522]:the path of ipcf plugin is /usr/hobot/lib/libhbipcfhal.so.1.
group name is bypass
[CANHAL][INFO][comps_mgr.cpp:158][2025-2-20 21:43:47.523]:channel constructor Register dev: ipcf success.
[CANHAL][INFO][node.cpp:17][2025-2-20 21:43:47.523]:channel id is 0
[INFO][hb_ipcf_hal.cpp:282] [channel] bypass [ins] 0 [id] 6 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] bypass [ins] 0 [id] 6 config success.
[CANHAL][INFO][node.cpp:39][2025-2-20 21:43:47.523]:io_channel init successful
[CANHAL][INFO][node.cpp:44][2025-2-20 21:43:47.523]:protocol is built_1.0
Send end, send package total: 1 frame total: 1
[CANHAL][INFO][can_hal_impl.cpp:120][2025-2-20 21:43:47.524]:Deinit node: bypass
[INFO][hb_ipcf_hal.cpp:553] [channel] bypass [ins] 0 [id] 6 deinit success.
[CANHAL][INFO][can_hal_impl.cpp:128][2025-2-20 21:43:47.524]:Deinit node: bypass
[CANHAL][INFO][channel_ctor.cpp:47][2025-2-20 21:43:47.524]:Deinit device: ipcf
*********************************************
[bypass] [pack]length: 1 soc_ts: 1186831 mcu_ts: 1191955
[canhal_get] [bypass] [canframe] canid is 0x00000131 timestamp is 0x123013 data is:
 0x0  0xaa  0xaa  0xaa  0xaa  0xaa  0xaa  0xfc
```
4. If the following output appears, the test succeeds:
```
[canhal_get] [bypass] [canframe] canid is 0x00000131 timestamp is 0x10b221 data is:
 0x0  0xaa  0xaa  0xaa  0xaa  0xaa  0xaa  0xfc
```


#### Multi-Channel Transmission

##### Directory Structure
```bash
// /app/Can/can_multi_ch
.
├── Makefile // Main build script
├── config  // Configuration directory
│   ├── channels.json // Channel mapping configuration file
│   ├── ipcf_channel.json // IPCF channel mapping configuration file
│   └── nodes.json // CAN virtual device mapping configuration file
├── main.cpp // Main program
├── readme.md // Documentation file
├── can_multich_log.h // Logging header file
└── run.sh // Execution script
```

This program implements multi-channel CAN bus data transmission and reception:
- **Hardware Connection**: Use a CAN closed-loop bus network. Connect CAN6 to CAN7, and CAN8 to CAN9. Leave CAN5 unconnected as a standalone channel. Install a 120Ω resistor between CAN_H and CAN_L (short the two pins behind the terminal blocks of all used CAN buses using jumper caps or Dupont wires).
- **Transmitting Threads**: Create an independent thread for each channel to send data. When using CAN FD, the data includes a counter and timestamp; when using classical CAN, the data is filled with 0x55.
- **Receiving Threads**: Create an independent thread for each channel to receive data and verify its correctness.

Transmission Strategy:
- Send data at fixed intervals via CAN. Adjust transmission frequency by modifying the delay. Excessively high frequencies may cause packet loss.
- Target Channels: Broadcast data across CAN5–CAN9.

Reception Strategy:
- Passively receive data, validate the counter in received data, and calculate transmission latency.
- Exit the program if no data is received for over 100 seconds (timeout defined in code).

##### Dependencies
- `pthread`: Threading library
- `hobot_can_hal`: CAN interface library
- `hb_ipcf_hal`: IPCF interface library
- `alog`: Android logging library

##### Channel Mapping

| Channel | Corresponding Thread Name | Device Name           |
|---------|----------------------------|------------------------|
| CAN5    | CAN5RX                     | "can5_ins0ch4"       |
| CAN6    | CAN6RX                     | "can6_ins0ch6"       |
| CAN7    | CAN7RX                     | "can7_ins0ch7"       |
| CAN8    | CAN8RX                     | "can8_ins0ch2"       |
| CAN9    | CAN9RX                     | "can9_ins0ch3"       |

##### DEBUG Toggle

```C
// can_multich_log.h
#define VERBOSE 0 // Set to 1 to enable debug logs
```
You can also use `logcat` to view more detailed logs.

##### Transmitter

Transmission Strategy:
- Send data at fixed intervals via CAN. Adjust transmission frequency by modifying the delay. Excessively high frequencies may cause packet loss.
- Data Content: Send extended CAN FD frames (64 bytes).
- Target Channels: Send data on each of CAN5–CAN9.

##### Receiver

Reception Strategy:
- Passively receive data, validate the counter in received data, and calculate transmission latency.
- Exit the program if no data is received for over 100 seconds (timeout defined in code).

##### Notes
- CAN device resources are automatically released upon program exit.
- Press Ctrl+C to interrupt program execution.
- Command-line arguments support configuring the number of frames to send, CAN frame type, and data length.

##### Build Commands
```bash
make # Compile
make clean # Clean compiled files
```

##### Command-Line Arguments
The program supports the following command-line arguments to customize runtime parameters:
```bash
-n <can_tran_num>                 Specify the number of frames to send (default: 1)
-t <can_type>                     Specify CAN frame type (0: Standard, 1: Extended, 2: FD Standard, 3: FD Extended) (default: 2)
-l <can_length>                   Specify CAN frame length (8: 8 bytes, 64: 64 bytes) (default: 64)
-h, --help                        Display help information
```

##### Run Command
```bash
export CAN_HAL_DEBUG_LEVEL=6 // Set CAN HAL debug level; setting to 6 disables all logs. Excessive logging may impact transmission frequency.
./can_multi_ch
```
Or run with parameters:
```bash  
./can_multi_ch -t 2 -l 64 -n 5
```

##### Log Analysis

Taking the execution of `./can_multi_ch -t 2 -l 64 -n 5` as an example:

- Current default parameter configuration:
    ```bash
    [param_config_display 389] [INFO]: Current parameter configuration:
    [param_config_display 390] [INFO]:   CAN Transmit Number: 1
    [param_config_display 391] [INFO]:   CAN Type: CAN_FD_STANDARD
    [param_config_display 392] [INFO]:   CAN Length: 64
    ```

- Hardware configuration used: CAN controllers, IPC instances, and channels
    ```
    group name is can6_ins0ch6
    group name is can7_ins0ch7
    group name is can8_ins0ch2
    group name is can9_ins0ch3
    group name is can5_ins0ch4
    ```

- Hardware configuration used: CAN controllers, IPC instances, and channels
    ```
    group name is can6_ins0ch6
    group name is can7_ins0ch7
    group name is can8_ins0ch2
    group name is can9_ins0ch3
    group name is can5_ins0ch4
    ```

- Channel initialization information
    ```bash
    [INFO][hb_ipcf_hal.cpp:282] [channel] can6_ins0ch6 [ins] 0 [id] 6 init success.
    [INFO][hb_ipcf_hal.cpp:333] [channel] can6_ins0ch6 [ins] 0 [id] 6 config success.
    [INFO][hb_ipcf_hal.cpp:282] [channel] can7_ins0ch7 [ins] 0 [id] 7 init success.
    [INFO][hb_ipcf_hal.cpp:333] [channel] can7_ins0ch7 [ins] 0 [id] 7 config success.
    [INFO][hb_ipcf_hal.cpp:282] [channel] can8_ins0ch2 [ins] 0 [id] 2 init success.
    [INFO][hb_ipcf_hal.cpp:333] [channel] can8_ins0ch2 [ins] 0 [id] 2 config success.
    [INFO][hb_ipcf_hal.cpp:282] [channel] can9_ins0ch3 [ins] 0 [id] 3 init success.
    [INFO][hb_ipcf_hal.cpp:333] [channel] can9_ins0ch3 [ins] 0 [id] 3 config success.
    [INFO][hb_ipcf_hal.cpp:282] [channel] can5_ins0ch4 [ins] 0 [id] 4 init success.
    [INFO][hb_ipcf_hal.cpp:333] [channel] can5_ins0ch4 [ins] 0 [id] 4 config success.
    ```

- Data printed by the sending thread, showing how many frames were successfully forwarded to the MCU and the transmission rate
    ```
    [send_frame_data 266] [INFO]: Target can5_ins0ch4 Time: 0.005230s
    [send_frame_data 268] [INFO]: Send success count: 5 Total:5
    [send_frame_data 271] [INFO]: 61185.468750 byte/s -> 59.751434kb/s
    [send_frame_data 274] [INFO]: Sending FPS: 956.02 frames/s
    [send_frame_data 276] [INFO]: Send end, send package total: 5 frame total: 5
    [send_frame_data 278] [INFO]: Send thread exiting...
    [send_frame_data 266] [INFO]: Target can6_ins0ch6 Time: 0.005074s
    [send_frame_data 268] [INFO]: Send success count: 5 Total:5
    [send_frame_data 271] [INFO]: 63066.613281 byte/s -> 61.588490kb/s
    [send_frame_data 274] [INFO]: Sending FPS: 985.42 frames/s
    [send_frame_data 276] [INFO]: Send end, send package total: 5 frame total: 5
    [send_frame_data 278] [INFO]: Send thread exiting...
    [send_frame_data 266] [INFO]: Target can7_ins0ch7 Time: 0.004979s
    .....
    ```

- Receive timeout: After the receive thread starts, it waits for incoming data. If no data is received for an extended period, error code -303 is reported. Depending on the actual situation, one can determine whether an anomaly exists. If no data is received for over 100 seconds, the thread exits.
    ```
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 307] [ERR]: canRecvMsgFrame failed ret: -303
    [recv_frame_data 314] [INFO]: No data received for 100 seconds. Exiting thread.

    ```

- After exiting with Ctrl+C, the number of packets received by each receive thread and the maximum transmission latency (supported only for CAN FD) can be observed. Since CAN5 is not connected to any device, the number of received packets is 0.
    ```
    Target :can7_ins0ch7 recv frame num: 5 Total recv frame num: 5 Maximum transmission time:32209 us
    Target :can6_ins0ch6 recv frame num: 5 Total recv frame num: 5 Maximum transmission time:41182 us
    Target :can8_ins0ch2 recv frame num: 5 Total recv frame num: 5 Maximum transmission time:22676 us
    Target :can9_ins0ch3 recv frame num: 5 Total recv frame num: 5 Maximum transmission time:11395 us
    Target :can5_ins0ch4 recv frame num: 0 Total recv frame num: 5 Maximum transmission time:0 us
    ```
- Resource release
    ```
    [INFO][hb_ipcf_hal.cpp:553] [channel] can5_ins0ch4 [ins] 0 [id] 4 deinit success.
    [INFO][hb_ipcf_hal.cpp:553] [channel] can8_ins0ch2 [ins] 0 [id] 2 deinit success.
    [INFO][hb_ipcf_hal.cpp:553] [channel] can9_ins0ch3 [ins] 0 [id] 3 deinit success.
    [INFO][hb_ipcf_hal.cpp:553] [channel] can7_ins0ch7 [ins] 0 [id] 7 deinit success.
    [INFO][hb_ipcf_hal.cpp:553] [channel] can6_ins0ch6 [ins] 0 [id] 6 deinit success.
    ```

#### Multi-CAN Networking Transmission

:::tip
Continuously being updated
:::

#### Modifying CAN Baud Rate and Other Configurations at the Application Layer

:::tip
Continuously being updated
:::

### Library Log Output Control

CAN_HAL_DEBUG_LEVEL is an environment variable used to control the log output level of the library file libhbcanhal.so. Its different values represent different log levels, which determine which log messages will be recorded.
```
0 (log_trace): Trace level.
1 (log_debug): Debug level.
2 (log_info): Info level.
3 (log_warn): Warning level.
4 (log_err): Error level.
5 (log_critical): Critical level.
6 (log_never): Do not print any logs.
```
By setting the value of CAN_HAL_DEBUG_LEVEL, you can control the verbosity of log output. For example, if set to 2, only logs of level log_info, log_warn, log_err, and log_critical will be printed.

### MCU-side DEBUG Application Instructions
1. Enter the MCU1 console.
2. Input the command: can_tran_debug
```
D-Robotics:/$ can_tran_debug
[01217.785483 0]Ipc2Can Statistics: // Statistics of data packets received by IPC
[01217.785712 0]Channel 0 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.786895 0]Channel 1 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.788077 0]Channel 2 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.789260 0]Channel 3 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.790443 0]Channel 4 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.791625 0]Channel 5 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.792808 0]Channel 6 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.793991 0]Channel 7 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.796039 0]Soc2Can Statistics: // Statistics after unpacking
[01217.796268 0]RecvAllNum: 4
[01217.796605 0]RollCntErr: 3 // This error does not affect functionality; it disappears when the A-core application sends data with properly sequenced packet numbers.
[01217.796941 0]InvaildChannelErr: 0
[01217.798140 0]CanLengthErr: 0
[01217.798424 0]CanDisableErr: 0
[01217.798793 0]RecvNum[0]: 0, OverFlowErr[0]: 0
[01217.800115 0]RecvNum[1]: 0, OverFlowErr[1]: 0
[01217.800570 0]RecvNum[2]: 0, OverFlowErr[2]: 0
[01217.801112 0]RecvNum[3]: 0, OverFlowErr[3]: 0
[01217.802343 0]RecvNum[4]: 0, OverFlowErr[4]: 0
[01217.802886 0]RecvNum[5]: 0, OverFlowErr[5]: 0
[01217.804158 0]RecvNum[6]: 1, OverFlowErr[6]: 0
[01217.804661 0]RecvNum[7]: 1, OverFlowErr[7]: 0
[01217.806059 0]RecvNum[8]: 1, OverFlowErr[8]: 0
[01217.806435 0]RecvNum[9]: 1, OverFlowErr[9]: 0
[01217.806978 0]Hb_CanIf Statistics: // CAN controller transmission/reception statistics
[01217.808159 0]Can 0 RxNum:          0 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.809520 0]Can 1 RxNum:          0 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.810941 0]Can 2 RxNum:          0 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.812362 0]Can 3 RxNum:          0 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.813784 0]Can 4 RxNum:          0 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.815205 0]Can 5 RxNum:          4 TxNum:          0 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.816626 0]Can 6 RxNum:          3 TxNum:          1 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.818048 0]Can 7 RxNum:          3 TxNum:          1 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.819469 0]Can 8 RxNum:          3 TxNum:          1 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
[01217.820891 0]Can 9 RxNum:          3 TxNum:          1 RxOverFlowErr:          0 TxCanBusyErr:          0 TxPduErr:          0
```

### Application Programming Interface (API)

#### void Can_Init(const Can_ConfigType* Config)

```shell
Description: This function initializes the module.

Sync/Async: Synchronous
Parameters(in)
    Config: Pointer to driver configuration.
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Can_GetVersionInfo(Std_VersionInfoType* versioninfo)

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

#### void Can_DeInit(void)

```shell
Description: This function de-initializes the module.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Can_SetControllerMode(uint8 Controller, Can_ControllerStateType Transition)

```shell
Description: This function performs software triggered state transitions of the CAN controller State machine.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which the status shall be changed.
    Transition: Transition value to request new CAN controller state.
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
	E_OK: request accepted.
    E_NOT_OK: request not accepted, a development error occurred.
```

#### void Can_DisableControllerInterrupts(uint8 Controller)

```shell
Description: This function disables all interrupts for this CAN controller.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which interrupts shall be disabled.
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Can_EnableControllerInterrupts(uint8 Controller)

```shell
Description: This function enables all allowed interrupts.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which interrupts shall be re-enabled.
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Can_GetControllerErrorState(uint8 ControllerId, Can_ErrorStateType* ErrorStatePtr)

```shell
Description: This service obtains the error state of the CAN controller.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: Abstracted CanIf ControllerId which is assigned to a CAN controller, which is requested for ErrorState.
Parameters(inout)
    None
Parameters(out)
    ErrorStatePtr: Pointer to a memory location, where the error state of the CAN controller will be stored.
Return value: Std_ReturnType
	E_OK: Error state request has been accepted.
    E_NOT_OK: Error state request has not been accepted.
```

#### Std_ReturnType Can_GetControllerMode(uint8 Controller, Can_ControllerStateType* ControllerModePtr)

```shell
Description: This service reports about the current status of the requested CAN controller.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which the status shall be requested.
Parameters(inout)
    None
Parameters(out)
    ControllerModePtr: Pointer to a memory location, where the current mode of the CAN controller will be stored.
Return value: Std_ReturnType
    E_OK: Controller mode request has been accepted.
    E_NOT_OK: Controller mode request has not been accepted.
```

#### Std_ReturnType Can_GetControllerRxErrorCounter(uint8 ControllerId, uint8* RxErrorCounterPtr)

```shell
Description: Returns the Rx error counter for a CAN controller.
             This value might not be available for all CAN controllers, in which case E_NOT_OK would be
             returned. Please note that the value of the counter might not be correct at the moment the
             API returns it, because the Rx counter is handled asynchronously in hardware. Applications
             should not trust this value for any assumption about the current bus state.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: CAN controller, whose current Rx error counter shall be acquired.
Parameters(inout)
    None
Parameters(out)
    RxErrorCounterPtr: Pointer to a memory location, where the current Rx error counter of the
                       CAN controller will be stored.
Return value: Std_ReturnType
    E_OK: Rx error counter available.
    E_NOT_OK: Wrong ControllerId, or Rx error counter not available.
```

#### Std_ReturnType Can_GetControllerTxErrorCounter(uint8 ControllerId, uint8* TxErrorCounterPtr)

```shell
Description: Returns the Tx error counter for a CAN controller. This value might not be available
             for all CAN controllers, in which case E_NOT_OK would be returned. Please note that the
             value of the counter might not be correct at the moment the API returns it, because the
             Tx counter is handled asynchronously in hardware. Applications should not trust this
             value for any assumption about the current bus state.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: CAN controller, whose current Tx error counter shall be acquired.
Parameters(inout)
    None
Parameters(out)
    TxErrorCounterPtr: Pointer to a memory location, where the current Tx error counter
                      of the CAN controller will be stored.
Return value: Std_ReturnType
    E_OK: Tx error counter available.
    E_NOT_OK: Wrong ControllerId, or Tx error counter not available.
```

#### Std_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType* PduInfo)

```shell
Description: This function is called by CanIf to pass a CAN message to CanDrv for transmission.

Sync/Async: Synchronous
Parameters(in)
    Hth: information which HW-transmit handle shall be used for transmit. Implicitly this is
        also the information about the controller to use because the Hth numbers are unique
        inside one hardware unit.
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: Write command has been accepted.
    E_NOT_OK: development error occurred.
    CAN_BUSY: No TX hardware buffer available or pre-emptive call of Can_Write that can’t be
              implemented re-entrant (see Can_ReturnType).
```

#### void Can_MainFunction_Write(void)

```shell
Description: This function performs the polling of TX confirmation when CAN_TX_PROCESSING
             is set to POLLING.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Can_MainFunction_Read(void)

```shell
Description: Returns the value of the specified CAN channel. This function performs the
             polling of RX indications when CAN_RX_PROCESSING is set to POLLING.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### void Can_MainFunction_BusOff(void)

```shell
Description: This function performs the polling of bus-off events that are configured statically
             as ‘to be polled’.
```Sync/Async: Synchronous  
Parameters(in)  
    None  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: None  
```

#### void Can_MainFunction_Mode(Void)

```shell
Description: This function performs the polling of CAN controller mode transitions.

Sync/Async: Synchronous  
Parameters(in)  
    None  
Parameters(inout)  
    None  
Parameters(out)  
    None  
Return value: None  
```