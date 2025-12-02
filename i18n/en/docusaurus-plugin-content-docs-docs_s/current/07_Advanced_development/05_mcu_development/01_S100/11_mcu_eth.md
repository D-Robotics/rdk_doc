---
sidebar_position: 11
---

# Eth Usage Guide

## Basic Overview

### Hardware Features

- Supports data transfer rates up to 1000 Mbps

- Supports full-duplex flow control operations (including IEEE 802.3x Pause packets and Priority Flow Control)

- Supports network statistics functionality

- Supports Ethernet packet timestamping as defined by IEEE 1588-2002/1588-2008 standards

- Supports output of PPS (Pulse Per Second) signal

- Supports programmable Ethernet frame length, up to a maximum of 16 KB

### Assumptions and Limitations

- FIFOs for transmit and receive directions support up to 6 each.

- Does not support transmission of data exceeding the available buffer size of the controller in use; longer data must be transmitted using Internet Protocol (IP) and Transmission Control Protocol (TCP).

- The length of a single received frame (including the 14-byte Ethernet header and 4-byte FCS) must be less than or equal to the configured RX buffer length.

- Module clock frequency is 300 MHz, and the PTP clock period is 20 ns.

## Code Paths

- McalCdd/Ethernet/inc # Header files

- McalCdd/Ethernet/src/Eth.c # Provides external API interfaces

- McalCdd/Ethernet/src/Eth_Interrupt.c # Interrupt handling callback function interfaces

- McalCdd/Ethernet/src/Mac_Lld.c # Wraps register control interfaces for use by API functions

- Config/McalCdd/gen_s100_sip_B_mcu1/Ethernet/src/Eth_PBcfg.c # Pre-compiled Eth configuration, used to provide initialization attributes for external API calls

- Config/McalCdd/gen_s100_sip_B_mcu1/Ethernet/src/Mac_Ip_PBcfg.c # Pre-compiled MAC driver configuration, statically dependent on Eth_PBcfg.c

- samples/Eth/Eth_Test/Eth_test.c # Eth functional test example program

## Application Sample

Taking S100's ```samples/Eth/Eth_Test/Eth_test.c``` sending an ARP packet as an example:

### Data Transmission

The Eth_test.c test program constructs and sends an ARP packet. A PC uses Wireshark to capture packets and verify whether the data is received correctly. The IP address is set by default and cannot be modified dynamically.

```
0xd4, 0xfd, 0x9b, 0xae, 0x48, 0xf5, // Sender MAC address: d4:fd:9b:ae:48:f5  // MCU
0xC0, 0xA8, 0x01, 0x32,             // Sender IP address: 192.168.1.50
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Target MAC address: 00:00:00:00:00:00  // PC
0xC0, 0xA8, 0x01, 0xf,              // Target IP address: 192.168.1.15
```

- Pseudocode calls

```
Eth_ProvideTxBuffer // Allocate buffer
Eth_Transmit        // Transmit data
Eth_TxConfirmation  // Release buffer
```

- Test instructions

By default, only Eth initialization is performed upon system startup. The data transmission steps are as follows:

```
# Enable periodic calling of EthTest_Mainfunc
setvar Eth_Test 1

# Bring Eth up
setvar eth_contrMode 1
setvar eth_testCase 3

# Send ARP packet
setvar eth_testCase 14
```

### Data Reception

Received packets are printed via the serial port within EthIf_RxIndication. Reference implementation as follows:

```
if(eth_getIngressTsFlag)
{
    eth_getIngressTsFlag = FALSE;
    /* QT-S01-API-60105 QT-S01-API-60106 QT-S01-API-60107 QT-S01-API-60194 */
    Eth_GetIngressTimeStamp(CtrlIdx,DataPtr,&Eth_TimeQual,&Eth_TimeStamp);
    //LogSync("Ingress timestamp quality: %s\r\n", (Eth_TimeQual==ETH_VALID)?"ETH_VALID":"ETH_INVALID");
    //LogSync("Ingress timestamp: %ds: %dns\r\n", ((uint32)(Eth_TimeStamp.secondsHi) << 16) + Eth_TimeStamp.seconds, Eth_TimeStamp.nanoseconds);
    if(Eth_TimeStamp.secondsHi!=0 || Eth_TimeStamp.nanoseconds!=0)
    {
        eth_checkIngressTsFlg=TRUE;
    }
}

if (count % 100 == 0) {
    LogSync("Eth packet is received, FrameType: %x, IsBroadcast: %s\r\n", FrameType, (IsBroadcast==TRUE)?"TRUE":"FALSE");
    LogSync("DstMac: %x-%x-%x-%x-%x-%x\r\n", *(DataPtr-14),*(DataPtr-13),*(DataPtr-12),*(DataPtr-11),*(DataPtr-10),*(DataPtr-9));
    LogSync("SrcMac: %x-%x-%x-%x-%x-%x\r\n", PhysAddrPtr[0],PhysAddrPtr[1],PhysAddrPtr[2],PhysAddrPtr[3],PhysAddrPtr[4],PhysAddrPtr[5]);
}
count++;
if(FrameType==0x800)
{
    LogSync("IP header checksum:%x,%x\r\n",DataPtr[10],DataPtr[11]);
    if(DataPtr[9]==0x11)//UDP
    {
        LogSync("UDP checksum:%x,%x\r\n",DataPtr[26],DataPtr[27]);
        eth_checkCksFlg=TRUE;
    }
    else if(DataPtr[9]==0x6)//TCP
    {
        LogSync("TCP checksum:%x,%x\r\n",DataPtr[36],DataPtr[37]);
        eth_checkCksFlg=TRUE;
    }
}
```

### Notes

- S100 uses polling mode for data transmission by default. In polling mode, note that after transmitting data using the allocated buffer, Eth_TxConfirmation must be called to release the buffer.
- Before calling Eth_Init, ensure the PHY is de-asserted from reset to guarantee successful initialization. On S100, this is achieved by pulling the PHY reset pin high.

### Application Programming Interface

#### void Eth_Init( const Eth_ConfigType* CfgPtr )

```
Description: Initializes the Ethernet Driver.

Sync/Async: Synchronous
Parameters (in)
    CfgPtr: Points to the implementation-specific configuration structure
Parameters (inout)
    None
Parameters (out)
    None
Return value: None
```

#### Std_ReturnType Eth_SetControllerMode(uint8 CtrlIdx, Eth_ModeType CtrlMode)

```
Description: Enables / disables the indexed controller.

Sync/Async: Asynchronous
Parameters (in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
    CtrlMode: ETH_MODE_DOWN: disable the controller; ETH_MODE_ACTIVE: enable the controller
Parameters (inout)
    None
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: controller mode could not be changed
```

#### Std_ReturnType Eth_GetControllerMode(uint8 CtrlIdx, Eth_ModeType* CtrlModePtr)

```
Description: Retrieves the current mode of the indexed controller.

Sync/Async: Synchronous
Parameters (in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters (inout)
    CtrlModePtr: Pointer to store the controller mode (ETH_MODE_DOWN: disabled; ETH_MODE_ACTIVE: enabled)
Parameters (out)
    None
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: controller mode could not be obtained
```

#### void Eth_GetPhysAddr(uint8 CtrlIdx, uint8* PhysAddrPtr)

```
Description: Obtains the physical source address (MAC address) used by the indexed controller.

Sync/Async: Synchronous
Parameters (in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters (inout)
    PhysAddrPtr: Physical source address (MAC address) in network byte order
Parameters (out)
    None
Return value: None
```

#### void Eth_SetPhysAddr(uint8 CtrlIdx, const uint8* PhysAddrPtr)  
```
Description: Sets the physical source address used by the indexed controller.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
    PhysAddrPtr: Pointer to memory containing the physical source address (MAC address) in network byte order
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Eth_GetCurrentTime(uint8 CtrlIdx, Eth_TimeStampQualType* timeQualPtr, Eth_TimeStampType* timeStampPtr)

:::info Note
Eth_GetCurrentTime may be called within an exclusive area.
:::

```
Description: Returns a time value out of the HW registers according to the capability of the HW. If the HW resolution is lower than the Eth_TimeStampType resolution resp. range, then the remaining bits will be filled with 0.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters(inout)
    None
Parameters(out)
    timeQualPtr: Quality of HW time stamp, e.g. based on current drift
    timeStampPtr: Current time stamp
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: failed
```

#### BufReq_ReturnType Eth_ProvideTxBuffer(uint8 CtrlIdx, uint8 Priority, Eth_BufIdxType *BufIdxPtr, uint8 **BufPtr, uint16 *LenBytePtr)

```
Description: Provides access to a transmit buffer of the FIFO related to the specified priority.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
    Priority: Frame priority for transmit buffer FIFO selection
Parameters(inout)
    LenBytePtr: In: desired length in bytes, out: granted length in bytes
Parameters(out)
    BufIdxPtr: Index to the granted buffer resource. To be used for subsequent requests
    BufPtr: Pointer to the granted buffer
Return value: BufReq_ReturnType
    BUFREQ_OK: success
    BUFREQ_E_NOT_OK: development error detected
    BUFREQ_E_BUSY: all buffers in use
    BUFREQ_E_OVFL: requested buffer too large
```

#### Std_ReturnType Eth_Transmit(uint8 CtrlIdx, Eth_BufIdxType BufIdx, Eth_FrameType FrameType, boolean TxConfirmation, uint16 LenByte, const uint8* PhysAddrPtr)

```
Description: Triggers transmission of a previously filled transmit buffer.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
    BufIdx: Index of the buffer resource
    FrameType: Ethernet frame type
    TxConfirmation: Activates transmission confirmation
    LenByte: Data length in byte
    PhysAddrPtr: Physical target address (MAC address) in network byte order
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: transmission failed
```

#### void Eth_Receive(uint8 CtrlIdx, uint8 FifoIdx, Eth_RxStatusType* RxStatusPtr)

```
Description: Receive a frame from the related fifo.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
    FifoIdx: Specifies the related fifo
Parameters(inout)
    None
Parameters(out)
    RxStatusPtr: Indicates whether a frame has been received and if so, whether more frames are available for the related fifo.
Return value: None
```

#### void Eth_TxConfirmation(uint8 CtrlIdx)

```
Description: Triggers frame transmission confirmation.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
Parameters(inout)
    None
Parameters(out)
    None
Return value: None
```

#### Std_ReturnType Eth_EnableSnapshot(uint8 CtrlIdx, GMAC_PPS_SOURCE PpsSource)

```
Description: Set snapshot source.

Sync/Async: Asynchronous
Parameters(in)
    CtrlIdx: Index of the ETH controller
    PpsSource: Index of the PPS Source
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: failed
```

#### Std_ReturnType Eth_GetSnapshotTime(uint8 CtrlIdx, Eth_TimeStampType * TimeStampPtr)

```
Description: Get snapshot time of PHC.

Sync/Async: Asynchronous
Parameters(in)
    CtrlIdx: Index of the ETH controller
    TimeStampPtr: Snapshot Time of PHC
Parameters(inout)
    None
Parameters(out)
    None
Return value: Std_ReturnType
    E_OK: success
    E_NOT_OK: failed
```