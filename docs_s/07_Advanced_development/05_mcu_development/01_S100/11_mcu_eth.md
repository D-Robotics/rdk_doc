---
sidebar_position: 11
---

# Eth使用指南

## 基本概述

### 硬件特性

- 最大支持1000Mbps数据传输速率

- 支持全双工流控操作(包括IEEE 802.3x Pause packets and Priority flow control)

- 支持网络统计功能

- 支持IEEE 1588-2002/1588-2008标准定义的以太网报文时间戳

- 支持输出PPS秒脉冲信号

- 支持可编程以太网帧长度，最大支持16KB

### 假设和限制

- 发送和接收方向的FIFO最多各支持6个。

- 不支持传输超过所使用控制器可用缓冲区大小的数据，较长的数据必须使用Internet协议(IP)和传输控制协议(TCP)传输。

- 单个接收帧的长度(包括14字节的以太网帧头和4字节的FCS)必须小于或等于RX buffer的配置长度。

- 模块时钟频率为300M，PTP时钟周期为20ns。

## 代码路径

- McalCdd/Ethernet/inc # 头文件

- McalCdd/Ethernet/src/Eth.c # 提供对外API接口

- McalCdd/Ethernet/src/Eth_Interrupt.c # 中断处理回调函数处理接口

- McalCdd/Ethernet/src/Mac_Lld.c # 封装寄存器控制接口，供API接口调用

- Config/McalCdd/gen_s100_sip_B_mcu1/Ethernet/src/Eth_PBcfg.c # Eth预编译配置，用于提供给对外接口API初始化属性调用

- Config/McalCdd/gen_s100_sip_B_mcu1/Ethernet/src/Mac_Ip_PBcfg.c # MAC驱动预编译配置，对Eth_PBcfg.c构成静态配置依赖

- samples/Eth/Eth_Test/Eth_test.c # Eth功能测试示例程序

## 应用sample

S100以```samples/Eth/Eth_Test/Eth_test.c```发送arp报文为例说明：

### 数据发送

Eth_test.c测试程序外发构造的arp报文，PC通过wiresharke抓包检查数据能否正常收到。其中，IP地址默认且不支持动态修改。

```
0xd4, 0xfd, 0x9b, 0xae, 0x48, 0xf5, //Sender MAC address: d4:fd:9b:ae:48:f5  //MCU
0xC0, 0xA8, 0x01, 0x32,             //Sender IP address: 192.168.1.50
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Target MAC address: 00:00:00:00:00:00  //PC
0xC0, 0xA8, 0x01, 0xf,             //Target IP address: 192.168.1.15
```

- 调用伪代码

```
Eth_ProvideTxBuffer //分配buffer
Eth_Transmit //数据发送
Eth_TxConfirmation //释放buffer
```

- 测试说明

系统启动默认只完成eth初始化，数据发送步骤如下：

```
# 使能EthTest_Mainfunc周期性调用
setvar Eth_Test1

# eth up
setvar eth_contrMode 1
setvar eth_testCase 3

# 发送arp报文
setvar eth_testCase 14
```

### 数据接收

在EthIf_RxIndication里把接收的报文通过串口打印出来。参考如下：

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
    else if(DataPtr[9]==0x6)//tcp
    {
        LogSync("TCP checksum:%x,%x\r\n",DataPtr[36],DataPtr[37]);
        eth_checkCksFlg=TRUE;
    }
}
```

### 注意事项

- S100默认数据传输模式是轮询。轮询模式下的数据发送需要注意申请的buffer在transmit发送之后，调用Eth_TxConfirmation释放buffer。
- Eth_Init之前需要phy解复位保证初始化成功，S100上将phy reset pin拉高实现解复位

### 应用程序接口

#### void Eth_Init( const Eth_ConfigType* CfgPtr )

```
Description：Initializes the Ethernet Driver.

Sync/Async: Synchronous
Parameters(in)
    CfgPtr: Points to the implementation specific structure
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Eth_SetControllerMode(uint8 CtrlIdx, Eth_ModeType CtrlMode)

```
Description：Enables / disables the indexed controller.

Sync/Async: Asynchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
    CtrlMode: ETH_MODE_DOWN: disable the controller; ETH_MODE_ACTIVE: enable the controller
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    E_NOT_OK: controller mode could not be changed
```

#### Std_ReturnType Eth_GetControllerMode(uint8 CtrlIdx, Eth_ModeType* CtrlModePtr)

```
Description：Enables / disables the indexed controller.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters(inout)
    CtrlModePtr: ETH_MODE_DOWN: disable the controller; ETH_MODE_ACTIVE: enable the controller
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    E_NOT_OK: controller mode could not be obtained
```

#### void Eth_GetPhysAddr(uint8 CtrlIdx, uint8* PhysAddrPtr)

```
Description：Obtains the physical source address used by the indexed controller.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters(inout)
    PhysAddrPtr: Physical source address (MAC address) in network byte order
Parameters(out)
    None
Return value：None
```

#### void Eth_SetPhysAddr(uint8 CtrlIdx, const uint8* PhysAddrPtr)

```
Description：Sets the physical source address used by the indexed controller.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
    PhysAddrPtr: Pointer to memory containing the physical source address (MAC address) in network byte order
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Eth_GetCurrentTime(uint8 CtrlIdx, Eth_TimeStampQualType* timeQualPtr, Eth_TimeStampType* timeStampPtr)

:::info 注意
Eth_GetCurrentTime may be called within an exclusive area.
:::

```
Description：Returns a time value out of the HW registers according to the capability of the HW. Is the HW resolution is lower than the Eth_TimeStampType resolution resp.range, than an the remaining bits will be filled with 0.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller within the context of the Ethernet Driver
Parameters(inout)
    None
Parameters(out)
    timeQualPtr: Quality of HW time stamp, e.g. based on current drift
    timeStampPtr: Current time stamp
Return value：Std_ReturnType
    E_OK: success
    E_NOT_

```

#### BufReq_ReturnType Eth_ProvideTxBuffer(uint8 CtrlIdx, uint8 Priority, Eth_BufIdxType *BufIdxPtr, uint8 **BufPtr, uint16 *LenBytePtr)

```
Description：Provides access to a transmit buffer of the FIFO related to the specified priority.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
    Priority: Frame priority for transmit buffer FIFO selection
Parameters(inout)
    LenBytePtr: In: desired length in bytes, out: granted length in bytes
Parameters(out)
    BufIdxPtr: Index to the granted buffer resource. To be used for subsequent requests
    BufPtr: Pointer to the granted buffer
Return value：BufReq_ReturnType
    BUFREQ_OK: success
    BUFREQ_E_NOT_OK: development error detected
    BUFREQ_E_BUSY: all buffers in use
    BUFREQ_E_OVFL: requested buffer too large
```

#### Std_ReturnType Eth_Transmit(uint8 CtrlIdx, Eth_BufIdxType BufIdx, Eth_FrameType FrameType, boolean TxConfirmation, uint16 LenByte, const uint8* PhysAddrPtr)

```
Description：Description：Triggers transmission of a previously filled transmit buffer.

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
Return value：Std_ReturnType
    E_OK: success
    E_NOT_OK: transmission failed
```

#### Void Eth_Receive(uint8 CtrlIdx, uint8 FifoIdx, Eth_RxStatusType* RxStatusPtr)

```
Description：Receive a frame from the related fifo.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
    FifoIdx: Specifies the related fifo
Parameters(inout)
    None
Parameters(out)
    RxStatusPtr: Indicates whether a frame has been received and if so, whether more frames are available for the related fifo.
Return value：None
```

#### void Eth_TxConfirmation(uint8 CtrlIdx)

```
Description：Triggers frame transmission confirmation.

Sync/Async: Synchronous
Parameters(in)
    CtrlIdx: Index of the controller
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Eth_EnableSnapshot(uint8 CtrlIdx, GMAC_PPS_SOURCE PpsSource)

```
Description：Set snapshot source.

Sync/Async: Asynchronous
Parameters(in)
    CtrlIdx: Index of the addresses ETH controller
    PpsSource: Index of the PPS Source
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    E_NOT_OK: failed
```

#### Std_ReturnType Eth_GetSnapshotTime(uint8 CtrlIdx, Eth_TimeStampType * TimeStampPtr)

```
Description：Get snapshot time of PHC.

Sync/Async: Asynchronous
Parameters(in)
    CtrlIdx: Index of the addresses ETH controller
    TimeStampPtr: Snapshot Time of PHC
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: success
    E_NOT_OK: failed
```
