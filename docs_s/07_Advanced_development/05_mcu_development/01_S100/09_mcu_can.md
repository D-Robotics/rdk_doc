---
sidebar_position: 9
---

# CAN使用指南

## 基本概述

- 最大可使用CAN controller数量：10。
- CAN最高传输速率：8M。(受限于transceiver的波特率限制，目前实验室只测试验证到5M波特率。)
- 个controller的Ram内划分的Block个数：
    - CAN0~CAN3：4 Block (可变payload);
    - CAN4~CAN9：4 Block (可变payload)+ 4 Block(固定payload)。
- 一个controller支持的最大Mailbox个数为128。
- 一个controller支持一路RxFIFO，FIFO深度为：
    - CAN0~CAN3：8 * 64 bytes;
    - CAN4~CAN9：32 * 64 bytes。
- 不支持 TTController，即不支持TTCAN（一种基于CAN总线的高层协议）。
- CAN支持多核使用，可将不同的CAN控制器绑定在不同的核心上，但不支持多个核心同时使用同一个CAN控制器。


## 软件架构

S100芯片的Can控制器位于MCU域，负责Can数据收发。由于感知等应用位于Acore，因此部分Can数据需要通过IPC核间通信机制转发到Acore。架构保证传输可靠性，转发机制实现数据正确性检测、丢包检测和传输超时检测等机制。此外，还需要规避MCU侧高频转发小数据块导致CPU占用率过高，造成MCU实时性降低等性能问题。

S100 Can转发方案的核心流程如下：
- 首先通过MCU侧CAN2IPC模块将CAN通道映射到对应IPC通道，然后通过Acore侧CANHAL模块将IPC通道反映射为虚拟Can设备通道。最后用户通过CANHAL提供的API接口获取虚拟Can设备中的数据。其中，CAN2IPC模块为MCU侧服务，CANHAL模块为Acore侧提供给应用程序的动态库。
- CAN2IPC模块周期性采集MCU侧CAN数据，按照指定传输协议进行打包，然后通过IPC核间通信转发到Acore。Ipc instance 0中的channel0~channel7默认分配给Can转发使用。
- CANHAL模块获取来自MCU侧的IPC数据，按照指定的传输协议解析数据，并支持业务软件通过API获取原始Can帧。


![Acore与MCU之间透传Can数据架构图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/mcu_can.png)

数据流如上图所示：
- 外设数据通过CAN的PHY和控制器器件被MCU域CAN驱动接收后，CAN驱动将数据上报并缓存在hobot CANIF模块。
- CAN2IPC Service周期性从CANIF模块取出CAN帧，按照可靠传输协议进行打包，然后通过IPC核间通信机制转发给Acore。
- CANHAL模块获取来自MCU侧的IPC数据，按照指定的传输协议解析数据，Acore 应用程序通过CANHAL Lib库提供的API获取Can帧。

方案特性说明：
- 支持数据透传正确性校验。
- 支持数据透传丢包检测。
- 支持传输超时检测。MCU侧CAN2IPC转发数据时将数据包打上MCU侧的时间戳，Acore CANHAL接收到数据后会读取Acore的时间戳，如果传输超时会报警。注意，需要提前启动时间同步完成MCU RTC时间和Acore 网卡phc0的时间同步。
- 支持多个CAN通道并行传输。MCU侧多个CAN控制器的数据可同时被转发给Acore，Acore应用程序通过CANHAL从不同通道号读出CAN数据。
- 由于CANHAL底层通过ipc核间通信进行传输，而ipc目前不支持多个进程或者线程读写同一个通道，因此CANHAL也不支持该特性。

## 波特率配置

CAN的标称位时（Nominal bit timing）可以分为四个段：
1. 同步段(sync_seg)‌：用于节点间的时钟同步，所有节点在此段内检测信号边沿。其长度固定为1个时间单位(TQ)
2. 传播段(prop_seg)‌：补偿信号在物理线路上的传播延迟。其长度可调整，用于确保信号在物理介质上的传输时间
3. 相位缓冲段1(phase_seg1)‌：用于调整相位误差，确保采样点的准确性，可以扩展重同步
4. 相位缓冲段2(phase_seg2)‌：同样用于调整相位误差，但可以缩短
这些段的总和决定了CAN的总位时间，通过调整这些段的长度，可以配置不同的波特率。
此外还有以下几个重要概念：
1. 同步跳转宽度(SJW，synchronization jump width)：CAN 总线同步机制中允许调整相位缓冲段的最大时间量，在 硬同步 和 重同步 过程中补偿节点间的时钟偏差，确保采样点对齐。
2. 延迟补偿偏移量(Transceiver Delay Compensation Offset)：仅 CAN FD 支持，用于解决数据段高速传输时的物理层时序偏移用于补偿 CAN FD 模式下 收发器环路延迟 和 信号传播时间 的固定修正值
3. 采样点：CAN控制器在位时间内对总线电平进行采样的精确时刻，用于判定位的逻辑值（显性0或隐性1）

### 取值范围和公式计算
1. 采样点计算：（sync_seg + prop_seg + phase_seg1）/（sync_seg + prop_seg + phase_seg1 + phase_seg2）×100%
2. 同步段固定一个tq
3. prop_seg + phase_seg1>phase_seg2
4. SJW ≤ min(Phase_Seg1, Phase_Seg2)
5. 当配置5M及以上波特率时，需配置补偿参数，补偿参数计算公式如下：
  - TDC offset = (PropSeg + Seg1 + 1) * Fd Prescaler

### 配置仲裁段1M数据段5M实例说明
#### 1. 基础参数确认
- CAN时钟频率（CAN_CLK）: 40 MHz
- 目标波特率（Bit Rate）: 5 Mbps
- 预分频值（Prescaler）: 1（不分频）
- 单Bit时间内的TQ总数: TQ = CAN_CLK/(Bit Rate×Prescaler)=40MHz/5Mbps×1=8TQ
- 时间量子：Tq time = 1 / (40M / prescaler) = 1/40M = 25ns

#### 2. 时间量子（TQ）分配
Sync_Seg（固定段）: 1 TQ（同步段不可修改）

剩余TQ分配: Prop_Seg+Phase_Seg1+Phase_Seg2=8−1=7TQ

此处取Prop_Seg=1，Phase_Seg1=4，Phase_Seg2=2，则采样点= (Sync_Seg+Prop_Seg+Phase_Seg1)/Total TQ x 100% = (1+4+1)/8 x 100%= 75%

#### 3. 延迟补偿偏移量（Transceiver Delay Compensation Offset）

Offset=(Prop_Seg+Phase_Seg1+1)×Prescaler = (1+4+1)×1=6TQ

**SJW（Synchronization Jump Width）设置**：

SJW必须满足: SJW≤min⁡(Phase_Seg1,Phase_Seg2)=min⁡(4,2)=2

因此，配置 SJW = 2 TQ。

#### 4. 终配置参数

根据上述的方式同理可以计算到1M情况下的参数，由于部分寄存器获取到的值会自动加一，所以实际写入的值要减一，具体可看下表

- 5M 75%数据段配置


| 参数名               | 值（TQ或时间） | 需写入寄存器的值 |
|----------------------|----------------|------------------|
| Sync_Seg             | 1 TQ           | 无需写入，固定为1 |
| Prop_Seg             | 1 TQ           | 1                |
| Phase_Seg1           | 4 TQ           | 3                |
| Phase_Seg2           | 2 TQ           | 1                |
| Prescaler            | 1              | 0                |
| SJW                  | 2 TQ           | 1                |
| 延迟补偿偏移量       | 6 TQ            | 6                |

- 1M 80%仲裁段配置

| 参数名        | 值（TQ或时间） | 需写入寄存器的值 |
|---------------|----------------|------------------|
| Sync_Seg      | 1 TQ           | 无需写入，固定为1  |
| Prop_Seg      | 7 TQ           | 6                |
| Phase_Seg1    | 8 TQ           | 7                |
| Phase_Seg2    | 4 TQ           | 3                |
| Prescaler     | 2              | 1                |
| SJW           | 2 TQ           | 1                |


#### 5. 8M的配置相对于5M较为特殊，使用60%的采样

| 参数名          | 值（TQ或时间） | 需写入寄存器的值 |
|-----------------|----------------|------------------|
| Sync_Seg        | 1 TQ           | 无需写入，固定为1  |
| Prop_Seg        | 1 TQ           | 1                |
| Phase_Seg1      | 1 TQ           | 0                |
| Phase_Seg2      | 2 TQ           | 1                |
| Prescaler       | 1              | 0                |
| SJW             | 1 TQ           | 0                |
| 延迟补偿偏移量  | 3 TQ           | 3                |


## 应用sample

### 使用指南

MCU侧CAN2IPC源码目录：mcu/Service/HouseKeeping/can_ipc/hb_CAN2IPC.c

- 源码中hb_CAN2IPC_MainFunction函数被OS周期性调用，其内部通过调用hb_CAN2IPC_Proc 函数将指定的CAN控制器数据通过IPC转发到Acore。
- hb_CAN2IPC_Proc 函数中三个传入参数分别为：CAN控制器、ipc instance、ipc 指定instance下的虚拟chennel。

S100 默认将Can5~Can9 转发给ACORE，示例如下
```C
void hb_CAN2IPC_MainFunction(void) {
hb_CAN2IPC_Proc(CANTRANS_INS0CH5_CONTROLLER, IpcConf_IpcInstance_IpcInstance_0, IpcConf_IpcInstance_0_IpcChannel_4);
//CAN4 for chassis,map to ipc channel 4 and 5，for Acore vechileio and pnc
hb_CAN2IPC_Proc(CANTRANS_INS0CH6_CONTROLLER, IpcConf_IpcInstance_IpcInstance_0, IpcConf_IpcInstance_0_IpcChannel_6);
//CAN6 for radar, map to ipc channel 6
hb_CAN2IPC_Proc(CANTRANS_INS0CH7_CONTROLLER, IpcConf_IpcInstance_IpcInstance_0, IpcConf_IpcInstance_0_IpcChannel_7);
//CAN7 for radar, map to ipc channel 7
hb_CAN2IPC_Proc(CANTRANS_INS0CH8_CONTROLLER, IpcConf_IpcInstance_IpcInstance_0, IpcConf_IpcInstance_0_IpcChannel_2);
//CAN8 for radar, map to ipc channel 2
hb_CAN2IPC_Proc(CANTRANS_INS0CH9_CONTROLLER, IpcConf_IpcInstance_IpcInstance_0, IpcConf_IpcInstance_0_IpcChannel_3);
//CAN9 for radar, map to ipc channel 3
}
```

Acore canhal使用可参考sample源码目录：source/hobot-io-samples/debian/app/Can，可以在S100的/app/Can目录下直接make编译使用。

以多路透传为例，目录结构如下：
```bash
$ tree /app/Can/can_multi_ch
.
├── Makefile
├── config
│   ├── channels.json
│   ├── ipcf_channel.json
│   └── nodes.json
├── main.c
└── readme.md

```
json文件配置主要包括3个json配置文件：node.json、ipcf_channel.json、channels.json。目前为了支持多进程，各个进程都会去当前路径下的config目录下寻找这3个配置文件。

node.json负责创建虚拟Can设备节点给CANHAL API访问。关键配置选项包括：
- channel_id字段指定该虚拟Can设备从ipc配置文件ipcf_channel.json中哪一个节点获取数据。
- target字段表示该虚拟Can设备节点的名称，CANHAL API通过该名称访问指定的节点。
- enable字段表示该节点是否使能。

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
ipcf_channel.json将node.json中用到的ipc节点映射到具体的instance和channel。
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
channels.json指定ipc配置文件，用户一般不需要更改。
```json
{
  "io_channels" : {
   "ipcf" : "./config/ipcf_channel.json"
 }
}
```

Acore无法直接操作CAN外设，需要通过借助Ipc模块来中转数据，与外设通道的映射关系可以查阅 [MCU IPC使用指南](../../../07_Advanced_development/05_mcu_development/01_S100/08_mcu_ipc.md) 中的IPC 使用情况章节。


Acore应用程序通过CANHAL获取MCU侧Can帧的流程伪代码如下：

```c
void send_frame_data(void *arg)
{
    for (int count = 1000; count > 0; count--) {
        canSendMsgFrame(test_params->target, &frame[0], &pack);
    }
}

void *recv_frame_data(void *arg)
{
    while (!exit_flag) {
        canRecvMsgFrame(target, frame, &pack); // non blocking
    }
}

int main(int argc, char *argv[])
{
	ret = canInit();
    pthread_create(&send_thread, NULL, send_frame_data, &tx_params);
    pthread_create(&rx_threads[i], NULL, recv_frame_data, &rx_params[i])

    pthread_join(send_thread, NULL);
    pthread_join(rx_threads[i], NULL);
	canDeInit();
}
```

- 首先执行canInit()完成初始化,然后创建发送线程和接收线程
- 发送线程调用canSendMsgFrame()发送数据包，接收线程调用canRecvMsgFrame()接收数据包，其中target参数为json文件中配置好的通道。
- pack信息包含这一包数据的信息，包括can帧数量、mcu侧的时间戳以及acore侧的monotic时间戳等信息。
- canhal会从这一包ipc数据中解析出can帧，用户通过frame指针读取出所有can帧。
- 最后执行canDeInit()释放资源。

### ACORE侧实例说明

#### 多通道传输
##### 此sample实现CAN总线多通道数据发送与接收：
- **硬件连接**：Can6连接Can7， Can8连接Can9,CAN5连接其他Can设备，不可不接。
- **发送线程**：为每个通道创建独立线程发送数据，数据内容固定。
- **接收线程**：为每个通道创建独立线程接收数据。

发送策略：
- 相隔固定时间通过Can发送数据，可通过修改延时调整发送频率，目前测试4路Can的收发，每路Can的最高发送频率为1000Hz，大于时会出现丢包情况。
- 数据内容：通过CANFD发送扩展帧(64bytes)的数据。
- 目标通道：CAN6~CAN9轮询发送

接收策略:
- 被动接收数据，不进行数据处理
- 超过5s未收到数据则退出程序

##### 目录介绍
```
.
├── Makefile // 主编译脚本
├── config  // 配置文件目录
│   ├── channels.json // 通道映射配置文件
│   ├── ipcf_channel.json // IPCF通道映射配置文件
│   └── nodes.json // Can虚拟设备映射配置文件
├── main.cpp // 主程序
├── readme.md // 说明文件
└── run.sh // 运行脚本
```
##### 依赖
- `pthread`线程库
- `hobot_can_hal` CAN接口库

##### 通道映射关系

| 通道   | 对应线程名称 | 设备名               |
|--------|--------------|----------------------|
| CAN5   | CAN5RX       | "can5_ins0ch4"     |
| CAN6   | CAN6RX       | "can6_ins0ch6"     |
| CAN7   | CAN7RX       | "can7_ins0ch7"     |
| CAN8   | CAN8RX       | "can8_ins0ch2"     |
| CAN9   | CAN9RX       | "can9_ins0ch3"     |


##### DEBUG开关

```C
#define VERBOSE 0 //修改为1时打印调试信息
```
##### 编译命令
```bash
make # 编译
make clean # 清除编译文件
```
##### 运行命令
```
./can_multi_ch
```
观察log可以看到收发数据的基本情况

:::tip
持续更新中
:::

#### 多can组网传输

:::tip
持续更新中
:::

#### 应用层修改can波特率等配置

:::tip
持续更新中
:::

### 库文件打印开关

CAN_HAL_DEBUG_LEVEL 是一个环境变量，用于控制库文件libhbcanhal.so日志输出的级别。它的不同值代表不同的日志级别，这些级别决定了哪些日志信息会被记录下来
```
0 (log_trace): 跟踪级别。
1 (log_debug): 调试级别。
2 (log_info): 信息级别。
3 (log_warn): 警告级别。
4 (log_err): 错误级别。
5 (log_critical): 致命级别。
6 (log_never): 不打印任何日志。
```
通过设置 CAN_HAL_DEBUG_LEVEL 的值，可以控制日志输出的详细程度。例如，如果设置为 2，那么只有 log_info、log_warn、log_err 和 log_critical 级别的日志会被打印

### MCU侧DEBUG应用说明
1. 进入MCU1的控制台
2. 输入命令：can_tran_debug
```
D-Robotics:/$ can_tran_debug
[01217.785483 0]Ipc2Can Statistics: // IPC 收到的数据包统计
[01217.785712 0]Channel 0 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.786895 0]Channel 1 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.788077 0]Channel 2 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.789260 0]Channel 3 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.790443 0]Channel 4 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.791625 0]Channel 5 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.792808 0]Channel 6 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.793991 0]Channel 7 FifoOverflowCnt:          0 MsgOverLengthCnt:          0 MsgCRCErrCnt:          0
[01217.796039 0]Soc2Can Statistics: // 解包之后的数据统计
[01217.796268 0]RecvAllNum: 4
[01217.796605 0]RollCntErr: 3 // 此错误不影响，当acore端应用发送数据填写好有序的序号时，此错误将消失。
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
[01217.806978 0]Hb_CanIf Statistics: // Can控制器收发数据统计
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

### 应用程序接口

#### void Can_Init(const Can_ConfigType* Config)

```shell
Description：This function initializes the module.

Sync/Async: Synchronous
Parameters(in)
    Config: Pointer to driver configuration.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Can_GetVersionInfo(Std_VersionInfoType* versioninfo)

```shell
Description：Returns the version information of this module.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    versioninfo: Pointer to where to store the version information of this module.
Return value：None
```

#### void Can_DeInit(void)

```shell
Description：This function de-initializes the module.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Can_SetControllerMode(uint8 Controller, Can_ControllerStateType Transition)

```shell
Description：This function performs software triggered state transitions of the CAN controller State machine.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which the status shall be changed.
    Transition: Transition value to request new CAN controller state.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
	E_OK: request accepted.
    E_NOT_OK: request not accepted, a development error occurred.
```

#### void Can_DisableControllerInterrupts(uint8 Controller)

```shell
Description：This function disables all interrupts for this CAN controller.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which interrupts shall be disabled.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Can_EnableControllerInterrupts(uint8 Controller)

```shell
Description：This function enables all allowed interrupts.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which interrupts shall be re-enabled.
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### Std_ReturnType Can_GetControllerErrorState(uint8 ControllerId, Can_ErrorStateType* ErrorStatePtr)

```shell
Description：This service obtains the error state of the CAN controller.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: Abstracted CanIf ControllerId which is assigned to a CAN controller, which is requested for ErrorState.
Parameters(inout)
    None
Parameters(out)
    ErrorStatePtr:Pointer to a memory location, where the error state of the CAN controller will be stored.
Return value：Std_ReturnType
	E_OK: Error state request has been accepted.
    E_NOT_OK: Error state request has not been accepted.
```

#### Std_ReturnType Can_GetControllerMode(uint8 Controller, Can_ControllerStateType* ControllerModePtr)

```shell
Description：This service reports about the current status of the requested CAN controller.

Sync/Async: Synchronous
Parameters(in)
    Controller: CAN controller for which the status shall be requested.
Parameters(inout)
    None
Parameters(out)
    ControllerModePtr: Pointer to a memory location, where the current mode of the CAN controller will be stored.
Return value：Std_ReturnType
    E_OK: Controller mode request has been accepted.
    E_NOT_OK: Controller mode request has not been accepted.
```

#### Std_ReturnType Can_GetControllerRxErrorCounter(uint8 ControllerId, uint8* RxErrorCounterPtr)

```shell
Description：Returns the Rx error counter for a CAN controller.
             This value might not be available for all CAN controllers, in which case E_NOT_OK would be
             returned.Please note that the value of the counter might not be correct at the moment the
             API returns it, because the Rx counter is handled as ynchronously in hardware.Applications
             should not trust this value for any assumption about the current bus state.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: CAN controller, whose current Rx error counter shall be acquired.
Parameters(inout)
    None
Parameters(out)
    RxErrorCounterPtr: Pointer to a memory location, where the current Rx error counter of the
                       CAN controller will be stored.
Return value：Std_ReturnType
    E_OK: Rx error counter available.
    E_NOT_OK: Wrong ControllerId, or Rx error counter not available.
```

#### Std_ReturnType Can_GetControllerTxErrorCounter(uint8 ControllerId, uint8* TxErrorCounterPtr)

```shell
Description：Returns the Tx error counter for a CAN controller. This value might not be available
             for all CAN controllers, in which case E_NOT_OK would be returned.Please note that the
             value of the counter might not be correct at the moment the API returns it, because the
             Tx counter is handled as ynchronously in hardware.Applications should not trust this
             value for any assumption about the current bus state.

Sync/Async: Synchronous
Parameters(in)
    ControllerId: CAN controller, whose current Rx error counter shall be acquired.
Parameters(inout)
    None
Parameters(out)
    TxErrorCounterPtr:Pointer to a memory location, where the current Tx error counter
                      of the CAN controller will be stored.
Return value：Std_ReturnType
    E_OK: Rx error counter available.
    E_NOT_OK: Wrong ControllerId, or Rx error counter not available.
```

#### Std_ReturnTypeCan_Write(Can_HwHandleType Hth, const Can_PduType* PduInfo)

```shell
Description：This function is called by CanIf to pass a CAN message to CanDrv for tran smission.

Sync/Async: Synchronous
Parameters(in)
    Hth:information which HW-transmit handle shall be used for transmit.Implicitly this is
        also the information about the controller to use because the Hth numbers are unique
        inside one hardware unit.
Parameters(inout)
    None
Parameters(out)
    None
Return value：Std_ReturnType
    E_OK: Write command has been accepted.
    E_NOT_OK: development error occurred.
    CAN_BUSY: No TX hardware buffer available or pre-emptive call of Can_Write that can’t be
              implemented re-entrant (see Can_ReturnType).
```

#### void Can_MainFunction_Write(Void)

```shell
Description：This function performs the polling of TX confirmation when CAN_TX_PROCESSING
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

#### void Can_MainFunction_Read(Void)

```shell
Description：Returns the value of the specified CAN channel.This function performs the
             polling of RX indications when CAN_RX_PROCESSING is set to POLLING.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Can_MainFunction_BusOff(Void)

```shell
Description：This function performs the polling of bus-off events that are configured statically
             as ‘to be polled’.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```

#### void Can_MainFunction_Mode(Void)

```shell
Description：This function performs the polling of CAN controller mode transitions.

Sync/Async: Synchronous
Parameters(in)
    None
Parameters(inout)
    None
Parameters(out)
    None
Return value：None
```
