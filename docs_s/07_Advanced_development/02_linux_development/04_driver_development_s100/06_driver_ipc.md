---
sidebar_position: 6
---

# IPC模块介绍

IPC（Inter-Processor Communication）模块是用于多核之间的通信，支持同构核和异构核之间的通信，软件上基于buffer-ring进行共享内存的管理，硬件上基于MailBox实现核间中断。IPCF具有多路通道，大数据传输，适用多种平台的特点。RPMSG基于开源协议框架，支持Acore与VDSP的核间通信。



## IPCF软硬件组件框图

Acore与MCU之间的核间通信，Acore侧主要使用IPCFHAL，MCU侧使用IPCF，其中IPCFHAL是基于IPCF封装了一层接口，用于用户态与内核态的数据传递。


![IPCF软硬件组件框图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/linux-ipc.jpg)


## IPC典型使用场景

![IPC典型使用场景图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/imageipcscen.png)


IPC典型应用场景有OTA模块、诊断模块、规控、CANHAL等。


## IPC实例分配方案

IPC Acore侧实例编号范围为[0-34]，分别用于Acore与MCU通信的实例[0-14]、Acore与VDSP通信的实例[22-24]、Acore与BPU通信的实例[32-34]，其余实例做其它私有用途。Acore与MCU通信可使用实例[0-8]，实例[4-6]默认为客户预留，若用户不需要CANHAL、规控等业务，可以自行修改配置文件。S100中AOCRE与MCU的IPC通信情况可以查阅 [MCU IPC使用指南](../../../07_Advanced_development/05_mcu_development/01_S100/08_mcu_ipc.md) 中的IPC 使用情况章节。

### Acore侧配置实例方法

Acore侧配置实例可通过设备树文件配置，配置路径为：

```dts
source/hobot-drivers/kernel-dts/drobot-s100-ipc.dtsi
source/hobot-drivers/kernel-dts/include/drobot_s100_ipc.h
```


设备树配置信息如下（仅供参考）：

```dts
ipcfhal_cfg: ipcfhal_cfg {
    status = "okay"; #节点状态，不需要改动
    compatible = "hobot,hobot-ipcfhal";  #节点属性，不可改动

    /****************instance--num_chans--num_bufs--buf_size****************/
    ipc-ins = <&ipc_instance0	8	8	0x2000>, #(Acore&MCU)用于CANHAL
            <&ipc_instance1	8	8	0x1000>, #(Acore&MCU)用于规控
            <&ipc_instance2	2	8	0x800>, #(Acore&MCU)用于规控
            <&ipc_instance3	8	8	0x1000>, #(Acore&MCU)用于crypto
            <&ipc_instance4	8	8	0x1000>, #(Acore&MCU)空闲，用户可自行配置
            <&ipc_instance5	8	8	0x1000>, #(Acore&MCU)空闲，用户可自行配置
            <&ipc_instance6	8	8	0x1000>, #(Acore&MCU)空闲，用户可自行配置
            <&ipc_instance7	8	8	0x1000>, #(Acore&MCU)透传uart，spi，i2c等外设和运行mcu侧cmd应用
            <&ipc_instance8	8	8	0x1000>, #(Acore&MCU)部分空闲，用户可自行配置
            <&ipc_instance9	2	5	0x400>,  #(Acore&MCU)私有实例，内部预留
            <&ipc_instance10	1	5	0x200>, #(Acore&MCU)私有实例，内部预留
            <&ipc_instance22	8	8	0x1000>, #(Acore&VDSP)VDSP预留，暂未对客户开放
            <&ipc_instance23	8	8	0x1000>, #(Acore&VDSP)VDSP预留，暂未对客户开放
            <&ipc_instance24	8	8	0x1000>; #(Acore&VDSP)VDSP预留，暂未对客户开放
};

```

### 设备树配置说明

设备树ipcfhal_cfg节点默认配置了一些实例的属性：
- 属性中第一列表示实例编号，必须唯一且在有效范围
- 属性的第二列表示该实例分配的通道数量，用户可以自行配置，最大值为32个
- 属性的第三列表示每个通道的缓冲buf的个数，用户可以自行配置，最大值为1024个，但受控制空间大小的限制。属性第四列表示缓冲buf的大小，单位是Bytes，用户可以自行配置，通道个数\*缓冲buf个数\*buf大小需要小于等于0.5MB（目前每个实例预分配了1MB的数据空间，暂不扩增）。

单个实例的设备树节点如下：

```dts
#Not used, User can apply for it
ipc_instance3: ipc_instance3 {
        status = "okay"; #节点状态，不需要改动
        compatible = "hobot,hobot-ipc"; #节点属性，不可改动
        mbox-names = "mbox-chan"; #mailbox名字属性，不可改动
        mboxes = <&mailbox0 3 19 3>; #mailbox通信方向，实例5和实例6需要改动，其它实例不需要改动
        instance = <3>; #实例id，不需要改动
        data_local_addr = /bits/ 64 <IPC_INS3_DATA_LOCAL>; #Acore数据段，实例5和实例6需要改动，其它实例不需要改动
        data_remote_addr = /bits/ 64 <IPC_INS3_DATA_REMOTE>; #MCU数据段，实例5和实例6需要改动，其它实例不需要改动
        data_size = <IPC_SINGLE_DATA_SIZE>; #数据段大小，不可改动，单位Bytes
        ctrl_local_addr = /bits/ 64 <IPC_INS3_CTRL_LOCAL>; #Acore控制段，实例5和实例6需要改动，其它实例不需要改动
        ctrl_remote_addr = /bits/ 64 <IPC_INS3_CTRL_REMOTE>; #MCU控制段，实例5和实例6需要改动，其它实例不需要改动
        ctrl_size = <IPC_SINGLE_CTRL_SIZE>; #数据段大小，不可改动，单位Bytes
};

```
由于实例5和实例6内部用于测试，外部客户若需要使用，需要自行配置，配置信息如下，不需要改动。

```dts
ipc_instance5: ipc_instance5 {
        status = "okay";
        compatible = "hobot,hobot-ipc";
        mbox-names = "mbox-chan";
        mboxes = <&mailbox0 5 21 5>;
        instance = <5>;
        data_local_addr = /bits/ 64 <IPC_INS5_DATA_LOCAL>;
        data_remote_addr = /bits/ 64 <IPC_INS5_DATA_REMOTE>;
        data_size = <IPC_SINGLE_DATA_SIZE>;
        ctrl_local_addr = /bits/ 64 <IPC_INS5_CTRL_LOCAL>;
        ctrl_remote_addr = /bits/ 64 <IPC_INS5_CTRL_REMOTE>;
        ctrl_size = <IPC_SINGLE_CTRL_SIZE>;
};

ipc_instance6: ipc_instance6 {
        status = "okay";
        compatible = "hobot,hobot-ipc";
        mbox-names = "mbox-chan";
        mboxes = <&mailbox0 6 22 6>;
        instance = <6>;
        data_local_addr = /bits/ 64 <IPC_INS6_DATA_LOCAL>;
        data_remote_addr = /bits/ 64 <IPC_INS6_DATA_REMOTE>;
        data_size = <IPC_SINGLE_DATA_SIZE>;
        ctrl_local_addr = /bits/ 64 <IPC_INS6_CTRL_LOCAL>;
        ctrl_remote_addr = /bits/ 64 <IPC_INS6_CTRL_REMOTE>;
        ctrl_size = <IPC_SINGLE_CTRL_SIZE>;
};
```

**设备树配置注意事项:**

- 实例3~8数据段默认预分配了1MB空间，Acore侧使用0.5MB，MCU侧使用0.5MB，所以通道个数\*缓冲buf个数\*buf大小需要小于等于0.5MB。
- 实例3~8控制段默认预分配了5KB空间，Acore侧使用2.5KB，MCU侧使用2.5KB，存放环形buf的控制信息和状态信息，所以(缓冲buf个数+2)\*16\*通道个数+8需要小于等于2.5KB。
- 实例5~6用于地瓜机器人内部测试，用户可按照上面配置，修改设备树节点即可。 // TODO:可以完全放开给到客户
- 每个实例的通道数量需要小于等于32，缓冲buf个数需要小于等于1024，同时需要满足前两点的不等式。
- 多个业务使用同一个实例的不同通道或者使用不同实例对传输影响不大，主要是参考buf_size/buf_num是否合适以及业务的开发和维护是否方便。
- 底层Mailbox中断分配不支持修改。
- Acore与MCU的通道个数、缓冲buf个数和大小需要保持一致，数据段和控制段的local和remote需要相反。
- 实例的控制段首地址存储实例是否初始化的状态，可用于判断实例是否初始化，其中Acore默认启动kernel时完成初始化。
- 若用户需要自行分配数据段和地址段，则需要修改Acore设备树文件、Uboot设备树文件以及MCU配置文件。
- 在同一个 channel 中，发送（push）和接收（pop）使用独立的 ring buffer 和中断机制，因此收发操作是相互独立、互不影响的。

## 用户态IPC应用和配置文件的使用方法

IPC Sample实现Acore与MCU之间的IPC收发通信，展示IPC多实例多通道多线程的使用示例。

![Acore与MCU之间的IPC收发通信](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/sampleframeware.png)

Sample软件架构图中Acore使用libipcfhal的接口进行数据收发，底层基于ipcf的驱动，MCU直接使用ipcf的接口进行收发。其中由于Acore侧有多套IPC接口，便于区分，分别描述为IPCFHAL、RPMSG、IPCF，MCU侧只有一套IPC接口，因此IPCF在MCU侧文档统一描述为IPC。

### 硬件数据流说明

Sample的共享内存数据流和中断信号流


![共享内存数据流和中断信号流](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/sampledataflow.png)

Sample中Acore与MCU通过共享内存传输数据，通过mailbox中断通知双方。

### Acore侧Sample代码位置与目录结构

**代码路径：**
```bash
# Sample源码路径
${SDK}/source/hobot-io-samples/debian/app/ipcbox_sample  # ipc C++  Sample
${SDK}/source/hobot-io-samples/debian/app/pyhbipchal_sample # ipc python Sample
${SDK}/source/hobot-io/debian/app/pyhbipchal # ipc C++库为转换pyhton库源码


# 源码随固件一同打包，可在S100自行编译, 路径如下
${S100}/app/ipcbox_sample
${S100}/app/pyhbipchal_sample
```

**目录结构：**

```bash
root@ubuntu:/app/ipcbox_sample# tree .
.
├── ipcbox_runcmd # 运行mcu侧cmd命令Sample
│   ├── Makefile # Sample编译框架
│   ├── ipcbox_runcmd.cpp # Sample代码
│   └── ipcfhal_sample_config.json # Sample配置文件
└── ipcbox_uart # ipc透传uart Sample
    ├── Makefile # Sample编译框架
    ├── ipcbox_uart.cpp # Sample代码
    └── ipcfhal_sample_config.json # Sample配置文件

```

### API流程说明

Acore与MCU(IRQ方式)之间API Sample运行流程图
![Acore与MCU(IRQ方式)之间API Sample运行流程图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/sampleirqapi.png)

Acore与MCU(POLL方式)之间API Sample运行流程图
![Acore与MCU(POLL方式)之间API Sample运行流程图](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/samplepollapi.png)

### 通路配置
可以增加Json文件中的通道数量config_num，并增加通道信息，本Sample未支持增加通道的功能，若需要增加通道，需要修改Acore和MCU两侧配置文件。
```json
{
        "log_level": 0, # 日志等级，可省略
        "config_num": 4, # 配置通道数量
        "config_num_max":256, #配置通道数量最大值
        "config_0": { # 配置通道
                "name": "cpu2mcu_ins7ch0", # 通道名字
                "instance": 7, # 实例id
                "channel": 0, # 通道id
                "pkg_size_max": 4096, # 发送包最大字节，推荐小于等于4096Bytes
                "fifo_size": 64000, # 缓冲fifo的大小，取决于需要缓冲的个数
                "fifo_type": 0, # 缓冲fifo的类型，仅支持0
                "ipcf_dev_path":"/dev/ipcdrv", # 字符设备驱动，仅支持/dev/ipcdrv
                "ipcf_dev_name":"ipcdrv" # 字符设备驱动名字，仅支持ipcdrv
        },
        "config_1": {
                "name": "cpu2mcu_ins7ch1",
                "instance": 7,
                "channel": 1,
                "pkg_size_max": 4096,
                "fifo_size": 64000,
                "fifo_type": 0,
                "ipcf_dev_path":"/dev/ipcdrv",
                "ipcf_dev_name":"ipcdrv"
        },
        "config_2": {
                "name": "cpu2mcu_ins8ch0",
                "instance": 8,
                "channel": 0,
                "pkg_size_max": 4096,
                "fifo_size": 64000,
                "fifo_type": 0,
                "ipcf_dev_path":"/dev/ipcdrv",
                "ipcf_dev_name":"ipcdrv"
        },
        "config_3": {
                "name": "cpu2mcu_ins8ch1",
                "instance": 8,
                "channel": 1,
                "pkg_size_max": 4096,
                "fifo_size": 64000,
                "fifo_type": 0,
                "ipcf_dev_path":"/dev/ipcdrv",
                "ipcf_dev_name":"ipcdrv"
        }
}

```

### 实例说明
### C++ 应用

#### RunCmd应用

此sample实现了对读取了ADC chanel的电压。
1. 开机进入S100后，打开应用目录/app/ipcbox_sample/ipcbox_runcmd
2. 编译：make
3. 运行: ./ipcbox_runcmd
4. 出现`Extracted adc data:{"adc_ch":1,"adc_result":2411,"adc_mv":1059}`打印则测试通过
其中表示adc对应pin口，adc_mv表示读出来的电压值

```
root@ubuntu:/app/ipcbox_sample/ipcbox_runcmd# ./ipcbox_runcmd
[INFO][hb_ipcf_hal.cpp:282] [channel] cpu2mcu_ins7ch0 [ins] 7 [id] 0 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] cpu2mcu_ins7ch0 [ins] 7 [id] 0 config success.
[INFO][hb_ipcf_hal.cpp:282] [channel] cpu2mcu_ins7ch1 [ins] 7 [id] 1 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] cpu2mcu_ins7ch1 [ins] 7 [id] 1 config success.
Extracted adc data:{"adc_ch":1,"adc_result":2411,"adc_mv":1059}
TxCmdItem(96)
44 2D 49 50 43 42 4F 58 01 00 00 00 55 07 00 00
69 70 63 73 65 6E 64 5F 61 64 63 20 31 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

rx_packet(160):D-IPCBOX
44 2D 49 50 43 42 4F 58 01 00 00 00 4B 10 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
A0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
7B 22 61 64 63 5F 63 68 22 3A 31 2C 22 61 64 63
5F 72 65 73 75 6C 74 22 3A 32 34 31 31 2C 22 61
64 63 5F 6D 76 22 3A 31 30 35 39 7D 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 C9 F5 5A 58 13 C3 E8 F0 1C E7 DE FF FF 00 00
CC 74 22 8A FF FF 00 00 74 47 67 8A FF FF 00 00
A4 0F 1F D4 AA AA 00 00 06 00 00 00 01 00 00 00
58 1D E7 DE FF FF 00 00 58 1D E7 DE FF FF 00 00
01 00 00 00 00 00 00 00 D8 1C 20 D4 AA AA 00 00
40 C0 69 8A FF FF 00 00 A4 0F 1F D4 AA AA 00 00

[INFO][hb_ipcf_hal.cpp:553] [channel] cpu2mcu_ins7ch0 [ins] 7 [id] 0 deinit success.
[INFO][hb_ipcf_hal.cpp:553] [channel] cpu2mcu_ins7ch1 [ins] 7 [id] 1 deinit success.
```


#### Uart透传
此sample实现了对Uart5的透传，测试时需要将uart5的TX和RX短接。

1. 开机进入S100后，打开应用目录cd /app/ipcbox_sample/ipcbox_uart
2. 编译：make
3. 运行: ./ipcbox_uart
4. 出现tx_data and rx_data are identical.打印则测试通过, 参考log如下：
```
# ./ipcbox_uart
[INFO][hb_ipcf_hal.cpp:282] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 config success.
tx_data(64)
69 70 63 5F 72 75 6E 63 6D 64 5F 73 65 6E 64 20
37 20 30 20 31 32 33 34 35 36 37 38 39 20 31 30
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

rx_data(64)
69 70 63 5F 72 75 6E 63 6D 64 5F 73 65 6E 64 20
37 20 30 20 31 32 33 34 35 36 37 38 39 20 31 30
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

tx_data and rx_data are identical.
[INFO][hb_ipcf_hal.cpp:553] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 deinit success.
```


### Python应用

S100提供python库文件使用IPC，其原理为通过pybind11调用C++接口，函数名与宏定义等两端保持一致。

1. 包的导入
```
import pyhbipchal as pyipc
import pyhbipchal_utils as ipc_utils
```

2. 源码路径

```bash
root@ubuntu:/app/pyhbipchal_sample# tree
.
├── ipcfhal_sample_config.json // 配置文件，用于初始化ipc
├── pyhbipchal_test.py // 使用pyhbipchal库编写基础python应用测试过用例
├── pyhbipchal_utils.py // pyhbipchal_utils对象源码，pyhbipchal进行二次封装,相较于pyhbipchal更符合pyhton的编程习惯
└── pyhbipchal_utils_test.py // pyhbipchal_utils测试用例
```

3. 示例

测试python库的效果与C++提供的接口是否一致
```bash
root@ubuntu:/app/pyhbipchal_sample# python pyhbipchal_test.py
Library version: 1.0.0
====================test error code==================

IPCF_HAL_E_OK (0): General OK

IPCF_HAL_E_NOK (-1): General Not OK

IPCF_HAL_E_CONFIG_FAIL (-2): Config fail

IPCF_HAL_E_WRONG_CONFIGURATION (-3): Wrong configuration

IPCF_HAL_E_NULL_POINTER (-4): A null pointer was passed as an argument

IPCF_HAL_E_PARAM_INVALID (-5): A parameter was invalid

IPCF_HAL_E_LENGTH_TOO_SMALL (-6): Length too small

IPCF_HAL_E_INIT_FAILED (-7): Initialization failed

IPCF_HAL_E_UNINIT (-8): Called before initialization

IPCF_HAL_E_BUFFER_OVERFLOW (-9): Source address or destination address Buffer overflow

IPCF_HAL_E_ALLOC_FAIL (-10): Source alloc fail

IPCF_HAL_E_TIMEOUT (-11): Expired the time out

IPCF_HAL_E_REINIT (-12): Re initilize

IPCF_HAL_E_BUSY (-13): Busy

IPCF_HAL_E_CHANNEL_INVALID (-14): Channel is invalid

=====================test OK=======================

[INFO][hb_ipcf_hal.cpp:282] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 config success.
tx_data(64)
69 70 63 5F 72 75 6E 63 6D 64 5F 73 65 6E 64 20
37 20 30 20 31 32 33 34 35 36 37 38 39 20 31 30
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

rx_data(64)
69 70 63 5F 72 75 6E 63 6D 64 5F 73 65 6E 64 20
37 20 30 20 31 32 33 34 35 36 37 38 39 20 31 30
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

tx_data and rx_data are identical.
[INFO][hb_ipcf_hal.cpp:553] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 deinit success.
root@ubuntu:/app/pyhbipchal_sample#

```

测试pyhbipchal_utils包的IPC通信功能是否正常。

```bash
root@ubuntu:/app/pyhbipchal_sample# python pyhbipchal_utils_test.py
[INFO][hb_ipcf_hal.cpp:282] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 init success.
[INFO][hb_ipcf_hal.cpp:333] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 config success.
Tx: b'ipc_runcmd_send 7 0 123456789 10' | Rx: b'ipc_runcmd_send 7 0 123456789 10'
[INFO][hb_ipcf_hal.cpp:553] [channel] cpu2mcu_ins7ch4 [ins] 7 [id] 4 deinit success.
```

## Acore与MCU的传输流程

Acore与MCU IPC通信使用MCU MDMA将数据在DDR与MCU SRAM之间搬运，MCU发送数据到Acore的流程中，MCU先使用MDMA将数据从SRAM搬运到DDR，再发送中断通知， Acore发送数据到MCU的流程中，MCU收到中断通知后，使用MDMA将数据从DDR搬运到SRAM。

### MCU发送数据到Acore

![MCU发送数据到Acore](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/imager52a78.png)

### Acore发送数据到MCU

![Acore发送数据到MCU](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/imagea78r52.png)

### IPCFHAL接口使用序列

![Acore发送数据到MCU](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/imageipcfhalapi.png)

IPCFHAL在Acore与MCU通信时，MCU侧用户使用IPCF接口。

### IPCFHAL使用注意事项

- ipcfhal同一个通道不支持多进程；若使用多线程收发，收发时序和逻辑需要用户保障。
- MCU侧SRAM fifo 太小仅用于内部使用。出于节省SRAM的角度，客户可以使用DDR方案。
- Acore侧每个实例的中断对应的优先级不可配置，MCU可配置。
- 若应用对系统调度延时的波动比较敏感，可调整应用程序的调度策略为SCHED_FIFO。

### IPCFHAL调试方法

#### 调试日志

IPCFHAL及底层的IPCF、Mailbox驱动都提供了较完整的log信息，若有调试问题，可查看log输出，帮助定位问题。

#### error code (api 返回值)

IPCFHAL定义了一些error code，覆盖了常见的错误类型，可使用接口：hb_ipcfhal_trans_err将error code转换为错误描述。

## sysfs调试节点(Acore侧)

### statistic调试节点
打印统计的通信信息，包括pkg，pkg_len，err_acq，err_shm_tx，err_cb。

#### 【节点路径】
```bash
//ins-X表示insid，若insid是5，则为ins-5
/sys/kernel/debug/ipcdrv-ins-5/statistic
```
#### 【节点作用】
```c
//以instance为单位，统计所有channel接收&发送的count
struct ipc_statistic_t {
        uint32_t acq_cnt;/**< tx: acquire buf count*/
        uint32_t shm_tx_cnt;/**< tx: send count*/
        uint32_t cb_cnt;/**< rx: callback count*/
        uint32_t err_acq;/**< tx: error acquire buf count*/
        uint32_t err_shm_tx;/**< tx: error send count*/
        uint32_t err_cb;/**< rx: error callback count*/

        uint32_t packages;/**< tx/rx: packages count*/
        uint64_t datalen;/**< tx/rx: datalen*/
};
```
#### 【节点使用】
```
//方法一：直接操作sys节点
//通道完成初始化，未收发数据时, enable
echo 0 > /sys/kernel/debug/ipcdrv-ins-5/statistic
//收发数据结束，且通道未去初始化时，get
cat /sys/kernel/debug/ipcdrv-ins-5/statistic


//方法二：使用open/write/read/close操作
fd= open(/sys/kernel/debug/ipcdrv-ins-5/statistic, O_RDWR);
write(fd, buf, 1024);
//tx/rx，收发数据
read(fd, buf, 1024);
close(fd);
```
#### 【log】
```bash
DataLink:
                                pkg              pkg_len    err_acq err_shm_tx     err_cb
        INS5CH0 TX:              1                   32          0          0          0
        INS5CH0 RX:              1                   32          0          0          0
        INS5CH1 TX:              0                    0          0          0          0
        INS5CH1 RX:              0                    0          0          0          0
        INS5CH2 TX:              0                    0          0          0          0
        INS5CH2 RX:              0                    0          0          0          0
        INS5CH3 TX:              0                    0          0          0          0
        INS5CH3 RX:              0                    0          0          0          0
        INS5CH4 TX:              0                    0          0          0          0
        INS5CH4 RX:              0                    0          0          0          0
        INS5CH5 TX:              0                    0          0          0          0
        INS5CH5 RX:              0                    0          0          0          0
        INS5CH6 TX:              0                    0          0          0          0
        INS5CH6 RX:              0                    0          0          0          0
        INS5CH7 TX:              0                    0          0          0          0
        INS5CH7 RX:              0                    0          0          0          0
```


### tsdump调试节点
以channel为单位，收发数据时打印时间戳。

#### 【节点路径】
```bash
//ins-X表示insid，若insid是5，则为ins-5
/sys/kernel/debug/ipcdrv-ins-5/tsdump
```
#### 【节点作用】
```c
//以channel为单位，使能收发数据时，打印时间戳log
int32_t tsdump;/**< >=0, 使能指定channel的时间戳打印log， <0, 关闭时间戳log*/
```

#### 【节点使用】
```c
//方法一：直接操作sys节点
//通道完成初始化，未收发数据时, enable
echo 0 > /sys/kernel/debug/ipcdrv-ins-5/tsdump
//收发数据过程中，会打印时间戳log
//收发数据结束，且通道未去初始化时，get
cat /sys/kernel/debug/ipcdrv-ins-5/tsdump


//方法二：使用open/write/read/close操作
fd= open(/sys/kernel/debug/ipcdrv-ins-5/tsdump, O_RDWR);
write(fd, buf, 1024);
//tx/rx，打印时间戳log
read(fd, buf, 1024);
close(fd);
```
#### 【log】
```bash
[ 1173.246630] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] tx wt sta: 1717558158.887241446
[ 1173.246642] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] tx wt end: 1717558158.887253646
[ 1173.246717] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx cb sta: 1717558158.887327971
[ 1173.246723] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx cb end: 1717558158.887334796
[ 1173.246725] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx rd sta: 1717558158.887336446
[ 1173.246727] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx sm sta: 1717558158.887338496
[ 1173.246729] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx sm end: 1717558158.887340121
[ 1173.246730] ipc-shm-hal: dev_print_timestamp()[515]: [5][0] rx rd end: 1717558158.887341646
libipcfhal-test: TestBody() [2328] info :
tsdump: 0

libipcfhal-test: TestBody() [2329] info :
tsdump: 0
```
wdump调试节点
以channel为单位，打印发送的数据。

#### 【节点路径】
```bash
//ins-X表示insid，若insid是5，则为ins-5
/sys/kernel/debug/ipcdrv-ins-5/wdump
```
#### 【节点作用】
```bash
//以channel为单位，使能发送数据dump，打印发送的数据
//打印的长度取决于dumplen，若未配置dumplen，则默认打印全部
int32_t wdump;/**< =chan_id, 使能发送dump， 否则, 关闭发送dump*/
```
#### 【节点使用】
```c
//方法一：直接操作sys节点
//通道完成初始化，未发送数据时, enable
echo 0 > /sys/kernel/debug/ipcdrv-ins-5/wdump
//发送数据过程中，会打印发送的数据
//收发数据结束，且通道未去初始化时，get
cat /sys/kernel/debug/ipcdrv-ins-5/wdump


//方法二：使用open/write/read/close操作
fd= open(/sys/kernel/debug/ipcdrv-ins-5/wdump, O_RDWR);
write(fd, buf, 1024);
//tx/rx，打印发送的数据
read(fd, buf, 1024);
close(fd);
```

#### 【log】
```c
[ 1022.271650] ipc-shm-hal: hal_ipc_shm_write()[926]: [5][0] tx size 32
[ 1022.271666] ipc-shm-hal: ipcf_dump_data()[519]: dump info: tx data len[32] mul[1] remain[0]
[ 1022.271700] ipc-shm-hal: ipcf_dump_data()[522]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[ 1022.273489] ipc-shm-hal: hal_ipc_shm_write()[926]: [6][0] tx size 32
[ 1022.273505] ipc-shm-hal: ipcf_dump_data()[519]: dump info: tx data len[32] mul[1] remain[0]
[ 1022.273733] ipc-shm-hal: ipcf_dump_data()[522]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
libipcfhal-test: TestBody() [2403] info :
wdump: 0

libipcfhal-test: TestBody() [2404] info :
wdump: 0
```
### rdump调试节点
以channel为单位，打印接收的数据。

#### 【节点路径】
```
//ins-X表示insid，若insid是5，则为ins-5
/sys/kernel/debug/ipcdrv-ins-5/rdump
```
#### 【节点作用】
```c
//以channel为单位，使能接收数据dump，打印接收的数据
//打印的长度取决于dumplen，若未配置dumplen，则默认打印全部
uint32_t rdump;/**< =chan_id, 使能接收dump，否则，关闭接收dump*/
```
#### 【节点使用】
```bash
//方法一：直接操作sys节点
//通道完成初始化，未收发数据时, enable
echo 0 > /sys/kernel/debug/ipcdrv-ins-5/rdump
//发送数据过程中，会打印接收的数据
//收发数据结束，且通道未去初始化时，get
cat /sys/kernel/debug/ipcdrv-ins-5/rdump


//方法二：使用open/write/read/close操作
fd= open(/sys/kernel/debug/ipcdrv-ins-5/rdump, O_RDWR);
write(fd, buf, DEV_README_BUFSIZE);
//tx/rx，打印接收的数据
read(fd, buf, DEV_README_BUFSIZE);
close(fd);
```
#### 【log】

```bash
[  983.730497] ipc-shm-hal: data_callback()[803]: [6][0] callback size 32
[  983.730524] ipc-shm-hal: ipcf_dump_data()[519]: dump info: callback rx len[32] mul[1] remain[0]
[  983.730540] ipc-shm-hal: ipcf_dump_data()[522]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[  983.731415] ipc-shm-hal: data_callback()[803]: [5][0] callback size 32
[  983.731431] ipc-shm-hal: ipcf_dump_data()[519]: dump info: callback rx len[32] mul[1] remain[0]
[  983.731443] ipc-shm-hal: ipcf_dump_data()[522]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
libipcfhal-test: TestBody() [2478] info :
rdump: 0

libipcfhal-test: TestBody() [2479] info :
rdump: 0
```

### dumplen调试节点
以channel为单位，配置dump数据长度。
【节点路径】
```bash
//ins-X表示insid，若insid是5，则为ins-5
/sys/kernel/debug/ipcdrv-ins-5/dumplen
```
#### 【节点作用】
```bash
//以channel为单位，配置dump数据的长度
//dumplen>0且dumplen<data_len时有效
//前提条件是使能wdump或rdump
uint32_t dumplen;/**< dump数据时，打印数据的字节数*/
```
#### 【节点使用】
```bash
//方法一：直接操作sys节点
//通道完成初始化，未发送数据时, enable
echo 0 > /sys/kernel/debug/ipcdrv-ins-5/dumplen
//发送数据过程中，会打印发送的数据
//收发数据结束，且通道未去初始化时，get
cat /sys/kernel/debug/ipcdrv-ins-5/dumplen


//方法二：使用open/write/read/close操作
fd= open(/sys/kernel/debug/ipcdrv-ins-5/dumplen, O_RDWR);
write(fd, buf, DEV_README_BUFSIZE);
//tx/rx，打印发送的数据
read(fd, buf, DEV_README_BUFSIZE);
close(fd);
```
#### 【log】
```bash
[  852.898250] ipc-shm-hal: hal_ipc_shm_write()[926]: [5][0] tx size 32
[  852.898283] ipc-shm-hal: ipcf_dump_data()[519]: dump info: tx data len[16] mul[0] remain[16]
[  852.898315] ipc-shm-hal: ipcf_dump_data()[539]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00
[  852.898519] ipc-shm-hal: data_callback()[803]: [6][0] callback size 32
[  852.898542] ipc-shm-hal: ipcf_dump_data()[519]: dump info: callback rx len[16] mul[0] remain[16]
[  852.898571] ipc-shm-hal: ipcf_dump_data()[539]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00
[  853.900076] ipc-shm-hal: hal_ipc_shm_write()[926]: [6][0] tx size 32
[  853.900112] ipc-shm-hal: ipcf_dump_data()[519]: dump info: tx data len[16] mul[0] remain[16]
[  853.900143] ipc-shm-hal: ipcf_dump_data()[539]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00
[  853.900359] ipc-shm-hal: data_callback()[803]: [5][0] callback size 32
[  853.900373] ipc-shm-hal: ipcf_dump_data()[519]: dump info: callback rx len[16] mul[0] remain[16]
[  853.900401] ipc-shm-hal: ipcf_dump_data()[539]: 0x0000: 00 01 02 03 04 05 06 07 00 00 00 00 00 00 00 00
libipcfhal-test: TestBody() [2576] info :
dumplen: 16

libipcfhal-test: TestBody() [2577] info :
dumplen: 16

libipcfhal-test: TestBody() [2584] info :
rdump: 0

libipcfhal-test: TestBody() [2585] info :
rdump: 0

libipcfhal-test: TestBody() [2592] info :
wdump: 0

libipcfhal-test: TestBody() [2593] info :
wdump: 0
```
