---
sidebar_position: 5
---

# 多路Camera及与Lidar同步

## 简述

在多路Camera接入使用场景中，为满足算法或应用需求，一般有多路Camera同步需要，同时还有与Lidar同步的要求，可通过ETH PPS等方式进行同步。

S100上LPWM模块主要是给Camera trigger使用，能提供具有延时输出能力的PWM波形，用于校准sensor的曝光同步，同时其有多个外部trigger源可选，可支持与MCU或GPS设备时间同步。

本文档主要基于该LPWM模块的基本功能，描述其原理与使用方法，提供多路Camera与Lidar典型场景的同步方案，便于实际项目中参考使用。

:::info 注意

  本文档只基于一种Camera与Lidar的硬件连接方案，提供推荐方案作为参考，若使用其他硬件可参考本文档中的配置使用方式自行适配。
:::

## 硬件通路

### LPWM模块

S100总共有3个LPWM chip，每个LPWM chip下面有4个LPWM通道，请根据实际的硬件连接使用配置。

S100的Camera同步功能的主体实现依赖LPWM模块，其支持S100多种trigger信号源，并产生多通道的可配置PWM信号，输出给外部Camera使用(可经SerDes转发)，从而实现trigger源与Camera的同步及所有多Camera之间的同步。

使用LPWM时主要需注意的是：

- 实际硬件连接配置使用的通道。

- LPWM的同步触发源选择: 支持MCU RTC/PPS0/PPS1/ETH PPS0/ETH PPS1/MCU ETH PPS等。

- LPWM基于同步触发源的offset偏移配置: 需求根据帧率要求、PPS周期、相位要求等适配。

- LPWM目标信号波形: period，duty\_time等参数。

- LPWM的扩展功能: 缓慢同步 threshold，adjust\_step 配置。

关于LPWM模块的功能及使用，更多可参考: [LPWM使用]

### LPWM同步源

LPWM工作原理：LPWM 受PPS触发源trigger，其作为Trigger Bus的Target侧设备，每个LPWM设备可独立配置选择trigger源。

收到PPS信号传输给cam-trig的过程如下图所示，PPS trigger会连接到LPWM模块，触发LPWM输出，LPWM的输出连接到camera：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_01.png)

关于同步源PPS的功能说明，更多可参考: [PPS说明](../../02_linux_development/04_driver_development_s100/12_driver_timesync.md#PPS)

:::info 注意

关于LPWM的trigger源使用，有注意如下：

- 多种外部trigger源为多选1，一个LPWM模块只能选其中一种使用。

- 不同的LPWM间通过外部trigger源来同步，选用同一源后可同步。

- 同一LPWM模块的不同channel使用同一trigger源，可配不同的offset/period/duty参数。
:::

### 多路Camera同步连接

对于单S100的Camera接入场景，其一般连接方式如下：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_02.png)

其中：

- S100的trigger源可来自外部GPS设备，或网络ETH PPS。

- MCU与S100之间也可同步。

- 同S100内的不同LPWM之间可选同一同步源实现。

- 不同的DES可以连接不同的LPWM通道。

- SerDes可通过Link线缆上的反向通道将LPWM信号透传到Sensor侧，连接为FSYNC，做同步触发。

### 与Lidar连接配合

在有Lidar使用的场景中，有连接方案有如下:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_03.png)

其中:

- S100的trigger源可来自外部GPS设备，或通过网络授时等方式

- Lidar设备可以通过相应的授时(如gPTP)方式，实现与S100同源时钟对齐

- Camera同步是由通过S100的LPWM输出触发给DES，再分发给各Camera实现同步曝光/出图

- Camera数据输入到S100内CIM模块时，会在Frame Start时刻打上时间戳记录该帧时间。

## 软件方案

### Camera与Lidar时间对齐需求

对于Camera与Lidar同时使用且有同步对齐的场景，其一般的功能需求有如下:

- Lidar与S100间有时间同步，在同源时间轴上工作;

- Lidar可基于同步时间按需求启动周期性扫描，此处以10Hz(100ms周期)为例，可在整百ms处开始扫描。

- Camera通过LPWM基于同步时间PPS作为触发源，触发每帧图像的曝光/读出，实现与Lidar扫描时间的对齐，此处以30fs为例。

- Camera图像数据的时间戳需与Lidar数据的时间戳在同一时间轴，并有一定的对齐关系，以便融合使用。

其期望的时间对齐目标有如下:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_04.png)

在PPS整秒开始: LPWM输出并曝光(若offset设为0)、Lidar开始扫描(若start为0)。

此处软件方案适配上述连接图中实线连接的硬件方案：

- S100与Lidar设备都连接网络交换机，通过网络实现时间同步。

- Lidar扫描频率为10Hz，在整百ms时刻开始扫描，数据带对齐时间戳。

- 多个Camera通过SerDes连接S100,同时接有LPWM，用于Camera的曝光同步的触发。

- LPWM的触发源使用Acore的ETH PPS0。

### LPWM触发源选择

LPWM模块有多种触发源可选，对于上述硬件连接方式，仍可有多种源可以选择，可参考: [LPWM推荐使用配置]

本方案中使用Acore ETH PPS0作为LPWM的触发源：

- 其与Lidar都使用网络同步，可使用PHC时间作为统一时间轴。

- Acore ETH PPS误差最小，建议优先使用。

对于使用Acore ETH PPS0同步源，使用fixed mode时，其上升沿基于PPS整秒时间有一固定偏移536.871ms，需要在使用时按需求进行offset计算与配置，更多可参考: [Acore ETH PPS说明](../../02_linux_development/04_driver_development_s100/12_driver_timesync.md#Acore\_Eth_\PPS)

### Camera同步模式选择

Sensor曝光输出一般有Master模式(主动曝光输出，按配置，只要开流自动出帧)与Slave模式(等待触发曝光输出、需trigger后才输出)，对于一般运行默认使用Master模式运行，而要同步输出则使用Slave模式。

以下为AR0820模组的Slave模式说明：以下以AR0820为例说明。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_05.png)

同样的slave模式，还有多种同步方式，其中常用的为shutter sync: 该模式在trigger后固定时间出图(可保证FS时间戳对齐)，且下不会丢失trigger信号(即在出图过程中来了trigger信号，不会被忽略)。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_06.png)

配置有如下:

```
// 参见: source/hobot-camera/drivers/sensor/ar0820/inc/ar0820_setting.h
uint16_t ar0820_trigger_standard_setting[] = {
    //0x301A, 0x0058, // RESET_REGISTER_RESET,RESET_REGISTER_STDBY_EOF
    0x301A, 0x0958, // RESET_REGISTER_GPI_EN, FORCED_PLL_ON
    0x31C6, 0x2000, // MASK_FRAMER_STANDBY
    0x30B0, 0x8100, // PIXCLK_ON
    0x30CE, 0x0000, // TRIGGER STANDARD MODE
    0x30CE, 0x0000, // TRIGGER STANDARD MODE
};
uint16_t ar0820_trigger_shutter_sync_setting[] = {
    //0x301A, 0x0058, // RESET_REGISTER_RESET,RESET_REGISTER_STDBY_EOF
    0x301A, 0x095C, // RESET_REGISTER_GPI_EN, FORCED_PLL_ON, STREAM
    0x31C6, 0x2000, // MASK_FRAMER_STANDBY
    0x30B0, 0x8100, // PIXCLK_ON
    0x30CE, 0x0120, // TRIGGER SHUTTER SYNC MODE
    0x30CE, 0x0120, // TRIGGER SHUTTER SYNC MODE
};
uint16_t ar0820_trigger_gpio_setting[][8] = {
    {
        0x340A, 0x00EE, // GPIO0_INPUT_ENABLE
        0x340A, 0x00EE, // GPIO0_INPUT_ENABLE
        0x340C, 0x0002, // GPIO_ISEL
        0x340E, 0x2100, // GPIO_OSEL
    },
    {
        0x340A, 0x00DD, // GPIO1_INPUT_ENABLE
        0x340A, 0x00DD, // GPIO1_INPUT_ENABLE
        0x340C, 0x0008, // GPIO_ISEL
        0x340E, 0x2100, // GPIO_OSEL
    },
    {
        0x340A, 0x00BB, // GPIO2_INPUT_ENABLE
        0x340A, 0x00BB, // GPIO2_INPUT_ENABLE
        0x340C, 0x0020, // GPIO_ISEL
        0x340E, 0x2010, // GPIO_OSEL
    },
    {
        0x340A, 0x0077, // GPIO3_INPUT_ENABLE
        0x340A, 0x0077, // GPIO3_INPUT_ENABLE
        0x340C, 0x0080, // GPIO_ISEL
        0x340E, 0x0210, // GPIO_OSEL
    },
};
```

在实际使用时，根据config\_index的使能同步功能(需要sensor库有相应的实现)。

:::info 注意

此上仅为AR0820的示例，其它Camera类似，此处Camera同步功能主要需配置:

Slave模式：根据实际需要配置模组的SYNC模式。

FSYNC选择: 根据模组实际硬件连接，选用正常的GPIO作为FSYNC使用。有的模组包含ISP，只需要配置好FSYNC即可，不需要配置Sensor为Slave模式。
:::

对于Camera的FSYNC信号输出，不同的模组可能有不同的特性，如：

- FSYNC信号只用于对齐曝光输出，并不能控制帧率，因此LPWM的周期需与实际帧率匹配设置。

- 可能是纯Slave模式，没有FSYNC信号就不出帧，也可能只是用于对齐，就算无FSYNC也会出帧。

- 需注意同步模式的选择，是同步曝光，还是同步出图？

- 若为同步曝光，则是曝光时刻同步，曝光完成后直接读图输出，出图时间受曝光时间影响，可能时间戳不完全一致。

- 若为出图同步，即曝光后在指定的同一时刻读图输出，可保证时间戳一致，本方案中使用出图同步方式。

- 不同的模组其曝光时间可能不同，因此在同步完成后输出的时间戳可能不一致。

## 推荐方案

### Camera与Lidar同步对齐方案

基于上述硬件连接与软件方案，有同步方案如下：

- 使用Acore ETH PPS0触发。

- Lidar在整百ms时刻开始扫描，数据带对齐时间戳。

- Camera(以AR0820为例)使用SHUTTER SYNC出图同步，自动曝光。

- 通过LPWM的offset调整相位整百ms，如30fps适配offset=463.129ms对33.333ms取余数=29.8ms。

- Camera在经offset正确配置后，可在整百ms(每3帧)同步输出，与Lidar数据对齐。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/03_multimedia_development/02_S100/cam_sync/05_camera_sync_07.png)

### Camera配置

对于以下为单路AR0820的同步配置:

```
{
             "deserial_0": {
             "deserial_name": "max96712",
             "deserial_addr": "0x29",
             "deserial_gpio": {
                     "camerr_pin": [4, 6, 8, 10],
                     "trig_pin": [5]
             },
             "poc": {
                     "poc_addr": "0x28",
                     "poc_map": "0x1320"
             }
     },
             "port_0": {
             "sensor_name": "ar0820std",
             "serial_addr": "0x41",
             "sensor_addr": "0x11",
             "eeprom_addr": "0x51",
             "sensor_mode": 5,
             "fps": 30,
             "width": 3840,
             "height": 2160,
             "extra_mode": 5,
             "config_index": 512,
             "deserial_index": 0,
             "deserial_port": 0
     }
}
```

此处同步的使能，主要通过config\_index的bit9(+512)完成：

```
{
    "config_0":{
        "port_0":{
            "config_index":512,
        },
    }
}
```

Deserial下的trig\_pin是对硬件连接上LPWM连接的MFP索引的配置，需deserial库中进行相应的支持，同时在json中配置使用:

```
"deserial_gpio": {
        "trig_pin": [5]
},
```

- 若一路LPWM同时触发多路Camera，则只需配置一个索引值，如MFP5：[5]。

- 若多路LPWM分别触发不同路Camera，则需配置多个索引值对应多个Link，如MFP5-A,MFP14-B：[5,14]。

### trigger配置

LPWM的配置由json完成，配置示例如下，更多可参考: [LPWM JSON配置]

```
{
    "lpwm_chn0": {
        "trigger_source": 8,
        "trigger_mode": 1,
        "period": 33333,
        "offset": 29800,
        "duty_time": 100,
        "threshold": 0,
        "adjust_step": 0
    },
    "lpwm_chn1": {
        "trigger_source": 8,
        "trigger_mode": 1,
        "period": 33333,
        "offset": 29800,
        "duty_time": 100,
        "threshold": 0,
        "adjust_step": 0
    },
    "lpwm_chn2": {
        "trigger_source": 8,
        "trigger_mode": 1,
        "period": 33333,
        "offset": 29800,
        "duty_time": 100,
        "threshold": 0,
        "adjust_step": 0
    },
    "lpwm_chn3": {
        "trigger_source": 8,
        "trigger_mode": 1,
        "period": 33333,
        "offset": 29800,
        "duty_time": 100,
        "threshold": 0,
        "adjust_step": 0
    }
}
```

若使用trigger\_mode为1，trigger\_source为8，则使用的是ETH PPS0进行同步。

## 总结

S100上多Camera同步主要通过LPWM模块的硬件连接与软件配置共同实现。

在硬件设计时即需考虑不同帧率的场景需求，不同Camera模组的同步要求，及外部trigger信号的选择。

在模组点亮配置时，也需进行SerDes上的LPWM同步透传，同时Sensor配置为Slave模式，选择相应的GPIO作为FSYNC信号，实现同步触发输出。

在与Lidar设备同步场景，使用ETH PPS0同步源，使用fixed mode，需要正确计算并配置LPWM的offset，以达到完全相位对齐的要求。
