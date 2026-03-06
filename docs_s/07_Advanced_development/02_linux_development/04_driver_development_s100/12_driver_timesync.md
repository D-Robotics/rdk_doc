---
sidebar_position: 12
---

# 时间同步方案

## 名词缩写及解释

| **缩写** | **解释**                                     |
| -------------- | -------------------------------------------------- |
| GPS            | Global Positioning System                          |
| RTC            | Real\_Time Clock                                   |
| PHC            | PTP hardware clock                                 |
| PTP            | precision time protocol                            |
| gPTP           | precision time protocol, extension of PTP Protocol |
| MCU            | Microcontroller Unit                               |
| UART           | Universal Asynchronous Receiver/Transmitter        |
| CAN            | Controller Area Network                            |
| PPS            | Pulse Per Second                                   |
| NMEA           | National Marine Electronics Association            |
| NTP            | Network Time Protocol                              |
| NIC            | Network Interface Card                             |

## ptp时间同步

### 功能

该软件包含两个程序：ptp4l和phc2sys。这两个软件结合使用，就可以实现从master获取时间，同步S100的PHC时间和Linux系统时间。

### ptp4l使用方法

#### 命令行参数

ptp4l支持gptp功能，可以作为master，也可以作为slave；如果作为slave，可以从master获取时间，同步S100的PHC时间和RTC时间。

可以通过ptp4l -h查看帮助信息：

```
  root@ubuntu:/userdata# ptp4l -h

  usage: ptp4l [options]

  Delay Mechanism

  -A Auto, starting with E2E
  -E E2E, delay request-response (default)
  -P P2P, peer delay mechanism

  Network Transport

  -2 IEEE 802.3
  -4 UDP IPV4 (default)
  -6 UDP IPV6

  Time Stamping

  -H HARDWARE (default)
  -S SOFTWARE
  -L LEGACY HW

  Other Options

  -f [file] read configuration from 'file'
  -i [dev] interface device to use, for example 'eth0'
  (may be specified multiple times)
  -p [dev] Clock device to use, default auto
  (ignored for SOFTWARE/LEGACY HW time stamping)
  -s slave only mode (overrides configuration file)
  -l [num] set the logging level to 'num'
  -m print messages to stdout
  -q do not print messages to the syslog
  -v prints the software version and exits
  -h prints this message and exits
```

#### 配置文件的使用

ptp4l可以通过-f参数指定配置文件。

配置文件按段划分，空行和\#开头的行会被忽略。

有三种段类型：

\[global\]段，用来配置program选项，clock选项，默认port选项。

port段使用被配置的网口的名字，如\[eth0\]段，其配置的选项会覆盖\[global\]段中默认port选项。port段可以为空内容，作用只是指定网口，这样命令行中不必使用-i选项。

\[unicast\_master\_table\]段，配置单播table。

配置文件的详细信息可以参考：[https://linuxptp.nwtime.org/documentation/ptp4l/](https://linuxptp.nwtime.org/documentation/ptp4l/)

#### automotive配置示例

automotive-master.cfg:

```
#
# Automotive Profile example configuration for master containing those
# attributes which differ from the defaults.  See the file, default.cfg, for
# the complete list of available options.
#
[global]
# Options carried over from gPTP.
gmCapable       1
priority1       248
priority2       248
logSyncInterval     -3
syncReceiptTimeout  3
neighborPropDelayThresh 800
min_neighbor_prop_delay -20000000
assume_two_step     1
path_trace_enabled  1
follow_up_info      1
transportSpecific   0x1
ptp_dst_mac     01:80:C2:00:00:0E
network_transport   L2
delay_mechanism     P2P
#
# Automotive Profile specific options
#
BMCA            noop
masterOnly      1
inhibit_announce    1
asCapable               true
inhibit_delay_req       1
```

automotive-slave.cfg:

```
#
# Automotive Profile example configuration for slaves containing those
# attributes which differ from the defaults.  See the file, default.cfg, for
# the complete list of available options.
#
[global]
#
# Options carried over from gPTP.
#
gmCapable       1
priority1       248
priority2       248
logSyncInterval     -3
syncReceiptTimeout  3
neighborPropDelayThresh 800
min_neighbor_prop_delay -20000000
assume_two_step     1
path_trace_enabled  1
follow_up_info      1
transportSpecific   0x1
ptp_dst_mac     01:80:C2:00:00:0E
network_transport   L2
delay_mechanism     P2P
#
# Automotive Profile specific options
#
BMCA            noop
slaveOnly       1
inhibit_announce    1
asCapable               true
ignore_source_id    1
# Required to quickly correct Time Jumps in master
step_threshold          1
operLogSyncInterval     0
operLogPdelayReqInterval 2
msg_interval_request     1
servo_offset_threshold   30
servo_num_offset_values  10
```

### phc2sys使用方法

phc2sys用于将linux系统时间同步到phc时间，或者将phc时间同步到linux系统时间。

可以通过phc2sys --h查看使用说明：

```
  root@ubuntu:/userdata# phc2sys -h

  usage: phc2sys [options]


  automatic configuration:
  -a turn on autoconfiguration
  -r synchronize system (realtime) clock
  repeat -r to consider it also as a time source
  manual configuration:
  -c [dev|name] slave clock (CLOCK_REALTIME)
  -d [dev] master PPS device
  -s [dev|name] master clock
  -O [offset] slave-master time offset (0)
  -w wait for ptp4l
  common options:
  -f [file] configuration file
  -E [pi|linreg] clock servo (pi)
  -P [kp] proportional constant (0.7)
  -I [ki] integration constant (0.3)
  -S [step] step threshold (disabled)
  -F [step] step threshold only on start (0.00002)
  -R [rate] slave clock update rate in HZ (1.0)
  -N [num] number of master clock readings per update (5)
  -L [limit] sanity frequency limit in ppb (200000000)
  -M [num] NTP SHM segment number (0)
  -u [num] number of clock updates in summary stats (0)
  -n [num] domain number (0)
  -x apply leap seconds by servo instead of kernel
  -z [path] server address for UDS (/var/run/ptp4l)
  -l [num] set the logging level to 'num' (6)
  -t [tag] add tag to log messages
  -m print messages to stdout
  -q do not print messages to the syslog
  -v prints the software version and exits
  -h prints this message and exits
```

### 综合示例

#### 命令行参数

下面示范如何通过ptp4l和phc2sys，将master的网卡时间同步到slave的网卡时间和系统时间。用户需要保证master设备和slave设备之间网络联通。

Master端执行如下命令:

```
  ptp4l -i eth0 -f /usr/hobot/lib/pkgconfig/automotive-master.cfg -m -l 7
```

Slave端执行如下命令:

```
  ptp4l -i eth0 -f /usr/hobot/lib/pkgconfig/automotive-slave.cfg -m -l 7 > ptp4l.log &
  phc2sys -s eth0 -c CLOCK_REALTIME --transportSpecific=1 -m --step_threshold=1000 -w > phc2sys.log &
```

#### Log示例

Slave端log:

```
  ptp4l[8330.884]: PI servo: sync interval 1.000 kp 0.700 ki 0.300000
  ptp4l[8330.885]: master offset 21 s3 freq -391 path delay 690
  ptp4l[8330.998]: port 1: delay timeout
  ptp4l[8330.999]: delay filtered 689 raw 687
  ptp4l[8331.884]: master offset 35 s3 freq -371 path delay 689
  ptp4l[8332.885]: master offset 47 s3 freq -349 path delay 689
  ptp4l[8333.885]: master offset 50 s3 freq -332 path delay 689
  ptp4l[8334.885]: master offset 22 s3 freq -345 path delay 689
```

  Master端log:
```
  ptp4l[3469.136]: config item /var/run/ptp4l.inhibit_delay_req is 1
  ptp4l[3469.136]: config item (null).uds_address is '/var/run/ptp4l'
  ptp4l[3469.136]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
  ptp4l[3469.136]: config item (null).slaveOnly is 0
  ptp4l[3469.136]: port 1: received link status notification
  ptp4l[3469.136]: interface index 2 is up
  ptp4l[3469.261]: port 1: master sync timeout
  ptp4l[3469.386]: port 1: master sync timeout
  ptp4l[3469.511]: port 1: master sync timeout
  ptp4l[3469.636]: port 1: master sync timeout
  ptp4l[3469.761]: port 1: master sync timeout
  ptp4l[3469.886]: port 1: master sync timeout
```

## 全局时间源配置

由于当前系统中存在多个timeline，比如systime、phc、rtc等，而各个模块支持选择不同的timeline来打印各自的时间戳。为了统一各个模块的日志中时间戳的timeline，添加了一个hb\_systime的模块，来统一系统各个模块的timeline的选择。该模块实现的具体功能如下：

解析dts里默认的timeline的配置，如下：

```
  globaltime: globaltime {
    compatible = "hobot,globaltime";
    globaltime = <2>; /* 0-systime, 1-phc, 2-rtc */
    phc-index = <0>; /* phc index */
    status = "okay";
  };
```

其中dts中 globaltime 属性表示系统默认用的全局timeline，S100默认使用RTC时间。对应关系如下：

```
  0:systime
  1:phc
  2:rtc
```

如果timeline选中网卡phc时间， 则通过phc-index属性进一步判断当前使用哪个网卡phc的时间。对应关系如下：

```
  0:phc0
  1:phc1
```

向应用层提供/sys接口用来设置或者获取当前系统全局的timeline的选择，命令如下：

查看当前系统全局的timeline的配置：

```
  cat /sys/devices/platform/soc/soc:globaltime/globaltime
```

设置系统全局timeline的选项：

```
  echo 0 >/sys/devices/platform/soc/soc:globaltime/globaltime
  或
  echo 1 >/sys/devices/platform/soc/soc:globaltime/globaltime
  或
  echo 2 >/sys/devices/platform/soc/soc:globaltime/globaltime
```

向应用层提供/sys接口用来设置或者获取当前选中的phc编号

查看当前选中的phc编号：

```
  cat /sys/devices/platform/soc/soc:globaltime/phcindex
```

设置phc编号：

```
  echo 0 >/sys/devices/platform/soc/soc:globaltime/phcindex
  或
  echo 1 >/sys/devices/platform/soc/soc:globaltime/phcindex
```

<div style={{ borderLeft: '4px solid orange', background: '#fff7f0', padding: '10px', borderRadius: '8px' }}>
📍 <strong>注意：</strong> phcindex设置不要超过网卡实际数量。
</div>

向内核其他模块提供接口，查看当前系统用的哪个timeline，以及使用的是哪个phc，如下：

```
  int32_t hobot_get_global_time_type(uint32_t *global_time_type);
  int32_t hobot_get_phc_index(uint32_t *phc_index);
```

## MCU时间同步说明

### MCU支持的时间类型

PHC时间：这个是网卡内部的一个计时器；

RTC时间：这个是S100 MCU侧带的一个实时时钟，不支持通过纽扣电池供电；

### MCU支持的时间同步方式

MCU支持下面的时间同步方式:

基于PPS的Timesync：在秒脉冲上升沿捕捉到两个时间的snapshot，根据snapshot计算误差，根据误差进行时间同步 ；

### 基于PPS的Timesync

代码目录： mcu/Service/TimeSync/src/

#### 支持的时间

- RTC
- PHC

#### 配置方法

通过修改配置决定使用哪种时间同步方式。配置代码在./Service/TimeSync/src/Hobot\_TimeSync.c文件中。目前只是支持下面一种时间同步方式：

PHC同步RTC

```
  /*TimeSync Config Begin*/
  uint8 ConfigSource[] = {
    TIMEKEEPER_NONE, //TimeKeeperRTC Source
    TIMEKEEPER_NONE, //TimeKeeperGPS Source
    TIMEKEEPER_NONE, //TimeKeeperStbmPhc Source //Not Used in S100
    TIMEKEEPER_NONE, //TimeKeeperSpi Source
    TIMEKEEPER_NONE, //TimeKeeperIPC Source
    TIMEKEEPER_NONE, //TimeKeeperPHC Source
    TIMEKEEPER_NONE //TimeKeeperGpt Source //Not Used in S100
  };

  _Bool EnableTimeKeeper[] = {
    FALSE,//TimeKeeperRTC
    FALSE,//TimeKeeperGPS
    FALSE,//TimeKeeperStbmPhc //Not Used in S100
    FALSE,//TimeKeeperSpi
    FALSE,//TimeKeeperIPC
    FALSE,//TimeKeeperPHC
    FALSE//TimeKeeperGpt //Not Used in S100
  };

  _Bool Enable_TimeSync_RTC_Once= FALSE;

  _Bool Enable_HobotTimesync_Debug = FALSE;
  _Bool EthTsync_Master_Use_Phc = TRUE;
  uint8 TimeSync_PPS_Index = TIMESYNC_MCU_ETH0_PPS;

  /*TimeSync Config End*/
```

ConfigSource：配置每个时间的时间源，如果一个时间不需要和其他时间同步，时间源配置成TIMEKEEPER\_NONE。

可以配置的时间源为 ：

```
  typedef enum TimeKeeperIndexEnum {
    TIMEKEEPER_RTC_INDEX,
    TIMEKEEPER_GPS_INDEX,
    TIMEKEEPER_STBMPHC_INDEX, //Not Used in S100
    TIMEKEEPER_SPI_INDEX,
    TIMEKEEPER_IPC_INDEX,
    TIMEKEEPER_PHC_INDEX,
    TIMEKEEPER_STBMGPT_INDEX, //Not Used in S100
    TIMEKEEPER_NONE,
    TIMEKEEPER_NUM_MAX,
  } TimeKeeperIndex;
```

EnableTimeKeeper： 配置时间是否打开 ；

Enable\_HobotTimesync\_Debug： 配置是否开启打印 ；

TimeSync\_PPS\_Index： 配置使用哪个PPS进行时间同步，可以使用的PPS如下 ：

```
  #define TIMESYNC_AON_RTC_PPS 0
  #define TIMESYNC_PPS0 2
  #define TIMESYNC_PPS1 3
  #define TIMESYNC_PPS2 4
  #define TIMESYNC_PCIE0_PTM_PPS 6
  #define TIMESYNC_PCIE1_PTM_PPS 7
  #define TIMESYNC_ACORE_ETH0_PPS 8
  #define TIMESYNC_ACORE_ETH1_PPS 9
  #define TIMESYNC_MCU_ETH0_PPS 10
  #define TIMESYNC_PPS_DISABLE 11
```

#### 配置举例

如果想配置RTC和GPS进行时间同步， 配置如下:

```
  /*TimeSync Config Begin*/
  uint8 ConfigSource[] = {
    TIMEKEEPER_GPS_INDEX, //TimeKeeperRTC Source
    TIMEKEEPER_NONE, //TimeKeeperGPS Source
    TIMEKEEPER_NONE, //TimeKeeperStbmPhc Source //Not Used in S100
    TIMEKEEPER_NONE, //TimeKeeperSpi Source
    TIMEKEEPER_NONE, //TimeKeeperIPC Source
    TIMEKEEPER_NONE, //TimeKeeperPHC Source
    TIMEKEEPER_NONE //TimeKeeperGpt Source //Not Used in S100
  };

  _Bool EnableTimeKeeper[] = {
    TRUE,//TimeKeeperRTC
    TRUE,//TimeKeeperGPS
    FALSE,//TimeKeeperStbmPhc //Not Used in S100
    FALSE,//TimeKeeperSpi
    FALSE,//TimeKeeperIPC
    FALSE,//TimeKeeperPHC
    FALSE//TimeKeeperGpt //Not Used in S100
  };

  _Bool Enable_TimeSync_RTC_Once= FALSE;

  _Bool Enable_HobotTimesync_Debug = FALSE;
  /*TimeSync Config End*/
```

## PPS说明{#PPS}

### S100 PPS 介绍

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image43.png)

如上图所示，S100上的PPS可以分为PPS Source和PPS Target。PPS Source产生PPS，经过中间的Trigger Bus送到PPS Target，PPS Target利用送来的PPS产生snapshot或者LPWM波形。

### PPS Target

PPS Target表示被PPS触发的对象，用于产生snapshot或者LPWM波形，S100使用PPS触发的对象主要有以下几个：

PPS OUT：PPS2的管脚复用在当前软件版本上默认是设置为PPS OUT的，即可以通过这个pad脚将芯片内的PPS向外输出到片外；

GIC：当前软件版本默认将MCU RTC，PPS0，PPS1，PPS2和MCU ETH PPS的中断给到了GIC；

Acore ETH：Acore ETH可以接受PPS源，并通过PPS打自身硬件的snapshot时间；

LPWM：LPWM可以接受PPS源输入，并通过PPS产生对应的LPWM波形；

### PPS Source

S100的PPS source有多个源可选，其中常用的源有以下几个，其它是reserve的：

编号0：MCU RTC PPS，是MCU侧RTC输出的PPS，当前软件版本默认已有输出，周期1秒1次；

编号2：PPS0，PPS的pad，可用于GPS PPS等外部PPS源输入到芯片内部；

编号3：PPS1，PPS的pad，可用于GPS PPS等外部PPS源输入到芯片内部；

编号4：PPS2，PPS的pad，可用于GPS PPS等外部PPS源输入到芯片内部，也可以用于将内部的PPS输出到外部；

编号8：Acore Eth0 PPS， 是Acore Eth0产生的PPS，可以设置灵活的时间周期，比如1秒1次，400毫秒1次等；

编号9：Acore Eth1 PPS， 是Acore Eth1产生的PPS；可以设置灵活的时间周期，比如1秒1次，400毫秒1次等；

编号10：MCU Eth PPS，是MCU侧Eth输出的PPS，当前软件版本默认已有输出，周期1秒1次;

### PPS的PAD配置

当前软件版本默认将PPS0和PPS1设置为了PPS的IN，即用于接收GPS PPS等外部PPS的输入；把PPS2设置为了PPS的OUT，即将S100芯片的PPS输出到外部；
它们的配置均由MCU侧的Port模块进行了配置，如需要修改，则在Port模块中进行修改。

### Acore Eth PPS介绍{#Acore\_Eth\_PPS}

Acore Eth PPS有两种输出方法，flex mode和fix mode两种。上电默认是fix mode。可以通过下文中ethtool命令设置为flex mode。若要重置为fix mode，可以通过重启网卡或者重启系统完成恢复。

flex mode: 灵活的pps模式，它的开始/结束时间、周期、占空比均可以灵活配置，其上升沿是整秒时刻，PPS输出不会随着PHC时间的变化而变化；当前软件版本下，其占空比是百分之一，周期可以根据下方配置方法灵活配置，常用的有1s，400ms。

fixed mode: 固定的pps模式，它的周期固定，且占空比也固定为46.3129%。PPS输出会随着PHC时间的变化而变化。当前软件版本下，其支持1s周期，波形的整秒时刻是下降沿，经过536.871ms的低电平后，再输出463.129ms的高电平。

整秒输出需求配置方法：

若在flex mode下有整秒时刻输出PPS的需求，需要在gptp时间同步完成后，再配置PPS输出；

若在fixed mode下有整秒时刻输出PPS的需求，需要注意ETH的整秒时刻在下降沿出现，而LPWM被ETH上升沿同步。因此需要参考下图，调整LPWM的offset。以camera一秒30帧举例，PPS上升沿在536.871ms，下降沿在1s整秒处，要求在整秒位置出图的话，offset=463.129对33.333取余数=29.8ms。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image44.png)

### Acore Eth PPS的输出配置方法

Acore Eth PPS的输出，可以借助ethtool工具，命令格式如下 :

配置Eth0的PPS输出1秒的周期：ethtool hobot\_gmac \--set-flex-pps eth0 index 0 fpps on interval 1000000000;

配置Eth1的PPS输出1秒的周期：ethtool hobot\_gmac \--set-flex-pps eth1 index 0 fpps on interval 1000000000;

如果需要修改输出周期的话，修改最后一个参数\<1000000000\>，该参数的单位是ns，如果要修改为400ms的话，则改为\<400000000\>。

:::tip
配置eth pps输出需要注意的两点，下面以修改eth pps0并设置PPS_INOUT脚为PPS_OUT功能为例进行说明。如果有其他需求，用户请根据实际使用调整配置项。
1. 修改Port配置的默认pps_source
```
 static const Port_Lld_PpsConfigType PpsConfig =
 {
     (boolean)TRUE,
-    PPS_SOURCE_AON_RTC,
+    PPS_SOURCE_ETH0_PTP,
 };
```
2. 将pin脚配置成PPS_OUT func
```
-    {(uint8)2, "PPS_OUT", (boolean)TRUE, {(boolean)TRUE, (boolean)FALSE, (boolean)TRUE, (boolean)TRUE, (boolean)TRUE, GPIO, PORT_PIN_CONFIG_TYPE0, PORT_PULL_NONE, PORT_DRIVE_DEFAULT, PORT_PIN_DIR_OUT, PORT_PIN_LEVEL_HIGH}},
+    {(uint8)2, "PPS_OUT", (boolean)TRUE, {(boolean)TRUE, (boolean)FALSE, (boolean)TRUE, (boolean)TRUE, (boolean)TRUE, PPS_OUT, PORT_PIN_CONFIG_TYPE0, PORT_PULL_NONE, PORT_DRIVE_DEFAULT, PORT_PIN_DIR_OUT, PORT_PIN_LEVEL_HIGH}},
```
以上通过修改`McalCdd/gen_s100_sip_B/Port/src/Port_PBcfg.c`实现，修改完成后更新MCU固件即可。
:::

### MCU Eth PPS的配置方法

MCU Eth PPS的信号周期和脉冲宽度在`Config/McalCdd/gen_s100_sip_B_mcu1/Ethernet/src/Mac_Ip_PBcfg.c`配置文件中设置默认值，其中PpsInterval的默认值为1000(ms)，参数PpsWidth的默认值为10(ms)。

限制条件：

- 配置项EthGlobalTimeSupport需要被使能，默认使能；
- 参数PpsWidth的值必须小于PpsInterval；

配置使能后通过查看sys节点assert变化确认PPS配置是否生效。

```
cat /sys/class/pps/pps[4]/assert
```

/sys/class/pps下包含多个pps，通过name确认哪个PPS对应MCU PPS。

```
cat /sys/class/pps/pps[*]/name
```

## 时间同步整体方案

### 功能概述

本文主要以S100为例，介绍时间同步方案。目前MCU主线默认支持的是时间源接入S100
Acore的单时间域方案。

#### 软件架构说明

##### 时间源接入Acore的单时间域方案

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image45.png)

上图时间同步流程总结如下：

- 外部gptp master设备作为整个S100的时间源；

- 首先通过ptp4l软件将gptp master的时间同步给Acore侧的网卡；然后通过phc2sys将网卡时间同步给Linux系统时间，该过程持续进行；

- 接下来通过Acore timesync\_sample软件将网卡时间同步给MCU侧Rtc；

特性说明：

- Rtc支持给图中的CIM和CAN打硬件时间戳；

- Acore 网卡可以发出pps，触发lpwm进行曝光同步；

#### 代码位置及目录结构

Acore侧timesync sample代码位于工程目录：``{sdk_dir}/source/hobot-sp-samples/debian/app/timesync_demo``

MCU侧timesync sample代码位于工程目录：``{mcu_dir}/mcu/Service/TimeSync``

### Acore编译

#### 编译环境

板端在安装hobot-sp-samples\_\*.deb包后，会包含codec\_demo源码内容。

#### 编译说明

本sample主要依赖ipcfhal和timesync提供的API头文件

```
#include "hb_ipcf_hal.h"
#include "hobot_clock_hal.h"
```

编译依赖的库有如下：

```
  LIBS := -lhbipcfhal -lhbtimesynchal -lpthread -lalog
```

编译命令：

板端进入/app/timesync\_demo/sample\_timesync目录，执行

```
  make
```

### MCU编译

#### 编译环境

MCU侧本sample的编译环境使用MCU代码中的build工具，请参考：[MCU编译](../../05_mcu_development/01_S100/01_basic_information.md#开发环境)。

编译FreeRtos镜像版本。注意

```
  # 进入Build/FreeRtos目录
  python build_freertos.py lite matrix B s100 gcc debug # 硬件板或者项目名
```

#### 运行

##### 支持平台

S100

##### 板端部署及配置

需要做好如下准备工作：

烧写好支持MCU侧时间同步服务的MCU固件。

若需要运行gptp时间同步，需要保证网络连通。

为了防止Linux操作系统自带时间同步服务产生干扰，需要执行如下命令关闭Linux自带时间同步服务。

```
  systemctl stop systemd-timesyncd
```

##### 运行指南

###### 时间源接入Acore的单时间域方案

MCU侧在./Service/TimeSync/src/Hobot\_TimeSync.c代码中进行如下配置修改：

```
/* Timesync Config Begin */
uint8_t ConfigSource[] = {
    TIMEKEEPER_IPC_INDEX,   // TimeKeeperRTC Source
    TIMEKEEPER_NONE,        // TimeKeeperGPS Source
    TIMEKEEPER_IPC_INDEX,   // TimeKeeperStbmPhc Source
    TIMEKEEPER_NONE,        // TimeKeeperSpi Source
    TIMEKEEPER_NONE,        // TimeKeeperIPC Source
    TIMEKEEPER_NONE,        // TimeKeeperPHC Source
    TIMEKEEPER_NONE         // TimeKeeperGpt Source
};

_Bool EnableTimeKeeper[] = {
    TRUE,
    FALSE,
    FALSE,
    FALSE,
    TRUE,
    FALSE,
    FALSE
};

_Bool Enable_TimeSync_RTC_Once = FALSE;

_Bool Enable_HobotTimesync_Debug = TRUE;
```

MCU侧的修改需要注意 PRODUCT\_IMAGE宏的影响，详细内容请参考上述注意事项章节。

刷写启动后，默认MCU执行如下命令，第一条命令指定PPS触发源为mcu eth；第二条命令启动时间同步服务

```
  TimeSyncCtrl 4 10
  TimeSyncCtrl 6
```

其中，打开MCU侧日志打印命令如下：

```
TimeSyncCtrl 1
```

:::tip
MCU默认不启动时间同步服务。如果配置默认启动，除了修改以上描述的配置，另外需要增加初始化的动作，参考如下：

```
TASK(OsTask_SysCore_Startup)
{
  ......
  Timesync_Init();
  ......
}
```

完成以上配置，在加载MCU1固件后，时间同步服务默认启动，不再需要执行``TimeSyncCtrl``的命令。
:::

Acore执行如下命令，第一条命令设置log等级，允许输出打印到控制台；第二条命令启动ptp时间同步，将外部ptp master时间同步到Acore网卡；
第三条命令将网卡时间同步到Linux系统时间；第五条命令启动时间同步程序，将Acore网卡时间同步给MCU侧Rtc。

```
export LOGLEVEL=15
ptp4l -i eth0 -f /usr/hobot/lib/pkgconfig/automotive-slave.cfg -m -l 7 > ptp4l.log &
phc2sys -s eth0 -c CLOCK_REALTIME --transportSpecific=1 -m --step_threshold=1000 -w > phc2sys.log &
cd /app/timesync_demo/sample_timesync
./timesync_sample -p 0 -P 4 -r
```

关键参数说明：

-p 1: 同步网卡1的时间； -p 0: 同步网卡0的时间(默认)

-r: 将Acore网卡时间同步给MCU Rtc时间

Acore时间同步log如下：

```
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:510] ************************************************
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:517] Monotonic_raw time: second = 222, nanosecond = 557846725
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:520] PPS fetch infobuf time: second = 1745925136, nanosecond = 454616475
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:531] rtc snapshot: sec = 1745925136, nsec = 455389596
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:538] ptp0 snapshot: sec = 1745925136, nsec = 455462040
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:582] Timesync Info: timesync ipc send data succeed
[INFO][][/app/timesync_demo/sample_timesync/timesync_sample.c:510] ************************************************
```

Acore log说明：

"snapshot"字段为MCU网卡发出pps秒脉冲时各个clock对应的硬件时间戳；

"PPS fetch infobuf time"字段为Acore pps驱动捕获到的系统时间；

"timesync ipc send data succeed"提示说明Acore网卡时间戳成功发送给MCU，由MCU侧进行时间同步，时间同步质量在MCU侧查看。

MCU时间同步log如下：

```
[0162.799758 0]*
[0162.802970 0]RTC snapshot time:1745925071.455114934
[0163.000408 0]Timesync_RecvCallback.
[0163.000661 0]Get Time From Acore success 1745925071 455188935
[0163.001958 0]TimeKeeperRTC and TimeKeeperIPC Offset : - 0s.74001ns
[0163.789774 0]*
```

MCU log说明：

"Get Time From Acore success" 表明MCU从Acore拿时间成功；

"TimeKeeperRTC and TimeKeeperIPC Offset" ，表明在同步时，Rtc使用IPC时间同步时的offset。
