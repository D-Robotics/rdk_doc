---
sidebar_position: 2
---

# 1.1.2 RDK X5

```mdx-code-block
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
```

## 接口总览

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

RDK X5提供了网口、USB、摄像头、LCD、HDMI、CANFD、40PIN等功能接口，方便用户进行图像多媒体、深度学习算法等应用的开发和测试。开发板接口布局如下：


![RDK_X5_interface](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/RDK_X5_interface.jpg)


| 序号 | 功能 | 序号 | 功能 | 序号 | 功能 |
| -------- | ---------------------------- | -------- | ----------------------- | ----------------------- | ----------------------- |
| 1 | 供电接口 （USB Type C） | 2 | RTC 电池接口 | 3 | 闪连接口 （USB Type C） |
| 4 | 调试串口（Micro USB） | 5 | 2 路 MIPI Camera 接口 | 6 | 千兆以太网口，支持 POE |
| 7 | 4 路 USB 3.0 Type A 接口 | 8 | CAN FD 高速接口 | 9 | 40PIN 接口 |
| 10 | HDMI 显示接口 | 11 | 多标准兼容耳机接口 | 12 | 板载 Wi-Fi 天线 |
| 13 | TF卡接口（底面） | 14 | LCD 显示接口（MIPI DSI） |  |  |

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module Carrier Board，作为 X5 模组的配套底板，提供了丰富的配置和接口，包括 USB3.0、以太网、HDMI、MIPI CSI、MIPI DSI、40pin header 等，方便用户对模组的功能验证和开发。 

![img-20250416-161040](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/img-20250416-161040.png)

| 序号 | 功能 | 序号 | 功能 | 序号 | 功能 |
| -------- | ---------------------------- | -------- | ----------------------- | ----------------------- | ----------------------- |
| 1	| 电源接口 | 9	| CAM2接口，4lane | 17 | Audio接口 |
| 2	| USB2.0配置header | 10	| CAM1接口，4lane | 18 | IO电平选择header |
| 3	| USB2.0 Device接口 | 11	| 40pin header | 19 | MIPI DSI接口 |
| 4	| USB3.0 HOST接口 | 12	| 核心模组接口 | 20 | Micro SD卡接口（背面） |
| 5	| USB3.0 HOST接口 | 13	| RTC电池接口 | 21 | HDMI接口 |
| 6	| 千兆以太网口 | 14	| CAN终端电阻接入开关 | 22 | Debug口，USB转串口（背面） |
| 7	| 风扇接口 | 15	| CAN总线接口 | 23 | Sleep按键 |
| 8	| POE 接口 | 16	| 功能控制IO header | 24 | 电源开关 |

</TabItem>
</Tabs>

:::caution
RTC在给电池供电的时候，对电池的电压和放电电流要求为：2~3.3V ，＞2.5uA。
开机后当 pmic 检测到 rtc 电压低到充电电压时，会自动给 rtc 充电，电池要求为：可承受的最大充电电压≥3.3V，最大可允许充入电流＞1mA。  
此外，非充电的RTC不可以使用进行供电
:::

## 核心模组接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

全板载设计， 无核心模组。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module载板提供一组300pin板板连接器，用于核心模组的安装。安装时需要首先确认正确的方向和定位，避免对核心模组、载板的连接器造成损伤。

![img-20250418-111059](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/img-20250418-111059.png)

模组安装方法如下：

1. 对照核心模组pin脚，确认安装方向正确。
2. 将核心模组放于载板正上方，并确认周围四个定位孔位置对齐。
3. 从核心模组中心向下按压，当模组发出咔哒的声响后，表示安装到位。

</TabItem>
</Tabs>

## 电源接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板提供一路 USB Type C 接口(接口1)，作为供电接口，需要使用支持**5V/5A**的电源适配器为开发板供电。将电源适配器接入开发板后，**<font color='Green'>绿色</font> 电源指示灯 亮起**，说明开发板供电正常，3.1.0版本后，**<font color='Orange'>橙色</font> 状态指示灯 闪烁**，说明系统运行正常。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板提供一路 USB Type C 接口(接口1)，作为供电接口，需要使用支持**5V/5A**的电源适配器为开发板供电。将电源适配器接入开发板后，**<font color='Green'>绿色</font> 5V指示灯 PWR指示灯 亮起**，说明开发板供电正常，**<font color='Green'>绿色</font> ACT指示灯 闪烁**，说明系统运行正常。

</TabItem>
</Tabs>

:::caution

请不要使用电脑USB接口为开发板供电，否则会因供电不足造成开发板**异常断电、反复重启**等情况。

:::


## 调试串口{#debug_uart}

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板提供一路调试串口(接口4)，以实现串口登录、调试功能。电脑串口工具的参数配置如下：

- 波特率（Baud rate）：115200
- 数据位（Data bits）：8
- 奇偶校验（Parity）：None
- 停止位（Stop bits）：1
- 流控（Flow Control）：无

串口连接时，需要将通过 Micro-USB 线，连接开发板接口 4 与 PC。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板提供一路调试串口(<font color='Red'>背面</font>接口22)，以实现串口登录、调试功能。电脑串口工具的参数配置如下：

- 波特率（Baud rate）：921600
- 数据位（Data bits）：8
- 奇偶校验（Parity）：None
- 停止位（Stop bits）：1
- 流控（Flow Control）：无

串口连接时，需要将通过 Micro-USB 线，连接开发板接口22 与 PC。

</TabItem>
</Tabs>

通常情况下，用户第一次使用该接口时需要在电脑上安装 CH340 驱动，用户可搜索`CH340串口驱动`关键字进行下载、安装。


## 有线网口

开发板提供一路千兆以太网接口(接口6)，支持1000BASE-T、100BASE-T标准，默认采用静态IP模式，IP地址为`192.168.127.10` 。如需确认开发板IP地址，可通过串口登录设备，并用`ifconfig`命令进行查看 `eth0`网口的配置.

此外，该接口支持 PoE（Power over Ethernet，以太网供电）功能，无需额外的电源线即可通过网线同时传输数据和电力，使设备的安装更加简便灵活。


## HDMI 显示接口{#hdmi_interface}

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板提供一路HDMI(接口10)显示接口，最高支持 1080P 分辨率。开发板通过HDMI接口在显示器输出Ubuntu系统桌面(Ubuntu Server版本显示logo图标)。此外，HDMI接口还支持实时显示摄像头、网络流画面功能。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板提供一路HDMI(接口21)显示接口，最高支持 1080P 分辨率。开发板通过HDMI接口在显示器输出Ubuntu系统桌面(Ubuntu Server版本显示logo图标)。此外，HDMI接口还支持实时显示摄像头、网络流画面功能。

</TabItem>
</Tabs>

## USB 显示接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板通过硬件电路实现了多路USB接口扩展，满足用户对多路USB设备接入的需求，接口描述如下：

| 接口类型       | 接口序号 | 接口数量 | 接口描述                                                 |
| -------------- | -------- | -------- | -------------------------------------------------------- |
| USB 2.0 Type C | 接口3    | 1路      | USB Device模式，用于连接主机实现ADB、Fastboot、系统烧录等功能 |
| USB 3.0 Type A | 接口7    | 4路      | USB Host模式，通过 HUB 扩展出 4 个 USB 端口，用于接入USB 3.0外设 |

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板通过硬件电路实现了多路USB接口扩展，满足用户对多路USB设备接入的需求，接口描述如下：

| 接口类型       | 接口序号 | 接口数量 | 接口描述                                                 |
| -------------- | -------- | -------- | -------------------------------------------------------- |
| USB 2.0 Type C | 接口3    | 1路      | USB Device模式，用于连接主机实现ADB、Fastboot、系统烧录等功能 |
| USB 3.0 Type A | 接口4 & 接口5 | 4路      | USB Host模式，通过 HUB 扩展出 4 个 USB 端口，用于接入USB 3.0外设 |

### USB 2.0 切换HOST

开发板可以短接接口2，将USB 2.0切换到HOST模式。

</TabItem>
</Tabs>

### 接入U盘

开发板 USB Type A 接口，支持U盘功能，可自动检测U盘接入并挂载，默认挂载目录为`/media/sda1`。

### 接入 USB 串口转接板

开发板USB Type A接口，支持USB串口转接板功能，可自动检测USB串口转接板接入并创建设备节点`/dev/ttyUSB*` 或者 `/dev/ttyACM*`（星号代表0开始的数字）。用户可参考 [使用串口](../../03_Basic_Application/03_40pin_user_guide/uart.md#40pin_uart_usage) 章节对串口进行使用。

### USB 摄像头

开发板 USB Type A 接口，支持 USB 摄像头功能，可自动检测USB摄像头接入并创建设备节点`/dev/video0`。

## MIPI Camera 接口{#mipi_port}

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板提供2路 22pin MIPI CSI接口(接口5)，可实现2路MIPI摄像头的接入,支持双目相机的接入。目前开发板适配了多种规格的摄像头模组，模组型号、规格如下：

| 序号 | Sensor |   分辨率  |  FOV  | I2C 设备地址 |
| --- | ------ | ------- | ------- | ------- |
|  1  | IMX219  | 800W |    |  |
|  2  | OV5647  | 500W |    |  |
| 3   | IMX477  | 1230W |   |  |

摄像头模组通过22pin 同向排线跟开发板连接，排线金属面背对黑色卡扣插入连接器。


安装完成后，用户可以通过i2cdetect命令确认模组I2C地址能否正常检测到。

查询靠近网口的mipi_host0 接口 上 Camera Sensor 的 I2C 设备地址：
```shell
echo 353 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio353/direction
echo 0 > /sys/class/gpio/gpio353/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio353/value

i2cdetect -y -r 6
```

查询远离网口的mipi_host2 接口 上 Camera Sensor 的 I2C 设备地址：
```shell
echo 351 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio351/direction
echo 0 > /sys/class/gpio/gpio351/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio351/value

i2cdetect -y -r 4
```

成功探测到Camera Sensor 的 I2C 设别地址时，可以看到如下所示的打印（以在接口 mipi_host2 上探测 IMX219 为例，可以发现 10 地址被打印出来了）：
```shell
root@ubuntu:~# i2cdetect -y -r 4
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --    
```

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板提供2路 22pin MIPI CSI接口 CAM1(接口10) CAM2(接口9)，可实现2路MIPI摄像头的接入,支持双目相机的接入。目前开发板适配了多种规格的摄像头模组，模组型号、规格如下：

| 序号 | Sensor |   分辨率  |  FOV  | I2C 设备地址 |
| --- | ------ | ------- | ------- | ------- |
|  1  | IMX219  | 800W |    |  |
|  2  | OV5647  | 500W |    |  |
| 3   | IMX477  | 1230W |   |  |

摄像头模组通过22pin 同向排线跟开发板连接，排线金属面背对黑色卡扣插入连接器。

安装完成后，用户可以通过i2cdetect命令确认模组I2C地址能否正常检测到。

查询靠CAM1(接口10) 接口 上 Camera Sensor 的 I2C 设备地址：
```shell
echo 353 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio353/direction
echo 0 > /sys/class/gpio/gpio353/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio353/value

i2cdetect -y -r 6
```

查询CAM2(接口9) 接口 上 Camera Sensor 的 I2C 设备地址：
```shell
echo 351 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio351/direction
echo 0 > /sys/class/gpio/gpio351/value
sleep 0.1
echo 1 > /sys/class/gpio/gpio351/value

i2cdetect -y -r 4
```

成功探测到Camera Sensor 的 I2C 设别地址时，可以看到如下所示的打印（以在CAM2(接口9) 上探测 IMX219 为例，可以发现 10 地址被打印出来了）：
```shell
root@ubuntu:~# i2cdetect -y -r 4
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --    
```

</TabItem>
</Tabs>

:::caution
重要提示：严禁在开发板未断电的情况下插拔摄像头，否则非常容易烧坏摄像头模组。
:::

## LCD 显示接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

RDK X5 提供一路 MIPI DSI 的 LCD 显示接口（接口14），可以用于 LCD 显示屏等接入。接口为22pin，可使用DSI-Cable-12cm线材直接接入树莓派多款 LCD 显示屏。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

RDK X5 Module 提供一路 MIPI DSI 的 LCD 显示接口（接口19），可以用于 LCD 显示屏等接入。接口为22pin，可使用DSI-Cable-12cm线材直接接入树莓派多款 LCD 显示屏。

</TabItem>
</Tabs>

## Micro SD 接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板提供1路Micro SD存储卡接口(接口13)。推荐使用至少16GB容量的存储卡，以便满足Ubuntu操作系统及相关功能包的安装要求。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板提供1路Micro SD存储卡接口(<font color='Red'>背面</font>接口22)。推荐使用至少16GB容量的存储卡，以便满足Ubuntu操作系统及相关功能包的安装要求。

</TabItem>
</Tabs>

:::caution

开发板使用中禁止热插拔TF存储卡，否则会造成系统运行异常，甚至造成存储卡文件系统损坏。

:::

## Wi-Fi 天线接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

开发板的无线网络支持板载和外置天线两种配置，通常情况下板载天线可以满足使用需求。当开发板安装金属材质外壳后，需要连接外置天线到（接口12旁的天线接口），以增强信号强度。

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">

开发板的无线网络支持板载和外置天线两种配置，通常情况下板载天线可以满足使用需求。当开发板安装金属材质外壳后，需要连接外置天线到核心板上，以增强信号强度。

</TabItem>
</Tabs>


## CANFD 接口

<Tabs groupId="rdk-type">
<TabItem value="x5" label="RDK X5">

RDK X5开发板提供 CANFD 接口（接口8）和 CAN终端电阻接入开关（接口8后的2pin座子，高速通信需要两段使能终端电阻，防止信号反射，提升抗干扰能力），可用于CAN及CAN FD通信，具体信息请参考 [CAN使用](../../07_Advanced_development/01_hardware_development/rdk_x5/can.md) 章节

</TabItem>
<TabItem value="x5md" label="RDK X5 Module">


RDK X5 Module 开发板提供 CANFD 接口（接口15）和 CAN终端电阻接入开关（接口14，高速通信需要两段使能终端电阻，防止信号反射，提升抗干扰能力），可用于CAN及CAN FD通信，具体信息请参考 [CAN使用](../../07_Advanced_development/01_hardware_development/rdk_x5/can.md) 章节

</TabItem>
</Tabs>

## 40PIN 接口

RDK X5开发板提供 40PIN 接口，IO 信号采用 3.3 V电平设计。管脚定义兼容树莓派等产品，详细管脚定义、复用关系参考硬件开发章节。
