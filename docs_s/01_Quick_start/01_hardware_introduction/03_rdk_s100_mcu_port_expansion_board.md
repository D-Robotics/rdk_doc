---
sidebar_position: 4
---

# 1.1.3 MCU 接口扩展板

![image-rdk_100_mcu_port_expansion_board](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_mcu_port_expansion_board.png)

RDK S100 MCU Port Expansion Board（含配套 FPC）是地瓜机器人 RDK S100 系列开发者套件的核心扩展模块，主要用于扩展 MCU 接口功能，支持以太网、CAN_FD、ADC 等。

:::warning

1. 本产品仅适配 RDK S100 系列主板使用，禁止与其他型号设备兼容。
2. 使用时需将本扩展板放置于稳固、平坦且不导电的表面，避免因支撑不稳导致设备跌落或短路。
3. 若将非兼容设备与本 MCU 扩展板连接，由此造成的设备损坏，本产品不提供维修服务。
4. 所有配套使用的外围设备（包括但不限于网络设备、CAN 设备）须符合使用国家/地区的安全与性能标准，并标注合规认证信息。
5. 与本扩展板连接的所有外围设备电缆及连接器需具备充足绝缘性能，确保满足电气安全要求。

:::

:::warning 安全使用

为避免本扩展板故障或损坏，请严格遵守以下事项：

1. 环境要求 ​：运行时请勿接触水、湿气或导电物体表面，远离热源（如暖气、阳光直射），确保工作环境温度符合产品规格书要求。
2. 装配操作 ​：装配过程中需轻拿轻放，避免对印刷电路板（PCB）及连接器施加机械压力或电气干扰（如静电触碰）。
3. 通电操作 ​：通电状态下禁止直接触摸 PCB 板面或设备边缘金属接口，降低静电放电（ESD）损坏风险。

:::

## 产品规格

| **名称** | **参数**                                                                                 |
| -------- | ---------------------------------------------------------------------------------------- |
| 接口     | 5 x CAN FD（最高 8Mbps） <br />1 x 30-pin，具有最多 7x ADC, 2x IIC, 2x SPI<br />1 x RJ45 |
| 板载模组 | IMU：BMI088                                                                              |
| 工作温度 | 0℃~45℃                                                                                   |

### 拓扑图

![image-rdk_s100_mcu_port_expansion_board_architecture_diagram.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_s100_mcu_port_expansion_board_architecture_diagram.png)

### 接口描述

![image-rdk_100_cam_expansionboard](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_mcu_port_expansion_board_interface.png)

| **序号** | **功能**                  | 序号 | 功能                   |
| -------- | ------------------------- | ---- | ---------------------- |
| J1       | MCU 扩展板 100-pin 连接器 | J6   | CAN7                   |
| J12      | 30-pin                    | J7   | CAN7 的 120 欧电阻跳线 |
| U4       | MCU 域的 RJ45 千兆网口    | J8   | CAN8                   |
| J2       | CAN5                      | J9   | CAN8 的 120 欧电阻跳线 |
| J3       | CAN5 的 120 欧电阻跳线    | J10  | CAN9                   |
| J4       | CAN6                      | J11  | CAN9 的 120 欧电阻跳线 |
| J5       | CAN6 的 120 欧电阻跳线    |      |                        |

### 组装说明

:::danger

1. 请在开发板电源关闭，且 DC 插头断开的情景下进行安装。
2. 安装时请确保**连接器保持平行**，**接口均匀受力完成扣合**，且连接紧密，以免损坏连接器。

:::

:::info 提示

FPC 正面丝印"MAIN"标识侧对应 RDK S100 主板的 J23 接口，"SUB"标识侧对应本扩展板的 J1 接口。

:::

<video controls width="90%" preload="metadata">
  <source src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/video/mcu_port_expansion_board_assembly_guide.mp4" type="video/mp4" />
  您的浏览器不支持 video 标签。
</video>

## 接口说明

### CAN FD 连接器（J2/J4/J6/J8/J10）

:::info 信息

扩展板背面标注了每个接口的`CAN_H`、`CAN_L`和`GND`

:::

扩展板提供 5 路 CAN FD 接口（J2/J4/J6/J8/J10），每路接口配备 120Ω 终端电阻，可通过跳帽连接对应插针（J3/J5/J7/J9/J11）实现选通。具体对应关系如下：

| CAN FD 通道 | 连接器位号 | 120 欧电阻跳线位号 |
| ----------- | ---------- | ------------------ |
| CAN5        | J2         | J3                 |
| CAN6        | J4         | J5                 |
| CAN7        | J6         | J7                 |
| CAN8        | J8         | J9                 |
| CAN9        | J10        | J11                |

### 以太网连接器（U4）

MCU 扩展板提供了一个千兆以太网接口。

### 30-Pin（J12）

接口定义: [drobotics_rdk_s100_mcu_port_expansion_board_pinlist_v1p0.xlsx](../../../static/asset/rdk_s100/drobotics_rdk_s100_camera_expansion_board_pinlist_v1p0_0924.xlsx)

:::warning 注意

1. 当系统处于 light sleep 和 deep sleep 模式时，VDD_5V，VDD_3V3，VDD_1V8 电源保持供电，最大输出电流分别为 300mA，600mA，300mA。
2. I2C9_SDA_3V3，I2C9_SCL_3V3 信号作为 GPIO 使用时不允许接外部下拉电阻。

:::

### IMU（U8）

:::warning 注意

RDKS100_LNX_SDK_V4.0.2 暂未实现对应功能

:::

集成惯性测量单元（IMU，型号 Bosch Sensortec BMI088），支持通过 SPI-5 串行总线实现通信控制。

## 指示灯

扩展板 100PIN 连接器（J1）下方设有 1 颗绿色 LED 指示灯（标记为"CONNECT"），用于指示电源状态和 MCU 扩展板与 RDK S100 的连接状态：

- 绿灯常亮：RDK S100 和 MCU 扩展板连接正常，5V 电源已正常供电；
- 绿灯熄灭：RDK S100 和 MCU 扩展板连接异常，无 5V 供电。

## 尺寸规格

板卡尺寸：70x70x17mm
