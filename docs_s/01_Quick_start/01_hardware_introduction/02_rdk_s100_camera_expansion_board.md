---
sidebar_position: 4
---

# 1.1.2 相机扩展板

![image-rdk_100_camera_expansion_board](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_camera_expansion_board.png)

RDK S100 Camera Expansion Board（以下简称“Camera 扩展板”）是地瓜机器人 RDK S100 系列开发者套件的核心扩展模块。Camera 扩展板基于 RDK S100 Camera Expansion Connector 进行二次开发，提供了 2 个 MIPI 相机接口和 4 个 GMSL 相机接口。

:::warning

1. 本产品仅适配 RDK S100 系列主板使用，禁止与其他型号设备兼容。
2. 使用时需将本扩展板放置于稳固、平坦且不导电的表面，避免因支撑不稳导致设备跌落或短路。
3. 若将非兼容设备与 RDK S100 CAMERA EXPANSION BOARD 连接，由此造成的设备损坏，本产品不提供维修服务。
4. 所有配套使用的外围设备（包括但不限于摄像头模组、电源适配器）须符合使用国家/地区的安全与性能标准，并标注合规认证信息。
5. 与本扩展板连接的所有外围设备电缆及连接器需具备充足绝缘性能，确保满足电气安全要求。

:::

:::warning 安全守则

为避免本扩展板故障或损坏，请严格遵守以下事项：

1. 环境要求 ​：运行时请勿接触水、湿气或导电物体表面，远离热源（如暖气、阳光直射），确保工作环境温度符合产品规格书要求。
2. 装配操作 ​：装配过程中需轻拿轻放，避免对印刷电路板（PCB）及连接器施加机械压力或电气干扰（如静电触碰）。
3. 通电操作 ​：通电状态下禁止直接触摸 PCB 板面或设备边缘金属接口，降低静电放电（ESD）损坏风险。

:::

:::warning 注意

1. 接入 MIPI 相机前，需确保 MIPI 相机的逻辑电平需求与拨码开关位置相匹配，防止出现通讯异常或设备损坏等问题。
2. 如需添加电平转换芯片，应关注该芯片支持的速率范围以及其对外部上下拉电阻的要求。
3. 注意 Camera 扩展板与相机上连接器 1 脚的位置及定义，根据产品结构形态采购或定制适配排线。
4. 制作 FPC 排线时，需保证 MIPI 信号参考平面的完整性并进行阻抗控制。

:::

## 产品规格

| **名称**    | **参数**                                         |
| ----------- | ------------------------------------------------ |
| 解串器      | Maxim MAX96712                                   |
| MIPI 连接器 | 2x 22-Pin MIPI CSI-2                             |
| GMSL 连接器 | Fakra-Mini 4in1                                  |
| 外部供电    | 12V DC，仅用于电流需求大于 700mA 时，最大 2.4A。 |
| 工作温度    | 0℃~45℃                                           |

### 拓扑图

![image-rdk_s100_camera_expansion_board_architecture_diagram.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_s100_camera_expansion_board_architecture_diagram.png)

### 接口说明

![camera_expansion_board_interface](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_camera_expansion_board_interface.png)

| 接口  | 功能          | 接口   | g 功能                    |
| :---- | :------------ | ------ | ------------------------- |
| J2000 | 100-Pin 接口  | J2200  | MIPI 相机接口 1           |
| D2000 | 电源指示灯    | J2201  | MIPI 相机接口 2           |
| J2001 | DC 电源输入   | SW2200 | MIPI 相机接口功能切换开关 |
| J2100 | GMSL 相机接口 | SW2201 | MIPI 相机接口电平切换开关 |

### 组装说明

| 型号                        | 硬件接口        | 功能切换 SW2200 | 电平切换 SW2201       |
|-----------------------------|------------------|------------------|------------------------|
| IMX219 摄像头（兼容树莓派 5） | J2200 / J2201     | lpwm             | 亚博 1.8V / 微雪 3.3V  |
| SC230AI 双目摄像头（V3 版）   | J2200 & J2201     | lpwm             | 3.3V                   |
| SC132GS 双目摄像头           | J2200 & J2201     | lpwm             | 3.3V                   |
| SG8S-AR0820C-5300-G2A       | J2100             | -                | -                      |
| LEC28736A11（X3C 模组）      | J2100             | -                | -                      |
| Intel RealSense D457        | J2100             | -                | -                      |
| Intel RealSense D435i       | USB               | -                | -                      |


:::danger

1. 请在开发板电源关闭，且 DC 插头断开的情景下进行安装。
2. 安装时请确保**开发板与子板保持平行**，**接口均匀受力完成扣合**，且连接紧密，以免损坏连接器。

:::

<video controls width="90%" preload="metadata">
  <source src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/video/camera_expansion_board_assembly_guide.mp4" type="video/mp4" />
  您的浏览器不支持 video 标签。
</video>

## 接口说明

### 100Pin 连接器（J2000）

Camera 扩展板与 RDK S100 的连接端口，为 Camera 扩展板提供功能接口（MIPI CSI 和 GPIO）以及电源（12V 和 3.3V）。

:::warning 注意

使用时，请确认 Camera 扩展板与 RDK S100 之间的连接器已完全扣合，并安装固定螺丝，以确保信号连接的可靠性。

:::

### DC 电源输入（J2001）

Camera 扩展板为 GMSL 相机配备了一个外部 12V 电源输入接口。当连接至该 Camera 扩展板的所有 GMSL 相机在 12V 电压下的供电电流需求超过 700mA 时，需通过此 DC 电源座为 GSML 相机供电。

:::info 信息

1. 适配器插头规格：内径 2.5mm，外径 6mm。
2. 适配器额定电压要求为 12V。根据需接入的 GMSL 相机模组要求选取合适的电流参数。

:::

### GMSL 相机接口（J2100）

Camera 扩展板集成了 MAX96712 解串芯片，能够接入 4 路 GMSL2 相机，并且可通过同轴线缆为 GMSL 相机提供 12V 电源。

:::info 信息

1. 当 GMSL 相机的 12V 电源电流需求在 700mA 以内时，无需接入外部 12V 适配器，此时 12V 电源由 RDK S100 提供。若电流需求超过 700mA，则必须接入外部 12V 适配器，以保障 GMSL 相机模组电源的稳定供给。
2. Camera 扩展板可为每路 GMSL 相机提供最大 550mA@12V 的电流。若超过该电流规格，将无法保证 GMSL 相机模组的稳定运行。
3. GMSL 接口采用 mini Fakra 4-in-1 z code 连接器，请选用地瓜机器人推荐的线缆与相机进行连接，以保障 GMSL 高速信号的稳定传输。

:::

### MIPI 相机接口（J2200, J2201）

接口定义: [drobotics_rdk_s100_camera_expansion_board_pinlist_v1p0.xlsx](../../../static/asset/rdk_s100/drobotics_rdk_s100_camera_expansion_board_pinlist_v1p0.xlsx)

Camera 扩展板配备 2×4 Lane MIPI CSI D PHY 接口，支持同时连接两路 MIPI 相机。MIPI 相机接口支持 1.8V 和 3.3V 两种逻辑电平，同时为开发者提供 LPWM 同步信号或 24MHz MCLK 信号。

:::warning 注意

1. 两个 MIPI 相机接口中 VDD_PERI_3V3 最大供电电流为 500mA，当系统处于 light sleep 和 deep sleep 模式时，VDD_PERI_3V3 电源关闭。
2. 两个 MIPI 相机接口中 MCLK 频率为 24MHz，由 Camera 扩展板上有源晶体提供。

:::

### MIPI 接口电平切换（SW2201）

MIPI 相机接口的控制信号支持 1V8 和 3V3 逻辑电平的切换，便于连接具有不同需求的相机模组。通过切换拨码开关 SW2201 的位置来实现逻辑电平切换。

| 开关编号   | 相机编号    | 3V3                              | 1V8                              |
| ---------- | :---------- | :------------------------------- | :------------------------------- |
| 1 （右侧） | MIPI 相机 1 | MIPI 相机 1 接口为 3.3V 逻辑电平 | MIPI 相机 1 接口为 1.8V 逻辑电平 |
| 2          | MIPI 相机 2 | MIPI 相机 2 接口为 3.3V 逻辑电平 | MIPI 相机 2 接口为 1.8V 逻辑电平 |

### MIPI 接口功能切换（SW2200）

MIPI 相机接口连接器的第 5 引脚支持 LPWM 和 MCLK(24MHz)两种功能切换，以满足不同开发需求。通过切换拨码开关 SW2200 的位置来实现功能切换。

| 开关编号  | 相机编号    | LPWM                                | MCLK                                |
| --------- | :---------- | :---------------------------------- | :---------------------------------- |
| 1（右侧） | MIPI 相机 1 | MIPI 相机 1 接口第 5 脚为 LPWM 信号 | MIPI 相机 1 接口第 5 脚为 MCLK 信号 |
| 2         | MIPI 相机 2 | MIPI 相机 2 接口第 5 脚为 LPWM 信号 | MIPI 相机 2 接口第 5 脚为 MCLK 信号 |

## 电源指示灯（D2000）

电源指示灯，其位置在 DC 电源输入接口旁边。

| 指示灯状态 | 说明                                                       |
| :--------- | :--------------------------------------------------------- |
| 绿色常亮   | Camera 扩展板与 RDK S100 已连接，RDK S100 已输出 3.3V 电源 |
| 熄灭       | Camera 扩展板与 RDK S100 连接异常或 3.3V 电源异常          |

## 连接器型号

| 连接器 | 连接器型号                      | 连接器厂商   |
| :----- | :------------------------------ | :----------- |
| J2000  | HC-PBB05-2-100-M-H4.0-G1-R-P-04 | 华灿天禄     |
| J2001  | ZX-DC-WC2.56.3                  | 兆星精密电子 |
| J2100  | 112038-161410                   | 信翰精密     |
| J2200  | AFC01-S22FCA-00                 | 钜硕电子     |
| J2201  | AFC01-S22FCA-00                 | 钜硕电子     |

## 适配模组

参考[7.1.2 配件清单](../../07_Advanced_development/01_hardware_development/02_accessory.md)
