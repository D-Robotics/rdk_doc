---
sidebar_position: 4
---

# 1.1.2 相机扩展板

![image-rdk_100_camera_expansion_board](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_camera_expansion_board.png)

地瓜机器人 RDK S100 相机扩展板是 RDK S100 开发者套件的一个扩展件，用于将开发者套件上的 Camera 拓展接口转换为 MIPI 接口和 GMSL 接口。便于开发者接入 MIPI 或者 GMSL 相机，进行功能验证和二次开发。

:::warning

1. 本产品应在通风良好的环境中使用，在密闭空间使用时，需要做好散热措施。
2. 使用时，本产品应放置在稳固、平坦、不导电的表面上。
3. 将不兼容的设备与本品连接时，导致设备损坏，将不支持维修。
4. 所有与本产品配套使用的外围设备均应符合使用国家的相关标准，并标明相应地确保满足安全和性能要求。 外围设备包括但不限于与本品结合使用时的键盘、显示器和鼠标。
5. 与本产品一起使用的所有外围设备的电缆和连接器必须有足够的绝缘，以便相关的满足安全要求。

:::

:::warning 安全守则

为避免本产品发生故障或损坏，请遵守以下事项：

1. 运行时，请勿接触水或湿气，或放置在导电物体表面上，不要接触任何热源，以确保本品在正常环境温度下可靠运行。
2. 装配时，避免对印刷电路板和连接器造成机械或电气损坏。
3. 通电时，避免手触摸印刷电路板及设备边缘，减少静电放电损坏的风险。

:::

## 产品规格

| **名称**    | **参数**                                         |
| ----------- | ------------------------------------------------ |
| 解串器      | Maxim MAX96712                                   |
| MIPI 连接器 | 2x 22-Pin MIPI CSI-2                             |
| GMSL 连接器 | Fakra-Mini 4in1                                  |
| 外部供电    | 12V DC，仅用于电流需求大于 700mA 时，最大 2.4A。 |
| 工作温度    | 0℃~45℃                                           |

## 接口描述

![camera_expansion_board_interface](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_camera_expansion_board_interface.png)

| **序号** | **功能**                                  |
| -------- | ----------------------------------------- |
| 1        | 相机扩展板 100-pin 连接器，用于和主板连接 |
| 2        | 拨码开关，用于切换 MIPI 相机的逻辑电平    |
| 3        | 22-pin 的 MIPI 接口                       |
| 4        | 22-pin 的 MIPI 接口                       |
| 5        | Fakra-Mini 4in1 接口                      |
| 6        | 外部供电 DC 接口                          |

## 适配模组

参考[7.1.2 配件清单](../../07_Advanced_development/01_hardware_development/02_accessory.md)
