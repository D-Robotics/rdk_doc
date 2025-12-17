---
sidebar_position: 4
---

# 1.1.3 MCU Port Expansion Board

![image-rdk_100_mcu_port_expansion_board](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_mcu_port_expansion_board.png)

The RDK S100 MCU Port Expansion Board (including the accompanying FPC) is a core expansion module of the D-Robotics RDK S100 series developer kit, primarily designed to extend MCU interface capabilities, supporting Ethernet, CAN_FD, ADC, and more.

:::warning

1. This product is exclusively compatible with RDK S100 series mainboards and must not be used with other device models.
2. During use, place this expansion board on a stable, flat, and non-conductive surface to prevent device drop or short circuits caused by inadequate support.
3. Damage resulting from connecting non-compatible devices to this MCU expansion board is not covered by our repair service.
4. All peripheral devices used with this product (including but not limited to network and CAN devices) must comply with the safety and performance standards of the country or region of use and bear appropriate compliance certification markings.
5. All cables and connectors of peripheral devices connected to this expansion board must provide sufficient insulation to meet electrical safety requirements.

:::

:::warning Safe Usage

To prevent malfunction or damage to this expansion board, strictly adhere to the following:

1. **Environmental Requirements**: Do not expose the board to water, moisture, or conductive surfaces during operation. Keep it away from heat sources (e.g., radiators, direct sunlight) and ensure the ambient temperature complies with specifications in the product datasheet.
2. **Assembly Handling**: Handle the board gently during assembly. Avoid applying mechanical stress or electrical interference (e.g., electrostatic discharge) to the printed circuit board (PCB) or connectors.
3. **Power-On Operation**: Never touch the PCB surface or metal interfaces on the board edges while powered on to minimize the risk of electrostatic discharge (ESD) damage.

:::

## Product Specifications

| **Name**       | **Specifications**                                                                                     |
| -------------- | ------------------------------------------------------------------------------------------------------ |
| Interfaces     | 5 × CAN FD (up to 8 Mbps) <br />1 × 30-pin header supporting up to 7× ADC, 2× I²C, 2× SPI<br />1 × RJ45 |
| Onboard Module | IMU: BMI088                                                                                            |
| Operating Temp | 0℃ ~ 45℃                                                                                               |

### Topology Diagram

![image-rdk_s100_mcu_port_expansion_board_architecture_diagram.png](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_s100_mcu_port_expansion_board_architecture_diagram.png)

### Interface Description

![image-rdk_100_cam_expansionboard](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/hardware_interface/image-rdk_100_mcu_port_expansion_board_interface.png)

| **No.** | **Function**                            | No.  | Function                              |
| ------- | --------------------------------------- | ---- | ------------------------------------- |
| J1      | 100-pin connector for MCU expansion board | J6   | CAN FD Connectors 7                                  |
| J12     | 30-pin header                           | J7   | 120Ω termination resistor jumper for CAN7 |
| U4      | Ethernet Connector      | J8   | CAN FD Connectors 8                                  |
| J2      | CAN FD Connectors 5                                    | J9   | 120Ω termination resistor jumper for CAN8 |
| J3      | 120Ω termination resistor jumper for CAN5 | J10  | CAN FD Connectors 9                                  |
| J4      | CAN FD Connectors 6                                    | J11  | 120Ω termination resistor jumper for CAN9 |
| J5      | 120Ω termination resistor jumper for CAN6 |      |                                       |

### Assembly Instructions

:::danger

1. Perform installation only when the development board is powered off and the DC power plug is disconnected.
2. During installation, ensure the **connector remains parallel**, **interfaces are evenly pressed together**, and the connection is secure to avoid damaging the connectors.

:::

:::info Tip

The side of the FPC marked with the silkscreen label "MAIN" corresponds to the J23 connector on the RDK S100 mainboard, while the side marked "SUB" connects to the J1 connector on this expansion board.

:::

<video controls width="90%" preload="metadata">
  <source src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/video/mcu_port_expansion_board_assembly_guide.mp4" type="video/mp4" />
  Your browser does not support the video tag.
</video>

## Interface Details

### CAN FD Connectors (J2/J4/J6/J8/J10)

:::info Information

Each interface on the back of the expansion board is labeled with `CAN_H`, `CAN_L`, and `GND`.

:::

The expansion board provides five CAN FD interfaces (J2/J4/J6/J8/J10). Each interface includes a 120Ω termination resistor, which can be enabled by placing a jumper on the corresponding pins (J3/J5/J7/J9/J11). The mapping is as follows:

| CAN FD Channel | Connector Pin | 120Ω Resistor Jumper Pin |
| -------------- | ------------- | ------------------------ |
| CAN5           | J2            | J3                       |
| CAN6           | J4            | J5                       |
| CAN7           | J6            | J7                       |
| CAN8           | J8            | J9                       |
| CAN9           | J10           | J11                      |

### Ethernet Connector (U4)

The MCU expansion board features one Gigabit Ethernet port.

### 30-Pin Header (J12)

Pinout Definition: [drobotics_rdk_s100_mcu_port_expansion_board_pinlist_v1p0.xlsx](https://archive.d-robotics.cc/downloads/en/hardware/rdk_s100/rdk_s100_mcu_port_expansion_board/drobotics_rdk_s100_mcu_port_expansion_board_pinlist_v1p0-eng.xlsx)

:::warning Note

1. When the system is in light sleep or deep sleep mode, the VDD_5V, VDD_3V3, and VDD_1V8 power rails remain active, with maximum output currents of 300mA, 600mA, and 300mA, respectively.
2. When I2C9_SDA_3V3 and I2C9_SCL_3V3 signals are used as GPIOs, external pull-down resistors must not be connected.

:::

### IMU (U8)

:::warning Note

The corresponding functionality is not yet implemented in RDKS100_LNX_SDK_V4.0.2.

:::

An inertial measurement unit (IMU, Bosch Sensortec BMI088) is integrated and supports communication and control via the SPI-5 serial bus.

## Status Indicator

A green LED labeled "CONNECT" is located beneath the 100-pin connector (J1) on the expansion board, indicating power status and the connection status between the MCU expansion board and the RDK S100:

- **Solid green**: RDK S100 and MCU expansion board are properly connected, and 5V power is supplied normally.
- **Off**: Connection between RDK S100 and MCU expansion board is abnormal or 5V power is not supplied.

## Dimensions

Board Dimensions: 70 × 70 × 17 mm