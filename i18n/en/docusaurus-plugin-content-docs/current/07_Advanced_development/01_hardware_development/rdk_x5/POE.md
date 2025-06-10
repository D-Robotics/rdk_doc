---
sidebar_position: 5
---

# PoE Power Supply Usage

Currently, there are several PoE standards available, each with different voltage and power specifications. To avoid power supply issues or potential damage to the board due to incompatible PoE standards, please read the following content.

## Protocol Overview

Power over Ethernet (PoE) is a standardized technology that allows power to be transmitted to devices over Ethernet cables through twisted pairs.

A PoE system consists of two components: Power Sourcing Equipment (PSE) and Powered Devices (PD). PSE is the device responsible for providing power over Ethernet cables, often a network switch. PD refers to any device powered by a PoE system. Typically, PD devices come with an optional external power input interface, allowing the power received from PoE to be converted into other derived power supplies (e.g., VDD).

Currently, the most common PoE protocols are IEEE 802.3af and IEEE 802.3at, which support maximum power delivery of 15W and 30W, respectively.

## Interface Usage

The RDK X5 supports power supply via a PoE system but is not a complete PSE or PD device. The RDK X5 acts as the **frontend of the PD device** and the **backend load of the PD device**, as shown in the diagram below.

![PoE Interface Diagram](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_x5/whiteboard_exported_image_en.png)

The RJ45 connector on the RDK X5 has an integrated transformer that outputs AC power. This connector is located next to the Ethernet port, with a 4-pin header for AC power output.

As shown in Path ①, users should connect the 4-pin header to an external third-party PoE Hat, which feeds into the PD device circuit.

The primary function of the PoE Hat is to convert the incoming AC power into DC power. For the RDK X5, **the required DC power is 5V (MAX 5.2V)**.

As shown in Path ②, users should connect the DC 5V power to the 5V pin on the RDK X5's 40-pin header.


Users should connect this 5V DC power to the 5V pin on the RDK X5's 40-pin header.

## PoE Hat

When designing or selecting a PoE Hat, users should ensure that the PoE Hat supports the correct protocol standard. It is recommended to use a PoE solution that follows the **IEEE 802.3at** standard to ensure the reliable operation of the RDK X5 and its peripherals.
