---
sidebar_position: 1
---

# 1.2.1.1 Flashing Instructions


:::warning Notes

- Do not plug or unplug any devices while powered on, except for USB, HDMI, and Ethernet cables.
- Use a power adapter from a reputable brand; otherwise, abnormal power supply may occur, leading to unexpected system shutdowns.
- It is recommended to use the onboard POWER ON/OFF button to power the mainboard on or off, and only plug or unplug the DC jack when the adapter is disconnected from power.

:::


## Flashing Modes

The RDK S100 development kit supports flashing the Ubuntu system using the PC-side tool D-Navigation, with two available flashing modes: U-Boot mode and USB mode.

### U-Boot Mode:

- This mode requires the RDK S100 to enter U-Boot flashing mode (i.e., fastboot mode).
- It is commonly used in everyday flashing scenarios and satisfies most standard system flashing requirements.


### USB Mode:

- This mode is based on the DFU protocol and is used in special situations—such as when the RDK S100 cannot enter U-Boot mode or when system corruption has rendered the device unbootable ("bricked")—to restore the system.
- Suitable for flashing blank chips or recovering devices with corrupted firmware that cannot enter U-Boot.