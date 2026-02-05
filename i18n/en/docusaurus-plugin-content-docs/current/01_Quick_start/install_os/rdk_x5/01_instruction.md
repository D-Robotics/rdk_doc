---
sidebar_position: 1
---

# 1.2.3.1 Flashing Instructions


:::warning Notes

- Do not plug or unplug any devices while powered on, except for USB, HDMI, and Ethernet cables.
- The Type-C USB port on the RDK X5 is for power supply only.
- Use a high-quality, branded USB Type-C power cable; otherwise, abnormal power delivery may occur, causing unexpected system shutdowns.
- Do not power the development board via a computer's USB port, as insufficient power supply may lead to **unexpected power-offs or repeated reboots**.
- For more power supply options, refer to [PoE Power Supply Usage](../../../07_Advanced_development/01_hardware_development/rdk_x5/POE.md).

:::


## System Flashing

The RDK X5 supports SD card flashing and on-board SD card flashing, which can be accomplished using PC-side tools such as RDK Studio and Rufus to flash the Ubuntu system.

### RDK Studio Tool

- Supports local image files and online downloads  
- Compatible with Windows, Linux, and macOS  
- Supports both SD card flashing and on-board SD card flashing  

### Rufus Tool

- Supports local image files  
- Compatible with Windows  
- Supports both SD card flashing and on-board SD card flashing  

## Firmware Flashing

:::warning Firmware Flashing Notes

- The RDK minimal system is stored in the `NAND Flash` and includes critical boot components such as the `Bootloader (Miniboot, U-Boot)`.
- Devices are pre-installed at the factory with the latest NAND firmware matching the hardware.
- To ensure compatibility and device stability, downgrading to an older firmware version is strictly prohibited, as it may render the device unable to boot properly.
- If your device fails to boot, please re-flash the NAND firmware.

:::

### XBurn Tool

- Used for NAND firmware flashing  
- Compatible with Windows, Linux, and macOS