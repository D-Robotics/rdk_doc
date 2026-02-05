# 1.2.2.1 Flashing Instructions


:::warning Notes

- Do not plug or unplug any devices while powered, except for USB, HDMI, and Ethernet cables.
- The RDK X3 Module is powered via the power connector on the carrier board. The [official carrier board](../../accessory.md#rdk-x3-module-accessories-list) is powered via a DC jack. It is recommended to use the certified 12V/2A adapter listed in the official accessories list.
- Do not power the development board via a computer's USB port, as insufficient power supply may cause abnormal behaviors such as **unexpected power-offs or repeated reboots**.

:::

## Boot Modes

### Boot from SD Card

#### RDK Studio Tool
  - Supports flashing from local images and downloading images online
  - Compatible with Windows, Linux, and macOS

#### Rufus Tool
  - Supports flashing from local images
  - Compatible with Windows

### Boot from eMMC

#### RDK Studio Tool
  - Supports flashing from local images and downloading images online
  - Compatible with Windows, Linux, and macOS
  - Uses UMS (USB Mass Storage) mode to flash system images

  
#### Rufus Tool
  - Supports flashing from local images
  - Compatible with Windows
  - Uses UMS (USB Mass Storage) mode to flash system images

#### hbupdate Tool
  - Supports flashing from local images
  - Compatible with Windows and Linux
  - Requires the RDK X3 Module to enter U-Boot flashing mode (i.e., fastboot mode)