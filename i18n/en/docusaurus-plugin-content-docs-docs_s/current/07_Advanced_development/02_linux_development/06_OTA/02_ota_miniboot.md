---
sidebar_position: 2
---

# miniboot Upgrade

:::note
- The miniboot upgrade described in this section can be used in non-OTA images;
- The miniboot upgrade requires a reboot to take effect;
:::

## Overview

To meet the system's requirement for standalone miniboot updates, the miniboot upgrade solution adopts a partition-level update mechanism based on OTA (Over-The-Air) technology, performing secure and controllable version updates for the critical partitions involved in miniboot. This solution focuses on system security, upgrade reliability, and stability of non-upgradable partitions, ensuring that the system will not brick under any abnormal upgrade circumstances and can automatically recover and continue booting.

## Features

This solution has the following core features:

- Only upgrades miniboot-related BAK partitions and partitions using A/B mechanism

    Partitions in miniboot are divided into three types: Permanent, BAK, and A/B. Permanent partitions are critical permanent areas that are not normally updated and have an extremely low update frequency. To ensure system stability and avoid irreversible brick risks caused by misoperation, this solution only supports OTA upgrades for BAK partitions and A/B partitions, without touching Permanent partitions. The specific partitions involved in the upgrade include: `HSM_FW, HSM_RCA, keyimage, SBL, scp, spl, MCU, acore_cfg, bl31, optee, uboot`.

- Uses onboard ota_tool to complete upgrades, with comprehensive verification and recovery mechanisms

    When performing upgrade operations on the device side, the system uses the built-in ota_tool, which provides complete upgrade process management including: partition package verification, partition write protection, upgrade status recording, and failure rollback mechanisms. Whether facing abnormal power failure, upgrade interruption, or data corruption, the system can automatically fall back to a normally bootable version, ensuring the device does not brick in case of upgrade failure.

- Automatically handles A/B synchronization issues for partitions not involved in the upgrade

    After OTA upgrade completion, during the A/B switching process, the system may switch to partitions that were not involved in this upgrade. To avoid risks caused by inconsistent content between A/B sides, ota_tool automatically synchronizes and copies all A/B partitions that were not involved in the upgrade, ensuring partition content consistency and guaranteeing normal system boot and operation.

- This method does not support partition table upgrades

    Before upgrading, this solution compares the partition table in the upgrade package with the partition table information in the device. If they are inconsistent, the upgrade will exit. Therefore, for upgrades involving partition table changes, the full system flashing must be completed using Dijia tools. Reference: [System Burning](../../../01_Quick_start/02_install_os/rdk_s100/01_instruction.md).

Overall, based on flexible OTA upgrade capabilities, strict verification mechanisms, and comprehensive rollback strategies, this solution achieves secure updates of miniboot partitions while minimizing risks during the system upgrade process. For the principles and detailed introduction of OTA upgrade, please refer to the [System OTA Upgrade](./01_ota_system.md) section.

## Usage Guide

### Updating the miniboot Package

Update the hobot-miniboot package on the device side:
```bash
sudo apt update
sudo apt-get install -y hobot-miniboot
```

### On-Device Upgrade

After updating the miniboot package, there are two ways to initiate the upgrade:
1. Upgrade using the `rdk-miniboot-update` command
    - Parameter Description

        | Parameter            | Description                                                        |
        | -------------------- | ------------------------------------------------------------------ |
        | `--build` / `--type` | Specifies the Miniboot upgrade version type, options `release` or `debug`, optional **Default: release** |
        | `--reboot`           | Specifies whether to reboot immediately after upgrade completion, values `y` or `n`, optional, if omitted, confirmation will be interactive |

    - Examples
        ```bash
        # Without any parameters, use interactive mode to confirm upgrade behavior (default uses release version)
        rdk-miniboot-update

        # Use release version for upgrade (default), reboot immediately after upgrade
        rdk-miniboot-update --reboot y

        # Use release version for upgrade (default), do not reboot immediately after upgrade
        rdk-miniboot-update --reboot n

        # Use debug version for upgrade, reboot immediately after upgrade
        rdk-miniboot-update --build debug --reboot y

        # Use debug version for upgrade, do not reboot immediately after upgrade
        rdk-miniboot-update --build debug --reboot n

        ```

2. Upgrade via srpi-config

    - This method can be referenced in the Update miniboot section of [srpi-config](../../../02_System_configuration/02_srpi-config.md#system-options).

    - **Note: This method only supports release version upgrades. For debug version upgrades, please use the `rdk-miniboot-update` command.**