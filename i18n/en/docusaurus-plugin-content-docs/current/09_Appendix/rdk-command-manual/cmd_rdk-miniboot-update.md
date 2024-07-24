---
sidebar_position: 1
---

# rdk-miniboot-update

The **rdk-miniboot-update command** is used to update the minimum boot image (miniboot) of RDK hardware.

## Syntax

```
sudo rdk-miniboot-update [options]... [FILE]
```

## Option Description

Options are optional and not required. If run without any option parameters, `rdk-miniboot-update` will use the latest version of the `miniboot` image for the upgrade.

- `-f`: Install the specified file instead of installing the latest available update.
- `-h`: Display the help text and exit.
- `-l`: Returns the complete path of the latest available `miniboot` image based on FIRMWARE_RELEASE_STATUS and FIRMWARE_IMAGE_DIR settings. This allows you to see what image file will be used for the update when running without option parameters.
- `-s`: Suppress progress messages.

## Common Commands

Update the `miniboot` image to the latest available version:

```
sudo rdk-miniboot-update
```

Update using a specified `miniboot` image:

```
sudo rdk-miniboot-update -f /userdata/miniboot.img
```

Check what image file will be used for the update when running without option parameters:

```
sunrise@ubuntu:~$ rdk-miniboot-update -l
/lib/firmware/rdk/miniboot/default/disk_nand_minimum_boot_2GB_3V3_20230413.img
```