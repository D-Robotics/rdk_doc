---
sidebar_position: 2
---

# 1.2.2.2 Bootloader (NAND) firmware flashing

## Overview

Bootloader flashing writes low-level firmware (e.g. Miniboot, U-Boot) to **NAND Flash**. It handles power-on initialization and loading the OS. Use it when the board will not boot, the bootloader is damaged, or you must update the boot stack.

:::warning About NAND firmware

- The minimal RDK system in NAND includes Bootloader (Miniboot, U-Boot).
- Boards ship with matching NAND firmware.
- **Do not downgrade**; that can brick the device.
- If the device does not boot, reflash NAND firmware.

:::

## Flashing tool

RDK X3 Module bootloader flashing uses **hbupdate** on a PC.

### hbupdate

- **Local image** files.
- Windows.
- Fastboot-based Bootloader flashing.

#### Download

[[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/) and choose the package for your OS and firmware.

#### Installation

**Windows**

1. Extract the `hbupdate` archive.

    :::warning Note

    Extract to a path without spaces, non-ASCII characters, or special symbols.

    :::

2. Double-click the `.exe` to launch hbupdate.

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_exe.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>


## Firmware download

[[Click here]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x3/) and download the `.img` for your product capacity.

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/miniboot_download.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

## Flashing steps


### Driver check

On **Windows**, confirm drivers before using the tool.

1. Connect the carrier **Micro USB** (flashing port) to the PC.

    <img
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/flashing_port.png"
    style={{ width: '100%', height: 'auto', align: 'center' }}
    />

2. Power on. If **USB download gadget** is unknown, install/update the driver.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

### Download and install the driver {#x3md-android-driver}

1. [[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) to download and extract `android_hobot.zip`.


2. Run `5-runasadmin_register-CA-cer.cmd` **as Administrator**.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/usb_driver_administrator.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. Update the **USB download gadget** driver from the extracted folder.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

4. **Android Device** should appear when done.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/Android_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />


### Flash the firmware

Run `hbupdate.exe`:

1. **Board model** (required):

    - **RDK_X3_2GB**: RDK X3 (Horizon X3 Pi), 2GB—minimal image only.
    - **RDK_X3_4GB**: RDK X3 (Horizon X3 Pi), 4GB—minimal image only.
    - **RDK_X3_MD_2GB**: RDK X3 Module, 2GB.
    - **RDK_X3_MD_4GB**: RDK X3 Module, 4GB.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_boardname.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

2. Click **Browse** and select the image (required).

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_select_image.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. Click **Start**:

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_start.png"
      style={{ width: '80%', height: 'auto', align: 'center' }}
    />

4. Power off, disconnect USB, remove the **BOOT** jumper, and power on again.

If boot is OK, the **ACT** LED shows two fast blinks and one slow blink.

### Verify the result

- Success:

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_success.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

- Failure—confirm **Android Device** exists:

<img
  src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_fail.png"
  style={{ width: '100%', height: 'auto', align: 'center' }}
/>

