---
sidebar_position: 2
---

# 1.2.1.2 Bootloader (NAND) firmware flashing

Bootloader flashing writes low-level firmware (e.g. Miniboot, U-Boot) to the on-board **NAND Flash**. It handles power-on init and loading the OS. Do this when the board will not boot, the bootloader is corrupted, or you need to update the boot stack—this controls whether the device **can boot at all**.

:::warning About NAND firmware

- The minimal RDK system in NAND includes Bootloader (Miniboot, U-Boot) and related components.
- Boards ship with matching NAND firmware preinstalled.
- **Do not downgrade** to older firmware; that can brick the device.
- If the device does not boot, reflash the NAND firmware.

:::

## Flashing tool

RDK X3 bootloader flashing on a PC uses **hbupdate**.

### hbupdate

- Supports **local image** files.
- Windows.
- eMMC flashing via **UMS** mode.

#### Download

[[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/) and choose the package for your OS and firmware version.

#### Installation

**Windows**

1. Extract the `hbupdate` archive.

    :::warning Note

    Extract to a path without spaces, non-ASCII characters, or special symbols.

    :::


2. Double-click the `.exe` to launch hbupdate.

  <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_exe.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

## Firmware download

[[Click here]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x3/) and download the `.img` file that matches your product capacity.

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/miniboot_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## Flashing steps

### Driver check

1. **Serial**: Connect dupont wires to connector **3** on the board and a USB–serial adapter to the PC. Verify wiring.

  <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
  />

2. On first use, install the **CH340** driver: [[click here]](https://archive.d-robotics.cc/downloads/software_tools/serial_to_usb_drivers/).

3. After installation, Device Manager should list the serial port.

    <img 
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/usb_serial.png" 
            style={{ width: '100%', height: 'auto', align:'center'}}
    />

4. Download [MobaXterm](https://mobaxterm.mobatek.net/download.html).
5. Open MobaXterm, click **Session** → **Serial**, set the port (e.g. `COM9`—use what Windows shows), then **OK**.

  

    Serial settings:

    | Setting | Value |
    | ------ | ------ |
    | Baud rate | 921600 |
    | Data bits | 8 |
    | Parity | None |
    | Stop bits | 1 |
    | Flow control | None |
    <img 
              src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/serial_parameter.png" 
              style={{ width: '100%', height: 'auto', align:'center'}}
      />

6. Power on the board and **hold Space** to stop autoboot and enter U-Boot. Type `fastboot 0` to enter fastboot mode.

    ```bash
    U-Boot 2022.10-gca2c6582a0 (Mar 13 2024 - 19:04:15 +0800)

    ... (omitted) ...
    Hit any key to stop autoboot:  0
    Hobot>
    Hobot>fastboot 0
    ```

7. **If the driver is missing**, Device Manager may show an unknown **USB download gadget** device.

    <img 
              src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/download_gadget.png" 
              style={{ width: '100%', height: 'auto', align:'center'}}
      />

### Download and install the Android/fastboot driver {#x3md-android-driver}

1. [[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) to download and extract `android_hobot.zip`.


2. In the extracted folder, run `5-runasadmin_register-CA-cer.cmd` **as Administrator** to register the driver.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/usb_driver_administrator.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. After installation, Device Manager should show a fastboot **Android Device**.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/Android_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />



### Flash the firmware

Run `hbupdate.exe` and follow the steps below.

1. **Board model** (required):

    - **RDK_X3_2GB**: RDK X3 (Horizon X3 Pi), 2GB RAM.
    - **RDK_X3_4GB**: RDK X3 (Horizon X3 Pi), 4GB RAM.
    - **RDK_X3_MD_2GB**: RDK X3 Module, 2GB RAM.
    - **RDK_X3_MD_4GB**: RDK X3 Module, 4GB RAM.

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_boardname.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. Click **Browse** and select the image to flash (required).

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_select_image.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. Click **Start** to begin:

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_start.png" 
      style={{ width: '80%', height: 'auto', align:'center'}}
    />

4. When finished, power off, disconnect from the PC, and power on again.

### Verify the result

- On success:

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_success.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

- On failure, you may see the message below—confirm **Android Device** appears in Device Manager:

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3/hbupdate_fail.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />
