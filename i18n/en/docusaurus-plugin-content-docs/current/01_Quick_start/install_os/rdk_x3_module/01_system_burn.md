---
sidebar_position: 1
---

# 1.2.2.1 Full image flashing

Full image flashing writes a complete OS image to storage (usually an SD card or eMMC). It prepares the board with user-space apps, drivers, and core services.

:::warning Notes

- Do not hot-plug any device other than USB, HDMI, or Ethernet while power is on.
- RDK X3 Module is powered from the **DC jack** on the carrier; use a **12V/2A** adapter as listed in the [accessory list](../../../07_Advanced_development/01_hardware_development/rdk_x3_module/accessory.md).
- Do not power from a computer USB port; insufficient power can cause abnormal shutdowns and repeated reboots.

:::

## Boot medium

RDK X3 Module has on-board **eMMC** and supports **microSD** or **eMMC** as the boot medium.

- Use at least an **8GB** microSD card.
- An SD card reader.

## Flashing tools

RDK X3 Module can flash Ubuntu with **RDK Studio** or **Rufus**.

### RDK Studio

- **Local image** or **download while flashing**.
- Windows, Linux, macOS.
- microSD flashing.
- eMMC flashing via **UMS**.

#### Download

        [[Click here]](https://developer.d-robotics.cc/en/rdkstudio) to open the RDK Studio download page and choose the **User Installer** for your OS.

        <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_download.PNG" 
          style={{ width: '100%', height: 'auto', align:'center'}}
        />

#### Installation

**Windows**

Double-click the `.exe` to install and launch.



**Linux**

Run `sudo dpkg -i` with the package in a terminal.

<img 
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_install_linux.JPEG" 
    style={{ width: '100%', height: 'auto', align:'center'}}
/>

**macOS**

Double-click the package and drag the app into **Applications**.

<img 
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_install_mac.JPEG" 
    style={{ width: '100%', height: 'auto', align:'center'}}
/>

### Rufus

- **Local image** only.
- Windows only.
- microSD or eMMC (UMS).

#### Download

[[Click here]](https://rufus.ie/en/) for Rufus.

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/rufus_install.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

#### Installation

Double-click the `.exe` to install and launch.

## Image download

1. [[Click here]](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/) and choose an **RDK X3** image.

    :::info About the images

    - RDK X3 provides Ubuntu **20.04** and **22.04** in **server** or **desktop** form:

      - **desktop**: Ubuntu with a desktop.
      - **server**: Headless Ubuntu.

    - Boards ship with a test image; reflash for the latest release.

      - **3.x.x** (Ubuntu 22.04): RDK X3 and RDK X3 Module.
      - **2.x.x** (Ubuntu 20.04): Same.
      - **1.x.x**: Legacy RDK X3 only; image name `system_sdcard.img`.

    :::

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. Open the version folder—for **3.0.3** as an example—and download **server** or **desktop**.

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download1.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. Extract the image, e.g. `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`.

## Flashing procedure

### Flashing the SD card

#### RDK Studio

See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

#### Rufus

Same steps as [RDK X3 — Using Rufus](../rdk_x3/01_system_burn.md#using-rufus-x3).


### Flashing eMMC {#x3md-emmc-system-burn}

#### Driver check

Before flashing on **Windows**, confirm drivers as follows.

1. Connect the carrier **Micro USB** (flashing port) to the PC with a USB cable (see figure).

    <img
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/flashing_port.png"
    style={{ width: '100%', height: 'auto', align: 'center' }}
    />

2. Power on. If Device Manager shows unknown **USB download gadget**, update the driver.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

#### Download and install the driver {#x3md-android-driver}

1. [[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/android_hobot.zip) to download and extract `android_hobot.zip`.


2. Run `5-runasadmin_register-CA-cer.cmd` **as Administrator**.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/usb_driver_administrator.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

3. Double-click the unknown **USB download gadget**, point to the driver folder, and continue.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/download_gadget.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

4. After installation, **Android Device** should appear.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/Android_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />


#### Flash the system {#x3md-emmc-ums}


1. Set the carrier jumper to **3.3V** power.

    <img
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/3.3v_power.png"
          style={{ width: '100%', height: 'auto', align: 'center' }}
      />

2. On first use of the **debug Micro USB**, install **CH340**: [[click here]](https://archive.d-robotics.cc/downloads/software_tools/serial_to_usb_drivers/). Connect the **debug Micro USB** to the PC.

      <img
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/debug_port.png"
            style={{ width: '100%', height: 'auto', align: 'center' }}
        />

3. Connect the **flashing Micro USB** to the PC.

      <img
        src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/flashing_port.png"
        style={{ width: '100%', height: 'auto', align: 'center' }}
      />

4. Install [MobaXterm](https://mobaxterm.mobatek.net/download.html).
5. **Session** → **Serial**, set the port (e.g. `COM3`), **OK**.

    Settings:

    | Setting | Value |
    | ------ | ------ |
    | Baud rate | 921600 |
    | Data bits | 8 |
    | Parity | None |
    | Stop bits | 1 |
    | Flow control | None |

    <img
        src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/serial_parameter.png"
        style={{ width: '100%', height: 'auto', align: 'center' }}
      />

6. Power on and **hold Space** to enter U-Boot.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/enter_uboot.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

7. In U-Boot run `watchdog off`, then `ums 0 mmc 0` to expose eMMC (device 0) as a USB mass-storage device.

    ```bash
    watchdog off
    ums 0 mmc 0
    ```

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x3_module/watch_dog_off.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

8. The PC should see a standard USB drive—the module’s eMMC.

    <img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/portable_device.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />



##### RDK Studio

At **Select storage device**, choose the USB drive. See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

<img
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/studio_select_storage_refresh.png"
      style={{ width: '100%', height: 'auto', align: 'center' }}
    />

##### Rufus

In Rufus, pick that drive letter; other steps match [RDK X3 — Using Rufus](../rdk_x3/01_system_burn.md#using-rufus-x3).

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

:::warning Note

If flashing is interrupted, repeat the steps above.

:::



