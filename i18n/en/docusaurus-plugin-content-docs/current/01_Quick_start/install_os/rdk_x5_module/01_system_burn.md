---
sidebar_position: 1
---

# 1.2.4.1 Full image flashing

Full image flashing writes a complete OS image to storage (usually an SD card or eMMC). It prepares the board with user-space apps, drivers, and core services.

:::warning Notes

- Do not hot-plug any device other than USB, HDMI, or Ethernet while power is on.
- RDK X5 Module is powered via the carrier **USB Type-C** with a **5V/5A** adapter.
- Do not power from a computer USB port; insufficient power can cause abnormal shutdowns and repeated reboots.


:::

## Boot medium

RDK X5 Module has on-board **eMMC** and supports **microSD** or **eMMC** as the boot medium.

- Use at least a **16GB** microSD card.
- An SD card reader.

## Flashing tools

RDK X5 Module supports **SD in reader**, **SD in-board**, and **eMMC** flashing with **RDK Studio** or **Rufus**.

### RDK Studio

- **Local image** or **download while flashing**.
- Windows, Linux, macOS.

#### RDK Studio Download Links
- [Click here to download Windows version](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22%20Setup.exe)
- [Click here to download macOS version](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22-arm64.dmg)

#### Installation

**Windows**

Double-click the `.exe` to install and launch.


**macOS**

Double-click the package and drag the app into **Applications**.

<img 
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_install_mac.JPEG" 
    style={{ width: '100%', height: 'auto', align:'center'}}
/>

### Rufus

- Local image only.
- Windows only.

#### Download

[[Click here]](https://rufus.ie/en/) for Rufus.

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/rufus_install.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

#### Installation

Double-click the `.exe` to install and launch.

## Image download

1. [[Click here]](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/) and pick an **RDK X5** release.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/x5_os_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. Open the version folder—for **3.3.3** as an example—and download **server** or **desktop**.

    :::info About the images

    - RDK X5 Module provides **Ubuntu 22.04** in **server** or **desktop** form:

        - **desktop**: Ubuntu with a desktop.
        - **server**: Headless Ubuntu.

    - Boards ship with a test image; reflash for the latest release.

    - RDK X5 Module requires **3.2.0** or newer images.

    :::

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/x5_os_download_type.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. Extract the image, e.g. `rdk-x5-ubuntu22-preinstalled-desktop-3.3.3-arm64.img`.


## Flashing procedure

### RDK Studio

#### SD in a reader

See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

#### SD in-board

1. Insert the SD card, connect USB Type-C to the PC, **hold Sleep**, power on, wait **5 s** for flashing mode.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5_module/os_insatll_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. The PC should see the SD as a USB drive. At **Select storage device**, choose that drive. See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

        <img
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/studio_select_storage_refresh.png"
            style={{ width: '100%', height: 'auto', align: 'center' }}
            />

#### eMMC flashing

1. With **no** SD card inserted, connect USB Type-C to the PC, **hold Sleep**, power on, wait **5 s**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5_module/os_insatll_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. The PC should see **eMMC** as a USB drive. At **Select storage device**, choose that drive. See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

        <img
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3_module/studio_select_storage_refresh.png"
            style={{ width: '100%', height: 'auto', align: 'center' }}
            />

### Rufus

#### SD in a reader {#module-rufus-sd-card}

1. Insert the SD into a reader and plug it into the PC.

2. Open Rufus and select the microSD under **Device**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

3. Click **SELECT** and choose the image, e.g. `rdk-x5-ubuntu22-preinstalled-desktop-3.3.3-arm64.img`.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/rufus_select_image.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

4. Leave defaults, click **START**, and wait.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/rufus_x5_install_finish-en.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

5. Close Rufus and eject the card.

#### SD in-board

1. Insert the SD, connect USB Type-C to the PC, **hold Sleep**, power on, wait **5 s**—the SD appears as a USB drive.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5_module/os_insatll_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. In Rufus, select the drive letter; other steps match [SD in a reader](#module-rufus-sd-card).

#### eMMC flashing

1. With **no** SD card, connect USB Type-C to the PC, **hold Sleep** (connector **23**), power on, wait **5 s**—on-board eMMC appears as a USB drive.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5_module/os_insatll_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. In Rufus, select the drive letter; other steps match [SD in a reader](#module-rufus-sd-card).
