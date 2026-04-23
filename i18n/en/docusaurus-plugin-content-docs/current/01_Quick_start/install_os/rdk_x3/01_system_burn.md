---
sidebar_position: 1
---

# 1.2.1.1 Full image flashing


Full image flashing writes a complete OS image to storage (usually an SD card or eMMC). It prepares the board with user-space apps, drivers, and core services.

:::warning Notes

- Do not hot-plug any device other than USB, HDMI, or Ethernet while power is on.
- The RDK X3 Type-C USB port is for power only.
- Use a **5V/3A** adapter as recommended in the [accessory list](../../../07_Advanced_development/01_hardware_development/rdk_x3/accessory.md).
- Do not power the board from a computer USB port; insufficient power can cause abnormal shutdowns and repeated reboots.

:::

## Boot medium

RDK X3 boots from a **microSD** card; flash the system onto the card.

- Use at least an **8GB** microSD card for Ubuntu and applications.
- An SD card reader.

## Flashing tools

RDK X3 can flash Ubuntu with **RDK Studio** or **Rufus** on a PC.

### RDK Studio

- Supports **local image** or **download while flashing**.
- Windows, Linux, and macOS.

#### RDK Studio Download Links
- [Click here to download Windows version](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22%20Setup.exe)
- [Click here to download macOS version](https://rdkstudio.bj.bcebos.com/rdkstudio/lastversion/RDKStudio-0.3.22-arm64.dmg)
#### Installation

**Windows**

Double-click the downloaded `.exe` to install and launch the app.

**macOS**

Double-click the package, then drag the app icon into **Applications**.

<img 
    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/studio_install_mac.JPEG" 
    style={{ width: '100%', height: 'auto', align:'center'}}
/>

### Using Rufus

- Supports **local image** only.
- Windows only.

#### Download

[[Click here]](https://rufus.ie/en/) to open the Rufus site and download the build for your platform.

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/rufus_install.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

#### Installation

Double-click the downloaded `.exe` to install and launch the app.

## Image download

1. [[Click here]](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/) to download an **RDK X3** image.

    :::info About the images

    - RDK X3 provides Ubuntu **20.04** and **22.04** images, in **server** (headless) or **desktop** form:

      - **desktop**: Ubuntu with a desktop; use a display, keyboard, and mouse.
      - **server**: Headless Ubuntu; use serial or network access.

    - Boards ship with a test image; for the latest release, reflash following this guide.

      - **3.x.x** (Ubuntu 22.04): Built from the RDK Linux open-source package; supports RDK X3 and RDK X3 Module.
      - **2.x.x** (Ubuntu 20.04): Same as above.
      - **1.x.x**: Legacy RDK X3 only; extracted image name is `system_sdcard.img`.

    :::

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

2. Open the version folder—for example for **3.0.3**—and download **server** or **desktop**.

    <img 
      src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download1.png" 
      style={{ width: '100%', height: 'auto', align:'center'}}
    />

3. Extract the Ubuntu image file, for example `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`.

## Flashing procedure

### Using RDK Studio

RDK Studio can flash the OS and manage the device on Windows, Linux, and macOS. See [Flashing with RDK Studio](../../09_RDK_Studio/04_flashing.md).

### Using Rufus{#using-rufus-x3}

#### Hardware

Insert the microSD into a reader and plug the reader into the PC.

#### Steps

1. Open Rufus and select the microSD card under **Device**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/rufus_select_device.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

2. Click **SELECT** and choose the extracted image, e.g. `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/rufus_select_image.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

3. Leave other options at default, click **START**, and wait until flashing completes.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x3/rufus_x3_install_finish-en.png" 
          style={{ width: '50%', height: 'auto', align:'center'}}
    />

4. When done, close Rufus and eject the card.
