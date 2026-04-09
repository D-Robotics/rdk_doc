---
sidebar_position: 2
---

# 1.2.4.2 Bootloader (NAND) firmware flashing


Bootloader flashing writes low-level firmware (e.g. Miniboot, U-Boot) to **NAND Flash**. It handles power-on initialization and loading the OS. Use it when the board will not boot, the bootloader is damaged, or you must update the boot stack.

:::warning About NAND firmware

- The minimal RDK system in NAND includes Bootloader (Miniboot, U-Boot).
- Boards ship with matching NAND firmware.
- **Do not downgrade**; that can brick the device.
- If the device does not boot, reflash NAND firmware.

:::

## Firmware download


### Download

[[Click here]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/) and download the latest `product_<release_date>.zip` (exact name depends on the release folder).

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/miniboot_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## Flashing tool

RDK X5 Module Bootloader flashing on a PC uses **XBurn**.

### XBurn

- For Bootloader firmware.
- Windows, Linux, macOS.


#### Download

[[Click here]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) and get **XBurn**:
- Windows: `XBurn-gui_<version>_x64-setup.exe`.
- Ubuntu: `XBurn-gui_<version>_x64-setup.deb`.
- macOS: `XBurn-gui_<version>_x64-setup.dmg`.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_download.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

#### Installation

**Windows**

1. Run the installer, e.g. `XBurn-gui_1.1.9_x64-setup.exe`, and click **Next**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_01.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

2. Choose **Uninstall before installing** / **Do not uninstall**, then **Next**:

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_02.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

3. Choose uninstall path if prompted.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_03.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

4. Click **Browse** for the install location.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_04.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

5. Click **Install**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_05.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

6. When you see **Setup was completed successfully**, click **Next**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_06.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

7. Check **Run xburn-gui** and **Create desktop shortcut**, then **Finish**.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_07.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

8. XBurn-gui opens; later you can start it from the desktop shortcut.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_08.png" 
          style={{ width: '70%', height: 'auto', align:'center'}}
    />

**Linux**

1. In the package folder run e.g. `sudo dpkg -i XBurn-gui_1.1.9_amd64.deb`. Example output:

```bash
(base) hobot@hobot-ThinkPad-T14-Gen-1:~/tools$ sudo dpkg -i XBurn-gui_1.1.5_amd64.deb 
[sudo] password for hobot: 
Selecting previously unselected package xburn-gui.
(Reading database ... 494898 files and directories currently installed.)
Preparing to unpack XBurn-gui_1.1.5_amd64.deb ...
Unpacking XBurn-gui (1.1.5) ...
Setting up XBurn-gui (1.1.5) ...
Udev rules installed and activated
User nobody added to plugdev group
User hobot added to plugdev group
User snapd-range-524288-root added to plugdev group
User snap_daemon added to plugdev group
User xpj added to plugdev group
Processing triggers for mailcap (3.70+nmu1ubuntu1) ...
Processing triggers for gnome-menus (3.36.0-1ubuntu3) ...
Processing triggers for desktop-file-utils (0.26-1ubuntu3) ...
Processing triggers for hicolor-icon-theme (0.17-2) ...
```

2. Run `sudo XBurn-gui` or launch **XBurn-gui** from the app menu (you may be prompted for your password).

**macOS**

Open `XBurn-gui_1.1.5_x64-setup.dmg`, drag **XBurn-gui** into **Applications**, then launch it from there.

<img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/zh/x5/xburn_install_mac.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
/>

## Flashing steps

### Check drivers

On first launch, open the **Drivers** tab to verify installation.

- **Driver name**: e.g. USB Driver (ADB, Fastboot, DFU) and USB to Serial Driver (CH341).
- **Current version**: installed version.
- **Actions**: install or uninstall.
- **Scan**: detect and install missing drivers. If a driver is missing, click **Install** and follow prompts.

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/xburn_check_driver.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

### Hardware

- **Serial to PC**: Micro-USB
- **Flashing port to PC**: USB Type-C
- **Power**: USB Type-C with a **5V/5A** adapter

    <img 
          src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5_module/nand_insatll_hardware_connection.png" 
          style={{ width: '100%', height: 'auto', align:'center'}}
    />

### Flash firmware

1. Set parameters:

    - **Product**: X5
    - **Connection**: Serial+USB
    - **Download mode**: `xmodem_fastboot`
    - **Image folder**: directory containing the images to flash
    - **Batch count**: concurrent devices (depends on PC and links; **up to 8** is suggested)
    - **Serial port**: the port the PC sees
    - **Baud rate**: **921600** for RDK X5 Module

        <img 
            src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5_module/xburn_parameter_config.png" 
            style={{ width: '100%', height: 'auto', align:'center'}}
        />

2. Click **Start upgrade**. When prompted, cycle power. If the serial port disappears after power cycle, leave power off until prompted, then power on.

        <img 
                src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/xburn_power.png" 
                style={{ width: '100%', height: 'auto', align:'center'}}
        />

3. Upgrade runs after power-on.

        <img 
                src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/xburn_start.png" 
                style={{ width: '100%', height: 'auto', align:'center'}}
        />

4. Success.

        <img 
                    src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/install_os_260408/en/x5/xburn_success.png" 
                    style={{ width: '100%', height: 'auto', align:'center'}}
        />



