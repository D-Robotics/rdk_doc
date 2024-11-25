---
sidebar_position: 3
---

# 1.2.3 RDK Ultra

## Preparation for Flashing

### **Power Supply**

The RDK Ultra development board is powered via a DC interface. It is recommended to use the power adapter included in the `official kit` or a power adapter with at least **12V/5A** output.

:::caution
Do not use a computer USB port to power the board. Insufficient power may cause **abnormal shutdowns or repeated reboots**.

For more troubleshooting tips, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section.
:::

---

### **Storage**

The RDK Ultra board includes 64GB of onboard eMMC storage, so no additional storage card is required.

---

### **Display**

The RDK Ultra development board supports an HDMI display interface. Connect the board to a monitor using an HDMI cable to enable graphical desktop display.

---

### **Network Connection**

The RDK Ultra development board supports both Ethernet and Wi-Fi network interfaces. Users can use either interface to connect to a network.

---

## System Flashing

The RDK Ultra currently provides an Ubuntu 20.04 system image, supporting graphical interaction with the Desktop version.

:::info Note
The **RDK Ultra** comes with a pre-installed test system image. To ensure the use of the latest version, <font color='Red'>it is recommended to follow this guide to flash the latest system image</font>.
:::

---

### Image Download {#img_download}

Click [**Download Image**](https://archive.d-robotics.cc/downloads/os_images), select the `rdk_ultra` directory, and choose the appropriate version to go to the file download page. For example, to download version 1.0.0 of the system image:

![image-20230510143353330](../../../../../../static/img/01_Quick_start/image/install_os/20231010120539.png)

After downloading, extract the Ubuntu system image file, such as `ubuntu-preinstalled-desktop-arm64-rdkultra.img`.

:::tip

- **Desktop**: Ubuntu system with a desktop interface, enabling operation with an external screen and mouse.  
- **Server**: Ubuntu system without a desktop, accessible via serial or remote network connection.

:::

---

### System Flashing

When flashing the Ubuntu system on the RDK Ultra development kit, use the D-Robotics `hbupdate` flashing tool. The tool supports both Windows and Linux versions, named `hbupdate_win64` and `hbupdate_linux`, respectively. Download the tool from [hbupdate](https://archive.d-robotics.cc/downloads/hbupdate/).

:::tip Notes

- Extract the tool archive, ensuring the extraction path does not contain **spaces, Chinese characters, or special symbols**.  
- The tool communicates with the RDK Ultra via the Ethernet port. For optimal flashing speed, ensure that the **PC supports a Gigabit Ethernet port and uses a direct connection**.  
- Configure the PC's network settings to use a **static IP address** as follows:
  - **IP**: 192.168.1.195  
  - **Netmask**: 255.255.255.0  
  - **Gateway**: 192.168.1.1  

:::

1. Use an Ethernet cable to connect the RDK Ultra directly to the PC, and ensure the connection is reachable via ping.

2. Short-circuit the `FC_REC` and `GND` signals on the function control interface (Interface 10).

   ![image-ultra-fc-rec](../../../../../../static/img/01_Quick_start/image/install_os/image-ultra-fc-rec.jpg)

3. Run the `hbupdate` tool, select the development board model as `RDK_ULTRA` (required).

   ![image-flash-system1](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system1.jpg)

4. Click the `Browse` button to select the system image file to be flashed (required).

   ![image-flash-system2](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system2.jpg)

5. Click the `Start` button to begin flashing. After confirming the prompts, click the `OK` button:

   ![image-flash-system3](../../../../../../static/img/01_Quick_start/image/install_os/image-system-download3.jpg)

6. When the tool displays the following log, it indicates that the flashing process has started. The process duration depends on the network transmission speed. Please wait patiently.

   ![image-flash-system4](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system4.jpg)

7. Wait for the tool to complete the flashing process and check the results:

- If successful, the tool will display the following message:

   ![image-flash-system6](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system6.png)

- If an error occurs, check if steps 1–3 were performed correctly:

   ![image-flash-system7](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system7.png)

- If an error occurs indicating slow network speed, use a higher-performance PC to retry the process:

   ![image-flash-system8](../../../../../../static/img/01_Quick_start/image/install_os/image-rdk-ultra-system8.jpg)

:::caution

If the flashing process is interrupted, repeat the steps above to retry.
:::

---

### System Boot

First, ensure the board is powered off, then remove the short circuit between the `FC_REC` and `GND` signals on the function control interface (Interface 10). Connect the board to a monitor using an HDMI cable, and then power on the board.

During the first boot, the system will perform default environment configuration, which takes approximately 45 seconds. After the configuration is complete, the Ubuntu system desktop will be displayed on the monitor.

Once the Ubuntu Desktop system boots, the system desktop will be displayed via the HDMI interface, as shown below:

![image-desktop_display.jpg](../../../../../../static/img/01_Quick_start/image/install_os/image-desktop_display.jpg)

---

## **FAQ**

### **Precautions**

- Do not plug or unplug any devices except USB, HDMI, and Ethernet cables while the board is powered on.

:::tip

For more troubleshooting tips, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section. You can also visit the [D-Robotics Developer Official Forum](https://developer.d-robotics.cc/forum) for help.

:::
