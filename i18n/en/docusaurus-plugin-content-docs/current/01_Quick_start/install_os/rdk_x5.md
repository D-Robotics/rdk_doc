---
sidebar_position: 2
---

# 1.2.2 RDK X5

Before using the RDK X5 development board, the following preparations are required.

## Preparation for Flashing

### **Power Supply**

The RDK X5 development board is powered via a USB Type C interface. Use a power adapter that supports **5V/5A** to power the board.

:::caution
Do not use a computer USB port to power the board. Insufficient power may cause **abnormal shutdowns or repeated reboots**.  

For more troubleshooting tips, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section.
:::

---

### **Storage**

The RDK X5 development board uses a Micro SD card as the system boot medium. It is recommended to use at least an 8GB card to meet the storage needs of the Ubuntu system and application software.

---

### **Display**

The RDK X5 development board supports HDMI output. Connect the board to a monitor via an HDMI cable to enable graphical desktop display.

---

### **Network Connection**

The RDK X5 development board supports both Ethernet and Wi-Fi for network connections. You can use either interface to connect to a network.

---

## System Flashing

The RDK kit currently provides an Ubuntu 22.04 system image, supporting graphical interaction with the Desktop version.

:::info Note
The **RDK X5 Module** comes with a pre-installed test system image. To ensure the use of the latest version, <font color='Red'>it is recommended to follow the guide on the current page to flash the latest system image</font>.
:::

---

### Image Download {#img_download}

Click [**Download Image**](https://archive.d-robotics.cc/downloads/en/os_images/rdk_x5/) to open the version selection page. Navigate to the version directory and go to the system download page for version 3.0.0.

After downloading, extract the Ubuntu system image file, such as `ubuntu-preinstalled-desktop-arm64.img`.

**Version Description:**
:::caution

- Version 3.1.0: For the 3.1.0 image, there is an issue where the latest version of balenaEtcher may fail during the flashing process. The current solutions are:
1. Use balenaEtcher version 1.18.11. Flashing will proceed normally, though the verification process may report an error, but the system will still boot up correctly.
2. It is strongly recommended by the official team to use the Rufus flashing tool for image installation, as it is relatively more stable.

:::

- **3.0 Version**: Based on the RDK Linux open-source package, it supports the entire hardware series, including the RDK X5 Pi and X3 modules.

:::tip

- **Desktop**: Ubuntu system with a desktop interface, enabling operation with an external screen and mouse.
- **Server**: Ubuntu system without a desktop, accessible via serial or remote network connection.

:::

---

### Flashing the System

:::tip

Before flashing the Ubuntu system image, please prepare the following:
- A Micro SD card with at least 16GB capacity
- An SD card reader
- Download the Rufus image flashing tool ([click here to download](https://rufus.ie/))

:::

Rufus is a bootable media creation tool for Windows. To create an SD boot card using Rufus, follow these steps:

1. Open Rufus and select the target Micro SD card from the "Device" dropdown menu.

    ![image-rufus-select-device](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-device.png)

2. Click the "Select" button and choose the extracted `rdk-x5-ubuntu-preinstalled-desktop-arm64.img` file as the image to flash.

    ![image-rufus-select-img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-select-img.png)

3. Keep the other parameters as default, then click "Start" to begin flashing. Once the process is complete, you can close Rufus and remove the SD card.

    ![image-rufus-flash](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-rufus-flash.png)


---

### System Boot

First, ensure the board is powered off. Insert the prepared SD card into the Micro SD card slot on the board, connect the board to a monitor using an HDMI cable, and then power on the board.

On the first boot, the system will perform default environment configuration, which takes approximately 45 seconds. After the configuration is complete, the Ubuntu system desktop will be displayed on the monitor.

:::tip Development Board Indicator Lights

- **<font color='Green'>Green</font>** indicator: Lights up to indicate normal power status.  

If the board does not display anything for an extended period after powering on (over 2 minutes), the board may not have booted properly. Debug via the serial port to check the board’s status.

:::

Once the Ubuntu Desktop system boots, the system desktop will be displayed via the HDMI interface, as shown below:

![image-desktop_display.jpg](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-desktop_display.jpg)

---

## **FAQ**

Here are common issues for first-time users of the development board:

- **<font color='Blue'>No power-on after connection</font>**: Ensure you are using the recommended power adapter mentioned in the [Power Supply](#power-supply) section. Ensure the Micro SD card has been flashed with the Ubuntu system image.  
- **<font color='Blue'>Hot-swapping the storage card during use</font>**: The board does not support hot-swapping of the Micro SD card. If this happens, restart the board.

---

### **Precautions**

- Do not plug or unplug any devices except USB, HDMI, and Ethernet cables while powered on.  
- The USB Type C interface on the RDK X5 is for power supply only.  
- Use certified USB Type C power cables. Poor-quality cables may cause power supply issues, leading to abnormal system shutdowns.

:::tip

For more troubleshooting tips, refer to the [FAQ](../../08_FAQ/01_hardware_and_system.md) section. You can also visit the [D-Robotics Developer Official Forum](https://developer.d-robotics.cc/forum) for help.

:::
