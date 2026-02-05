---
sidebar_position: 2
---

# 1.2.5.2 Flashing Preparation


## Image Download


1. Click [**Download Image**](https://archive.d-robotics.cc/downloads/os_images/rdk_ultra/).


        ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/ultra_os_image_download.png)


2. Enter the directory of your chosen version and select either the “server” or “desktop” image, then click to download.
   
          :::info Image Description
        
            Supports both headless server system and desktop graphical interface.
            - desktop: Ubuntu system with a desktop environment; supports external monitor and mouse operation.
            - server: Ubuntu system without a desktop environment; can be operated via serial port or remote network connection.
            
          :::

          ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/ultra_os_image_download1.png)


3. After downloading, extract the Ubuntu system image file, e.g., `ubuntu-preinstalled-desktop-arm64-rdkultra.img`.
  
## Flashing Tool Download
When flashing the Ubuntu system onto the RDK Ultra development kit, you need to use the D-Robotics hbupdate flashing tool. Currently, the tool is available for both Windows and Linux platforms, named starting with `hbupdate_win64` and `hbupdate_linux`, respectively.

[[Click here]](https://archive.d-robotics.cc/downloads/hbupdate/) to access the download page and select the appropriate version based on your operating system.

:::warning Notes

  - Extract the tool archive, ensuring that the extraction path **does not contain spaces, Chinese characters, or special symbols**.
  - The tool communicates with the RDK Ultra via Ethernet. To ensure optimal flashing speed, please confirm that your **PC supports a Gigabit Ethernet port and is connected directly**.
  - Configure your PC’s network settings to use a **static IP address** as follows:
    - IP: 192.168.1.195
    - Netmask: 255.255.255.0
    - Gateway: 192.168.1.1
:::