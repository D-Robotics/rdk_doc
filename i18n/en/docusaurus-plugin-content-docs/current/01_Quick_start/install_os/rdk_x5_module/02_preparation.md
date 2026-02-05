---
sidebar_position: 2
---

# 1.2.4.2 Flashing Preparation

## Storage Device

- Prepare a storage card with at least 16GB capacity to meet the storage requirements of the Ubuntu system and application software.

- SD card reader

## Image Download

:::warning Flashing Tip

The **RDK X5 Module** comes pre-flashed with a test version of the system image. To ensure you are using the latest system version, <font color='Red'>it is recommended to follow this document to flash the latest system image</font>.
:::

:::warning Note

The RDK X5 Module only supports system versions 3.2.0 and later.

:::


1. Click [**Download Image**](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/) and select the desired RDK X5 image version.


        ![Image Version Selection Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download.png)


2. Enter the selected version directory and choose either the “server” or “desktop” image version, then click to download.
   
          :::info Image Description
        
            Supports both headless server system and desktop graphical interface.
            - desktop: Ubuntu system with a desktop environment; supports external monitor and mouse operation
            - server: Ubuntu system without a desktop; can be operated via serial port or remote network connection
            
          :::

          ![Image Version Selection Interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download1.png)


  3. After the download completes, extract the Ubuntu system image file, e.g., `rdk-x5-ubuntu22-preinstalled-desktop-3.2.3-arm64.img`

  
## Flashing Tool Download


### RDK Studio Tool Download

[[Click here]](https://developer.d-robotics.cc/en/rdkstudio) to access the download page. Select the installer version matching your operating platform; the User Installer is recommended.

![RDK Studio Download Page](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rdkStudio_download.png)

### Rufus Tool Download

[[Click here]](https://rufus.ie/en/) to visit the official website and select the appropriate version for your platform.