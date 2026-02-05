---
sidebar_position: 2
---

# 1.2.1.2 Flashing Preparation

## Storage Device

- Prepare a Micro SD card with at least 8GB capacity to meet the storage requirements of the Ubuntu system and application software.

- SD card reader


## Image Download

Video: https://www.bilibili.com/video/BV1rm4y1E73q/?p=1


  1. Click [**Download Image**](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/) and select the RDK X3 image.

        :::info Version Notes

        Currently, Ubuntu 20.04/22.04 system images are provided.
        
        - Version 3.0 (Ubuntu 22.04): Built from the RDK Linux open-source code package, supporting both RDK X3 and RDK X3 Module development kits.
        - Version 2.0 (Ubuntu 20.04): Built from the RDK Linux open-source code package, supporting both RDK X3 and RDK X3 Module development kits.
        - Version 1.0: Legacy version for RDK X3 only; the system image is named `system_sdcard.img`.
        
        :::

        ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download.png)


  2. Enter the directory of the selected version and choose either the “server” or “desktop” image, then click to download.
   
          :::info Image Description
        
            Supports both headless server system and desktop graphical interface.
            - desktop: Ubuntu system with a desktop environment; supports external monitor and mouse operation.
            - server: Ubuntu system without a desktop; can be operated via serial port or remote network connection.
            
          :::

          ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x3_os_image_download1.png)


  3. After downloading, extract the Ubuntu system image file, e.g., `rdk-x3-ubuntu22-preinstalled-desktop-3.0.3-arm64.img`.

  
## Flashing Tool Download


### RDK Studio Tool Download

[[Click here]](https://developer.d-robotics.cc/en/rdkstudio) to access the download page. Select the installer version according to your operating platform. The User Installer is recommended.

![RDK Studio download interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rdkStudio_download.png)

### Rufus Tool Download

[[Click here]](https://rufus.ie/en/) to visit the official website and select the appropriate version for your platform.