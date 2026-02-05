---
sidebar_position: 2
---

# 1.2.3.2 Flashing Preparation

## System Flashing Preparation

### Storage Device

- Prepare a Micro SD card with at least 16GB capacity to meet the storage requirements of the Ubuntu system and application software.

- SD card reader


### Image Download

1. Click [**Download Image**](https://archive.d-robotics.cc/downloads/os_images/rdk_x5/) and select the RDK X5 image version.

        ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download.png)

2. Enter the selected version directory and choose either the "server" or "desktop" image version, then click to download.
   
          :::info Image Description
        
            Supports both headless server system and desktop graphical interface.
            
            - desktop: Ubuntu system with a desktop environment; supports external monitor and mouse operation.
            - server: Ubuntu system without a desktop environment; can be operated via serial port or remote network connection.
            
          :::

          ![Image version selection interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/x5_os_image_download1.png)

3. After the download completes, extract the Ubuntu system image file, e.g., `rdk-x5-ubuntu22-preinstalled-desktop-3.0.1-arm64.img`.

  
### Flashing Tool Download

#### RDK Studio Tool Download

[[Click here]](https://developer.d-robotics.cc/en/rdkstudio) to access the download page. Select the installer version according to your operating platform. The User Installer is recommended.

![RDK Studio download interface](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/rdkStudio_download.png)

#### Rufus Tool Download

[[Click here]](https://rufus.ie/en/) to visit the official website and select the appropriate tool version for your platform.

## NAND Firmware Flashing Preparation

### Firmware Download

1. [[Click here]](https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/) to download the latest `product_ReleaseDate.zip`.

        ![image-20251031-170821](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-170821.png)

2. After extraction, you will obtain a `product` folder, which serves as the image directory for subsequent flashing.

### Flashing Tool Download

#### XBurn Tool Download

[[Click here]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) to download the XBurn image flashing tool:

        ![image-20251031-194712](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/install_os/image-20251031-194712.png)
        
        - For Windows: download `XBurn-gui_VersionNumber_x64-setup.exe`.
        
        - For Ubuntu: download `XBurn-gui_VersionNumber_x64-setup.deb`.
        
        - For macOS: download `XBurn-gui_VersionNumber_x64-setup.dmg`.