---
sidebar_position: 2
---

# 1.2.1.2 Flashing Preparation


## Image Download

:::warning Note

The **RDK S100** comes pre-flashed with a test version of the system image. To ensure you are using the latest system version, <font color='Red'>we recommend following this guide to flash the latest system image</font>.

:::

The RDK S100 kit currently provides an Ubuntu 22.04 system image that supports Desktop graphical user interface interaction.

1. [[Click here]](../../download.md#system-images-and-tools) to download the image.
   
    ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/download_web.png)

2. After decompression, you will obtain a `product` folder with the following structure. Ensure that both the `img_packages` folder and the `xmodem_tools` file are located in the same directory:
    ![product folder interface](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)


## Flashing Tool Download

[[Click here]](https://archive.d-robotics.cc/downloads/software_tools/download_tools/) to access the flashing tool download page. The RDK S100 development kit uses the PC-based D-Navigation tool to flash the Ubuntu system. Select the appropriate tool and version based on your operating system:

Taking D-navigation-v2.4 as an example:
   
   - For Windows: D-navigation-win32-x64_v2.4.zip 
   - For Linux: D-navigation-linux-x64-v2.4.tar.gz 
   - For Mac: D-navigation-darwin-arm64_v2.4.zip