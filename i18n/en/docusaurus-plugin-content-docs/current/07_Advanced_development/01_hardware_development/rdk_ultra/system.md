---
sidebar_position: 3
---

# System Flashing

:::tip
For a complete guide on downloading and flashing the system image, please [**click here to view the full chapter**](../../../01_Quick_start/install_os.md).
:::

When flashing the Ubuntu system on the RDK Ultra development kit, you need to use the D-Robotics `hbupdate` flashing tool. The tool is available for both Windows and Linux versions, named `hbupdate_win64` and `hbupdate_linux`, respectively. The tool can be downloaded from the following link: [hbupdate](https://archive.d-robotics.cc/downloads/en/hbupdate/).

### Tool Usage Notes:
- Unzip the tool's archive, ensuring the extraction path does **not** contain any **spaces, Chinese characters, or special symbols**.
- The tool communicates with the RDK Ultra via Ethernet. To ensure fast flashing, **make sure the PC has a gigabit Ethernet port and is directly connected** to the development kit.
- Set the PC's network configuration to **static IP mode**, as follows:
  - IP: 192.168.1.195
  - Netmask: 255.255.255.0
  - Gateway: 192.168.1.1

## Flashing Procedure {#flash_system}

1) Connect the RDK Ultra to your PC using an Ethernet cable and ensure that the network is reachable by pinging the device.

2) Short the `FC_REC` and `GND` pins of the Functional Control Interface (Interface 10).

3) Run the `hbupdate` main program, open the flashing tool, and select the development board model as `RDK_ULTRA` (this is a required option).

![image-flash-system1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system1.jpg)

4) Click the `Browse` button to select the image file you want to flash (this is a required option).

![image-flash-system2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system2.jpg)

5) Click the `Start` button to begin flashing. Confirm the operation based on the prompts, and then click the `OK` button:

![image-flash-system3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-system-download3.jpg)

6) When the tool shows the following printout, it indicates that the flashing process has started. The duration depends on the network transfer speed, so please be patient.

![image-flash-system4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system4.jpg)

7) Wait for the tool to complete the flashing process and check the flashing result.

- If the flashing is successful, the tool will display the following message:

![image-flash-system6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system6.png)

- If the following error message appears, please double-check steps 1-3 to ensure the procedure was followed correctly:

![image-flash-system7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system7.png)

- If the following error message appears, it indicates that the network transfer speed is too slow. We recommend switching to a more powerful PC and retrying the upgrade:

![image-flash-system8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/01_hardware_development/rdk_ultra/image/rdk_ultra/image-rdk-ultra-system8.jpg)
