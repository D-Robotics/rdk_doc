---
sidebar_position: 1
---

# System OTA Upgrade

## Overview

**OTA** (Over-the-Air Technology) refers to technology that enables remote software upgrades through wireless networks. First introduced by the Android system to mobile devices, OTA technology has significantly simplified the traditional software upgrade process. Without requiring computer connections, users can directly download and install updates on their devices. This technology greatly facilitates users and improves device maintenance efficiency.

![ota_intro](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_intro.png)

- In the broad application of OTA, it can be divided into two main components: the cloud side and the device side. The cloud side is responsible for handling device upgrade requests, including performing upgrade verification, delivering upgrade packages, and collecting upgrade results. The device side primarily relies on upgrade packages delivered from the cloud to complete updates and upgrades of system software (FOTA, Firmware Over-the-Air) or applications (SOTA, Software Over-the-Air).
- This document aims to provide a user manual for OTA on the underlying device side, elaborating on the mechanisms and implementation methods of OTA in upgrading underlying system software and applications, while also providing relevant development guidance. It should be particularly noted that the system software and applications upgraded via OTA mainly refer to updating data stored in external storage (such as eMMC).
- The external deliverables of OTA mainly consist of a set of APIs and their corresponding implementation libraries (such as libupdate.so), which implement key functions such as underlying writing and verification. The upper-layer OTA service architecture is implemented by the customer to interface with the customer's cloud services. After successfully downloading the upgrade package from the cloud, the OTA service calls interfaces in libupdate.so to perform version upgrades and verification operations, thereby ensuring that the device can complete software updates smoothly and securely.

**Abbreviations**

| Abbreviation | English Full Name                | Chinese Explanation       |
|--------------|----------------------------------|---------------------------|
| SoC          | System on Chip                   | System on Chip            |
| BL[x]        | Boot Loader Stage [x]            | Boot Stage x              |
| SPL          | Secondary Program Loader         | Secondary Program Loader  |
| GPT          | GUID Partition Table             | GUID Partition Table      |
| GUID         | Globally Unique IDentifier       | Globally Unique Identifier |
| RSA          | RSA Algorithm                    | RSA Public Key Cryptography |
| eMMC         | embedded MultiMedia Card         | Embedded Non-Volatile Memory |

## System Partition Table

During OTA, target partitions are updated on a per-partition basis. These partitions are distinguished according to different types as follows:

| Partition Type | Attributes | Upgrade Method | Example |
|----------------|------------|----------------|---------|
| Persistent Partition | Parameter partition: Generally stores configuration files and parameters that need to be loaded during system runtime, such as ubootenv in the partition table.<br /><br /> User partition: Refers to partitions unrelated to system startup. Generally mounted only after system startup, such as the userdata partition. | Single partitions typically have no image, partition data needs long-term preservation, and OTA upgrade is not supported. | ubootenv, veeprom, userdata |
| AB Partitions | Partitions with the same prefix and suffixes _a and _b are called AB partitions. | AB partitions upgrade alternately. | boot_a, boot_b |
| BAK Partitions | Partitions with the same prefix and the suffix `bak` are called BAK partitions, mainly consisting of one primary partition and several backup partitions. | BAK partitions only upgrade the primary partition. After the primary partition is upgraded and verified successfully, its content is synchronized to the backup partition. | SBL, SBL_bak |

## Enabling OTA
OTA functionality is not enabled by default on RDK. To enable it, please follow the process below:
1. Before enabling OTA, if the project has not been compiled and the out directory does not exist, execute the following commands. The project must be compiled or the build environment must be established.
        ```bash
        # Compile the project and generate images
        sudo ./pack_image.sh

        # Only establish the build environment
        sudo ./pack_image.sh -p
        ```
2. In the `ubuntu-22.04_desktop_rdk-s100_beta.conf` and `ubuntu-22.04_desktop_rdk-s100_release.conf` files under the build_params directory, configure PARTITION_FILE for the OTA version: `export PARTITION_FILE="s100-ota-gpt.json"`, and configure RDK_DM_VERIFY_ENABLE to enable: `export RDK_DM_VERIFY_ENABLE="yes"`;

3. In the `board_s100_debug.mk` and `board_s100_release.mk` files under the source/bootloader/device/rdk/s100 directory, configure the RDK_OTA variable to enable: `export RDK_OTA="yes"`;

4. Compilation
   - Create a new miniboot deb package
        ```bash
        ./mk_debs.sh hobot-miniboot
        ```
   - Recompile the local image
        ```bash
        sudo ./pack_image.sh -l
        ```

## Root File System Description (OTA Mode)
### Overview

After enabling OTA, the system adopts a root file system structure of system + overlayfs.
This solution separates the system's read-only part from the user-writable part, allowing users to perform write operations in the root file system while ensuring the system partition is mounted in read-only mode. This guarantees system stability and security while supporting differential updates for the system partition during OTA upgrades.

### Brief Description of OverlayFS

OverlayFS is a union filesystem that merges multiple directories into a unified view.
In overlayfs:

- Lowerdir: The read-only layer, providing the base file content of the system.
- Upperdir: The writable layer, saving user modifications, additions, and deletions of files.
- Merged: The mount point view providing unified access for users.

When files are modified or deleted, operations only take effect in the Upperdir, while the Lowerdir remains unchanged. The system prioritizes reading files from the Upperdir when accessing, thereby achieving incremental overlay and read-only protection.

### Root File System Structure Solution

This system uses the following partition structure to implement overlayfs:

| Partition Type        | Mount Role | Permissions | Description                                           |
| --------------------- | ---------- | ----------- | ----------------------------------------------------- |
| system_A / system_B   | Lowerdir   | Read-Only   | Partition for system base files, dual AB partition structure supports seamless OTA upgrades |
| overlay               | Upperdir   | Read-Write  | User data and modification save area, does not participate in OTA upgrades |
| root (`/`)            | Merged     | Merged View | Provides a unified root directory access view         |

The mounting logic for the root file system is as follows:

- system_A / system_B: System content is updated through OTA upgrades.
- overlay partition: As the upper directory of overlayfs, it saves runtime and user modifications.
- Root directory /: The merged view of the file system seen by the user.

### Precautions

1. Initial Flashing Behavior

    During whole-device flashing, the overlay partition will be formatted, and read-only system images will be written to the system_A / system_B partitions. Afterward, the system will automatically establish the overlayfs merge layer upon startup.

2. OTA Upgrade Behavior

    During an OTA upgrade, only the content of the system partition is updated; the overlay partition remains unchanged. User configurations, application installations, or modified files will be preserved.

3. File Override Priority

    If a user manually modifies a file like /etc/xxx, this modification will be written to the overlay partition. Even if an OTA upgrade updates the same file in the system partition, the system will still prioritize displaying the user-modified version after the upgrade (overlay has higher priority).

## OTA Packaging Tool

**This document uses OTA packages in ZIP format as examples for explanation. Unless otherwise specified, the usage method for TAR (images compressed with Zstandard) format is identical to the ZIP format, simply replace the .zip suffix in the document with .zst.tar.**

Currently, OTA packages support zip format and tar format with images compressed by zstd, with suffixes .zip and .zst.tar respectively. .zst.tar compresses the img into img.zst, offering higher compression ratios and faster decompression speeds, while utilizing the tar format to support indexing.

### Introduction to the ota_tools Directory

The OTA packaging tool is located in the `ota_tools/` path. This directory contains the core tools and scripts required for building OTA packages. The contents of this folder are as follows:
```bash
tree
.
├── hdiffz                 # Binary diff tool for generating OTA differential images
├── hooks                  # Hook script directory for pre/post OTA partition upgrade
│   ├── README.md          # Hook script usage instructions and naming conventions
│   ├── postinst_MCU.sh    # Post-processing script after MCU partition upgrade completion
│   └── preinst_MCU.sh     # Pre-processing script before MCU partition upgrade
├── hpatchz                # Binary patch tool for applying OTA differential images
├── mk_otapackage.py       # Main script for building OTA upgrade packages
├── ota_pack_tool.sh       # Shell wrapper script for the OTA upgrade package build process
├── ota_process            # Core OTA upgrade processing logic directory
├── ota_sign.py            # OTA upgrade package signing script
├── parse_env.py           # Build environment and configuration parsing script
├── part_ota_cmd.json      # Partition OTA upgrade command and configuration description file
├── private_key.pem        # Private key used for signing OTA upgrade packages (please keep secure)
└── zstd.py                # Zstandard compression/decompression helper module
```
The hooks directory is used to provide automatically executed scripts before and after partition writing, which can be used to insert custom processing logic during the upgrade process. Before using the packaging tool, it is recommended to understand the purpose and usage of the hooks directory.

#### OTA Upgrade Hook Mechanism
- Hook Script Placement

    The hooks directory stores pre-install / post-install processing scripts for the OTA partition upgrade process, used to execute some custom logic before or after partition writing, including but not limited to:

    - Environment checks before upgrade
    - Data backup and migration

    **If a specific partition does not require any pre or post-processing logic, the corresponding script file can be omitted.**

- Script Naming Rules (Must Follow)

    To ensure scripts are correctly packaged and executed during the OTA upgrade process, script file names must strictly follow the naming rules below. Otherwise, they will not be recognized or included in the upgrade package.

    - Pre-install Hook Script (runs before partition writing)

        ```text
        preinst_<partition_name>.sh
        ```

    - Post-install Hook Script (runs after partition writing)

        ```bash
        postinst_<partition_name>.sh
        ```

    Script files that do not conform to the above naming rules will not be included in the OTA upgrade package and will not be executed during the upgrade process.

    The `<partition_name>` in the script file name must be **exactly the same** as the partition name used in the OTA system. If unsure about the partition name, you can confirm it via the `base_name` field in the `out/product/img_packages/ota_tools/ota_info.json` file.

- Usage Example

    Taking the MCU partition as an example

    - Open the `out/product/img_packages/ota_tools/ota_info.json` file and check the `base_name` field of the `MCU` section to confirm the partition name.

        **The following is just an example, not linked to actual code. Please refer to the source code for the actual partition table.**
        ```json
            "MCU_a_S600_V1.0": {
                "base_name": "MCU",
                "out": "/home/zxs/s600/source/bootloader/out/target/product/img_packages/MCU_S600_V1.0.img",
                "medium": "nor",
                "nose_support": null,
                "secure": true,
                "part_type": "AB",
                "multi_img": true,
                "life_img": false,
                "multi_key": false,
                "have_anti_ver": null,
                "size": 6291456,
                "ota_update_mode": "image"
            }
        ```

    - Create the following scripts in the hooks directory:

        ```text
        hooks/
        ├── preinst_MCU.sh     # Executes before partition writing
        ├── postinst_MCU.sh    # Executes after partition writing
        ```

    If the partition does not require any pre or post-processing logic, no script files need to be created.

- Precautions

    - Hook scripts are **optional**. They will be skipped automatically if they do not exist.
    - Only script files located in the hooks directory with correctly named specifications will be recognized and processed.
    - Please ensure the script content has correct execution logic and necessary error handling.
    - Depending on the system configuration, script execution failure may cause the OTA upgrade process to be interrupted. Please script the logic carefully.

#### OTA Packaging Tool Usage Method

Generally, use the `ota_pack_tool.sh` script located in the `ota_tools/` path to create the required OTA upgrade packages. It supports unpacking OTA upgrade packages, repackaging after unpacking, creating upgrade packages, and creating differential packages.

Usage method is as follows:
```bash
Usage: ./ota_pack_tool.sh [OPTIONS]...
Options:
  -x, -unpack <ota_package>        Unpack the given OTA package file. (compress inferred from suffix)
  -r, -repack [-t <tar|zip>]       Repack the files into a new OTA package. (default: tar)
  -c, -create <pack_type> -d <source_dir> [-t <tar|zip>]
       Generate OTA package by source dir or img_packages. (default: tar)
       <pack_type>: sys, sys_signed, miniboot_signed ... etc.
       <source_dir>: sys or sys_signed require img dir.
       (e.g.)run "./ota_pack_tool.sh -c sys -d ../out/product/img_packages/ -t tar"
  -i, -inc <pack_type> -old <old_pkg> -new <new_pkg>
       Create an incremental OTA package. (compress inferred from suffix of -old)
       <pack_type>: sys, sys_signed.
       <old_pkg>, <new_pkg>: ota pack, xxx.zip or xxx.zst.tar.
       (e.g.)run "./ota_pack_tool.sh -i sys_signed -old all_in_one_old.zip -new all_in_one_new.zip".
  -h, -help                                 Display this help message.
```

#### Creating a Standard OTA Upgrade Package

A system upgrade package can be created using the following command:

```BASH
# tar format upgrade package (-t parameter omitted), sys_signed means packaging a secure version upgrade package
 ./ota_pack_tool.sh -c sys_signed -d ~/s600/out/product/img_packages/

# zip format upgrade package, sys means packaging a non-secure version upgrade package
./ota_pack_tool.sh -c sys -d ~/s600/out/product/img_packages -t zip
```
The generated OTA upgrade package will be output to the `out/product/ota_packages` directory. In this directory, you will see upgrade packages ending with `zip` or `.zst.tar` and their signature files ending with `signature`:
```BASH
all_in_one_signed.signature     #secure upgrade package signature file
all_in_one_signed.zst.tar       #secure upgrade package file

all_in_one.signature            #non-secure upgrade package signature file
all_in_one.zip                  #non-secure upgrade package file
```

#### Creating a Differential OTA Upgrade Package
The `ota_pack_tool` can be used to create an `all_in_one_signed_inc.zip` system differential package. The OTA configuration file and ota_process used during creation are obtained from unpacking the new package.

1. Purpose of Differential Upgrade

    Saves network traffic, but does not save upgrade time.

2. Principle of Differential Upgrade

    Uses a differential algorithm to generate a differential image between the old and new images. During upgrade, the differential image is inversely restored to the target partition. (Images on flash media are not differentiated; their size is only a few MB, and flash reading is slow). Differential library: hpatchz. Supported images: All images in the OTA package that are not on flash media.

3. Limitations of Differential Upgrade
   - Partitions on flash do not support differential upgrades.
   - The boot partition has write behavior and does not support differential upgrades.
   - Only supported when the image package size is greater than 10M.
   - If a partition requiring differential upgrade needs to be mounted, it must be mounted in read-only mode with the noload option added; otherwise, MD5 verification may fail.
4. Creating a Differential Upgrade Package

    - Differential image upgrade relies on the already flashed old image package. Therefore, when planning to use differential upgrade, **be sure to properly save the old image package to avoid loss or damage.**
   ```bash
    # Create differential package all_in_one_signed_inc.zip based on all_in_one_signed_old.zip and all_in_one_signed_new.zip
    ./ota_pack_tool.sh -i sys_signed -old all_in_one_signed_old.zip -new all_in_one_signed_new.zip
   ```

#### Unpacking and Repackaging OTA Upgrade Packages
Command to unpack an upgrade package:
```bash
./ota_pack_tool.sh -x all_in_one.zip
```
- After unpacking the upgrade package, images under `out/deploy/ota_deploy/unpack` can be updated. To repackage the OTA package, the OTA configuration file and ota_process used are located in the `out/deploy/ota_deploy/unpack` directory. This method cannot modify the gpt.conf OTA configuration file.

Command to repackage an upgrade package:
```BASH
./ota_pack_tool.sh -r -t zip
```
- Source folder path for packaging: out/deploy/ota_deploy/unpack
- Target file path after packaging: out/product/ota_packages

### Signature Keys

The private key `private_key.pem` used for signing is placed in the project's `ota_tools` directory. The public key `public_key.pem` used for signing is placed in the project's `source/bootloader/miniboot/ota_flash_tools/` directory. On the device side, the path for this public key is /usr/hobot/share/ota/public_key.pem.

If you need to replace them with your own keys, follow these steps:

- Generate private key:

  ```bash
  openssl genrsa -out private_key.pem 4096
  ```

- Generate public key:

  ```bash
  openssl rsa -RSAPublicKey_out -in private_key.pem  -out public_key.pem
  ```

- Replace `public_key.pem` in the path `source/bootloader/miniboot/ota_flash_tools/` and replace `private_key.pem` in the path `ota_tools/`.
- Execute the following commands to recompile

    ```bash
    # Go back to the project's top-level directory, create the miniboot deb package
    ./mk_debs.sh hobot-miniboot

    # Compile the local project
    sudo ./pack_image.sh -l

**Note:**
- The generated OTA package is named `all_in_one_xxx` by default. The upgrade program verifies the package name; the package name **must** contain the keyword "all_in_one". Additionally, the package name must not contain the following keywords: "app", "APP", "middleware", "param".

### Introduction to OTA Upgrade Package

#### Upgrade Package Structure

```bash
Archive:  all_in_one_signed.zip
  Length      Date    Time    Name
---------  ---------- -----   ----
     1047  2025-05-14 12:11   gpt.conf
     5326  2025-05-14 12:11   data.json
   526336  2025-05-13 20:37   HSM_FW_signed.img
   264192  2025-05-13 20:37   HSM_RCA_signed.img
   264192  2025-05-13 20:37   keyimage_signed.img
   264192  2025-05-13 20:37   keyimage_ohp_signed.img
   526336  2025-05-13 20:37   scp_signed.img
   526336  2025-05-13 20:37   spl_signed.img
  1312768  2025-05-13 20:37   MCU_S100_V1.0_signed.img
   256288  2025-05-13 20:37   acore_cfg.img
   348928  2025-05-13 20:37   bl31.img
   986080  2025-05-13 20:37   optee.img
  1076928  2025-05-13 20:37   uboot.img
 36388864  2025-05-13 20:37   boot.img
8589934592  2025-05-13 20:37   system.img
   310144  2025-05-13 19:47   ota_process
---------                     -------
8632992549                     16 files
```

The above shows the file structure within a current OTA upgrade package, mainly including the following four types of files. The number of image files may vary based on the actual configuration.

| File         | Description          |
|--------------|----------------------|
| gpt.conf     | Partition table file |
| data.json    | OTA configuration file |
| *.img        | Images for each partition |
| ota_process  | OTA flashing program |

#### OTA Configuration File

The OTA upgrade package contains a configuration file named data.json. This file is generated during compilation and contains partition and image information for the upgrade package.

General Configuration

| Configuration           | Type      | Function                      |
|-------------------------|-----------|-------------------------------|
| backup_dir              | arr[obj]  | HSM backup directory          |
| ab_sync                 | str       | reserved field, default fixed false |
| nor_sign                | bool      | NOR Flash image signature verification switch |
| update_partition        | arr[str]  | Partitions to upgrade         |
| partition_info          | arr[str]  | Configuration for each partition |


Per-Partition Configuration (partition_info)

| Configuration | Type     | Function                                           |
|---------------|----------|----------------------------------------------------|
| md5sum        | arr[obj] | MD5 values for each image                          |
| md5_scope     | arr[obj] | MD5 verification length for each image             |
| medium        | str      | External storage medium (NOR/eMMC/NAND)            |
| part_type     | str      | Partition type (AB/BAK/GOLDEN)                     |
| upgrade_method| str      | Upgrade method (image)                             |
| imgname       | str      | Image name, only supports files suffixed with `.img/.bin/.ubifs` |

Below is an example of a data.json file:

```JSON
{
    "antirollbackUpdate_host": false,
    "antirollbackUpdate_hsm": false,
    "backup_dir": "/tmp/ota/backup",
    "ab_sync": false,
    "update_partition": [
        "HSM_FW",
        "HSM_RCA",
        "keyimage",
        "scp",
        "spl",
        "MCU",
        "acore_cfg",
        "bl31",
        "optee",
        "uboot",
        "boot",
        "system"
    ],
    "nor_sign": true,
    "partition_info": {
        "HSM_FW": {
            "md5sum": {
                "HSM_FW_signed.img": "3e19e04f97e0cb4e37899958e3aec34f"
            },
            "md5_scope": {
                "HSM_FW_signed.img": 524288
            },
            "medium": "nor",
            "part_type": "BAK",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "HSM_FW_signed.img"
        },
        "HSM_RCA": {
            "md5sum": {
                "HSM_RCA_signed.img": "9b2f9cb4d00586dd49112f50fdb90952"
            },
            "md5_scope": {
                "HSM_RCA_signed.img": 262144
            },
            "medium": "nor",
            "part_type": "BAK",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "HSM_RCA_signed.img"
        },
        "keyimage": {
            "md5sum": {
                "keyimage_signed.img": "df28a9900fa504a90a4c85368cb7879d",
                "keyimage_ohp_signed.img": "af81665e36a9a274084e2dc02b7a3830"
            },
            "md5_scope": {
                "keyimage_signed.img": 262144,
                "keyimage_ohp_signed.img": 262144
            },
            "medium": "nor",
            "part_type": "BAK",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "keyimage%s_signed.img"
        },
        "scp": {
            "md5sum": {
                "scp_signed.img": "6ea63e573ae3bb50dc8cb0052c29fddb"
            },
            "md5_scope": {
                "scp_signed.img": 524288
            },
            "medium": "nor",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "scp_signed.img"
        },
        "spl": {
            "md5sum": {
                "spl_signed.img": "ba48f24f989de1ddbc6eb3aedd139b4f"
            },
            "md5_scope": {
                "spl_signed.img": 524288
            },
            "medium": "nor",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "spl_signed.img"
        },
        "MCU": {
            "md5sum": {
                "MCU_S100_V1.0_signed.img": "d44784323de5f5d57fa606ea178fd854"
            },
            "md5_scope": {
                "MCU_S100_V1.0_signed.img": 1310720
            },
            "medium": "nor",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "MCU_S100_V1.0_signed.img"
        },
        "acore_cfg": {
            "md5sum": {
                "acore_cfg.img": "fe36a8ad522a2ca5ee811da8208828ef"
            },
            "md5_scope": {
                "acore_cfg.img": 256288
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "acore_cfg.img"
        },
        "bl31": {
            "md5sum": {
                "bl31.img": "5be617cd08128e89318f61964696509a"
            },
            "md5_scope": {
                "bl31.img": 348928
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "bl31.img"
        },
        "optee": {
            "md5sum": {
                "optee.img": "f0f663462f523f9e8722e0c26d29209e"
            },
            "md5_scope": {
                "optee.img": 986080
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "optee.img"
        },
        "uboot": {
            "md5sum": {
                "uboot.img": "78b07c6a4264bd757c4d992e2dc0ee0b"
            },
            "md5_scope": {
                "uboot.img": 1076928
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "uboot.img"
        },
        "boot": {
            "md5sum": {
                "boot.img": "aca98db8136ad4f1d55cd3e7bdb4c856"
            },
            "md5_scope": {
                "boot.img": 36388864
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "boot.img"
        },
        "system": {
            "md5sum": {
                "system.img": "a6f61c86ae0045ad3167b3daf7c33b3a"
            },
            "md5_scope": {
                "system.img": 8589934592
            },
            "medium": "emmc",
            "part_type": "AB",
            "have_anti_ver": null,
            "upgrade_method": "image",
            "imgname": "system.img"
        }
    }
}
```

With the above configuration, the OTA upgrade package can ensure that the image for each partition is correctly verified and updated during the upgrade process.

## Detailed OTA Implementation
### OTA Process
The following process takes the implementation in ota_tool as an example (developers can refer to this tool for their own implementation).

- Preparation Phase:

    ![ota_tool_sequence_step1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_tool_sequence_step1-en.jpg)

- Upgrade Phase:

    ![ota_tool_sequence_step2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_tool_sequence_step2-en.jpg)

- Verification Phase:

    ![ota_tool_sequence_step3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_tool_sequence_step3-en.jpg)   

### OTA State Machine

#### Upgrade Package State
The state is stored in the OTA process space. The process description is as follows:
```c
/**
 * @enum otahl_update_result
 * @brief ota upgrade result
 * @NO{S21E03C02}
 */
typedef enum otahl_update_result {
    OTA_UPGRADE_NOT_START = 0, /**< OTA upgrade not start */
    OTA_UPGRADE_IN_PROGRESS, /**< OTA upgrading */
    OTA_UPGRADE_SUCCESS, /**< OTA upgrade success*/
    OTA_UPGRADE_FAILED, /**< OTA upgrade failed*/
} otahl_update_result_e;
```

![ota_package_state](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_package_state-en.jpg)

#### OTA Upgrade Process State
The state is stored in veeprom. The process description is as follows:
```c
typedef enum ota_update_flag {
    OTA_FLAG_NORMAL = 0,     // normal state
    OTA_FLAG_BURN,           // burning state
    OTA_FLAG_VERIFY,         // pending verification state
    OTA_FLAG_VERIFIED,       // verified state
} ota_update_flag_e;
```

![ota_upgrade_state](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_upgrade_state-en.jpg)

#### misc (AB State Machine)
1. Area Allocation

    Dijia uses the Android AB mechanism to implement the AB system. The following information introduces some principles of this mechanism. For detailed principles, please refer to: https://source.android.google.cn/docs/core/ota

    bootloader_message_ab occupies a total of 4K: The bootloader_control used by AB is located at struct bootloader_message_ab->slot_suffix, occupying 32 bytes.
    ```c
    struct bootloader_message_ab {
        struct bootloader_message message;
        char              slot_suffix[32];
        char              update_channel[128];

        // Round up the entire struct to 4096-byte.
        char reserved[1888];
    };

    /**
    * Be cautious about the struct size change, in case we put anything post
    * bootloader_message_ab struct (b/29159185).
    */
    #if (__STDC_VERSION__ >= 201112L) || defined(__cplusplus)
    static_assert(sizeof(struct bootloader_message_ab) == 4096,
            "struct bootloader_message_ab size changes");
    #endif
    ```

2. AB Structure Data
    ```c
    #define ARRAY_32    (32U)

    struct slot_metadata {
        // Slot priority with 15 meaning highest priority, 1 lowest
        // priority and 0 the slot is unbootable.
        uint8_t priority : 4;
        // Number of times left attempting to boot this slot.
        uint8_t tries_remaining : 3;
        // 1 if this slot has booted successfully, 0 otherwise.
        uint8_t successful_boot : 1;
        // 1 if this slot is corrupted from a dm-verity corruption, 0 otherwise.
        uint8_t verity_corrupted : 1;
        // Reserved for further use.
        uint8_t reserved : 7;
    } __attribute__((packed));

    struct bootloader_control {
        // NUL terminated active slot suffix.
        char slot_suffix[4];
        // Bootloader Control AB magic number (see BOOT_CTRL_MAGIC).
        uint32_t magic;
        // Version of struct being used (see BOOT_CTRL_VERSION).
        uint8_t version;
        // Number of slots being managed.
        uint8_t nb_slot : 3;
        // Number of times left attempting to boot recovery.
        uint8_t recovery_tries_remaining : 3;
        // Ensure 4-bytes alignment for slot_info field.
        uint8_t reserved0[2];
        // Per-slot information.  Up to 4 slots.
        struct slot_metadata slot_info[4];
        // Reserved for further use.
        uint8_t reserved1[8];
        // CRC32 of all 28 bytes preceding this field (little endian
        // format).
        uint32_t crc32_le;
    } __attribute__((packed));
    ```

3. State Machine Description

    ![ab_state](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ab_state.png)
- State 1: Default state, both AB slots are bootable. b has higher priority than a, boots from b by default.

- State 2: Upgrade state (burning state), a slot is unbootable and boot success is 0.

- State 3: Burning successful, not yet rebooted. At this time, the slot to be upgraded is set to active state, and the current slot is set to non-active state (priority adjustment).

- State 4: Reboot verification successful, set a to successful boot state.

#### veeprom Area

The state machine for OTA updates is stored in this area. Below is the application of OTA in this region:

```c
#define VEEPROM_OTA_STAT_OFFSET \
    (1024) /**< offset of OTA status in the veeprom partition */
#define VEEPROM_OTA_STAT_SIZE \
    (2048) /**< size of OTA status in the veeprom partition */

#define VEEPROM_RECOVERY_STAT_OFFSET \
    (VEEPROM_OTA_STAT_OFFSET +   \
     VEEPROM_OTA_STAT_SIZE) /**< offset of recovery information in the veeprom partition */
#define VEEPROM_RECOVERY_STAT_SIZE \
    (128) /**< size of recovery information in the veeprom partition */
```

1.  **OTA Area**

    Start Offset: 1024

    Size: 2048

    Function: OTA status storage

    Stored data structure:

    ```c
    /**
    * @ota_status_t
    * @brief ota status
    * @NO{S21E03C04U}
    */
    typedef struct ota_status_s {
        uint32_t           magic;               /**< magic number */
        ota_update_flag_e  up;
        ota_update_flag_e  up_system;           /**< system partition update flag */
        ota_update_flag_e  up_backup;           /**< backup partition update flag */
        ota_update_flag_e  up_app;              /**< application partition update flag */
        ota_update_flag_e  up_middleware;       /**< middleware partition update flag */
        ota_update_flag_e  up_param;            /**< parameter partition update flag */
        ota_update_owner_e owner;
        uint32_t           next_slot;           /**< expected slot for next boot */
        update_part_t      update_part;
        uint8_t            reserved[ARRAY_32];
        uint32_t           crc32_le;            /**< crc verify value */
    } ota_status_t;
    ```

### Boot State Switching
The diagram below illustrates how A/B slots are switched during normal boot (this process applies regardless of whether an OTA update is performed):
    ![boot_slot_select](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/boot_slot_select-en.jpg)

- During boot, the ROM boot count automatically increments (this flag is located in the AON domain and resets upon power-off).

- During an OTA update, the priority of the slot to be updated is set to 15, while the other slot is set to 14. The retry count for the slot to be upgraded is set to 1, and if the slot is corrupted, it is set to 0.

- If the boot verification of the OTA update is successful, the slot is marked as `successboot`. If the boot fails, the slot is marked as corrupted.

- When booting from a slot that has never been marked as `successboot`, one retry attempt will be consumed. If the retry count reaches 0 or the slot is marked as corrupted, this slot will be skipped.

### Reboot Verification and Rollback
In the S100 reference implementation, after the OTA update completes and the system reboots, the kernel triggers the systemd OTA service to perform a reboot check, thereby completing the full OTA process (essentially executing `ota_tool -b`).
    ![ota_boot_check_state](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/ota_boot_check_state-en.jpg)

### Partition Flashing Method
OTA performs upgrades on a per-partition basis. Each partition to be upgraded has its own image. During the upgrade process, the primary task is to write this image to the corresponding partition in the external storage. Image types are further divided into two forms: full images and delta (differential) images.

#### Full Image Upgrade
  - A full image refers to the complete image of the target partition provided. The upgrade process involves writing this image directly to the corresponding partition in the external storage.
#### Delta Image Upgrade
  - A delta image is generated by applying a delta algorithm to compute the differences between the original image and the target image. Differential analysis typically extracts the differences between the target and original images, streamlining redundant information. The size of the resulting delta image is generally much smaller than the target image (the exact size depends on the degree of difference between the original and target – the smaller the difference, the smaller the image). This method is useful for saving bandwidth/data traffic.

  - During a delta upgrade, S100 OTA uses the delta image and the original partition data on the device to reconstruct the target image through a reverse delta process. It then writes this reconstructed image to the corresponding external storage partition, completing the final upgrade.

  - The S100 platform utilizes the open-source delta algorithm tool `hdiffz/hpatch`. For more information, please refer to: [github | HDiffPatch](https://github.com/sisong/HDiffPatch).


### OTA Security Protection Measures

#### Partition Table Verification
    OTA supports verification of the fpt/GPT partition files within the image. This is done by comparing them with the fpt/GPT partitions in the current system to verify if the fpt/GPT partition table has been modified. If the fpt/GPT partition table is found to be adjusted, the upgrade process is halted.
    - The OTA upgrade package includes the partition file `gpt.conf` generated by the build system. Its specific content format is as follows:
        ```bash
        fpt:0:262143:0
        recovery:262144:6291455:0
        misc:6291456:6553599:0
        HB_APDP:6553600:6815743:0
        keystorage:6815744:7340031:0
        HSM_FW:7340032:7864319:0
        HSM_FW_bak:7864320:8388607:0
        HSM_RCA:8388608:8650751:0
        ···
        ```
    - As the S100 partition scheme supports automatic expansion of the last partition, its end address can change dynamically. Therefore, the GPT verification only checks partitions up to the `userdata` partition. The last partition typically does not contain an image, so this limitation does not affect normal usage.
### Typical Upgrade Process
1.  The OTA Service downloads and verifies the upgrade package from the cloud, then calls `otaInitLib` to initialize the dynamic library.

2.  It calls `otaRequestStart` to initiate the upgrade. This API extracts the `ota_process` program from the upgrade package, forks a process, and executes the `ota_process` program to perform the actual flashing. Concurrently, this API also creates a file lock (to prevent multiple simultaneous upgrades) and creates a pipe file (for communication with `ota_process`). Additionally, it creates a thread to periodically read from the pipe, obtaining real-time upgrade progress, results, and information about the partition being updated.

3.  During the child process upgrade phase, the OTA Service can call `otaGetResult` to get the upgrade result, `otaGetProgress` to get the upgrade progress, and `otaGetUpdatingImageName` to get the name of the image currently being upgraded.

4.  When `otaGetResult` returns `OTA_UPGRADE_SUCCESS`, it indicates successful image flashing, and the verification phase begins.

5.  It calls `otaSetPartition` to set the next boot partition to the opposite partition (the newly updated one) and initiates the reboot process.

6.  After reboot, the OTA Service calls `otaGetOwnerFlag` to get the upgrade owner. If the owner is the OTA Service, the OTA Service is responsible for verifying this upgrade and proceeds to the verification process.

7.  It calls `otaCheckUpdate` to get the upgrade result. This API primarily checks whether the image was written completely and whether the system booted from the expected A/B slot or Backup slot.

8.  It calls `otaMarkOTASuccessful` to mark the current partition as booted successfully. This API operates on the A/B state machine, marking the current slot as `boot_successful`, ensuring subsequent boots use this slot. If any reboot occurs before this step, the next boot will use the slot containing the old image version, and the upgrade will be considered failed.

9.  It calls `otaPartitionSync` to synchronize the primary and backup (BAK) partitions.

10. It calls `otaClearFlags` to clear the upgrade flags and conclude the upgrade process.

    ![otaservice](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/otaservice-en.jpg)

## OTA Client-Side Tool Introduction

### ota_tool Usage

**This section uses a ZIP format OTA package as an example for illustration. Unless otherwise specified, the usage method for TAR format (images compressed with Zstandard) is exactly the same as the ZIP format; simply replace the `.zip` suffix in the documentation with `.zst.tar`.**

On the device side, OTA upgrades can be initiated manually using `ota_tool`. Detailed information can be obtained by entering the `ota_tool -h` command.

```bash
ota_tool Usage:
   -v, --version                      get this library's version.
   -b, --boot                         check ota update status when boot.
   -s, --setpartition [partition]     set A/B slot partition, 0--A; 1--B.
   -g, --getpartition                 get A/B slot partition, 0--A; 1--B.
   -p, --package [package_path]       specify the path of package, the package paths can be relative or absolute, it's length must be smaller than 64 bytes.
   -n, --noreboot                     request ota without reboot.
   -c, --checksign                    signature check.
   -i, --signature                    signature information file.
   -h, --help                         Display this help screen.
```

Before upgrading using `ota_tool`, you need to upload the OTA upgrade package to the device.

Parameter Description:

- `-h` Used to get help information.
- `-v` Used to get the `libupdate.so` version and the current system software version.
- `-b` Checks the upgrade result after booting (this check is performed automatically by the system on boot; user intervention is not required).
- `-s` Sets the A/B slot for the next boot, `0` indicates A, `1` indicates B.
- `-g` Gets the current A/B slot.
- `-p` Specifies the upgrade package.
- `-n` Does not automatically reboot after successful upgrade.
- `-c` Enables package integrity verification.
- `-i` Specifies the signature file (must follow the `-p` parameter).

Example:

```BASH
# Full upgrade, without package integrity verification
ota_tool -p all_in_one.zip

# Full upgrade, with package integrity verification
ota_tool -c -p all_in_one.zip -i all_in_one.signature

# Delta upgrade, without package integrity verification
ota_tool -p all_in_one_inc.zip

# Delta upgrade, with package integrity verification
ota_tool -c -p all_in_one_inc.zip -i all_in_one_inc.signature
```

### ota_tool Implementation
`ota_tool` is implemented in C language, with the source code contained in a single file, `otainterface.c`. It implements functions including: obtaining system software version, checking upgrade results, setting/getting AB slots, performing OTA upgrades, forcing upgrades, and verifying OTA package signatures.

If the `-c` parameter is passed, the provided signature file is used to verify the signature of the upgrade package.

Finally, it calls `ota_update_all_img` to start the upgrade.

#### ota_update_all_img
```c
static int32_t ota_update_all_img(const char *zip_path)
{
     int32_t progress = 0;

     uint8_t             slot = 0;
     uint8_t             next_slot = 0;
     int32_t             ret = 0;
     ota_update_result_e result = 0;
     char                part_name[ARRAY_32] = { 0 };

     ret = otaGetPartition(&slot);
     if (ret < 0) {
             return ret;
     }

     if (slot == 2) {
             next_slot = 0;
             printf("The slot [%d] to be burned\n", next_slot);
     } else {
             next_slot = 1 - slot;
             printf("The slot [%d] to be burned\n", next_slot);
     }

     ret = otaInitLib();
     if (ret < 0) {
             printf("error:init failed!\n");
             return ret;
     }

     ret = otaRequestStart(zip_path, OTA_TOOL);
     if (ret < 0) {
             printf("error: start ota update failed!\n");
             ret = -1;
             goto err;
     }
     while (otaGetResult() != OTA_UPGRADE_SUCCESS && otaGetResult() != OTA_UPGRADE_FAILED) {
             progress = otaGetProgress();
             result = otaGetResult();
             otaGetUpdatingImageName(part_name, sizeof(part_name));

             if (result == UPGRADE_FAILED) {
                     printf("error: ota update failed!\n");
                     ret = -1;
                     break;
             }
             OTA_show_Process_Bar(part_name, progress,
                                  "OTA is upgrading ...");
             usleep(100 * 1000);
     }

err:
     if (otaGetResult() == OTA_UPGRADE_SUCCESS) {
             ret = otaSetPartition(next_slot);
             if (ret < 0) {
                     printf("error: set partition failed!\n");
                     return ret;
             }
             if (g_is_reboot == true) {
                     printf("reboot system!\n");
                     ota_system_exe("reboot");
             } else {
                     printf("ota update success and waiting for reboot!\n");
             }
     }
     otaDeinitLib();

     return ret;
}
```

1.  Calls `otaGetPartition` to get the current active AB slot.

2.  Calls `otaInitLib` to initialize `libupdate.so`.

3.  Calls `otaRequestStart` with the upgrade package path and owner (`OTA_TOOL`) to begin the upgrade.

4.  Waits for the result from `otaGetResult` to become either `OTA_UPGRADE_SUCCESS` or `OTA_UPGRADE_FAILED`. While waiting, it calls `otaGetProgress`, `otaGetResult`, and `otaGetUpdatingImageName` to retrieve the upgrade progress, result, and the name of the image currently being updated. It then calls `OTA_show_Process_Bar` to print this information to the console.

5.  If the upgrade result (`otaGetResult`) is `OTA_UPGRADE_SUCCESS`, it considers the upgrade successful, calls `otaSetPartition` to set the AB slot to the opposite slot, and then reboots the SoC.

    ![otatool-ota_update_all_img](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/otatool-ota_update_all_img-en.jpg)

After the upgrade process finishes and the system reboots, `ota_tool -b` should be launched to check and verify the upgrade result, and to perform subsequent operations.

#### ota_boot_check
After the S100 boots, it starts a `hobot-otatool.service` service. This service invokes `ota_tool -b`. This option checks if the `/ota/ota_tool_force_upgrade` file exists. If it exists, it enters the upgrade process (the process re-invokes the upgrade command). If the file does not exist, it proceeds to the upgrade verification process `ota_boot_check`.
```c
int32_t ota_boot_check(void)
{
     int32_t               ret = 0;
     enum ota_update_owner owner = 0;

     if ((ret = otaInitLib()) != 0) {
             printf("error: init failed!\n");
             goto exit;
     }

     if ((ret = otaGetOwnerFlag(&owner)) != 0) {
             printf("error: Get owner flag failed!\n");
             goto exit;
     }

     if (owner == NORMAL_BOOT) {
             printf("Normal boot\n");
             if ((ret = otaMarkOTASuccessful()) != 0) {
                     printf("error: mark boot success failed\n");
             }
             return ret;
     }

     if (owner != OTA_TOOL) {
             printf("ota_tool is not owner, owner is [%d]\n", owner);
             return 0;
     }

     if ((ret = otaCheckUpdate()) != 0) {
             printf("error: boot check failed\n");
             goto exit;
     }

     if ((ret = otaMarkOTASuccessful()) != 0) {
             printf("error: mark boot success failed\n");
             goto exit;
     }

     if ((ret = otaPartitionSync()) != 0) {
             printf("error: partition sync failed\n");
             goto exit;
     }

     ret = otaDeinitLib();

exit:
     otaClearFlags();
     return ret;
}
```
1.  Calls `otaInitLib`.

2.  Calls `otaGetOwnerFlag` to get the upgrade owner.

3.  If the owner is `NORMAL_BOOT`, it calls `otaMarkOTASuccessful` to mark the boot as successful and then exits.

4.  If the owner is not `OTA_TOOL`, it exits normally.

5.  Calls `otaCheckUpdate` to get the upgrade result. If the upgrade result is abnormal, it calls `otaClearFlags` to clear the OTA flags and terminates the OTA process.

6.  Calls `otaMarkOTASuccessful` to mark the boot as successful.

7.  Calls `otaPartitionSync` to synchronize AB partitions and BAK partitions.

    ![otatool-ota_boot_check](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/image/ota/otatool-ota_boot_check-en.jpg)


## OTA API Introduction
The S100 platform provides a low-level flashing library, `libupdate.so`, which implements a set of cross-platform APIs for flashing OTA packages.

The `ota_tool` utility was developed by the underlying software team based on the OTA HighLevel API.

Header file: `hobot_ota_hl.h`

Link library: `libupdate.so`

### Dynamic Library Interface Error Code List
```c
enum ota_err_e {
    OTA_SUCCESS = 0,
    OTAERR_IO,
    OTAERR_PLAT_UNSUPPORT,
    OTAERR_REPEAT,
    OTAERR_MUTEX_INIT_LOCK_ERR,
    OTAERR_NOTINIT,
    OTAERR_NULLPOINTER,
    OTAERR_SHORTBUF,
    OTAERR_THREAD_CREATE,
    OTAERR_RANGE,
    OTAERR_STAGE,
    OTAERR_IMAGE_WRITE,
    OTAERR_BOOT_FAILED,
    OTAERR_VEEPROM,
    OTAERR_FILE_TYPE,
    OTAERR_UNZIP,
    OTAERR_NO_EXISTS,
    OTAERR_MALLOC,
    OTAERR_VERIFY,
    OTAERR_IMG_SIZE,
    OTAERR_UPDATE_STATUS,
};
```

### Interface List
| Interface Prototype | Description |
|----------|------|
| `int32_t otaInitLib(void);` | Dynamic library initialization |
| `int32_t otaDeinitLib(void);` | Dynamic library de-initialization |
| `int32_t otaGetLibVersion(char *version, int32_t len);` | Get dynamic library version |
| `int32_t otaRequestStart(const char *image_name, enum ota_update_owner owner);` | Start the upgrade thread |
| `int32_t otaGetResult(void);` | Get upgrade status and result |
| `int32_t otaGetProgress(void);` | Get upgrade progress |
| `int32_t otaGetUpdatingImageName(char *image_name, int32_t len);` | Get the name of the package currently being upgraded |
| `int32_t otaGetPartition(uint8_t *partition);` | Get the currently booted AB partition |
| `int32_t otaSetPartition(uint8_t partition);` | Set the AB partition for the next boot |
| `int32_t otaGetOwnerFlag(enum ota_update_owner *owner);` | Get the OTA owner (after reboot) |
| `int32_t otaMarkOTASuccessful();` | Mark OTA upgrade as successful (after reboot) |
| `int32_t otaCheckUpdate();` | Check if the upgrade was successful (after reboot) |
| `int32_t otaPartitionSync(void);` | Synchronize AB partitions and BAK partitions |
| `void otaClearFlags(void);` | End the upgrade and clear OTA flags |

### Interface: otaInitLib
| Interface Name | int32_t otaInitLib(void); |
|:---------|:-------------|
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0: Success;<br/> -OTAERR_REPEAT: Repeated initialization |
| Description | Initializes the dynamic library for device-side flashing interfaces, primarily initializing the global structure `g_upgrade_info`. |

**Example Code**
```c
#include <stdio.h>
#include <hobot_ota_hl.h>
int main(void) {
    int32_t ret;
    ret = otaInitLib();
    if (ret != 0) {
        printf("otaInitLib return: %d\n", ret);
        return ret;
    }
    ret = otaDeinitLib();
    if (ret != 0) {
        printf("otaDeinitLib return: %d\n", ret);
        return ret;
    }
    return 0;
}
```
### Interface: otaDeinitLib
| Interface Name | int32_t otaDeinitLib(void); |
|:---------|:--------------------------|
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0: Success;<br/>-OTAERR_NOTINIT: Currently not initialized, cannot call de-initialization |
| Description | De-initializes the dynamic library. |

**Example Code**

Refer to `otaInitLib`.

### Interface: otaGetLibVersion
| Interface Name | int32_t otaGetLibVersion(char *version, int32_t len); |
|:---------|:----------------------------------------------|
| Interface Type | C function interface |
| Input Parameters | `len`: Length of the provided buffer. |
| Output Parameters | `version`: Buffer to store the version information. The version is returned as a three-part string (e.g., "1.0.0"). |
| Return Value | 0: Success;<br/> -OTAERR_NULLPOINTER: `version` is a null pointer;<br/> -OTAERR_SHORTBUF: The provided buffer is too small. |
| Description | Retrieves the version of this dynamic library. |

**Example Code**
```c
#include <stdio.h>
#include <hobot_ota_hl.h>
int main(void) {
    int32_t ret;
    char version[128];
    if (ret = otaGetLibVersion(version, sizeof(version))) {
        return ret;
    }
    printf("version: %s\n", version);
}
```

### Interface: otaRequestStart
| Interface Name | int32_t otaRequestStart(const char *image_name, enum ota_update_owner owner); |
|:---------|:---------------------------------------------------------------------|
| Interface Type | C function interface |
| Input Parameters | `image_name`: Absolute path to the upgrade package. Supports passing multiple different types of packages simultaneously, separated by semicolons.<br/>`owner`: The process initiating this upgrade, defined by the `enum ota_update_owner`. |
| Output Parameters | N/A |
| Return Value | 0: Success;<br/>-OTAERR_NULLPOINTER: `image_name` is a null pointer;<br/>-OTAERR_RANGE: `owner` value is out of defined range;<br/>-OTAERR_NOTINIT: Dynamic library not initialized;<br/>-OTAERR_REPEAT: Another process is currently performing an upgrade;<br/>-OTAERR_IO: I/O failure;<br/>-OTAERR_THREAD_CREATE: Thread creation failed. |
| Description | Starts the upgrade process for the provided `image_name` and sets the current upgrade owner. |

**Example Code**
```c
#include <stdio.h>
#include <hobot_ota_hl.h>
int main(void) {
    int32_t ret;
    int32_t result;
    int32_t progress;
    char imgname[256];
    ret = otaInitLib();
    if (ret != 0) {
        printf("otaInitLib return: %d\n", ret);
        return ret;
    }
    if (ret = otaRequestStart("/ota/all_in_one-secure_signed.zip;/ota/middleware.zip;/ota/param.zip", OTA_TOOL)) {
        printf("otaRequestStart return: %d\n", ret);
        return ret;
    }
    do {
        result = otaGetResult();
        if (ret = otaGetUpdatingImageName(imgname, sizeof(imgname))) {
            printf("otaGetUpdatingImageName return: %d\n", imgname);
        }
        progress = otaGetProgress();
        printf("current image: %s, progress: %d\n", imgname, progress);
        sleep(1);
    } while(result != OTA_UPGRADE_SUCCESS && result != OTA_UPGRADE_FAILED);

    ret = otaGetPartition(&cur_part);
    if (ret != 0) {
        printf("otaGetPartition returned with %d\n", ret);
    }
    ret = otaSetPartition(1 - cur_part);
    if (ret != 0) {
        printf("otaSetPartition returned with %d\n", ret);
    }
    /* Then reboot system */
    return 0;
}
```
### Interface: otaGetResult
| Interface Name | int32_t otaGetResult(void); |
|:---------|:-------------------------------------------------------------------|
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | -OTAERR_NOTINIT: Dynamic library not initialized<br/>`OTA_UPGRADE_NOT_START`: Upgrade has not started;<br/>`OTA_UPGRADE_IN_PROGRESS`: Upgrade in progress;<br/>`OTA_UPGRADE_SUCCESS`: Upgrade finished, successful;<br/>`OTA_UPGRADE_FAILED`: Upgrade finished, failed. |
| Description | Gets the upgrade result. |

**Example Code**

Refer to `otaRequestStart`.

### Interface: otaGetProgress
| Interface Name | int32_t otaGetProgress(void); |
|:---------|:-------------|
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0 ~ 100: Current upgrade progress (%);<br/>-OTAERR_NOTINIT: Dynamic library not initialized. |
| Description | Gets the current upgrade progress. |


**Example Code**

Refer to `otaRequestStart`.

### Interface: otaGetUpdatingImageName
| Interface Name | int32_t otaGetUpdatingImageName(char *image_name, int32_t len); |
|:---------|:-------------|
| Interface Type | C function interface |
| Input Parameters | `len`: Length of the buffer. |
| Output Parameters | `image_name`: Buffer to store the result. Possible contents: `"idle_state"` – upgrade not started, `"all_img_finish"` – upgrade completed, `"app_param"` – application parameters, other package names ending with `".img"`. |
| Return Value | 0: Success<br/>-OTAERR_NULLPOINTER: `image_name` is a null pointer<br/>-OTAERR_NOTINIT: Dynamic library not initialized<br/>-OTAERR_SHORTBUF: Buffer length is too short. |
| Description | Gets the name of the image currently being upgraded. |


**Example Code**

Refer to `otaRequestStart`.

### Interface: otaGetPartition
| Interface Name | int32_t otaGetPartition(uint8_t *partition); |
| :------------ | :------------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | `partition`: Pointer to a variable receiving the current partition. <br/>`*partition=0`: Currently in A partition, `*partition=1`: Currently in B partition. |
| Return Value | 0: Success<br/>-OTAERR_NULLPOINTER: `partition` is a null pointer<br/>-OTAERR_IO: I/O error, please retry. |
| Description | Gets the currently booted AB partition. |

**Example Code**

Refer to `otaRequestStart`.

### Interface: otaSetPartition
| Interface Name | int32_t otaSetPartition(uint8_t partition); |
| :------------ | :------------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | `partition`: Partition for the next boot. `0`: A partition, `1`: B partition. |
| Output Parameters | N/A |
| Return Value | 0: Success<br/>-OTAERR_NOTINIT: Dynamic library not initialized<br/>-OTAERR_RANGE: `partition` value is out of range (must be 0 or 1)<br/>-OTAERR_IO: I/O error, please retry<br/>-OTAERR_STAGE: Operation not supported in the current upgrade stage (e.g., during upgrade or after failure). |
| Description | Sets the AB partition for the next boot. |


**Example Code**

Refer to `otaRequestStart`.

### Interface: otaGetOwnerFlag
| Interface Name | int32_t otaGetOwnerFlag(enum ota_update_owner *owner); |
| :------------ | :------------------------------------------ |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | `owner`: Pointer to a variable receiving the current upgrade owner. Refer to `enum ota_update_owner`. |
| Return Value | `OTA_SUCCESS`: Success<br/> -OTAERR_NOTINIT: Not initialized<br/> -OTAERR_NULLPOINTER: `owner` is a null pointer<br/> -OTAERR_IO: I/O error, please retry. |
| Description | Gets the current upgrade owner. |


**Example Code**

```c
static int check_and_mark(void) {
    enum ota_update_owner owner;
    int32_t ret;
    ret = otaGetOwnerFlag(&owner);
    if (ret != 0) {
            printf("otaGetOwnerFlag returned with %d\n", ret);
            return -1;
    }
    if (owner != OTA_TOOL) {
            printf("The OTA update is not launched by ota_tool, owner: %d\n", owner);
            return -1;
    }
    ret = otaCheckUpdate();
    if (ret == -OTAERR_IMAGE_WRITE) {
            printf("ota_tool: OTA image write failed, is there a reboot during update?\n");
            goto clearFlags;
    }
    if (ret == -OTAERR_BOOT_FAILED) {
            printf("ota_tool: The new package boot failed. Please check the packages\n");
            goto clearFlags;
    }
    if (ret != 0) {
            printf("otaCheckUpdate returned with %d\n", ret);
            goto clearFlags;
    }
    ret = otaMarkOTASuccessful();
    if (ret != 0) {
            printf("otaMarkOTASuccessful returned with %d\n", ret);
            goto clearFlags;
    }
    ret = otaPartitionSync();
    if (ret != 0) {
            printf("otaPartitionSync returned with %d\n", ret);
            goto clearFlags;
    }
clearFlags:
    otaClearFlags();
    return ret;
}
```

### Interface: otaMarkOTASuccessful
| Interface Name | int32_t otaMarkOTASuccessful(); |
| :------------ | :---------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0: Success<br/>-OTAERR_NOTINIT: Not initialized<br/>-OTAERR_IO: I/O error, please retry. |
| Description | Marks the current boot as successful for this upgrade. |


**Example Code**

Refer to `otaGetOwnerFlag`.

### Interface: otaCheckUpdate
| Interface Name | int32_t otaCheckUpdate(); |
| :------------ | :---------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0: Success<br/>-OTAERR_IO: I/O error, please retry<br/>-OTAERR_STAGE: No upgrade was performed<br/>-OTAERR_IMAGE_WRITE: Image write failed<br/>-OTAERR_BOOT_FAILED: New image boot failed or partition switch did not occur. |
| Description | Checks whether this upgrade was successful. |


**Example Code**

Refer to `otaGetOwnerFlag`.

### Interface: otaClearFlags
| Interface Name | void otaClearFlags(void); |
| :------------ | :---------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | N/A |
| Description | Clears the OTA flags. |


**Example Code**

Refer to `otaGetOwnerFlag`.


### Interface: otaPartitionSync
| Interface Name | int32_t otaPartitionSync(void); |
| :------------ | :---------------------------------------- |
| Interface Type | C function interface |
| Input Parameters | N/A |
| Output Parameters | N/A |
| Return Value | 0: Success<br/> -OTAERR_NOTINIT: Not initialized<br/> -OTAERR_IO: I/O error <br/>-OTAERR_REPEAT: Conflict with another upgrade process. |
| Description | Synchronizes AB partitions and BAK partitions. |


**Example Code**

Refer to `otaGetOwnerFlag`.