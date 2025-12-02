---
sidebar_position: 06
---
# 7.6 RDK S100 Build System Development Guide
## 7.6.1 Overview
This section is primarily intended for users who need to customize the RDK build system. For instructions on using `rdk_gen`, please refer to the README.md in the `rdk_gen` repository.

Basic usage instructions:
``` bash
# Online image building: download required deb packages from D-Robotics and third-party APT repositories
sudo ./pack_image.sh

# Offline image building: only install deb packages under out/product/deb_packages.
# When using this option, ensure that the deb packages in out/product/deb_packages meet expectations,
# and that precompiled rootfs packages already exist under out/product/rootfs_packages.
sudo ./pack_image.sh -l

# Only set up the deb package build environment without packing an image
sudo ./pack_images.sh -p

# Build all deb packages
./mk_debs.sh

# Build a specific deb package (e.g., hobot-configs)
./mk_debs.sh hobot-configs
```

## 7.6.2 Precompiled Root Filesystem Package Build Instructions
The root filesystem for RDK S100 is generated using `multistrap` + `chroot`.

### multistrap
#### Official Documentation
  - [Debian Wiki | Multistrap](https://wiki.debian.org/Multistrap)
  - [Debian manpage](https://manpages.debian.org/bookworm/multistrap/multistrap.1.en.html)

#### Tool Introduction
In summary, `multistrap` is an alternative tool to `debootstrap` for generating root filesystems based on Debian/Ubuntu APT repositories. It uses one or more configuration files to define which APT sources to use and which packages should be included by default in the generated root filesystem.

Key differences from `debootstrap` include:
1. **Flexibility**: `multistrap` allows users to fully customize all packages within the newly generated root filesystem—including those marked as essential in APT sources—but users must ensure the completeness and usability of the resulting root filesystem themselves.
2. **Build Process**: Unlike `debootstrap`, the `multistrap` generation process can be summarized in the following steps. The key difference lies in step 4: `multistrap` only decompresses packages without configuring them (i.e., without executing `[pre/post]install` scripts):
   1. Read configuration file(s)
   2. Fetch metadata from specified APT sources according to the configuration
   3. Attempt to download specified packages per the configuration
   4. Decompress specified packages per the configuration

In D-Robotics's provided `multistrap` build script, based on practical experience, we implement package configuration using `binfmt-support + chroot` under `sudo` privileges, enabling users to directly flash the generated root filesystem onto the board for immediate use.

#### Configuration File Introduction
`multistrap` supports both single-file and multi-file configurations. In multi-file mode, the `include` field allows including all content from a base configuration file, enabling version-specific customization—greatly simplifying maintenance across multiple filesystem variants.

Configuration files referenced in the following sections are located at the S100 code path: `samplefs/configs`:
```bash
$ tree samplefs/configs/
samplefs/configs/
├── jammy-base.conf
├── jammy-desktop.conf
├── jammy-server.conf
└── pip-requirements.list

0 directories, 4 files
```

##### Basic Format Introduction
For detailed field descriptions, please refer to the official documentation. Here, we focus on explaining key configurations used in D-Robotics's setup.

1. **Field format**: `key1=value1` defines the value of field "key1" as "value1".
2. **Field sets (stanzas/sections)**: Defined by `[Some-Section]`, grouping all fields from that line until the next `[Next-Section]` into a single "Section".

Key fields explained:
- **include**  
  Specifies the path to configuration files to be included.
- **bootstrap**  
  Defines the stanza containing APT sources used to generate the root filesystem and the packages to be downloaded and decompressed.
- **aptsources**  
  Defines the stanza specifying APT sources to be saved under `/etc/sources.list.d/` in the generated root filesystem. **Note**: These sources do not necessarily need to match those used in `bootstrap`, but we strongly recommend that the sources defined in `aptsources` are a superset of those in `bootstrap`.
- **source / suite / components / omitdebsrc**  
  Define key parameters for APT sources. Refer to the [APT source format definition](https://manpages.ubuntu.com/manpages/xenial/man5/sources.list.5.html) for details:
  - **source**: Root URL of the APT repository, corresponding to the "uri" field in APT source format.
  - **suite**: Suite name of the APT repository, corresponding to the "suite" field (e.g., Ubuntu code names like `jammy`, `focal`, or updates/security suites like `jammy-updates`, `jammy-security`).
  - **components**: Repository components, matching the "component" field; multiple components can be specified.
  - **omitdebsrc**: Whether to skip downloading source packages (`*.dsc`, etc.) when fetching metadata and packages. Typically set to `true` to accelerate builds.
- **packages**  
  Specifies packages to be fetched. Multiple packages can be listed in one `packages` field (space-separated), and the union of all `packages` fields forms the final package list.

##### Multi-file Configuration Example
Refer to `samplefs/configs/jammy-desktop.conf`.

### RDK S100 Implementation
#### Configuration File Design
By default, RDK S100 splits `multistrap` configuration into three parts:
- **jammy-base.conf**: Configures common defaults for all RDK S100 root filesystems, including APT sources and packages included in all variants.
- **jammy-desktop.conf**: Adds desktop-specific packages on top of `jammy-base.conf`.
- **jammy-server.conf**: Adds server-specific packages on top of `jammy-base.conf`.

#### Build Process Instructions
Users typically build the root filesystem using the script `samplefs/make_ubuntu_samplefs.sh`. If invoked with `sudo`, the build process includes package configuration, significantly increasing build time.

If invoked without `sudo`, package configuration is skipped, reducing build time—but on first boot, the board must run the following command to initialize the system and reboot for proper operation:
```bash
rm -rf /var/lock/* ; dpkg --configure -a --force-confdef --force-confold ; systemctl enable /etc/systemd/system/S*.service
```

:::info **Tip**
It is **recommended to invoke** `samplefs/make_ubuntu_samplefs.sh` **with `sudo`**.
:::

The script must be run from within the `samplefs` directory.

Running `sudo ./make_ubuntu_samplefs.sh` without arguments defaults to building the desktop image using `jammy-desktop.conf`.

To specify a custom config file:  
`sudo ./make_ubuntu_samplefs.sh build <config_file_name>`  
For example, if you create a new config `new-desktop.conf`, place it in `samplefs/configs/` and run:  
`sudo ./make_ubuntu_samplefs.sh build new-desktop.conf`

Build workflow diagram:
![samplefs_flowchart](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/06_rdk_gen/samplefs_flowchart-en.jpg)

#### Methods for Trimming/Customizing the Root Filesystem
:::info Tip
The **Priority** field in APT source (deb package control info) determines trimming/customization behavior.  
By default, `multistrap` installs all packages with Priority "**Required**".  
D-Robotics also ensures packages marked as "**important**" are installed.
:::

##### Trimming/Customizing Packages Not Marked "important" or "required"
Users can directly remove unwanted packages from the `packages` fields in config files like `samplefs/configs/jammy-base.conf`, or define a new stanza and update the `bootstrap` field in `[General]` to exclude the original stanza and include the new one.

##### Trimming/Customizing Packages Marked "important" or "required"
**Note**: APT maintainers typically mark the minimal system package set (for Ubuntu/Debian versions) as "Required", but further trimming is possible. At this level, users **must guarantee the completeness and usability** of the resulting root filesystem.

Steps:
1. Add `omitrequired=true` in the `[General]` stanza.
2. Explicitly define all required packages in the `Packages` stanza.

## 7.6.3 RDK S100 Deb Package Build Process
### Introduction
RDK S100 manages D-Robotics-customized user-space features as deb packages. Source code for building these deb packages is stored under the SDK's `source/` directory.

### Build Script Introduction
The entry script for building debs is `mk_debs.sh`, located in the SDK root directory. Users can build repository-contained deb packages via this script.

### Deb Package Source Directory Structure
All directories under `source/`, except `bootloader`, `kernel`, and `hobot-drivers`, contain source code for D-Robotics-customized deb packages. Directories like `hobot-spdev`, `hobot-camera`, and `hobot-io` include source code for corresponding dynamic libraries; compiling these packages via `mk_debs.sh` triggers compilation of their source code.

Basic structure of a deb source directory (using `hobot-configs` as an example):
```bash
hobot-configs/
├── LICENSE          # License information for the deb source
├── README.md        # Brief description of the deb package
├── VERSION          # Version number in major.minor.patch format; timestamp appended during build
└── debian           # Root directory for the deb package (mirrors root filesystem upon installation)
    ├── DEBIAN       # Deb metadata
    |   ├── postinst # Standard deb script: runs after file copying during dpkg install
    │   ├── postrm   # Standard deb script: runs after file deletion during dpkg removal
    │   ├── preinst  # Standard deb script: runs before file copying during dpkg install
    │   └── prerm    # Standard deb script: runs before file deletion during dpkg removal
    ├── etc          # Files to copy to rootfs (e.g., this dir → /etc on target)
    ├── lib          # Files to copy to rootfs
    └── usr          # Files to copy to rootfs
```

For deb packages containing source code (e.g., `hobot-camera`), additional source directories appear alongside `debian`:
```bash
hobot-camera/
├── LICENSE
├── README.md
├── VERSION
├── debian
│   ├── DEBIAN
│   ├── etc
│   └── usr
├── drivers             # Source code directory requiring compilation
│   ├── Makefile        # Entry Makefile for compilation
│   ├── deserial        # Subdirectory
│   ├── inc
│   └── sensor
├── lib -> ../hobot-multimedia/debian/usr/hobot/lib # Symlink for easier compilation
├── sensor_calibration       # Sensor tuning library (example only; actual libs may vary)
│   └── lib_imx219_linear.so
└── tuning_tool              # D-Robotics-provided sensor tuning tool
    ├── bin
    ├── cfg
    ├── control_tool
    ├── res
    └── scripts
```

### Build Process Overview
For full details, refer to the `mk_debs.sh` script implementation. Simplified workflow:
![mk_debs_flowchart](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/06_rdk_gen/mk_debs_flowchart-en.jpg)

### Custom Deb Package Creation Guide
1. Create a new directory under `source/` named after your package (e.g., `new_package`).
2. Inside `source/new_package`, create a `debian` directory containing four (possibly empty) standard deb scripts:
   - `preinst`: Runs before file copying during package installation.
   - `postinst`: Runs after file copying during package installation.
   - `prerm`: Runs before file deletion during package removal.
   - `postrm`: Runs after file deletion during package removal.
3. To auto-build this package when running `mk_debs.sh` without arguments, add `new_package` to the `deb_pkg_list` variable in `mk_debs.sh`.
4. In the `switch` block of the `make_debian_deb` function in `mk_debs.sh`, add a `case` for `new_package`:
   - Call `gen_control_file` to generate the required `control` file.
   - Use `sed` to replace the default "Depends" field with actual dependencies. For example, if `new_package` depends on `dep_pkg1` and `dep_pkg2`:
     - With dependencies:  
       `sed -i 's/Depends: .*$/Depends: dep_pkg1,dep_pkg2/' "${deb_dst_dir}"/DEBIAN/control;`
     - Without dependencies:  
       `sed -i 's/Depends: .*$/Depends: /' "${deb_dst_dir}"/DEBIAN/control;`
   - *(Optional)* If source compilation is needed before packaging, add compilation commands—ensuring all outputs go to `out/build/debs/new_pkg/debian/`.
   - Set `is_allowed=1`.

## 7.6.4 Including Deb Packages in Images
During image building, deb packages are integrated into the board's root filesystem.

### Online Image Building
#### Process Overview
When `pack_image.sh` is run without the `-l` option, it follows the online image build process:
1. Retrieve the default build config file from the `DEFAULT_CONFIG` field in `pack_image.sh` (e.g., `build_params/ubuntu-22.04_desktop_rdk-s100_release.conf`).- This configuration file can be specified via the `-c` option.
2. Retrieve the `RDK_DPKG_DEB_PKG_LIST` field from `build_params/ubuntu-22.04_desktop_rdk-s100_release.conf`;
3. chroot into the `out/deploy/rootfs` directory:
   1. Attempt to download all deb packages from the existing on-device apt repositories based on the `RDK_DPKG_DEB_PKG_LIST` field;
   2. Install all deb packages;

#### Method to Add Additional Deb Packages
1. Identify the name of the package to be installed;
2. Add the corresponding package name to the `RDK_DPKG_DEB_PKG_LIST` variable in the specified build configuration file.

:::info Note
For instructions on how to obtain package names, please refer to [Method to Obtain Required Deb Package Names](#get_package_name)
:::

### Offline Image Building
#### Process Description
When the `-l` option is added to `pack_image.sh` during execution, the offline image building process is triggered. The process is described as follows:
1. chroot:
   1. Install each deb package based on the existing deb packages in `out/product/deb_packages`;

#### Method to Add Additional Deb Packages
1. Identify the name of the package to be installed;
2. Execute the following command to download the corresponding package into `out/product/deb_packages`:
```bash
cd out/product/deb_packages
apt download <package names>
```

:::info Note
For instructions on how to obtain package names, please refer to [Method to Obtain Required Deb Package Names](#get_package_name)
:::

### Method to Obtain Required Deb Package Names {#get_package_name}
Package names can be obtained via two methods:
1. You only know the required file, and the host has never installed the package:
   - It is recommended to use search engines such as Google or Baidu to identify the deb package containing the file and confirm its apt repository. The reference process is as follows:
     - Search for the deb package name by filename on [debian.org](https://www.debian.org/distrib/packages), noting that you must perform the search under the "Search the contents of packages" section;
     - After confirming the package name, search for its repository on [Ubuntu Launchpad](https://launchpad.net/ubuntu/+search) to verify that the package exists in components such as main/universe/multiverse under jammy/jammy-updates;
2. You know the required file, and the host has already installed it:
   - You can obtain the package name using the command `dpkg -S <filename>`.

## 7.6.5 Custom Partition Description
The partition definition file for RDK S100 is stored in the folder `source/bootloader/device/rdk/s100/partition_config_files`. The default partition table used is: `source/bootloader/device/rdk/s100/partition_config_files/s100-gpt.json`
```json
{
	"global": {
		"antirollbackUpdate_host": true,
		"antirollbackUpdate_hsm": false,
		"ab_sync": false,
		"backup_dir": "/tmp/ota/backup",
		"AB_part_a": "_a",
		"AB_part_b": "_b",
		"BAK_part_bak": "_bak"
	},
	"fpt": "fpt_common",                //------------------//
	"recovery": "recovery_common",      //                  //
	"misc": "misc_common",              //                  //
	"HB_APDP":"HB_APDP_common",         //                  //
	"keystorage": "keystorage_common",  //                  //
	"HSM_FW": "HSM_FW_common",          //  miniboot_flash  //
	"HSM_RCA": "HSM_RCA_common",        //                  //
	"keyimage": "keyimage_common",      //                  //
	"SBL": "SBL_common",                //                  //
	"scp": "scp_common",                //                  //
	"spl": "spl_common",                //                  //
	"MCU": "MCU_common",                //------------------//

	"quickboot": "quickboot_common",    //------------------// //------------------//
	"veeprom": "veeprom_common",        //                  // //                  //
	"ubootenv": "ubootenv_common",      //                  // //                  //
	"acore_cfg": "acore_cfg_common",    //   miniboot_emmc  // //                  //
	"bl31": "bl31_common",              //                  // //                  //
	"optee": "optee_common",            //                  // //                  //
	"uboot": "uboot_common",            //------------------// //     emmc_disk    //
	"boot": "boot_common",                                     //                  //
	"ota": "ota_common",                                       //                  //
	"log": "log_common",                                       //                  //
	"userdata": "userdata_common",                             //                  //
	"system": "system_common"                                  //------------------//
}
```
### Default Images and Partition Descriptions
1. **miniboot_flash**: Basic boot image stored on the S100 Nor Flash, including images for system components such as HSM/MCU0;
2. **miniboot_emmc**: Basic boot image stored on the S100 eMMC, including images for system components such as BL31/Uboot;
3. **emmc_disk**: Full image on the S100 eMMC, which includes miniboot_emmc. During compilation, it is automatically converted into an Android Sparse image ([Android Sparse Image Format Explanation (third-party website, for reference only)](https://www.2net.co.uk/tutorial/android-sparse-image-format)) to reduce system storage space usage;

### Configuration File Description
The overall partition table configuration consists of global shared settings and individual partition settings. Global shared settings are placed under the `"global"` field and apply to all partitions.

**Supported Global Parameters:**
- `antirollbackUpdate_host`: Whether to update the host's anti-rollback version (true or false);
- `antirollbackUpdate_hsm`: Whether to update the HSM's anti-rollback version (true or false);
- `ab_sync`: Reserved field for D-Robotics;
- `backup_dir`: HSM backup directory;
- `AB_part_a`: Suffix for the A partition in AB partitioning;
- `AB_part_b`: Suffix for the B partition in AB partitioning;
- `BAK_part_bak`: Suffix for backup partitions;

Individual partition configurations follow the format `"partition_name": "partition_config_type"`, allowing different partition configuration types to be selected as needed. For example, for the boot partition, you first need to add a description for the boot partition in the global partition configuration, then create `boot.json` in the folder `source/bootloader/device/rdk/s100/partition_config_files/sub_config` and define `boot_common` as follows:
```json
{
	"boot_common": {
		"components":[
			"${TARGET_DEPLOY_DIR}/rootfs/boot/:60m"
		],
		"pre_cmd": [
			"pack_boot.sh;[ $? -ne 0 ] && exit 1;rm -rf ${TARGET_DEPLOY_DIR}/rootfs/boot/System.map*"
		],
		"post_cmd": [
			"pack_avb_img.sh boot boot"
		],
		"fs_type": "ext4",
		"medium": "emmc",
		"ota_is_update": false,
		"ota_update_mode": "image",
		"part_type": "AB",
		"part_type_guid": "C12A7328-F81F-11D2-BA4B-00A0C93EC93B",
		"size": "60m"
	},
	"boot_ota": {
		"components":[
			"${TARGET_DEPLOY_DIR}/rootfs/boot/:60m"
		],
		"pre_cmd": [
			"pack_boot.sh;[ $? -ne 0 ] && exit 1;rm -rf ${TARGET_DEPLOY_DIR}/rootfs/boot/System.map*"
		],
		"post_cmd": [
			"pack_avb_img.sh boot boot"
		],
		"fs_type": "ext4",
		"medium": "emmc",
		"ota_is_update": true,
		"ota_update_mode": "image",
		"part_type": "AB",
		"part_type_guid": "C12A7328-F81F-11D2-BA4B-00A0C93EC93B",
		"size": "60m"
	}
}
```
**Supported Partition Parameters:**
- `depends`: Specifies partition dependencies; lists partitions that must be built before the current partition can be built;
- `components`: Specifies the contents included in the current partition, which can be sub-image paths or file directories. File directories will be formatted as filesystems. Currently supported filesystems are ext4 and fat16. Append `":"` followed by a size to indicate the space this component occupies within the partition. Multiple sub-images are allowed, and it is recommended to place sub-images under the corresponding partition directory in `out/xxx/deploy`;
- `components_nose`: Components for non-secure boot images;
- `pre_cmd`: Commands to execute before building the image from the `components`;
- `post_cmd`: Commands to execute after building the image from the `components`;
- `fs_type`: Image filesystem type: None/ext4/fat16/misc;
- `medium`: Storage medium for the image: emmc/nor;
- `ota_is_update`: Whether the partition is included in full OTA packages;
- `ota_update_mode`: OTA update method for the partition; `image` for full image updates (default), `image_diff` for differential image updates;
- `is_rootfs`: Whether this is the rootfs partition;
- `part_type`: Partition type; currently supports AB, BAK, PERMANENT;
- `size`: Partition size, with units k, m, or g;
- `magic`: Magic value for the partition; only valid for partition images with a flash image header;
- `have_anti_ver`: Whether the partition image includes an anti-rollback version (true or false);
- `load_addr`: Load address for the image; only valid for partition images with a flash image header;
- `entry_addr`: Entry address for the image; only valid for partition images with a flash image header;
- `nose_support`: Support for non-secure boot images;

:::info Note
1. The order of partitions in the JSON file corresponds to the actual partition order on the device;
2. Partition sizes must be aligned according to the sector size of the storage medium to which the partition belongs;
:::

### Partition Modification Instructions
S100X supports modifying partitions on eMMC/UFS by simply adding, deleting, or modifying the corresponding partition fields in the partition configuration. For modifications to flash partitions, please contact D-Robotics.

:::warning Partition Modification Notes
1. In the partition table, partitions up to and including `log` are boot partitions, and modifications are generally not recommended.
2. Other partitions, such as `userdata` and `system`, can have their sizes freely adjusted. If adding a new partition, remember to update the `fstab` file to ensure the partition is properly mounted;
:::

After modifying the partition table, the following steps are required for the changes to take effect:
1. Build the hobot-miniboot deb package:
	```shell
	# In RDK Source Root Directory
	./mk_debs.sh hobot-miniboot
	```
2. Build the image locally:
	```shell
	# In RDK Source Root Directory
	sudo ./pack_image.sh -l
	```

:::tip
The commercial version provides more comprehensive feature support, deeper hardware capability access, and exclusive customization options. To ensure compliance and secure delivery, we will grant access to the commercial version through the following process:

**Commercial Version Access Process:**
1. **Complete a questionnaire**: Submit basic information about your organization and intended use case.
2. **Sign an NDA**: We will contact you based on your submission, and both parties will sign a Non-Disclosure Agreement upon mutual confirmation.
3. **Content release**: After the agreement is signed, we will provide access to commercial version materials through a private channel.

If you wish to obtain the commercial version, please complete the questionnaire below. We will contact you within 3–5 business days:

Questionnaire link: https://horizonrobotics.feishu.cn/share/base/form/shrcnJQBMIkRm6K79rjXR0hr0Fg
:::