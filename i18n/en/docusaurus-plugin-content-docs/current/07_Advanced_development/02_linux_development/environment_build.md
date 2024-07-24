---
sidebar_position: 1
---
# 7.2.1 Development Environment Setup and Compilation Instructions

This chapter introduces the requirements and setup of the cross-compilation development environment, as well as instructions for downloading the source code and compiling the system image.

## Cross-compilation Development Environment

Cross-compilation refers to developing and building software on a host machine, and then deploying the built software to a development board for execution. The host machine generally has higher performance and memory than the development board, which can accelerate code building and install more development tools for convenient development.

![image-20220329140159441](./image/environment_build/image-20220329140159441.png)

**Host Compilation Environment Requirements**

It is recommended to use Ubuntu operating system. If using other system versions, adjustments may be needed for the compilation environment.

For Ubuntu 18.04 system, install the following packages:

```shell
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
flex python-numpy mtd-utils zlib1g-dev debootstrap \
libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
curl git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
android-tools-fsutils mtools parted dosfstools udev rsync
```

For Ubuntu 20.04 system, install the following packages:

```shell
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
flex python-numpy mtd-utils zlib1g-dev debootstrap \
libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
curl git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
android-sdk-libsparse-utils android-sdk-ext4-utils mtools parted dosfstools udev rsync
```

For Ubuntu 22.04 system, install the following packages:

```shell
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
flex python3-numpy mtd-utils zlib1g-dev debootstrap \
libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
curl repo git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
android-sdk-libsparse-utils mtools parted dosfstools udev rsync
```

**Install Python**

Compiling the Linux kernel requires a Python 2 environment. Starting from version Ubuntu 22.04, Python 2 is no longer installed by default. Therefore, you need to execute the following command to install it:

```shell
sudo apt install python2
```

Compiling Debian packages requires a Python 3 environment. Use the following commands to set the Python selector and install python3-setuptools and pip, which are dependency tools for generating wheel packages:

```shell
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 2
sudo update-alternatives --list python

apt-get install python3-setuptools
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py
```

**Installing Cross-Compilation Toolchain**

Execute the following command to download the cross-compilation toolchain:

```shell
curl -fO http://sunrise.horizon.cc/toolchain/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu.tar.xz
```

Unzip and install it. It is recommended to install it under the /opt directory. Usually, writing data to the /opt directory requires sudo permission, for example:

```shell
sudo tar -xvf gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu.tar.xz -C /opt
```

Configure environment variables for the cross-compilation toolchain:

```shell
export CROSS_COMPILE=/opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/aarch64-linux-gnu-
export LD_LIBRARY_PATH=/opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PATH=$PATH:/opt/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/
export ARCH=arm64
```

The above commands configure the environment variables temporarily. To make the configuration permanent, you can add the above commands to the environment variable files `~/.profile` or `~/.bash_profile` at the end.

## Compiling Environment Source Code (rdk-gen)

rdk-gen is used to build a custom operating system image for the D-Robotics RDK X3. It provides a scalable framework that allows users to customize and build the Ubuntu operating system for RDK X3 according to their needs.

Download the source code:

```shell
git clone https://github.com/D-Robotics/rdk-gen.git
```

After downloading, the directory structure of rdk-gen is as follows:

| **Directory**              | **Description**                                                  |
| -------------------------- | ---------------------------------------------------------------- |
| pack_image.sh              | Code entry for building system images                             |
| download_samplefs.sh       | Download the pre-made base Ubuntu file system                     |
| download_deb_pkgs.sh       | Download D-Robotics's deb packages, including kernel, multimedia libraries, sample code, tros.bot, etc., which need to be pre-installed in the system image |
| hobot_customize_rootfs.sh  | Customized modification of the Ubuntu file system                 |
| source_sync.sh             | Download source code, including bootloader, uboot, kernel, example code, etc. |
| mk_kernel.sh               | Compile kernel, device tree, and driver modules                   |
| mk_debs.sh                 | Generate deb packages                                             |
| make_ubuntu_samplefs.sh    | Code for creating Ubuntu system filesystem, which can be modified to customize samplefs |
| config                     | Store the contents that need to be placed in the /hobot/config directory of the system image, a VFAT-rooted partition. If the SD card boot method is used, users can directly modify the contents of this partition in the Windows system. |

## Compiling the System Image

Run the following command to package the system image:

```shell
cd rdk-gen
sudo ./pack_image.sh
```

sudo privileges are required for compilation. After successful compilation, the system image file `*.img` will be generated in the deploy directory.

### Introduction to the Compilation Process of pack_image.sh

1. Call the scripts download_samplefs.sh and download_deb_pkgs.sh to download samplefs and the required pre-installed deb packages from D-Robotics's file server.
2. Extract samplefs and call the hobot_customize_rootfs.sh script to customize the filesystem configuration.
3. Install deb packages into the filesystem.
4. Generate the system image. Refer to [Install OS](../installation/install_os) for how to use the system image.

## Downloading Source Code

Downloading source code is not required when running `pack_image.sh` to compile the system image, because `pack_image.sh` will directly download the official debian packages from D-Robotics's file server and install them into the system. Only when you need to modify the content of debian packages and re-create custom packages, you need to download the source code.

The source code of rdk-linux related linux kernel, bootloader, hobot-xxx packages are hosted on [GitHub](https://github.com/). Before downloading the code, please register and log in to [GitHub](https://github.com/), and add the `SSH Key` of the development server to user settings through [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) method.

`source_sync.sh` is used to download the source code, including bootloader, uboot, kernel, example code, etc. This download program downloads all the source code to the local computer by executing `git clone git@github.com:xxx.git`.

Run the following command to download the main branch code (the latest release version branch maintained by the official):

```shell
./source_sync.sh -t main
```
Please execute the following command to download the code from the development branch (development branch, not fully tested, may have issues):

```shell
./source_sync.sh -t develop
```

If you want to download the source code that corresponds to the official system image version, such as the source code for the 2.0.0 version, please use the following command:

```shell
./source_sync.sh -t os-image_2.0.0
```

By default, the program will download the source code to the `source` directory:

```
source
├── bootloader
├── hobot-boot
├── hobot-bpu-drivers
├── hobot-camera
├── hobot-configs
├── hobot-display
├── hobot-dnn
├── hobot-dtb
├── hobot-io
├── hobot-io-samples
├── hobot-kernel-headers
├── hobot-multimedia
├── hobot-multimedia-dev
├── hobot-spdev
├── hobot-sp-samples
├── hobot-utils
├── hobot-wifi
└── kernel
```

## Compile kernel

The kernel source code is in `source/kernel`, to facilitate the compilation of the kernel, the program `mk_kernel.sh` is provided for users to use.

Execute the following command to compile the Linux kernel:

```shell
sudo ./mk_kernel.sh
```

After the compilation is completed, the kernel image, driver modules, device tree, and kernel header files will be generated in the `deploy/kernel` directory.

```shell
dtb  Image  Image.lz4  kernel_headers  modules
```

These contents will be used by three Debian packages: hobot-boot, hobot-dtb, and hobot-kernel-headers. If you want to customize and modify these three packages, you need to compile the kernel first.

## Compile hobot-xxx packages

The hobot-xxx packages are the source code and configuration of Debian packages maintained by D-Robotics. After downloading the source code, you can execute `mk_debs.sh` to rebuild the Debian packages.

The help information is as follows:

```shell
$ ./mk_debs.sh help
The debian package named by help is not supported, please check the input parameters.
./mk_debs.sh [all] | [deb_name]
    hobot-multimedia-dev, Version 2.0.0
    hobot-wifi, Version 2.0.0
    hobot-camera, Version 2.0.0
    hobot-dtb, Version 2.0.0
    hobot-configs, Version 2.0.0
    hobot-io, Version 2.0.0
    hobot-spdev, Version 2.0.0
    hobot-boot, Version 2.0.0
    hobot-sp-samples, Version 2.0.0
    hobot-bpu-drivers, Version 2.0.0
    hobot-multimedia-samples, Version 2.0.0
    hobot-dnn, Version 2.0.0
    hobot-io-samples, Version 2.0.0
    hobot-kernel-headers, Version 2.0.0
    hobot-utils, Version 2.0.0
    hobot-multimedia, Version 2.0.0
    hobot-display, Version 2.0.0
```

The descriptions and relationships of each Debian package are shown as follows:

![Flowchart](./image/environment_build/image-20221102173111002.jpg)

| Package Name                       | Content Description or Example                                 |
| ---------------------------------- | ------------------------------------------------------------ |
| hobot-sp-samples_xxx.deb           | Sample code for multimedia and algorithms: including video streaming and display output of vio, encoding and decoding examples, sample algorithms for image classification, object detection, segmentation, etc. |
| hobot-io-samples_xxx.deb           | Sample code for using the 40-pin interface: Python language examples for the 40-pin interface. |
| hobot-spdev_xxx.deb                | Encapsulation libraries and header files for C/C++ interfaces of multimedia and algorithms, encapsulation libraries and header files for Python interfaces of multimedia and algorithms. |
| hobot-multimedia-dev_xxx.deb       | Low-level multimedia header files.                            |
| hobot-multimedia_xxx.deb           | Runtime library files for multimedia: runtime library files (so files), configuration files, firmware, etc., for all multimedia-related components. |
| hobot-multimedia-samples_xxx.deb   | Reference examples based on the low-level multimedia interface. |
| hobot-camera_xxx.deb               | Drivers and ISP parameter libraries for compatible camera sensors. |
| hobot-dnn_xxx.deb                  | Runtime libraries and header files for algorithm-related components. |
| hobot-io_xxx.deb                   | Interfaces and header files for 40-pin GPIO usage (implemented in Python). |
| hobot-configs_xxx.deb              | D-Robotics's custom system configuration: udev configuration, apt source configuration, network, Bluetooth, USB configuration, autostart item configuration, etc. || **hobot-utils_xxx.deb**     | Common command set provided by D-Robotics |
| **hobot-display_xxx.deb**   | Image display related, HDMI, LCD display configuration |
| **hobot-wifi_xxx.deb**      | Configuration for Wi-Fi and Bluetooth modules |
| **hobot-kernel-headers_xxx.deb** | Configuration files and header files compiled after the kernel, used to support users to compile kernel drivers separately |
| **hobot-boot_xxx.deb**     | Kernel image file Image and driver module file |
| **hobot-bpu-driver_xxx.deb**  | BPU driver |
| **hobot-dtb_xxx.deb**   | Kernel device tree |

### Building Debian Packages as a Whole

Running the following command will rebuild all debian packages (kernel compilation is required beforehand):

```shell
./mk_debs.sh
```

After the build is complete, deb packages will be generated in the `deploy/deb_pkgs` directory.

### Building a Debian Package Individually

`mk_debs.sh` supports building specified packages individually by passing the package name as an argument, for example:

```shell
./mk_debs.sh hobot-configs
```

### Using Custom Debian Packages

When running `pack_image.sh` without arguments, it will download the latest release of debian packages from the D-Robotics file server and install them into the system. If you modify a package with the same name, you need to skip the process of downloading debian packages from the file server. You can use any optional parameters when executing the `pack_image.sh` command. For example, the following command will not redownload the debian package. Replace the original downloaded software package with your own package and rebuild it. For example, if you regenerate `hobot-boot` and name it `hobot-boot_2.0.0-customer_arm64.deb`, use that file to replace the `hobot-boot-xxx_arm64.deb` file in the `deb_packages` directory.

```shell
sudo ./pack_image.sh c
```

If you have added a custom-named software package and want to install it into the system, you can create a `third_packages` directory under the `rdk-gen` directory, and place the debian package you want to install in that directory. The software packages placed in the `third_packages` directory will be installed together with the software packages in the `deb_packages` directory, and they will not affect each other.

## Compiling Bootloader

The bootloader source code is used to generate the minimal boot image `disk_xxx_miniboot.img`, which contains the partition table, spl, ddr, bl31, and uboot.

The minimal boot image of RDK X3 is generally maintained and released by D-Robotics. You can download the corresponding version from [miniboot](http://sunrise.horizon.cc/downloads/miniboot/). If there are no modifications to uboot, you can directly use the official release image.

Follow the steps below to recompile and generate `miniboot`.

### Syncing Uboot Code

Execute the following command to download uboot code:

```shell
cd source/bootloader/
git submodule init
git submodule update
```

### Select board-level configuration file

```shell
cd source/bootloader/build
./xbuild.sh lunch

You're building on #221-Ubuntu SMP Tue Apr 18 08:32:52 UTC 2023
Lunch menu... pick a combo:
      0. horizon/x3/board_ubuntu_emmc_sdcard_config.mk
      1. horizon/x3/board_ubuntu_emmc_sdcard_samsung_4GB_config.mk
      2. horizon/x3/board_ubuntu_nand_sdcard_config.mk
      3. horizon/x3/board_ubuntu_nand_sdcard_samsung_4GB_config.mk
Which would you like? [0] :  
```

Choose the board-level configuration file according to the prompt.

The above preset configuration files are for different development boards' hardware configurations, the differences are in the use of emmc or nand to burn miniboot, different ddr models and capacities, and different root file systems:

| Board-level Configuration File                    | Memory                | rootfs        | Minimum Boot Image Storage | Main Storage |
| ------------------------------------------------- | --------------------- | ------------- | ------------------------- | ------------ |
| board_ubuntu_emmc_sdcard_config.mk                | LPDDR4 2GB    | ubuntu-20.04  | eMMC                      | sdcard       |
| board_ubuntu_emmc_sdcard_samsung_4GB_config.mk    | LPDDR4 4GB    | ubuntu-20.04  | eMMC                      | sdcard       |
| board_ubuntu_nand_sdcard_config.mk                | LPDDR4 2GB    | ubuntu-20.04  | nand                      | sdcard/eMMC  |
| board_ubuntu_nand_sdcard_samsung_4GB_config.mk    | LPDDR4 4GB    | ubuntu-20.04  | nand                      | sdcard/eMMC  |

**Minimum Boot Image Storage:** Storage for burning miniboot, users of RDK X3 and RDK X3 Module please select nand

**Main Storage:** Storage for the ubuntu system image, the system image of sdcard is compatible with eMMC, which means it can be burned to a Micro SD storage card or eMMC



The lunch command also supports specifying numbers and board-level configuration file names to complete the configuration directly.

```shell
$ ./xbuild.sh lunch 2

You're building on #221-Ubuntu SMP Tue Apr 18 08:32:52 UTC 2023
You are selected board config: horizon/x3/board_ubuntu_nand_sdcard_config.mk

$ ./xbuild.sh lunch board_ubuntu_nand_sdcard_config.mk

You're building on #221-Ubuntu SMP Tue Apr 18 08:32:52 UTC 2023
You are selected board config: horizon/x3/board_ubuntu_nand_sdcard_config.mk
```

### Compiling Bootloader as a Whole

Go to the build directory and execute xbuild.sh to compile the bootloader as a whole:

```shell
cd build
./xbuild.sh
```

After successful compilation, the image files, including miniboot.img, uboot.img, and disk_nand_minimum_boot.img, will be generated in the output directory of the compilation (deploy_ubuntu_xxx). The disk_nand_minimum_boot.img is the minimum boot image.

### Modular Compilation of Bootloader

Use the xbuild.sh script to compile individual modules and the generated image files will be output to the compilation output directory (deploy_ubuntu_xxx).

```shell
./xbuild.sh miniboot | uboot
```

**miniboot:** Calls mk_miniboot.sh to generate miniboot.img

**uboot:** Calls mk_uboot.sh to generate uboot.img

After modular compilation, the pack command can be executed to package disk_nand_minimum_boot.img.

```shell
./xbuild.sh pack
```

## Creating Ubuntu File System

This section describes how to create the `samplefs_desktop-v2.0.0.tar.gz` file system. D-Robotics maintains this file system, but if customization is required, it needs to be recreated according to the instructions in this section.

### Environment Configuration

It is recommended to use an Ubuntu host to create the Ubuntu file system for the development board. First, install the following software packages in the host environment:

```shell
sudo apt-get install wget ca-certificates device-tree-compiler pv bc lzop zip binfmt-support \
build-essential ccache debootstrap ntpdate gawk gcc-arm-linux-gnueabihf qemu-user-static \
u-boot-tools uuid-dev zlib1g-dev unzip libusb-1.0-0-dev fakeroot parted pkg-config \
libncurses5-dev whiptail debian-keyring debian-archive-keyring f2fs-tools libfile-fcntllock-perl \
rsync libssl-dev nfs-kernel-server btrfs-progs ncurses-term p7zip-full kmod dosfstools \
libc6-dev-armhf-cross imagemagick curl patchutils liblz4-tool libpython2.7-dev linux-base swig acl \
python3-dev python3-distutils libfdt-dev locales ncurses-base pixz dialog systemd-container udev \
lib32stdc++6 libc6-i386 lib32ncurses5 lib32tinfo5 bison libbison-dev flex libfl-dev cryptsetup gpg \
gnupg1 gpgv1 gpgv2 cpio aria2 pigz dirmngr python3-distutils distcc git dos2unix apt-cacher-ng
```

### Key Tool Introduction

#### debootstrap

debootstrap is a tool for building a basic system (root file system) in debian/ubuntu. The generated directory complies with the Linux File System Standard (FHS), including directories such as /boot, /etc, /bin, /usr, etc. However, it is much smaller in size compared to the Linux distribution version, and its functionality is not as powerful. Therefore, it can only be called a "basic system", so it can be customized according to your own needs for the ubuntu system.

Installation of debootstrap on Ubuntu (PC):

```shell
sudo apt-get install debootstrap
```

Usage:

```shell
# Additional parameters can be added to specify the source
sudo debootstrap --arch [platform] [distribution code] [directory] [source]
```

#### chroot

chroot, i.e. change root directory. In the Linux system, the default directory structure starts with `/`, which is the root. After using chroot, the system's directory structure will be changed to the specified location as the new root (/).

#### parted

parted is a powerful disk partition and resizing tool developed by the GNU organization. Unlike fdisk, it supports resizing partitions. Designed for Linux, it does not build multiple partition types associated with fdisk, but it can handle the most common partition formats, including ext2, ext3, fat16, fat32, NTFS, ReiserFS, JFS, XFS, UFS, HFS, and Linux swap partition.

### Script Code for Creating Ubuntu rootfs

Download `rdk-gen` source code:

```shell
git clone https://github.com/D-Robotics/rdk-gen.git
```

Execute the following command to generate the Ubuntu file system:

```shell
mkdir ubuntu_rootfs
cd ubuntu_rootfs
cp ../make_ubuntu_rootfs.sh .
chmod +x make_ubuntu_rootfs.sh
sudo ./make_ubuntu_rootfs.sh
```

Successful compilation output:

```shell
desktop/                                   # Compilation output directory
├── focal-xj3-arm64                        # Root file system generated after successful compilation, with various temporary system files── samplefs_desktop-v2.0.0.tar.gz         # Compressed package including the necessary content for the focal-xj3-arm64 
└── samplefs_desktop-v2.0.0.tar.gz.info    # Information about the currently installed apt packages

rootfs/                                    # After extracting samplefs_desktop-v2.0.0.tar.gz, the following files should be included
├── app
├── bin -> usr/bin
├── boot
├── dev
├── etc
├── home
├── lib -> usr/lib
├── media
├── mnt
├── opt
├── proc
├── root
├── run
├── sbin -> usr/sbin
├── srv
├── sys
├── tmp
├── userdata
├── usr
└── var

21 directories, 5 files
```

In the code, key variable definitions are as follows:

**PYTHON_PACKAGE_LIST**: A list of Python packages to be installed.

**DEBOOTSTRAP_LIST**: Debian packages to be installed during the execution of debootstrap.

**BASE_PACKAGE_LIST**: The essential Debian packages required for a minimal Ubuntu system installation.

**SERVER_PACKAGE_LIST**: Additional Debian packages that will be installed on top of the base version for an Ubuntu Server edition.

**DESKTOP_PACKAGE_LIST**: Software packages needed to support a graphical desktop environment.

The officially maintained `samplefs_desktop` filesystem by D-Robotics includes all configurations from these package lists. Users can customize this by adding or removing packages according to their specific requirements, maintaining the original format or structure.