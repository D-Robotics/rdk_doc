---
sidebar_position: 1
---

# Configure U-Boot and Kernel Option Parameters

In system software development, it is often necessary to configure U-Boot and Kernel option parameters. This chapter introduces some common configuration methods for user reference.

## Configure U-Boot Option Parameters

:::info Note

The following instructions take modifying the `board_x5_rdk_ubuntu_nand_sdcard_debug_config` configuration file as an example.

The specific U-Boot configuration file used can be checked in the `bootloader/device/.board_config.mk` board-level configuration file under the `HR_UBOOT_CONFIG_FILE` variable value after running `./xbuild.sh lunch`.

:::

### Configuring via the xbuild Command

First, enter the `source/bootloader` directory. The directory structure is as follows:


```
├── build           # System code compilation directory, provides shell scripts for compiling various functional modules, and tools used in compilation
├── device          # Board-level configuration directory, each hardware platform has its corresponding configuration file where you can set compilation options and partition tables
├── miniboot        # Generates the minimal boot firmware containing GPT, MBR, BL2, DDR, and BL3x
├── out             # Compilation output directory
└── uboot           # U-Boot source code

```
`build/xbuild.sh` is the main build script that provides the following commands to help users configure U-Boot options. This command will automatically use the U-Boot configuration file set in the board-level configuration file. After the configuration is completed, it will automatically save the `savedefconfig` and other relevant files.


```
./xbuild.sh uboot menuconfig
```

After the command is successfully executed, the U-Boot graphical configuration interface will open. You can complete the option configurations in this interactive interface, including removing unnecessary features and enabling the required ones.

![U-Boot Graphical Configuration](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241120-201418.png)

After completing the configuration in the `menuconfig` interface, select `Exit` to exit. Follow the prompts to choose either `Yes` or `No` to save the changes to the `.config` file.

![Save Configuration](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development/image-20220518111506018.png)

After exiting the configuration, the configuration file will be automatically saved. The automatic tasks include:


```
# Call `savedefconfig` to clean the configuration file, keep the necessary items, remove dependencies, and generate the `defconfig` file
make savedefconfig

# Use the `defconfig` file to overwrite the U-Boot configuration file set in the board-level configuration file
cp -f defconfig <U-Boot configuration file set in the board-level configuration file>

```

### Manual Configuration

First, navigate to the `source/bootloader/uboot` directory and execute the command `make ARCH=arm64 hobot_x5_rdk_nand_defconfig`. The `make` command will first execute the Makefile in the top-level directory. For targets ending with `config`, there is a common entry point:

```makefile
%config: scripts_basic outputmakefile FORCE
        $(Q)$(MAKE) $(build)=scripts/kconfig $@
```

The expanded command to be executed is:

```
make -f ./scripts/Makefile.build obj=scripts/kconfig hobot_x5_rdk_nand_defconfig
```

After executing this command, a `.config` file will be generated in the root directory of the `uboot` source code.


```bash
make ARCH=arm64 hobot_x5_rdk_nand_defconfig

  HOSTCC  scripts/basic/fixdep
  HOSTCC  scripts/kconfig/conf.o
  YACC    scripts/kconfig/zconf.tab.c
  LEX     scripts/kconfig/zconf.lex.c
  HOSTCC  scripts/kconfig/zconf.tab.o
  HOSTLD  scripts/kconfig/conf
#
# configuration written to .config
#
```

Then, you can execute `make ARCH=arm64 menuconfig` to open the graphical configuration interface for configuring U-Boot options.

After completing the configuration in the `menuconfig` interface, select `Exit` to exit. Follow the prompts to choose either `Yes` or `No` to save the changes to the `.config` file.

Once the configuration is saved, you can run the command `diff .config configs/hobot_x5_rdk_nand_defconfig` to compare the differences and verify that the changes are as expected.

If the modifications are correct, execute `cp .config configs/hobot_x5_rdk_nand_defconfig` to replace the default configuration file.

To clean up the `.config` and other files in the source directory, otherwise, when recompiling the system, you may encounter a warning saying `xxx is not clean, please run 'make mrproper'`:

```bash
make distclean
# or
make mrproper

```

## Configuring Kernel Option Parameters

:::info Note

The following instructions use the `hobot_x5_rdk_ubuntu_defconfig` configuration file as an example.

The specific configuration file used by the kernel can be found in the `kernel_config_file` variable in the `mk_kernel.sh` script.

:::

### Configuring via `mk_kernel` Command

The `mk_kernel.sh` script provides the following commands to help users configure Kernel options. This command will automatically use the Kernel configuration file set in the board-level configuration file. After the configuration is complete, it will automatically perform `savedefconfig` and save the changes.


```
./mk_kernel.sh menuconfig
```
After executing the command successfully, the Kernel graphical configuration interface will open. You can complete the configuration options in this interactive interface, including removing unnecessary features and enabling the required ones.

![Kernel Configuration Interface](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241120-195703.png)

After completing the configuration in the `menuconfig` interface, select `Exit` to exit. Follow the prompts to choose either `Yes` or `No` to save the changes to the `.config` file.

![Save Configuration](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development/image-20220518111506018.png)

After exiting the configuration, the configuration file will be automatically saved. The automatic tasks include:


```
# Call `savedefconfig` to clean the configuration file, keep the necessary items, remove dependencies, and generate the `defconfig` file
make savedefconfig

# Use the `defconfig` file to overwrite the Kernel configuration file set in the board-level configuration file
cp defconfig <Kernel configuration file set in the board-level configuration file>

```
### Manual Configuration

The process of configuring the `kernel` via `menuconfig` is the same as configuring `uboot`. The execution steps are as follows:

First, navigate to the `boot/kernel` directory and then follow these steps to configure the `kernel` options.

- Use `hobot_x5_rdk_ubuntu_defconfig` to generate the `.config` file. If a full build has been performed on the source code, the `.config` file will already be configured.


```
make ARCH=arm64 hobot_x5_rdk_ubuntu_defconfig
```

- Execute the following command to modify the configuration


```
make ARCH=arm64 menuconfig
```

- After making the changes, you can first check the differences between the modified and the original configuration.


```
diff .config arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig
```

- Overwrite the `hobot_x5_rdk_ubuntu_defconfig` with the new configuration.

```
cp .config arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig
```

- Clean up the `.config` and other files in the source directory; otherwise, when recompiling the system, you may encounter a message saying `xxx is not clean, please run 'make mrproper'`.

```
make distclean
# or
make mrproper
```

## Modifying the Kernel DTS

This section introduces the DTS (Device Tree Source) file structure of the RDK product and explains how to modify it to support secondary development.

The kernel DTS files are located in
`x5-rdk-gen/source/kernel/arch/arm64/boot/dts/hobot/`.

:::info Note
- DTS files describe hardware resource allocations (such as GPIO, I²C, SPI, interrupt numbers, memory addresses, etc.). Incorrect modifications may cause the device to fail to boot, drivers to fail to load, or even damage the hardware.
- Misconfigurations may also lead to kernel build failures or prevent the system from booting properly. In severe cases, you may need to re-flash the system image to recover.
:::

### Kernel Device Tree

| DTS File | Purpose |
| --------- | ---------- |
| pinmux-func.dtsi | Defines the functional groups of each pin. |
| pinmux-gpio.dtsi | Defines GPIO groups. |
| x5.dtsi | Defines the default configuration of all modules on the X5 platform. It serves as the base hardware description and is not recommended for user modification. |
| x5-rdk.dtsi | Defines common configuration items for different SDK hardware versions. It includes the default configuration from x5.dtsi. |
| x5-board.dts | Board-level device tree file. It includes the default configuration from x5.dtsi and defines enabling/disabling of functional modules as well as hardware-specific peripheral configurations. |

### Inclusion Hierarchy

`x5-board.dts` → `x5-rdk.dtsi` → `x5.dtsi` → `(pinmux-func.dtsi and pinmux-gpio.dtsi)`

### Board-Level DTS Files

The hardware version can be identified from the silkscreen on the board, or by running the `hrut_boardid` command and checking the `board_id`.

| Hardware Version | board_id | DTS File |
| ------- | ---------- | ----------- |
| RDK X5 V0.1 | 0x301 | x5-rdk.dts |
| RDK X5 V1.0 | 0x302 | x5-rdk-v1p0.dts |
| RDK X5 MD V0.1 | 0x501 | x5-md-v0p1.dts |
| RDK X5 MD V0.2 | 0x502 | x5-md-v0p2.dts |
| RDK X5 MD V0.3 | 0x503 | x5-md-v0p2.dts |
| RDK X5 MD V1.0 | 0x504 | x5-md-v0p2.dts |
| RDK X5 MD V1.1 | 0x505 | x5-md-v0p2.dts |

### Build

In the `x5-rdk-gen` directory, use the `mk_kernel.sh` script to build the kernel:

```
sudo ./mk_kernel.sh
```

After compilation, the DTB files will be generated in the `x5-rdk-gen/deploy/kernel/dtb/` directory.
You can copy them to the `/boot/hobot/` directory on the board, and the system will automatically load them on boot.

Alternatively, you can run the following command to package the kernel DTB files into a .deb package for system installation.

```
sudo ./mk_debs.sh hobot-dtb
```

The generated .deb file will be located at`x5-rdk-gen/deploy/deb_pkgs/hobot-dtb_version-date_arm64.deb`