---
sidebar_position: 1
---
# Configuration of Uboot and Kernel Options

In software development, it is often necessary to configure the functional options of Uboot and Kernel. This chapter introduces several commonly used configuration methods for users' reference.

## Configuration of Uboot Options

:::info Note

The following instructions are provided with the example of modifying the `xj3_ubuntu_nand_defconfig` configuration file.

The specific configuration file used by Uboot can be found in the `bootloader/device/.board_config.mk` board-level configuration file after executing `./xbuild.sh lunch`, in the `HR_UBOOT_CONFIG_FILE` variable.

:::

First, enter the `uboot` directory and execute `make ARCH=arm64 xj3_ubuntu_nand_defconfig`. Because the `make` command will first execute the Makefile file in the top-level directory. The targets ending with `config` have a common entry:

```makefile
%config: scripts_basic outputmakefile FORCE
        $(Q)$(MAKE) $(build)=scripts/kconfig $@
```

The expanded command is:

```
make -f ./scripts/Makefile.build obj=scripts/kconfig xj3_ubuntu_nand_defconfig
```

After executing this command, a `.config` file will be generated in the root directory of the Uboot source code.

```bash
make ARCH=arm64 xj3_ubuntu_nand_defconfig

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

Then, you can execute `make ARCH=arm64 menuconfig` to open the graphical configuration interface for configuring Uboot options.

![image-20220518111319607](./image/driver_develop_guide/image-20220518111319607.png)After completing the configuration in the menuconfig interface, select "Exit" and choose "Yes" or "No" to save the modifications to the `.config` file according to the prompt.

![image-20220518111506018](./image/driver_develop_guide/image-20220518111506018.png)

After saving the configuration, execute the command `diff .config configs/xj3_ubuntu_nand_defconfig` to compare the differences and confirm whether the modifications meet expectations.

If the modifications are correct, execute `cp .config configs/xj3_ubuntu_nand_defconfig` to replace the default configuration file.

## Configure Kernel Options

:::info Note

The following instructions are based on modifying the `xj3_perf_ubuntu_defconfig` configuration file.

The specific configuration file used by the kernel can be found in the `kernel_config_file` variable in the `mk_kernel.sh` script.

:::

Configuring the kernel through `menuconfig` is similar to configuring U-Boot. Follow these steps to configure the kernel options.

- Use `xj3_perf_ubuntu_defconfig` to generate `.config`. If the source code has been fully compiled, the `.config` file will already be configured.

```
make ARCH=arm64 xj3_perf_ubuntu_defconfig
```

- Execute the following command to modify the configuration.

```
make ARCH=arm64 menuconfig
```

- After making modifications, you can see the differences between the modified and original configurations.

```
diff .config arch/arm64/configs/xj3_perf_ubuntu_defconfig
```

- Overwrite `xj3_perf_ubuntu_defconfig` with the new configuration.

```
cp .config arch/arm64/configs/xj3_perf_ubuntu_defconfig
```