---
sidebar_position: 2
---

# 配置uboot和kernel选项参数

在系统软件开发中，经常需要对uboot和Kernel的功能选项进行配置，本章节介绍几个常用的配置方法，供用户参考使用。

## 配置uboot选项参数

:::info 注意

​	以下说明以修改 `board_x5_rdk_ubuntu_nand_sdcard_debug_config`配置文件为例。

​	Uboot具体使用的配置文件可以在`./xbuild.sh lunch`之后查看`bootloader/device/.board_config.mk`板级配置文件中 `HR_UBOOT_CONFIG_FILE`的变量值。

:::

### 通过 xbuild 命令配置

首先进入`source/bootloader`目录,目录结构如下

```
├── build # 编译系统代码目录，提供编译各个功能模块的shell脚本，编译用到的tools
├── device # 板级配置目录，每种硬件对应一份配置文件，可以设置编译选项和分区表等
├── miniboot  # 生成包含gpt、mbr、bl2、ddr、bl3x 一体的最小启动固件
├── out # 编译输出目录
└── uboot # Uboot源代码
```

`build/xbuild.sh`为主编译脚本，提供了以下命令帮助用户进行 Uboot 的选项配置，该命令会自动使用板级配置文件中设置的 Uboot 配置文件，在配置完成后，自动完成savedefconfig 和保存工作。
```
./xbuild.sh uboot menuconfig
```

命令执行成功后会打开 Uboot 图形化配置界面，您可以在这个交互界面下完成选项的配置，包括删除不需要的功能和启用需要的功能。

![image-20220518111319607](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241120-201418.png)

在menuconfig的配置界面上完成配置后，选择 `Exit`退出，根据提示选择 `Yes` 或者`No`保存修改到`.config`文件中。

![image-20220518111506018](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development/image-20220518111506018.png)

在配置退出后，会自动执行配置的文件的保存。自动完成的内容为：

```
# 调用savedefconfig对配置文件进行清理，保留必须项，删除被依赖项，生成 defconfig 文件
make savedefconfig
# 使用 defconfig 文件覆盖板级配置文件中设置的Uboot配置文件
cp -f defconfig <板级配置文件中设置的Uboot配置文件>
```

### 手动配置

首先进入`source/bootloader/uboot`目录，执行`make ARCH=arm64 hobot_x5_rdk_nand_defconfig `。因为`make`命令将首先执行顶级目录下的 Makefile 文件。其中对于以config结尾的目标都有一个共同的入口：

```makefile
%config: scripts_basic outputmakefile FORCE
        $(Q)$(MAKE) $(build)=scripts/kconfig $@
```

展开后的执行命令是：

```
make -f ./scripts/Makefile.build obj=scripts/kconfig hobot_x5_rdk_nand_defconfig
```

本命令执行后会在`uboot`的源码根目录下会生成 `.config`的文件。

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

然后就可以执行`make ARCH=arm64 menuconfig`打开图形化的配置界面进行uboot的选项参数配置。

在menuconfig的配置界面上完成配置后，选择 `Exit`退出，根据提示选择 `Yes` 或者`No`保存修改到`.config`文件中。

保存配置后，可以执行命令 `diff .config configs/hobot_x5_rdk_nand_defconfig` 对比一下差异，再次确认一下修改是否符合预期。

如果修改正确，请执行 `cp .config configs/hobot_x5_rdk_nand_defconfig`替换默认的配置文件。

清理源码目录下的 .config 等文件，否则在重新编译系统时会提示需要 xxx is not clean, please run 'make mrproper'
```bash
make distclean
# 或者
make mrproper
```

## 配置kernel选项参数

:::info 注意

​	以下说明以修改 `hobot_x5_rdk_ubuntu_defconfig`配置文件为例。

​	kernel具体使用的配置文件可以查看 `mk_kernel.sh` 脚本中 `kernel_config_file` 的变量值。

:::

### 通过 mk_kernel 命令配置

`mk_kernel.sh`提供了以下命令帮助用户进行 Kernel 的选项配置，该命令会自动使用板级配置文件中设置的 Kernel 配置文件，在配置完成后，自动完成savedefconfig 和保存工作。

```
./mk_kernel.sh menuconfig
```

命令执行成功后会打开 Kernel 图形化配置界面，您可以在这个交互界面下完成选项的配置，包括删除不需要的功能，启用需要的功能。

![image-20220518111319607](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_x5/screenshot-20241120-195703.png)

在menuconfig的配置界面上完成配置后，选择 `Exit`退出，根据提示选择 `Yes` 或者`No`保存修改到`.config`文件中。

![image-20220518111506018](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development/image-20220518111506018.png)

在配置退出后，会自动执行配置的文件的保存。自动完成的内容为：

```
# 调用savedefconfig对配置文件进行清理，保留必须项，删除被依赖项，生成 defconfig 文件
make savedefconfig
# 使用 defconfig 文件覆盖板级配置文件中设置的Kernel配置文件
cp defconfig <板级配置文件中设置的Kernel配置文件>
```

### 手动配置

通过`menuconfig`方式配置`kernel`与配置`uboot`的的过程是一样的。命令执行过程如下：

首先进入`boot/kernel`目录，然后按照以下步骤配置`kernel`选项。

- 使用`hobot_x5_rdk_ubuntu_defconfig`来配置生成`.config`，如果源码做过全量编译，则`.config`文件会配置好

```
make ARCH=arm64 hobot_x5_rdk_ubuntu_defconfig
```

- 执行以下命令来修改配置

```
make ARCH=arm64 menuconfig
```

- 修改后，可以先看看修改后和修改前的差异

```
diff .config arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig
```

- 把新配置覆盖`hobot_x5_rdk_ubuntu_defconfig`

```
cp .config arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig
```

- 清理源码目录下的 .config 等文件，否则在重新编译系统时会提示需要 xxx is not clean, please run 'make mrproper'

```
make distclean
# 或者
make mrproper
```

## 修改内核dts

本节介绍 RDK 产品的 DTS（Device Tree Source）文件结构，以及如何进行修改，以指导用户开展二次开发。

内核dts文件位于`x5-rdk-gen/source/kernel/arch/arm64/boot/dts/hobot/`目录下。

:::info 注意
- DTS 文件用于描述硬件资源分配（如 GPIO、I²C、SPI、中断号、内存地址等）。错误修改可能导致设备无法启动、驱动加载失败，甚至造成硬件损坏。
- 错误配置可能导致编译失败、系统无法正常引导，甚至需要重新刷写系统镜像恢复。
:::

### Kernel 设备树

| dts文件 | 作用 |
| ------- | ---------- |
| pinmux-func.dtsi | 定义各个 PIN 的功能组。 |
| pinmux-gpio.dtsi | 定义 GPIO 组。 |
| x5.dtsi | 定义 X5 平台上所有模块的默认配置，作为底层功能描述，不建议用户修改里面配置。 |
| x5-rdk.dtsi | 定义了不同 SDK 硬件版本间的通用配置项，包含 x5.dtsi 的默认配置。 |
| x5-板级产品.dts | 板级产品设备树文件，包含 x5.dtsi 的默认配置，同时完成功能模块的启用 / 禁用和与硬件强相关外设的定义。 |

### 调用关系

`x5-板级产品.dts` -> `x5-rdk.dtsi` -> `x5.dtsi` -> `(pinmux-func.dtsi and pinmux-gpio.dtsi)`

### 板级dts介绍

硬件编号可以通过板卡上的丝印确认，也可以通过`hrut_boardid`命令获取`board_id`来辨认。

| 硬件编号 | board_id | dts文件 |
| ------- | ---------- | ----------- |
| RDK X5 V0.1 | 0x301 | x5-rdk.dts |
| RDK X5 V1.0 | 0x302 | x5-rdk-v1p0.dts |
| RDK X5 MD V0.1 | 0x501 | x5-md-v0p1.dts |
| RDK X5 MD V0.2 | 0x502 | x5-md-v0p2.dts |
| RDK X5 MD V0.3 | 0x503 | x5-md-v0p2.dts |
| RDK X5 MD V1.0 | 0x504 | x5-md-v0p2.dts |
| RDK X5 MD V1.1 | 0x505 | x5-md-v0p2.dts |

### 编译

`x5-rdk-gen`目录下编译脚本为`mk_kernel.sh`，执行该脚本即可编译内核。

```
sudo ./mk_kernel.sh
```

编译完成后，会在`x5-rdk-gen/deploy/kernel/dtb/`目录下生成dtb文件，可以拷贝到板端的`/boot/hobot/`目录下，系统启动时会自动加载。

也可以执行以下命令把内核dtb文件打包成deb包，供系统安装使用。

```
sudo ./mk_debs.sh hobot-dtb
```

生成的deb包位于`x5-rdk-gen/deploy/deb_pkgs/hobot-dtb_version-data_arm64.deb`