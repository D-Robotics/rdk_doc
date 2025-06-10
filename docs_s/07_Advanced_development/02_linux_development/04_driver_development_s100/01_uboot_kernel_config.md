---
sidebar_position: 1
---

# 配置U-Boot和Kernel选项参数

在系统软件开发中，经常需要对u-boot和kernel的功能选项进行配置，本章节介绍几个常用的配置方法，供用户参考使用。

## 配置uboot选项参数

:::info 注意

​	以下说明以修改 `hobot_s100_defconfig`配置文件为例。

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

首先进入`source/bootloader/uboot`目录，执行`make ARCH=arm64 hobot_s100_defconfig `。因为`make`命令将首先执行顶级目录下的 Makefile 文件。其中对于以config结尾的目标都有一个共同的入口：

```makefile
%config: scripts_basic outputmakefile FORCE
        $(Q)$(MAKE) $(build)=scripts/kconfig $@
```

展开后的执行命令是：

```
make -f ./scripts/Makefile.build obj=scripts/kconfig hobot_s100_defconfig
```

本命令执行后会在`uboot`的源码根目录下会生成 `.config`的文件。

```bash
make ARCH=arm64 hobot_s100_defconfig

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

然后就可以执行`make ARCH=arm64 menuconfig`打开图形化的配置界面进行`uboot`的选项参数配置。

在menuconfig的配置界面上完成配置后，选择 `Exit`退出，根据提示选择 `Yes` 或者`No`保存修改到`.config`文件中。

保存配置后，可以执行命令 `diff .config configs/hobot_s100_defconfig` 对比一下差异，再次确认一下修改是否符合预期。

如果修改正确，请执行 `cp .config configs/hobot_s100_defconfig`替换默认的配置文件。

清理源码目录下的 .config 等文件，否则在重新编译系统时会提示需要 xxx is not clean, please run 'make mrproper'
```bash
make distclean
# 或者
make mrproper
```

## 配置Kernel选项参数

:::info 注意

​	以下说明以修改 `drobot_s100_defconfig`配置文件为例。

​	kernel具体使用的配置文件可以查看 `mk_kernel.sh` 脚本中 `kernel_config_file` 的变量值。

:::

### 通过 mk_kernel 命令配置

`mk_kernel.sh`提供了以下命令帮助用户进行 Kernel 的选项配置，该命令会自动使用板级配置文件中设置的 Kernel 配置文件，在配置完成后，自动完成savedefconfig 和保存工作。

```
./mk_kernel.sh menuconfig
```

命令执行成功后会打开 Kernel 图形化配置界面，您可以在这个交互界面下完成选项的配置，包括删除不需要的功能，启用需要的功能。

![image-20220518111319607](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/image-s100-kernel.png)

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

- 使用`drobot_s100_defconfig`来配置生成`.config`，如果源码做过全量编译，则`.config`文件会配置好

```
make ARCH=arm64 drobot_s100_defconfig
```

- 执行以下命令来修改配置

```
make ARCH=arm64 menuconfig
```

- 修改后，可以先看看修改后和修改前的差异

```
diff .config hobot-drivers/configs/drobot_s100_defconfig
```

- 把新配置覆盖`drobot_s100_defconfig`

```
cp .config hobot-drivers/configs/drobot_s100_defconfig
```

- 清理源码目录下的 .config 等文件，否则在重新编译系统时会提示需要 xxx is not clean, please run 'make mrproper'

```
make distclean
# 或者
make mrproper
```

