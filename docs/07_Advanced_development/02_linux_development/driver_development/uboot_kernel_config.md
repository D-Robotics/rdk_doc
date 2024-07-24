---
sidebar_position: 1
---

# 配置uboot和kernel选项参数

在系统软件开发中，经常需要对uboot和Kernel的功能选项进行配置，本章节介绍几个常用的配置方法，供用户参考使用。

## 配置uboot选项参数

:::info 注意

​	以下说明以修改 `xj3_ubuntu_nand_defconfig`配置文件为例。

​	Uboot具体使用的配置文件可以在`./xbuild.sh lunch`之后查看`bootloader/device/.board_config.mk`板级配置文件中 `HR_UBOOT_CONFIG_FILE`的变量值。

:::

首先进入`uboot`目录，执行`make ARCH=arm64 xj3_ubuntu_nand_defconfig `。因为`make`命令将首先执行顶级目录下的 Makefile 文件。其中对于以config结尾的目标都有一个共同的入口：

```makefile
%config: scripts_basic outputmakefile FORCE
        $(Q)$(MAKE) $(build)=scripts/kconfig $@
```

展开后的执行命令是：

```
make -f ./scripts/Makefile.build obj=scripts/kconfig xj3_ubuntu_nand_defconfig
```

本命令执行后会在`uboot`的源码根目录下会生成 `.config`的文件。

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

然后就可以执行`make ARCH=arm64 menuconfig`打开图形化的配置界面进行uboot的选项参数配置。

![image-20220518111319607](./image/driver_develop_guide/image-20220518111319607.png)

在menuconfig的配置界面上完成配置后，选择 `Exit`退出，根据提示选择 `Yes` 或者`No`保存修改到`.config`文件中。

![image-20220518111506018](./image/driver_develop_guide/image-20220518111506018.png)

保存配置后，可以执行命令 `diff .config configs/xj3_ubuntu_nand_defconfig` 对比一下差异，再次确认一下修改是否符合预期。

如果修改正确，请执行 `cp .config configs/xj3_ubuntu_nand_defconfig`替换默认的配置文件。

## 配置kernel选项参数

:::info 注意

​	以下说明以修改 `xj3_perf_ubuntu_defconfig`配置文件为例。

​	kernel具体使用的配置文件可以查看 `mk_kernel.sh` 脚本中 `kernel_config_file` 的变量值。

:::

通过`menuconfig`方式配置`kernel`与配置`uboot`的的过程是一样的。命令执行过程如下：

首先进入`boot/kernel`目录，然后按照以下步骤配置`kernel`选项。

- 使用`xj3_perf_ubuntu_defconfig`来配置生成`.config`，如果源码做过全量编译，则`.config`文件会配置好

```
make ARCH=arm64 xj3_perf_ubuntu_defconfig
```

- 执行以下命令来修改配置

```
make ARCH=arm64 menuconfig
```

- 修改后，可以先看看修改后和修改前的差异

```
diff .config arch/arm64/configs/xj3_perf_ubuntu_defconfig
```

- 把新配置覆盖`xj3_perf_ubuntu_defconfig`

```
cp .config arch/arm64/configs/xj3_perf_ubuntu_defconfig
```

