---
sidebar_position: 2
---

# 7.2.2 内核头文件

如果你需要在开发板上编译内核模块或类似的代码，你需要安装 Linux 内核头文件。这些头文件包含Linux内核的各种常量定义、宏定义、函数接口定义和数据结构定义，是完成内核模块代码编译所必须的依赖代码。

## 安装

你可以通过以下命令安装内核头文件及内核编译依赖。

```bash
sudo apt update
sudo apt install linux-headers-6.1.112-rt43
sudo apt install bison flex
```
命令运行成功后，内核头文件会被安装到`/usr/src`目录下
```bash
sunrise@ubuntu:~$ ls /usr/src/linux-headers-$(uname -r)
Documentation  Makefile        arch   crypto   include   ipc     mm    samples   sound  virt
Kbuild         Module.symvers  block  drivers  init      kernel  net   scripts   tools
Kconfig        System.map      certs  fs       io_uring  lib     rust  security  usr
```

:::warning
请**不要**在`/usr/src/linux-headers-$(uname -r)`目录下执行`make clean`命令，这会将板端内核模块的编译环境破坏；
如果执行了`make clean`动作，请重新安装`linux-headers-6.1.112-rt43`包进行环境恢复。
:::

## 使用示例
### Hello World内核模块
我们用一个简单的 `Hello World` 内核模块的开发介绍如果使用内核头文件。步骤概要如下：

- 准备程序代码，以编译路径为`sunrise`用户的`${HOME}/test_ko`为例
- 编写Makefile，完成驱动模块的编译
- 对驱动模块进行签名
- 测试加载、卸载模块
- （可选）配置开机自动加载

#### 编写Hello World内核模块
打开你熟悉的编辑器（比如VIM），创建文件 `hello.c`，输入下面的内容：
```c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple Hello World kernel module");

static int __init hello_init(void)
{
	printk(KERN_INFO "Hello, World! Kernel module loaded.\n");
	return 0;
}

static void __exit hello_exit(void)
{
	printk(KERN_INFO "Goodbye, World! Kernel module unloaded.\n");
}

module_init(hello_init);
module_exit(hello_exit);

```
模块加载时打印`Hello, World!`, 模块卸载时打印`Goodbye, World!`。

#### 编写Makefile
打开你熟悉的编辑器（比如VIM），创建文件 `Makefile`，输入下面的内容：
```makefile
ifneq ($(KERNELRELEASE),)
obj-m := hello.o
else
KERN_VER ?= $(shell uname -r)

KERN_DIR ?= /lib/modules/$(KERN_VER)/build

PWD := $(shell pwd)

.PHONY := all prepare clean

all: prepare
$(MAKE) -C $(KERN_DIR) M=$(PWD) modules
	$(MAKE) -C $(KERN_DIR) KERNELRELEASE=$(KERN_VER) M=$(PWD) modules_install

prepare:
	$(MAKE) -C $(KERN_DIR) prepare

clean:
	$(MAKE) -C $(KERN_DIR) M=$(PWD) clean

endif
```
- `PWD`指定源码路径，即hello.c的路径；
- `KERN_DIR`指定内核源码路径；
- `KERNELRELEASE`是在内核源码的顶层Makefile里定义的变量，一般用于判断当前编译是否是在`Kbuild`框架下；
- 调用内核原生的`prepare`目标，来将内核模块编译和安装所需的工具准备好；
- 调用内核原生的`modules`目标，来将内核模块编译出来；
- 调用内核原生的`modules_install`目标，来将编译好的内核模块签名并安装到系统模块目录。

保存`Makefile`后，执行`make`命令完成模块的编译，生成`hello.ko`文件。
```bash
sunrise@ubuntu:~/test_ko$ sudo make
make -C /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8/build prepare
make[1]: Entering directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
  SYNC    include/config/auto.conf.cmd
  HOSTCC  scripts/basic/fixdep
  HOSTCC  scripts/kconfig/conf.o
  HOSTCC  scripts/kconfig/confdata.o
  HOSTCC  scripts/kconfig/expr.o
  LEX     scripts/kconfig/lexer.lex.c
  YACC    scripts/kconfig/parser.tab.[ch]
  HOSTCC  scripts/kconfig/lexer.lex.o
  HOSTCC  scripts/kconfig/menu.o
  HOSTCC  scripts/kconfig/parser.tab.o
  HOSTCC  scripts/kconfig/preprocess.o
  HOSTCC  scripts/kconfig/symbol.o
  HOSTCC  scripts/kconfig/util.o
  HOSTLD  scripts/kconfig/conf
  HOSTCC  scripts/dtc/dtc.o
  HOSTCC  scripts/dtc/flattree.o
  HOSTCC  scripts/dtc/fstree.o
  HOSTCC  scripts/dtc/data.o
  HOSTCC  scripts/dtc/livetree.o
  HOSTCC  scripts/dtc/treesource.o
  HOSTCC  scripts/dtc/srcpos.o
  HOSTCC  scripts/dtc/checks.o
  HOSTCC  scripts/dtc/util.o
  LEX     scripts/dtc/dtc-lexer.lex.c
  YACC    scripts/dtc/dtc-parser.tab.[ch]
  HOSTCC  scripts/dtc/dtc-lexer.lex.o
  HOSTCC  scripts/dtc/dtc-parser.tab.o
  HOSTLD  scripts/dtc/dtc
  HOSTCC  scripts/dtc/libfdt/fdt.o
  HOSTCC  scripts/dtc/libfdt/fdt_ro.o
  HOSTCC  scripts/dtc/libfdt/fdt_wip.o
  HOSTCC  scripts/dtc/libfdt/fdt_sw.o
  HOSTCC  scripts/dtc/libfdt/fdt_rw.o
  HOSTCC  scripts/dtc/libfdt/fdt_strerror.o
  HOSTCC  scripts/dtc/libfdt/fdt_empty_tree.o
  HOSTCC  scripts/dtc/libfdt/fdt_addresses.o
  HOSTCC  scripts/dtc/libfdt/fdt_overlay.o
  HOSTCC  scripts/dtc/fdtoverlay.o
  HOSTLD  scripts/dtc/fdtoverlay
  HOSTCC  scripts/kallsyms
  HOSTCC  scripts/sorttable
  HOSTCC  scripts/asn1_compiler
  HOSTCC  scripts/sign-file
  UPD     include/config/kernel.release
  UPD     include/generated/utsrelease.h
  UPD     include/generated/compile.h
  GEN     arch/arm64/include/generated/asm/cpucaps.h
  GEN     arch/arm64/include/generated/asm/sysreg-defs.h
  CC      scripts/mod/empty.o
  HOSTCC  scripts/mod/mk_elfconfig
  MKELF   scripts/mod/elfconfig.h
  HOSTCC  scripts/mod/modpost.o
  CC      scripts/mod/devicetable-offsets.s
  UPD     scripts/mod/devicetable-offsets.h
  HOSTCC  scripts/mod/file2alias.o
  HOSTCC  scripts/mod/sumversion.o
  HOSTLD  scripts/mod/modpost
  CALL    scripts/checksyscalls.sh
  CHKSHA1 include/linux/atomic/atomic-arch-fallback.h
  CHKSHA1 include/linux/atomic/atomic-instrumented.h
  CHKSHA1 include/linux/atomic/atomic-long.h
  LD      arch/arm64/kernel/vdso/vdso.so.dbg
  VDSOSYM include/generated/vdso-offsets.h
  OBJCOPY arch/arm64/kernel/vdso/vdso.so
make[1]: Leaving directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
make -C /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8/build M=/home/sunrise/test_ko modules
make[1]: Entering directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
  CC [M]  /home/sunrise/test_ko/hello.o
  MODPOST /home/sunrise/test_ko/Module.symvers
  CC [M]  /home/sunrise/test_ko/hello.mod.o
  LD [M]  /home/sunrise/test_ko/hello.ko
make[1]: Leaving directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
make -C /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8/build KERNELRELEASE=6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8 M=/home/sunrise/test_ko modules_install
make[1]: Entering directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
  INSTALL /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8/extra/hello.ko
  SIGN    /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8/extra/hello.ko
  DEPMOD  /lib/modules/6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8
make[1]: Leaving directory '/usr/src/linux-headers-6.1.112-rt43-DR-4.0.2-2507191817-g7253c2-g30e6e8'
```

:::info
- 上方打印中的内核版本会随版本变化而变化；
- 更多关于内核模块编译的说明，请参考：[Linux Kernel Doc | Building External Modules](https://kernel.org/doc/html/v6.1/kbuild/modules.html)。
:::

#### 模块签名
编译好的驱动模块文件，已经通过modules_install自动签名，如果想要手动签名，请执行以下命令：
```bash
# Create a singining script.
sunrise@ubuntu:~/test_ko#
cat << EOF > drobot-sign-file.sh
#!/bin/bash

signing_ko=\$1
output_f=\$2

if [ ! -f "\${signing_ko}" ];then
	echo "\${signing_ko} not found!"
	exit 1
fi

module_build_dir=/lib/modules/\$(uname -r)/build

signing_algo=\$(cat \${module_build_dir}/.config | grep -w CONFIG_MODULE_SIG_HASH | awk -F '=' '{print \$2}' | tr -d '"')
signing_key_name=\$(cat \${module_build_dir}/.config | grep -w CONFIG_MODULE_SIG_KEY | awk -F '=' '{print \$2}' | tr -d '"')

signing_key_f=\${module_build_dir}/\${signing_key_name}
signing_key_x509_f=\${module_build_dir}/certs/signing_key.x509

if [ -z "\${output_f}" ];then
	echo -n "Saving signed module to \${signing_ko} ..."
else
	echo -n "Saving signed module to \${output_f} ..."
fi

\${module_build_dir}/scripts/sign-file \${signing_algo} \${signing_key_f} \${signing_key_x509_f} \${signing_ko} \${output_f}

echo "Done!"
EOF

sunrise@ubuntu:~/test_ko# chmod +x ./drobot-sign-file.sh
sunrise@ubuntu:~/test_ko# ./drobot-sign-file.sh hello.ko
Saving signed module to hello.ko ...Done!
```
如果不对驱动模块文件签名而直接加载，则会报以下错误:
```
ERROR: could not insert module hello.ko: Required key not available
```

#### 加载模块

加载ko：`modprobe hello`
```bash
sunrise@ubuntu:~/test_ko# sudo modprobe hello
```

查看ko是否加载：`lsmod | grep hello`
```bash
sunrise@ubuntu:~/test_ko# lsmod | grep hello
hello                  262144  0
```

卸载ko：`modprobe -r hello`
```bash
sunrise@ubuntu:~/test_ko# sudo modprobe -r  hello
[ 3136.909409] Goodbye, World!
```

执行命令`dmesg`查看内核打印信息如下：
```bash
sunrise@ubuntu:~/test_ko$ dmesg | tail -n 2
[  525.710661] Hello, World! Kernel module loaded.
[  612.716435] Goodbye, World! Kernel module unloaded.
```

#### 配置开机自动加载

在 `/lib/modules-load.d` 目录下新建一个`conf`扩展名的配置文件，例如 `hello.conf`,在配置文件里添加需要自动加载的模块名（模块名不需要`.ko` 扩展名），例如需要自动加载`hello.ko`,就写一行`hello`，如果有多个模块需要加载，一个配置文件可以添加多个自加载模块，一行一个模块名，可以通过以下命令简便的完成配置文件的新建和配置：
```bash
sudo echo hello > /lib/modules-load.d/hello.conf
```

### 第三方内核驱动模块编译示例
:::info
**免责声明**：

- 地瓜不对任何第三方驱动模块承担任何形式的开发/维护责任。

- 地瓜不对任何由于第三方源码在地瓜平台运行所导致的问题负责。
:::

#### PCAN
1. 请确认您的RDK开发板已正确连入互联网，能够访问PCAN官网，例如使用`ping peak-system.com`命令，示例输入如下：
    ```shell
    sunrise@ubuntu:~$ ping peak-system.com -c 5
    PING peak-system.com (37.202.3.222) 56(84) bytes of data.
    64 bytes from r2096.agenturserver.it (37.202.3.222): icmp_seq=1 ttl=45 time=280 ms
    64 bytes from r2096.agenturserver.it (37.202.3.222): icmp_seq=3 ttl=45 time=279 ms
    64 bytes from r2096.agenturserver.it (37.202.3.222): icmp_seq=4 ttl=45 time=278 ms
    64 bytes from r2096.agenturserver.it (37.202.3.222): icmp_seq=5 ttl=45 time=277 ms

    --- peak-system.com ping statistics ---
    5 packets transmitted, 4 received, 20% packet loss, time 4024ms
    rtt min/avg/max/mdev = 277.180/278.460/280.280/1.152 ms
    ```
2. 根据`peak-system`官方说明，下载`pcan-kernel-version.sh`脚本，并确认需要下载的驱动版本：
    ```shell
    # Download pcan-kernel-version.sh
    wget https://www.peak-system.com/fileadmin/media/linux/files/pcan-kernel-version.sh.tar.gz
    # Extract script
    tar -xzf pcan-kernel-version.sh.tar.gz
    # Run the script
    ./pcan-kernel-version.sh
    ```
    :::info
    地瓜RDK Super系列仅支持Linux6.1版本内核。
    :::
3. 下载PCAN驱动
    ```shell
    # Depending on the actual internet environment, the download may take minutes to hours
    wget --content-disposition https://www.peak-system.com/quick/PCAN-Linux-Driver
    ```
    - `--content-disposition`选项：将保存到本地的文件名改为与链接指向的服务器一致；
4. 解压PCAN驱动压缩包，这里我们以2025-08-16的PCAN驱动版本“8.20.0”为例，请用户根据下载的实际文件修改命令
    ```shell
    tar -xzf peak-linux-driver-8.20.0.tar.gz
    ```
5. 执行编译：
    :::info
    请确保您板端的linux-headers-6.1.112-rt43版本为4.0.3或以上
    :::

    ```shell
    # Install local module build prerequisites
    sudo apt install flex bison -y

    # Install PCAN driver prerequisites
    sudo apt install libpopt.h -y

    # Setup local module build environment
    sudo make -C /usr/src/linux-headers-$(uname -r) prepare

    # Build PCAN Driver modules
    cd peak-linux-driver-8.20.0
    make

    # Install PCAN drivers and libraries
    sudo make install
    ```
6. 对PCAN驱动进行签名并使用，以下示例以8.20.0版本的PCAN驱动为例，该版本下，PCAN驱动模块被默认安装到了路径`/lib/modules/6.1.112/misc/`内：
     1. 参考[模块签名](#模块签名)章节，创建脚本；
     2. 执行签名命令：
        ```shell
        # Sign file
        sudo bash drobot-sign-file.sh /lib/modules/6.1.112/misc/pcan.ko
        ```
     3. 使用PCAN驱动：
        ```shell
        # Insert PCAN driver
        sudo insmod /lib/modules/6.1.112/misc/pcan.ko
        ```
