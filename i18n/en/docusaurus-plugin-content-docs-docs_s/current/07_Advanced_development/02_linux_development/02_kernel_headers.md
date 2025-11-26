---
sidebar_position: 2
---

# 7.2.2 Kernel Headers

If you need to compile kernel modules or similar code on your development board, you must install the Linux kernel headers. These header files contain various constant definitions, macro definitions, function interface declarations, and data structure definitions from the Linux kernel, and are essential dependencies required to compile kernel module code.

## Installation

You can install the kernel headers and kernel compilation dependencies using the following commands:

```bash
sudo apt update
sudo apt install linux-headers-6.1.112-rt43
sudo apt install bison flex
```

After successful execution, the kernel headers will be installed under the `/usr/src` directory:

```bash
sunrise@ubuntu:~$ ls /usr/src/linux-headers-$(uname -r)
Documentation  Makefile        arch   crypto   include   ipc     mm    samples   sound  virt
Kbuild         Module.symvers  block  drivers  init      kernel  net   scripts   tools
Kconfig        System.map      certs  fs       io_uring  lib     rust  security  usr
```

:::warning
**Do not** run the `make clean` command inside the `/usr/src/linux-headers-$(uname -r)` directory, as this will break the kernel module compilation environment on your board.  
If you have already executed `make clean`, please reinstall the `linux-headers-6.1.112-rt43` package to restore the environment.
:::

## Usage Example
### Hello World Kernel Module

We'll demonstrate how to use kernel headers by developing a simple `Hello World` kernel module. The general steps are as follows:

- Prepare the source code (using `${HOME}/test_ko` under the `sunrise` user as an example)
- Write a Makefile to compile the driver module
- Sign the driver module
- Test loading and unloading the module
- (Optional) Configure automatic module loading at boot

#### Writing the Hello World Kernel Module

Open your preferred editor (e.g., VIM) and create a file named `hello.c` with the following content:

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

This module prints `Hello, World!` when loaded and `Goodbye, World!` when unloaded.

#### Writing the Makefile

Open your preferred editor (e.g., VIM) and create a file named `Makefile` with the following content:

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

- `PWD` specifies the source code path (i.e., the location of `hello.c`);
- `KERN_DIR` specifies the kernel source path;
- `KERNELRELEASE` is a variable defined in the top-level kernel Makefile, typically used to determine whether the current build is running under the `Kbuild` framework;
- Invoke the kernel's native `prepare` target to set up the tools required for compiling and installing kernel modules;
- Invoke the kernel's native `modules` target to compile the kernel module;
- Invoke the kernel's native `modules_install` target to sign and install the compiled kernel module into the system's module directory.

After saving the `Makefile`, run the `make` command to compile the module and generate the `hello.ko` file:

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
- The kernel version shown in the output above will vary depending on your system version;
- For more details on compiling external kernel modules, please refer to: [Linux Kernel Doc | Building External Modules](https://kernel.org/doc/html/v6.1/kbuild/modules.html).
:::

#### Module Signing

The compiled driver module has already been automatically signed via `modules_install`. If you wish to sign it manually, execute the following commands:

```bash
# Create a signing script.
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
If you load the driver module without signing it, the following error will occur:
```
ERROR: could not insert module hello.ko: Required key not available
```

#### Load the module

Load the ko: `modprobe hello`
```bash
sunrise@ubuntu:~/test_ko# sudo modprobe hello
```

Check if the ko is loaded: `lsmod | grep hello`
```bash
sunrise@ubuntu:~/test_ko# lsmod | grep hello
hello                  262144  0
```

Unload the ko: `modprobe -r hello`
```bash
sunrise@ubuntu:~/test_ko# sudo modprobe -r  hello
[ 3136.909409] Goodbye, World!
```

Run the command `dmesg` to view the kernel log messages as follows:
```bash
sunrise@ubuntu:~/test_ko$ dmesg | tail -n 2
[  525.710661] Hello, World! Kernel module loaded.
[  612.716435] Goodbye, World! Kernel module unloaded.
```

#### Configure automatic loading at boot

Create a new configuration file with the `.conf` extension under the `/lib/modules-load.d` directory, for example `hello.conf`. Add the name of the module you want to load automatically in this configuration file (without the `.ko` extension). For example, to automatically load `hello.ko`, add a line containing `hello`. If multiple modules need to be loaded, you can list each module on a separate line in a single configuration file. You can easily create and configure the file using the following command:
```bash
sudo echo hello > /lib/modules-load.d/hello.conf
```

### Third-party kernel driver module compilation example
:::info
**Disclaimer**:

- D-Robotics assumes no responsibility for the development/maintenance of any third-party driver modules.

- D-Robotics is not liable for any issues arising from running third-party source code on the D-Robotics platform.
:::

#### PCAN
1. Please ensure your RDK development board is properly connected to the internet and can access the PCAN official website. For example, use the command `ping peak-system.com`. Sample output is as follows:
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
2. According to the official `peak-system` instructions, download the `pcan-kernel-version.sh` script and confirm the driver version you need to download:
    ```shell
    # Download pcan-kernel-version.sh
    wget https://www.peak-system.com/fileadmin/media/linux/files/pcan-kernel-version.sh.tar.gz
    # Extract script
    tar -xzf pcan-kernel-version.sh.tar.gz
    # Run the script
    ./pcan-kernel-version.sh
    ```
    :::info
    D-Robotics RDK Super series only supports Linux kernel version 6.1.
    :::
3. Download the PCAN driver:
    ```shell
    # Depending on the actual internet environment, the download may take minutes to hours
    wget --content-disposition https://www.peak-system.com/quick/PCAN-Linux-Driver
    ```
    - The `--content-disposition` option: saves the file locally using the filename provided by the server;
4. Extract the PCAN driver archive. Here we use the PCAN driver version "8.20.0" released on 2025-08-16 as an example. Please adjust the command according to the actual file you downloaded:
    ```shell
    tar -xzf peak-linux-driver-8.20.0.tar.gz
    ```
5. Compile the driver:
    :::info
    Please ensure that the linux-headers-6.1.112-rt43 package on your board is version 4.0.3 or higher.
    :::

    ```shell
    # Install local module build prerequisites
    sudo apt install flex bison -y

    # Install PCAN driver prerequisites
    sudo apt install libpopt-dev -y

    # Setup local module build environment
    sudo make -C /usr/src/linux-headers-$(uname -r) prepare

    # Build PCAN Driver modules
    cd peak-linux-driver-8.20.0
    make

    # Install PCAN drivers and libraries
    sudo make install
    ```
6. Sign and use the PCAN driver. The following example uses PCAN driver version 8.20.0, where the driver modules are installed by default to `/lib/modules/6.1.112/misc/`:
     1. Create a signing script as described in the [Module Signing](#module-signing) section;
     2. Run the signing command:
        ```shell
        # Sign file
        sudo bash drobot-sign-file.sh /lib/modules/6.1.112/misc/pcan.ko
        ```
     3. Use the PCAN driver:
        ```shell
        # Insert PCAN driver
        sudo insmod /lib/modules/6.1.112/misc/pcan.ko
        ```