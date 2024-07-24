---
sidebar_position: 4
---
# 7.2.4 Kernel Header Files

If you need to compile kernel modules or similar code on a development board, you need to install the Linux kernel header files. These header files contain various constant definitions, macro definitions, function interface definitions, and data structure definitions of the Linux kernel. They are essential dependencies for compiling kernel module code.

## Installation

You can install the kernel header files using the following command.

```bash
sudo apt install hobot-kernel-headers
```

After the command runs successfully, the kernel header files will be installed in the `/usr/src/linux-headers-4.14.87` directory.

```bash
root@ubuntu:~# ls /usr/src/linux-headers-4.14.87/
arch   certs   Documentation  firmware  include  ipc      kernel  Makefile  Module.symvers  samples  security  System.map  usr
block  crypto  drivers        fs        init     Kconfig  lib     mm        net             scripts  sound     tools       virt
```

## Usage Example

We will use a simple `Hello World` kernel module to demonstrate how to use the kernel header files. The steps are outlined as follows:

- Prepare program code
- Write Makefile to compile the driver module
- Sign the driver module
- Test loading and unloading the module
- (Optional) Configure automatic loading at startup

### Write Hello World Program

Open your favorite editor (e.g. VIM) and create a file `hello.c` with the following content:

```c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xxx.xxx");
MODULE_DESCRIPTION("Hello World");

static int __init hello_init(void)
{
    printk(KERN_ERR "Hello, World!\n");
    return 0;
}
static void __exit hello_exit(void)
{
```printk(KERN_EMERG "Goodbye, World!\n");
}

module_init(hello_init);
module_exit(hello_exit);
```
The module prints "Hello, World!" when it is loaded, and prints "Goodbye, World!" when it is unloaded.

### Writing the Makefile
Open your favorite editor (such as VIM) and create a file called `Makefile`. Then input the following content:
```c
ifneq ($(KERNELRELEASE),)
    obj-m := hello.o
else
    PWD = $(shell pwd)
    KDIR := /usr/src/linux-headers-4.14.87

all:
    make -C $(KDIR) M=$(PWD) modules
clean:
    rm -rf *.ko *.o *.mod.o *.mod.c *.symvers  modul* .*.ko.cmd .*.o.cmd .tmp_versions
endif
```
- `PWD` specifies the source code path, i.e., the path of hello.c.
- `KDIR` specifies the path of the kernel source code.
- `KERNELRELEASE` is a variable defined in the top-level Makefile of the kernel source code.

Save the `Makefile` and then execute the `make` command to compile the module and generate the `hello.ko` file.
```bash
root@ubuntu:~# make 
make -C /usr/src/linux-headers-4.14.87 M=/root modules
make[1]: Entering directory '/usr/src/linux-headers-4.14.87'
  CC [M]  /root/hello.o
  Building modules, stage 2.
  MODPOST 1 modules
  CC      /root/hello.mod.o
  LD [M]  /root/hello.ko
make[1]: Leaving directory '/usr/src/linux-headers-4.14.87'
```

### Module Signing
After compiling the driver module file, it needs to be signed before it can be loaded into the RDK X3 kernel. The command is as follows:
```bash
root@ubuntu:~# hobot-sign-file hello.ko
Sign Kernel Module File Done.
```
If the driver module file is not signed and loaded directly, the following error will occur:
```
insmod: ERROR: could not insert module hello.ko: Required key not available
```

### Load Module

Load ko: `insmod hello.ko`
```bash
root@ubuntu:~# insmod hello.ko
[ 3104.480703] Hello, World!
```
Unload ko: `rmmod hello`
```bash
root@ubuntu:~# rmmod hello 
[ 3136.909409] Goodbye, World!
```

Check if ko is loaded: `lsmod | grep hello`
```bash
root@ubuntu:~# lsmod | grep hello
hello                  16384  0
```

Execute command `dmesg` to view kernel print information as follows:
```bash
[ 3104.480361] hello: loading out-of-tree module taints kernel.
[ 3104.480703] Hello, World!
[ 3136.909409] Goodbye, World!
```

### Configure Automatic Loading at Startup

To automatically load a custom driver module at startup, follow these steps:

Copy `hello.ko` to the directory `/lib/modules/4.14.87`, the command is as follows:
```bash
sudo cp -f hello.ko /lib/modules/4.14.87/
```
Execute the command `depmod` to update the module dependency:
```bash
sudo depmod
```
Finally, create a configuration file with `conf` extension in the directory `/lib/modules-load.d`, for example, `hello.conf`, and add the module name to be automatically loaded (the module name does not need the `.ko` extension). For example, if you need to automatically load `hello.ko`, write the line `hello`. If there are multiple modules to be loaded, you can add multiple self-loading modules in one configuration file, one module per line. You can use the following command to quickly create and configure the configuration file:
```bash
sudo echo hello > /lib/modules-load.d/hello.conf
```