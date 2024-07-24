---
sidebar_position: 4
---

# 7.2.4 内核头文件

如果你需要在开发板上编译内核模块或类似的代码，你需要安装 Linux 内核头文件。这些头文件包含Linux内核的各种常量定义、宏定义、函数接口定义和数据结构定义，是完成内核模块代码编译所必须的依赖代码。

## 安装

你可以通过以下命令安装内核头文件。

```bash
sudo apt install hobot-kernel-headers
```
命令运行成功后，内核头文件会被安装到`/usr/src/linux-headers-4.14.87`目录下
```bash
root@ubuntu:~# ls /usr/src/linux-headers-4.14.87/
arch   certs   Documentation  firmware  include  ipc      kernel  Makefile  Module.symvers  samples  security  System.map  usr
block  crypto  drivers        fs        init     Kconfig  lib     mm        net             scripts  sound     tools       virt
```

## 使用示例

我们用一个简单的 `Hello World` 内核模块的开发介绍如果使用内核头文件。步骤概要如下：

- 准备程序代码
- 编写Makefile，完成驱动模块的编译
- 对驱动模块进行签名
- 测试加载、卸载模块
- （可选）配置开机自动加载

### 编写Hello World程序
打开你熟悉的编辑器（比如VIM），创建文件 `hello.c`，输入下面的内容：
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
    printk(KERN_EMERG "Goodbye, World!\n");
}

module_init(hello_init);
module_exit(hello_exit);
```
模块加载时打印`Hello, World!`, 模块卸载时打印`Goodbye, World!`。

### 编写Makefile
打开你熟悉的编辑器（比如VIM），创建文件 `Makefile`，输入下面的内容：
```c
ifneq ($(KERNELRELEASE),)
    obj-m := hello.o
else
    PWD=$(shell pwd)
    KDIR := /usr/src/linux-headers-4.14.87

all:
    make -C $(KDIR) M=$(PWD) modules
clean:
    rm -rf *.ko *.o *.mod.o *.mod.c *.symvers  modul* .*.ko.cmd .*.o.cmd .tmp_versions
endif
```
- `PWD`指定源码路径，即hello.c的路径。
- `KDIR`指定内核源码路径。
- `KERNELRELEASE`是在内核源码的顶层Makefile里定义的变量。

保存`Makefile`后，执行`make`命令完成模块的编译，生成`hello.ko`文件。
```bash
root@ubuntu:~# make 
make  -C /usr/src/linux-headers-4.14.87 M=/root modules
make[1]: Entering directory '/usr/src/linux-headers-4.14.87'
  CC [M]  /root/hello.o
  Building modules, stage 2.
  MODPOST 1 modules
  CC      /root/hello.mod.o
  LD [M]  /root/hello.ko
make[1]: Leaving directory '/usr/src/linux-headers-4.14.87'
```

### 模块签名
编译好的驱动模块文件，需要进行签名后才能加载到RDK X3的内核里，命令如下：
```bash
root@ubuntu:~# hobot-sign-file hello.ko
Sign Kernel Module File Done.
```
如果不对驱动模块文件签名而直接加载，则会报以下错误:
```
insmod: ERROR: could not insert module hello.ko: Required key not available
```

### 加载模块

加载ko：`insmod hello.ko`
```bash
root@ubuntu:~# insmod hello.ko
[ 3104.480703] Hello, World!
```
卸载ko：`rmmod hello`
```bash
root@ubuntu:~# rmmod hello 
[ 3136.909409] Goodbye, World!
```

查看ko是否加载：`lsmod | grep hello`
```bash
root@ubuntu:~# lsmod | grep hello
hello                  16384  0
```

执行命令`dmesg`查看内核打印信息如下：
```bash
[ 3104.480361] hello: loading out-of-tree module taints kernel.
[ 3104.480703] Hello, World!
[ 3136.909409] Goodbye, World!
```

### 配置开机自动加载

如果想要自定义的驱动模块在开机时自动加载，请按照以下步骤进行配置：

拷贝`hello.ko`到 `/lib/modules/4.14.87` 目录，命令如下：
```bash
sudo cp -f hello.ko /lib/modules/4.14.87/
```
执行`depmod`命令更新模块的依赖关系：
```bash
sudo depmod
```
最后在 `/lib/modules-load.d` 目录下新建一个`conf`扩展名的配置文件，例如 `hello.conf`,在配置文件里添加需要自动加载的模块名（模块名不需要`.ko` 扩展名），例如需要自动加载`hello.ko`,就写一行`hello`，如果有多个模块需要加载，一个配置文件可以添加多个自加载模块，一行一个模块名，可以通过以下命令简便的完成配置文件的新建和配置：
```bash
sudo echo hello > /lib/modules-load.d/hello.conf
```
