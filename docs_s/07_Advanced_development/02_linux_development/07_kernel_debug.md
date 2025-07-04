---
sidebar_position: 7
---

# 7.2.7 Linux调试功能介绍

## crash分析ramdump

### 抓取ramdump

当前ramdump功能默认为关闭状态，在Linux下可以通过工具`hrut_ddr_misc`手动开启:

```Shell
root@ubuntu:/userdata# hrut_ddr_misc s bit 0 1
update misc para begin
------------------------------------
print new misc para:
Bit idx  Function name   Status
0        RAMDUMP         on
------------------------------------
```

查看当前ramdump功能是否开启：

```Shell
root@ubuntu:/userdata# hrut_ddr_misc g
Bit idx  Function name   Status
0        RAMDUMP         on
```

**当前ramdump功能只支持抓取由Kernel panic触发的场景**

**ramdump的时候可能会损坏保存dump文件的分区，请务必将dump文件保存到非根文件系统分区，且分区容量大于DDR容量**

#### 自动抓取

暂不支持

#### 手动抓取

触发Kernel panic重启到U-Boot之后，在U-Boot下执行以下命令，数据存储到eMMC或者ufs的/map/目录。

```Shell
Hobot$ setenv enable_ramdump 1
Hobot$ setenv ramdump_part_name map
Hobot$ setenv ramdump_in map
Hobot$ memdump userdata
intf mmc,dev 0,part 17 directory /Recovery required
file found, deleting
update journal finished
File System is consistent
update journal finished
cpu core context dumped to //cpu-contexts.bin
DRAM bank= 0x0
-> start   = 0x0000000080000000
-> size    = 0x0000000080000000
-> dumpfile = //DDRCS0-0.bin, from memory 0x80000000, length=536870912
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
536870912 bytes written in 10723 ms
skip secure wolrd memory region
-> dumpfile = //DDRCS0-2.bin, from memory 0xaa000000, length=1442840576
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
1442840576 bytes written in 23719 ms
DRAM bank= 0x1
-> start   = 0x0000000400000000
-> size    = 0x00000000FFFFF000
-> dumpfile = //DDRCS1-0.bin, from memory 0x400000000, length=2147483648
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
2147483648 bytes written in 33548 ms
-> dumpfile = //DDRCS1-1.bin, from memory 0x480000000, length=2147479552
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
2147479552 bytes written in 33429 ms
DRAM bank= 0x2
-> start   = 0x0000000800000000
-> size    = 0x00000000FFFFF000
-> dumpfile = //DDRCS2-0.bin, from memory 0x800000000, length=2147483648
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
2147483648 bytes written in 33528 ms
-> dumpfile = //DDRCS2-1.bin, from memory 0x880000000, length=2147479552
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
2147479552 bytes written in 33656 ms
DRAM bank= 0x3
-> start   = 0x0000000C80000000
-> size    = 0x000000007FFFF000
-> dumpfile = //DDRCS3-0.bin, from memory 0xc80000000, length=2147479552
File System is consistent
file found, deleting
update journal finished
File System is consistent
update journal finished
2147479552 bytes written in 33372 ms
```

### crash介绍

crash主要是用来离线分析linux内核内存转存文件，它整合了gdb工具，具有很强的功能，可以查看堆栈，dmesg日志，内核数据结构，反汇编等等。其支持多种工具生成的内存转储文件格式，包括：

- Live linux系统。

- kdump产生的正常的和压缩的内存转储文件。

- 由makedumpfile命令生成的压缩的内存转储文件。

- 由netdump生成的内存转储文件。

- 由diskdump生成的内存转储文件。

- 由kdump生成的Xen的内存转储文件。

- LKCD生成的内存转储文件。

- Mcore生成的内存转储文件。

- ramdump格式的raw内存转储文件。

### crash使用方法

本文主要使用crash来分析ramdump文件。ramdump文件几乎是对整个内存的镜像，除了一些security类型的memory抓不出来之外，几乎所有的DRAM都能被抓下来。有些问题的复现概率低，而且有些问题是由于踩内存导致的，这种问题靠log往往是无法分析出来的，所以如果可以在问题发生时候把内存镜像保存下来，就可以分析了。

crash工具代码获取及编译方法如下：

```Shell
sudo apt install -y texinfo
git clone --depth=1 https://github.com/crash-utility/crash.git
make target=arm64
```

在服务器命令行下使用如下命令，分析抓取的ramdump文件。工具输出如下：

```Shell
./crash ./vmlinux DDRCS0-0.bin@0x80000000,/dev/zero@0xa0000000,DDRCS0-2.bin@0xaa000000,DDRCS1-0.bin@0x400000000,DDRCS1-1.bin@0x480000000,DDRCS2-0.bin@0x800000000,DDRCS2-1.bin@0x880000000,DDRCS3-0.bin@0xc80000000 --machdep vabits_actual=48

crash 9.0.0
Copyright (C) 2002-2025  Red Hat, Inc.
Copyright (C) 2004, 2005, 2006, 2010  IBM Corporation
Copyright (C) 1999-2006  Hewlett-Packard Co
Copyright (C) 2005, 2006, 2011, 2012  Fujitsu Limited
Copyright (C) 2006, 2007  VA Linux Systems Japan K.K.
Copyright (C) 2005, 2011, 2020-2024  NEC Corporation
Copyright (C) 1999, 2002, 2007  Silicon Graphics, Inc.
Copyright (C) 1999, 2000, 2001, 2002  Mission Critical Linux, Inc.
Copyright (C) 2015, 2021  VMware, Inc.
This program is free software, covered by the GNU General Public License,
and you are welcome to change it and/or distribute copies of it under
certain conditions.  Enter "help copying" to see the conditions.
This program has absolutely no warranty.  Enter "help warranty" for details.

NOTE: setting vabits_actual to: 48

GNU gdb (GDB) 16.2
Copyright (C) 2024 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=x86_64-pc-linux-gnu --target=aarch64-elf-linux".
Type "show configuration" for configuration details.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...

WARNING: kernel version inconsistency between vmlinux and dumpfile

WARNING: cpu 0: cannot find NT_PRSTATUS note
WARNING: cpu 1: cannot find NT_PRSTATUS note
WARNING: cpu 2: cannot find NT_PRSTATUS note
WARNING: cpu 3: cannot find NT_PRSTATUS note
WARNING: cpu 4: cannot find NT_PRSTATUS note
WARNING: cpu 5: cannot find NT_PRSTATUS note
      KERNEL: ./vmlinux
   DUMPFILES: /var/tmp/ramdump_elf_VSntZ8 [temporary ELF header]
              DDRCS0-0.bin
              /dev/zero
              DDRCS0-2.bin
              DDRCS1-0.bin
              DDRCS1-1.bin
              DDRCS2-0.bin
              DDRCS2-1.bin
              DDRCS3-0.bin
        CPUS: 6 [OFFLINE: 5]
        DATE: Thu Feb 20 21:24:21 CST 2025
      UPTIME: 00:00:20
LOAD AVERAGE: 2.67, 0.65, 0.22
       TASKS: 792
    NODENAME: ubuntu
     RELEASE: 6.1.112-rt43-DR-4.0.2-2506251650-gac1c88-g0b202b-dirty
     VERSION: #2 SMP PREEMPT_RT Wed Jun 25 16:57:38 CST 2025
     MACHINE: aarch64  (unknown Mhz)
      MEMORY: 12 GB
       PANIC: ""
         PID: 0
     COMMAND: "swapper/0"
        TASK: ffff80000968ec00  (1 of 6)  [THREAD_INFO: ffff80000968ec00]
         CPU: 0
       STATE: TASK_RUNNING (ACTIVE)
     WARNING: panic task not found

crash> ps
```

接下来可以输入命令来分析了。
