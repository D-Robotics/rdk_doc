---
sidebar_position: 7
---

# 7.2.7 Introduction to Linux Debugging Features

## Analyzing ramdump with crash

### Capturing ramdump

The ramdump feature is disabled by default. On Linux, you can manually enable it using the tool `hrut_ddr_misc`:

```Shell
root@ubuntu:/userdata# hrut_ddr_misc s bit 0 1
update misc para begin
------------------------------------
print new misc para:
Bit idx  Function name   Status
0        RAMDUMP         on
------------------------------------
```

To check whether the ramdump feature is currently enabled:

```Shell
root@ubuntu:/userdata# hrut_ddr_misc g
Bit idx  Function name   Status
0        RAMDUMP         on
```

After capturing the ramdump, you can disable the DDR ramdump feature:
```Shell
root@ubuntu:~# hrut_ddr_misc s bit 0 0
update misc para begin
------------------------------------
print new misc para:
Bit idx  Function name   Status
0        RAMDUMP         off
------------------------------------
```

**Currently, the ramdump feature only supports capturing dumps triggered by Kernel panic.**

**Capturing ramdump may corrupt the partition where the dump file is saved. Please ensure that the dump file is saved to a non-root filesystem partition with sufficient capacity (larger than DDR size).**

**It is recommended to create a dedicated partition for ramdump. See [Custom Partition Instructions](../rdk_gen#Custom-Partition-Instructions), for example, name the partition "ramdump".**

#### Automatic Capture

- Set environment variables in Uboot:
```Shell
setenv enable_ramdump 1
setenv ramdump_part_name ramdump # "ramdump" here indicates the actual partition where dump files will be saved; replace it according to your board's partition layout
setenv ramdump_in map # "map" here instructs ramdump to save files to UFS or eMMC (depending on boot mode); ensure this is set to "map"
saveenv
```

- For secure boot devices, automatic ramdump capture requires flashing the HB_APDP partition image and enabling secure debug. Refer to the "HB_APDP Generation" section in the RDK S100 Commercial Customer Documentation Addendum. Please contact your FAE to obtain the RDK S100 Commercial Customer Documentation Addendum.

- With these settings, ramdump will be automatically captured upon reboot after a panic occurs.

#### Manual Capture

After triggering a Kernel panic and rebooting into Uboot, execute the following commands in Uboot. The data will be stored in the `/ramdump/` directory on eMMC or UFS.

```Shell
Hobot$ setenv enable_ramdump 1
Hobot$ setenv ramdump_part_name ramdump # "ramdump" here indicates the actual partition where dump files will be saved; replace it according to your board's partition layout
Hobot$ setenv ramdump_in map # "map" here instructs ramdump to save files to UFS or eMMC (depending on boot mode); ensure this is set to "map"
Hobot$ memdump userdata # This command triggers ramdump; "userdata" here refers to DRAM userdata
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
skip secure world memory region
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

### Introduction to crash

Crash is primarily used for offline analysis of Linux kernel memory dump files. It integrates the gdb debugger and offers powerful capabilities, such as viewing stack traces, dmesg logs, kernel data structures, disassembly, and more. Crash supports memory dump file formats generated by various tools, including:

- Live Linux systems.
- Normal and compressed memory dump files produced by kdump.
- Compressed memory dump files generated by the makedumpfile command.
- Memory dump files generated by netdump.
- Memory dump files generated by diskdump.
- Xen memory dump files generated by kdump.
- Memory dump files generated by LKCD.
- Memory dump files generated by Mcore.
- Raw memory dump files in ramdump format.

### Using crash

This document primarily uses crash to analyze ramdump files. Ramdump files are nearly complete memory images—except for certain security-related memory regions that cannot be captured, almost all DRAM contents can be dumped. Some issues are difficult to reproduce or caused by memory corruption, making them impossible to diagnose solely from logs. Therefore, preserving a memory snapshot at the moment an issue occurs enables thorough analysis.

#### Obtaining and compiling crash tool source code:

```Shell
sudo apt install -y texinfo
git clone --depth=1 https://github.com/crash-utility/crash.git
make target=arm64
```

**Currently, crash is only supported on x86-64 platforms.**

#### Copying ramdump files to the server

Copy the DDR*.bin and cpu-contexts.bin files saved in the board's ramdump partition to the directory containing the crash binary. Since DDR*.bin files are nearly as large as the DDR capacity, we recommend using the `scp` command for transfer.

#### Obtaining crash extension files and CPU context parsing scripts

These files are hosted on a public server at:  
[https://archive.d-robotics.cc/ubuntu-rdk-s100-beta/host-tools/crash-tools/](https://archive.d-robotics.cc/ubuntu-rdk-s100-beta/host-tools/crash-tools/)

Download `parse-cpu-contexts.py` and `arm64-regs.so`, and save them to the directory containing the crash binary.

#### Parsing CPU register information

```Shell
python3 parse-cpu-contexts.py cpu-contexts.bin >./coreregs.txt
```

#### Launching crash to analyze the crash scene

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

WARNING: cpu 0: cannot find NT_PRSTATUS note  
WARNING: cpu 1: cannot find NT_PRSTATUS note  
WARNING: cpu 2: cannot find NT_PRSTATUS note  
WARNING: cpu 3: cannot find NT_PRSTATUS note  
WARNING: cpu 4: cannot find NT_PRSTATUS note  
WARNING: cpu 5: cannot find NT_PRSTATUS note  
      KERNEL: ./vmlinux  
   DUMPFILES: /var/tmp/ramdump_elf_LKd4AH [temporary ELF header]  
              DDRCS0-0.bin  
              /dev/zero  
              DDRCS0-2.bin  
              DDRCS1-0.bin  
              DDRCS1-1.bin  
              DDRCS2-0.bin  
              DDRCS2-1.bin  
              DDRCS3-0.bin  
        CPUS: 6 [OFFLINE: 5]  
        DATE: Thu Jun  5 00:01:41 CST 2025  
      UPTIME: 01:44:00  
LOAD AVERAGE: 3.95, 4.11, 4.05  
       TASKS: 672  
    NODENAME: ubuntu  
     RELEASE: 6.1.112-rt43-DR-4.0.2-2507251105-g2eb711-g0e5746  
     VERSION: #6 SMP PREEMPT_RT Fri Jul 25 11:14:37 CST 2025  
     MACHINE: aarch64  (unknown Mhz)  
      MEMORY: 12 GB  
       PANIC: "Kernel panic - not syncing: sysrq triggered crash"  
         PID: 4240  
     COMMAND: "bash"  
        TASK: ffff00040f10f000  [THREAD_INFO: ffff00040f10f000]  
         CPU: 0  
       STATE: TASK_RUNNING (PANIC)  

crash>  
```  

#### Add extension file  
```Shell  
crash> extend arm64-regs.so  
./arm64-regs.so: shared object loaded  
```  

#### Add CPU register information  
```Shell  
crash> arm64_core_set -l coreregs.txt  
loading cpu core regs from coreregs.txt  
loading cpu core regs from coreregs.txt done  
```  

#### View stack trace at the time of panic  
```Shell  
crash> bt  
PID: 4240     TASK: ffff00040f10f000  CPU: 0    COMMAND: "bash"  
 #0 [ffff8000279cfab0] __arm_smccc_smc at ffff800008029cd0  
 #1 [ffff8000279cfad0] __invoke_psci_fn_smc at ffff8000089a3394  
 #2 [ffff8000279cfb10] psci_sys_reset at ffff8000089a3854  
 #3 [ffff8000279cfb20] atomic_notifier_call_chain at ffff8000080cc094  
 #4 [ffff8000279cfb60] do_kernel_restart at ffff8000080ce818  
 #5 [ffff8000279cfb70] machine_restart at ffff800008019b9c  
 #6 [ffff8000279cfb90] emergency_restart at ffff8000080cdaec  
 #7 [ffff8000279cfba0] panic at ffff800008c0ad68  
 #8 [ffff8000279cfc80] sysrq_handle_crash at ffff800008719354  
 #9 [ffff8000279cfc90] __handle_sysrq at ffff800008719c84  
#10 [ffff8000279cfce0] write_sysrq_trigger at ffff80000871a318  
#11 [ffff8000279cfd00] proc_reg_write at ffff8000083a9478  
#12 [ffff8000279cfd20] vfs_write at ffff80000831b9f4  
#13 [ffff8000279cfdc0] ksys_write at ffff80000831be6c  
#14 [ffff8000279cfe00] __arm64_sys_write at ffff80000831bf20  
#15 [ffff8000279cfe10] invoke_syscall at ffff800008029e5c  
#16 [ffff8000279cfe40] el0_svc_common.constprop.0 at ffff800008029f80  
#17 [ffff8000279cfe70] do_el0_svc at ffff80000802a0f4  
#18 [ffff8000279cfe80] el0_svc at ffff800008c22554  
#19 [ffff8000279cfea0] el0t_64_sync_handler at ffff800008c239ac  
#20 [ffff8000279cffe0] el0t_64_sync at ffff8000080115e4  
     PC: 0000ffffa9a67e10   LR: 0000ffffa9a0506c   SP: 0000ffffebb5a030  
    X29: 0000ffffebb5a030  X28: 0000000000000000  X27: 0000aaaaab07b000  
    X26: 0000aaaaab041468  X25: 0000aaaaab085810  X24: 0000000000000002  
    X23: 0000aaaabe0f0cd0  X22: 0000ffffa99707e0  X21: 0000ffffa9b2c5d8  
    X20: 0000aaaabe0f0cd0  X19: 0000000000000001  X18: 0000000000000001  
    X17: 0000ffffa9a01d40  X16: 0000ffffa9a06450  X15: 0000aaaaab08f2e8  
    X14: 0000000000000000  X13: 0000000000000001  X12: 0000ffffa9ad7720  
    X11: 0000ffffa9ad7440  X10: 0000000000000063   X9: 0000aaaabe092110  
     X8: 0000000000000040   X7: 00000000ffffffff   X6: 0000000000000063  
     X5: 0000aaaabe0f0cd1   X4: 0000aaaabe092111   X3: 0000ffffa9970020  
     X2: 0000000000000002   X1: 0000aaaabe0f0cd0   X0: 0000000000000001  
    ORIG_X0: 0000000000000001  SYSCALLNO: 40  PSTATE: 20001000  
crash>  
```