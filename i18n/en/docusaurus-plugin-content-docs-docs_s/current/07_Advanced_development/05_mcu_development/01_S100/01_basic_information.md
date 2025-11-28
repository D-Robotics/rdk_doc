---
sidebar_position: 1
---

# MCU Quick Start Guide

## Scope

This section provides an overview of the RDK-S100 MCU system, aiming to help readers quickly understand and master the relevant content to facilitate MCU1 development. Since MCU0 handles functions such as booting the Acore, MCU1, and power management, it is not recommended for customers to modify this part. The source code for MCU0 is not released by default; instead, pre-validated binary files from D-Robotics are provided. This chapter only briefly describes parts that might conflict with MCU1, helping users avoid resource contention issues between MCU0 and MCU1 during development.

## Basic Information

1. The MCU compilation toolchain is GCC, specifically version gcc-arm-none-eabi-10.3-2021.10.
2. The MCU core is ARM R52+. You can refer to the ARM R52 Technical Reference Manual: [Official Link](https://developer.arm.com/documentation/100026/latest).
3. Both MCUs run FreeRTOS, version FreeRTOS Kernel V10.0.1.
4. The MCU system is primarily divided into two parts: MCU0 and MCU1. MCU0 handles booting the Acore, MCU1, and power management, and its source code is currently not open-sourced. MCU1 runs user applications and is open-sourced, allowing customers to modify it according to their needs.

## MCU Architecture

MCU0 is the starting point of board initialization and is critically important, as it manages booting the Acore, MCU1, and power management. The Linux OS running on the Acore serves as the primary platform for customer application development, while the FreeRTOS OS on MCU1 ensures reliable execution of real-time tasks.

MCU1 is implemented via Linux's remoteproc framework. The Acore controls MCU1's startup and shutdown by sending notifications to MCU0 through sysfs. Additionally, during RDK-S100's sleep mode, the Acore notifies MCU0 to manage MCU1, enabling low-power sleep functionality.

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_frame-en.jpg)

## Development Environment

Cross-compilation refers to developing and building software on a host machine, then deploying the built software onto the development board for execution. Host machines typically offer higher performance and more memory than development boards, enabling efficient code building and supporting a wider range of development tools.

### Host Compilation Environment Requirements

Ubuntu 22.04 is recommended to maintain consistency with the RDK S100 system version and minimize dependency issues caused by version mismatches.

Install the following packages on Ubuntu 22.04:

```c
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
                        flex python3-numpy mtd-utils zlib1g-dev debootstrap \
                        libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
                        curl repo git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
                        android-sdk-libsparse-utils mtools parted dosfstools udev rsync python3-pip scons

pip install "scons>=4.0.0"
pip install ecdsa
pip install tqdm
```

## Compiling the MCU System

1. Python3 is used for compilation. The RDK S100 development environment uses Python 3.8.10.
2. MCU1 images are available in two versions: debug and release. The debug version includes debugging information, while the release version does not.

```shell
# Compile MCU1 Debug version
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py s100_sip_B debug
# 1. On the first compilation, the toolchain will be downloaded from ARM's official website and extracted (takes ~10 minutes). Poor network connectivity may cause download failures or incomplete downloads. In such cases, delete the partially downloaded toolchain and retry.
# 2. If you already have the toolchain, move it to /Build/ToolChain/Gcc/. The build script will skip downloading if the toolchain is detected.
# mv <toolchain_path>/gcc-arm-none-eabi-10.3-2021.10/ <new_code_path>/Build/ToolChain/Gcc/gcc-arm-none-eabi-10.3-2021.10
*/

# Compile MCU1 Release version
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py s100_sip_B release
```

## Compilation Success Indication

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/build_success.png)

### Compilation Output Directory

```c
output/
├── debug                               # Contains files generated for the debug version
|    ├── objs                           # Generated .i/.s/.o files
|    └── S100_MCU_SIP_V2.0              # Generated .bin/.map/.elf files
|         ├── custom_compiler_flags.py
|         ├── S100_MCU_DEBUG.elf        # MCU1 firmware file
|         ├── S100_MCU_DEBUG.map
|         ├── S100_MCU_SIP_V2.0.bin
├── objs                                # Generated .i/.s/.o files (varies by build version)
├── release                             # Contains files generated for the release version
|    ├── objs                           # Generated .i/.s/.o files
|    └── S100_MCU_SIP_V2.0              # Generated .bin/.map/.elf files
```

## MCU1 Startup/Shutdown Process

MCU1 startup and shutdown are controlled by the Acore through the remoteproc framework, which sends commands to MCU0 to manage MCU1.

### MCU1 Startup Principle and Steps {#start_mcu1}

#### MCU1 Startup Principle

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_start-en.jpg)

#### MCU1 Startup Steps

The following steps use the debug version as an example; the release version is similar but with fewer log outputs.

1. After compilation, the debug build generates `S100_MCU_DEBUG.elf` (similarly, the release build generates its own .elf file) in the `S100_MCU_SIP_V2.0` folder. This file is the MCU1 firmware and must be pushed to the board's `/lib/firmware` directory. For example:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/push_elf.png)

2. Board-side startup procedure:
```c
cd /sys/class/remoteproc/remoteproc_mcu0
echo S100_MCU_DEBUG.elf > firmware
echo start > state
```

Upon successful startup, the serial logs appear as shown below.

Acore-side serial log:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/Acore_start_log.png)

MCU-side serial log:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_start_log.png)

### MCU1 Shutdown Principle and Steps

#### MCU1 Shutdown Principle

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_stop-en.jpg)

#### MCU1 Shutdown Steps

The following steps use the debug version as an example; the release version is similar but with fewer log outputs.
```c
cd /sys/class/remoteproc/remoteproc_mcu0
echo S100_MCU_DEBUG.elf > firmware
echo stop > state
```

Upon successful shutdown, the serial logs appear as shown below.

Acore-side serial log:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/Acore_stop_log.png)

MCU-side serial log:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_stop_log.png)

:::caution
After stopping MCU1, you must wait until the system enters WFI (Wait For Interrupt) mode before restarting MCU1, as shown below. Explanation: If MCU1 is restarted before the system enters WFI mode, the firmware will be reloaded into the MCU SRAM, overwriting the previous code and potentially causing system crashes or hangs.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_enter_wfi.png)
:::

## MCU0/MCU1 Module Partitioning

The overall MCU system includes modules such as ICU, RTC, IPC, port, and CAN. For user convenience, functionalities are partitioned as shown in the table below.

| Module       | Location        |
|--------------|-----------------|
| ppslcu       | MCU0            |
| port         | MCU0            |
| uart         | MCU0/MCU1       |
| log          | MCU0/MCU1       |
| shell_init   | MCU0/MCU1       |
| mDma         | MCU0/MCU1       |
| I2c          | MCU0: i2c6, i2c7 / MCU1: i2c8, i2c9 |
| tca9539      | MCU0            |
| ICU          | MCU0            |
| GPT          | MCU0            |
| pmic         | MCU0            |
| fls_init     | MCU0            |
| otafiash     | MCU0            |
| ipc          | MCU0: instance8 / MCU1: instance0 (other instances are unassigned and available for use) |
| crypto       | MCU0            |
| pvt          | MCU0            |
| canGW        | MCU1            |
| Rtc          | MCU0            |
| RTC_pps      | MCU0            |
| Eth_Init     | MCU1            |
| Scmi         | MCU0            |

## MCU Debug Features in sysfs

The MCU currently supports several debug features via sysfs, including checking system status (`alive`), system uptime (`taskcounter`), MCU version (`mcu_version`), and SBL version (`sbl_version`).

1. **System status (`alive`)**: Indicates whether MCU0/MCU1 is alive or dead. The `alive` status updates every 1 second, so there is a 1-second delay when retrieving the status.
2. **System uptime (`taskcounter`)**: Shows the duration (in seconds) since the MCU was started.
3. **MCU version (`mcu_version`)**: Displays MCU version information, including whether it's a debug or release build and the compilation timestamp.
4. **SBL version (`sbl_version`)**: Shows SBL version information and compilation timestamp. This is only available under `remoteproc_mcu0`.
5. **MCU serial logs**: Allows viewing MCU serial logs. `remoteproc_mcu0` corresponds to MCU0 logs, and `remoteproc_mcu1` corresponds to MCU1 logs.
6. **MCU CPU loads (`cpuloads`)**: Provides task status, priority, remaining stack, execution count (FreeRTOS tick count), and CPU usage for MCU0/MCU1 tasks, aiding debugging. Retrieving `cpuloads` data involves significant data copying to the sysfs output buffer, resulting in a 1-second delay. This feature is only available when MCU0/MCU1 is powered on.

:::info
The information shown in the images may vary with version updates. The examples provided here are for reference only.
:::

1. System status (`alive`):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/alive_state.png)

2. System uptime (`taskcounter`):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/taskcounter_state.png)

3. MCU version (`mcu_version`):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_version.png)

4. SBL version (`sbl_version`):

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/sbl_version.png)

5. MCU serial log retrieval:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/log.png)  

6. Obtain MCU CPU loads, as shown below:

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/cpuload.jpg)

## MCU UART Usage
If the RDK-S100 uses the following connection method, the MCU UART and Acore UART share the same serial port. Check it yourself via: Device Manager → Ports → MCU-COM, Baud Rate 921600.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_COM1.jpg)

## MCU0 Flashing Procedure
### Manual Flashing
#### Flashing on a Non-Blank Board
1. Power on the board and continuously press Enter on the Acore UART to enter Uboot (keep pressing Enter).
```c
fastboot 0
```
2. Locate the compiled MCU0 image under the `/output_sysmcu/` directory (MCU0 source code is only provided in the commercial version).
```c
fastboot oem interface:mtd
/* Compiled MCU0 image: MCU_S100_SIP_V2.0.img */
fastboot flash MCU_a "xxx/MCU_S100_SIP_V2.0.img"
fastboot flash MCU_b "xxx/MCU_S100_SIP_V2.0.img"
```

#### Flashing on a Blank Board
**For blank board flashing, please refer to the following tool-based flashing method.**

### Tool-Based Flashing
1. When you can successfully enter Uboot, configure as follows:
   1. Select "Uboot" for "Download Mode";
   2. Select "eMMC" for "Storage Medium";
   3. Select "Secure" for "Type";
   4. For "Image Directory", choose the folder containing `img_packages` and `xmodem_tools`;
   5. Select the appropriate "Acore UART Port" based on your actual setup;
   6. Set "Baud Rate" to "921600";
   7. Click the small arrow next to "Other Settings", then click "Partition Selection" and check only "miniboot_flash".

  ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_uboot-en.png)

2. If you cannot enter Uboot normally, select "USB" for download mode—no need to specify UART port or baud rate. Keep all other settings consistent with the Uboot scenario:

  ![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_usb-en.png)

## MCU1 Undefined/Abort Exception Handling Principle

Under normal circumstances, when the system enters an Undefined/Abort exception, it eventually enters an infinite loop. The system can only resume normal operation after a full power cycle. Since the RDK-S100 cannot power-cycle MCU1 independently, modifications to the system workflow are required to achieve the desired behavior.

Specific principle: When an Undefined/Abort exception occurs, the system also ends up in an infinite loop. By using Acore’s sysfs interface to perform a software power-down of MCU1—i.e., notifying MCU1 to enter WFI mode—MCU1 will perform a software reboot upon the next start, thus achieving the intended recovery.

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_exception-en.jpg)

Taking the Undefined exception as an example: when it occurs, the UART outputs the log “EL1_Undefined_Handler” and eventually enters the `S100_Exception_Handler` function, where it loops indefinitely based on the `exception_on` variable. When Acore stops MCU1 via the remoteproc framework, an inter-processor interrupt modifies the `exception_on` variable, disables the periodic tick interrupt, and puts MCU1 into WFI mode (STANDBY mode):

```c
void Os_Isr_Cross_Core_Ins0_Isr(void)
{
  LogSync("mcu1 enter WFI mode!\r\n");
  power_on = 0;
  ClearCrossCoreISR0();
  if (exception_on)
  {
    exception_on = 0;
  }
}

void S100_Exception_Handler(void)
{
    LogSync("os enter %s!\r\n", __func__);
    while (exception_on){};
    LogSync("%s enter wfi mode!\r\n", __func__);
    Os_Disable_Millisecond();
    Os_Clear_Millisecond();
    STANDBY();
};

void EL1_Undefined_Handler(void)
{
    int32_t func_ptr;
    *(volatile unsigned int *)(UART_0_BASE) = ('E');
    *(volatile unsigned int *)(UART_0_BASE) = ('L');
    *(volatile unsigned int *)(UART_0_BASE) = ('1');
    *(volatile unsigned int *)(UART_0_BASE) = ('_');
    *(volatile unsigned int *)(UART_0_BASE) = ('U');
    *(volatile unsigned int *)(UART_0_BASE) = ('n');
    *(volatile unsigned int *)(UART_0_BASE) = ('d');
    *(volatile unsigned int *)(UART_0_BASE) = ('e');
    *(volatile unsigned int *)(UART_0_BASE) = ('f');
    *(volatile unsigned int *)(UART_0_BASE) = ('i');
    *(volatile unsigned int *)(UART_0_BASE) = ('n');
    *(volatile unsigned int *)(UART_0_BASE) = ('e');
    *(volatile unsigned int *)(UART_0_BASE) = ('d');
    *(volatile unsigned int *)(UART_0_BASE) = ('_');
    *(volatile unsigned int *)(UART_0_BASE) = ('H');
    *(volatile unsigned int *)(UART_0_BASE) = ('a');
    *(volatile unsigned int *)(UART_0_BASE) = ('n');
    *(volatile unsigned int *)(UART_0_BASE) = ('d');
    *(volatile unsigned int *)(UART_0_BASE) = ('l');
    *(volatile unsigned int *)(UART_0_BASE) = ('e');
    *(volatile unsigned int *)(UART_0_BASE) = ('r');
    *(volatile unsigned int *)(UART_0_BASE) = ('\r');
    *(volatile unsigned int *)(UART_0_BASE) = ('\n');
    LogSync("os enter %s!\r\n", __func__);
    exception_on = 1;

    func_ptr = &S100_Exception_Handler;
    __asm volatile (
        "mov lr, %[func_ptr]\t"
        :
        : [func_ptr] "r" ((uintptr_t)func_ptr)
        : "lr"
    );

    __asm volatile ("ERET");
}
```

## Overview of MCU1 main() Function
The `main()` function is critical code executed after system startup. The following code is essential for normal MCU1 boot-up. Do **not** arbitrarily delete any part of it, as doing so may cause boot failures.

```c
int main(void)
{
    Ipc_MainPowerUp = TRUE;   /* IPC power-on flag; MCU1 is powered on by default since MCU0 is already powered */
    PpsIcu_Irq_Init();        /* Configure PPS-related interrupts as edge-triggered */
    Uart_Init();              /* Initialize UART for debugging */
    Log_Init();               /* Initialize log UART; after this, MCU logs can be retrieved from Acore */
    #ifdef SHELL_ENABLE
    Shell_Init();             /* Initialize shell commands; controlled by the SHELL_ENABLE macro */
    #endif
    Version_into_AonSram();   /* Store MCU version info in AON SRAM; accessible from Acore afterward */
    LogSync("MCU FreeRtos Lite Init Success!\r\n");
    FreeRtos_Irq_Init();      /* Initialize FreeRTOS interrupts */
    FreeRtos_Task_Init();     /* Initialize FreeRTOS tasks and start the scheduler */
    for(;;){};
}
```

## MCU Log Overview
The MCU provides basic logging functionality, primarily used for debugging and runtime status recording. The current version of the Log module supports formatted string output, enabling developers to quickly locate issues and inspect variable states during debugging.

Currently supported format specifiers in MCU logs include:
- %s —— String  
- %d —— Signed decimal integer  
- %u —— Unsigned decimal integer  
- %x —— Lowercase hexadecimal  
- %X —— Uppercase hexadecimal  
- %c —— Single character  

Other format specifiers are not currently supported. Future versions will gradually expand support for additional data types and formatting options to meet more diverse debugging needs.