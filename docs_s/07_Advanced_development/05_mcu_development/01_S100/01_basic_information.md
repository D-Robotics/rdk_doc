---
sidebar_position: 1
---

# MCU快速入门指南

## 范围

本章节概述了 RDK-S100 mcu 系统，用于帮助读者快速了解并掌握，以便进行mcu1相关开发

## 基础信息

1. MCU编译工具链为GCC工具链，版本为gcc-arm-none-eabi-10.3-2021.10；
2. MCU核为ARM R52+，可以用ARM R52 technical reference manual文档作为参考，请至ARM官网查询；
3. MCU运行的操作系统均为FreeRTOS，版本为FreeRTOS Kernel V10.0.1
4. MCU主要分为两部分：MCU0和MCU1。MCU0主要负责启动Acore、MCU1以及电源管理等功能，目前不开源；MCU1主要负责跑业务等功能，开源，客户可根据自己需求进行修改。

## MCU框架
MCU0是板子启动的开始，也是重中之重。因为MCU0负责启动Acore、MCU1以及电源管理等功能。Acore所运行的linux操作系统是客户开发功能的重要载体，而MCU1运行的FreeRTOS操作系统为客户的实时任务进行保驾护航。
MCU1通过linux的remoteproc框架实现，在Acore的sysfs通过向MCU0发送通知，从而控制MCU1的启动和关闭。同时在RDK-S100的休眠模式下，也是通知Acore通知MCU0从而操作MCU1，实现低功耗休眠功能。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_frame.png)

## 开发环境
交叉编译是指在主机上开发和构建软件，然后把构建的软件部署到开发板上运行。主机一般拥有比开发板更高的性能和更多的内存，可以高效完成代码的构建，可以安装更多的开发工具。

### 主机编译环境要求

推荐使用 Ubuntu 22.04 操作系统，保持和RDK S100相同的系统版本，减少因版本差异产生的依赖问题。

Ubuntu 22.04 系统安装以下软件包：

```c
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
                        flex python3-numpy mtd-utils zlib1g-dev debootstrap \
                        libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
                        curl repo git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
                        android-sdk-libsparse-utils mtools parted dosfstools udev rsync python3-pip scons

pip install scons>=4.0.0
pip install ecdsa
pip install tqdm
```

## 编译MCU系统

1. 编译会使用python3，RDK S100开发使用的python3的版本为3.8.10；
2. mcu1的镜像分为debug和release两个版本。debug版本的镜像会有调试信息，而release版本不含调试信息。

```c
/* 编译mcu1 */
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py s100_sip_B debug/release

/*
1.首次编译会从arm官网下载一份工具链然后解压缩（10min左右），网速不好可能会存在工具链下载不成功或者工具链下载不完整的问题，可删除已下载的工具链，再多尝试下载几次。
2.如果已有相关工具链，可以将其移至/Build/ToolChain/Gcc/内，当检测到有工具链，就不会从官网下载。
mv 工具链地址/gcc-arm-none-eabi-10.3-2021.10/ 新代码/Build/ToolChain/Gcc/gcc-arm-none-eabi-10.3-2021.10
*/
```

## 编译成功标志

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/build_success.png)

### 编译输出目录

```c
output/
├── dbg                                 # 该文件夹下包含debug版本的编译生成文件
|    ├── objs                           # 编译生成的i/s/o文件
|    └── S100_MCU_SIP_V2.0              # 编译生成的bin/map/elf等文件
|         ├── custom_compiler_flags.py
|         ├── S100_MCU_DEBUG.elf        # mcu1启动文件
|         ├── S100_MCU_DEBUG.map
|         ├── S100_MCU_SIP_V2.0.bin
├── objs                                # 编译生成的i/s/o文件，根据编译的版本变化
├── rel                                 # 该文件夹下包含release版本的编译生成文件
|    ├── objs                           # 编译生成的i/s/o文件
|    └── S100_MCU_SIP_V2.0              # 编译生成的bin/map/elf等文件
```

## MCU1启动/关闭流程
MCU1的启动/关闭是由Acore经过remoteproc框架传递信息给mcu0进而实现启动/关闭mcu1。
### MCU1启动原理与步骤
#### MCU1启动原理

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_start.png)

#### MCU1启动步骤
下述启动流程以debug版本为例，release版本与其类似，只是少一些log打印。

1. 经过上述编译流程，编译debug版本会在S100_MCU_SIP_V2.0文件夹下产生S100_MCU_DEBUG.elf文件（release版本类似），该文件为mcu1的firmware文件，因此需要将该文件推送到板端的/lib/firmware目录。举例子如下：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/push_elf.png)

2. 板端启动流程
```c
cd /sys/class/remoteproc/remoteproc_mcu0
echo S100_MCU_DEBUG.elf > firmware
echo start > state
```
正常启动后，串口log打印下图所示
Acore侧串口打印

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/Acore_start_log.png)

MCU侧串口打印

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_start_log.png)

### MCU1关闭原理与步骤

#### MCU1关闭原理

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_stop.png)

#### MCU1关闭步骤
下述关闭流程以debug版本为例，release版本与其类似，只是少一些log打印。
```c
cd /sys/class/remoteproc/remoteproc_mcu0
echo S100_MCU_DEBUG.elf > firmware
echo stop > state
```
正常关闭后，串口log打印下图所示
Acore侧串口打印

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/Acore_stop_log.png)

MCU侧串口打印

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_stop_log.png)

:::caution
stop mcu1之后，如果需要再次启动mcu1，必须等待系统进入wfi模式之后，才能再次start mcu1，见下图所示。原因解释：避免系统还没有进入wfi模式时，start mcu1会重新加载firmware至 mcu sram位置导致之前位置代码被覆盖，导致系统运行跑飞挂死

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_enter_wfi.png)
:::

## MCU0/MCU1模块划分
MCU整个系统含有ICU、RTC、IPC、port、CAN等模块，但是为了用户开发的方便，对于功能进行了划分，划分细节如下图所示。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_functions.png)

## MCU在sysfs上debug功能介绍

MCU目前在sysfs上支持查看系统状态alive，系统存活时间taskcounter，mcu版本mcu_version，sbl版本sbl_version等功能。
1. 系统状态alive：表示mcu0\mcu1所处状态，分别为alive和dead两种；
2. 系统存活时间taskcounter：表示mcu启动后持续的时间，单位：秒；
3. mcu版本mcu_version：可以查看mcu版本信息，包括debug版本还是release版本，以及编译的时间；
4. sbl版本sbl_version：可以查看sbl版本信息以及编译的时间，但是只有在remoteproc_mcu0下可以查看;
5. MCU串口log:可以查看MCU串口log信息，分别remoteproc_mcu0对应MCU0，remoteproc_mcu1对应MCU1。

系统状态alive，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/alive_state.png)

系统存活时间taskcounter，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/taskcounter_state.png)

mcu版本mcu_version，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_version.png)

sbl版本sbl_version，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/sbl_version.png)

MCU串口log获取，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/log.png)

## MCU串口使用
如果RDK-S100含有连接方式如下，mcu串口和Acore串口共用一个串口，自行查看：设备管理器 -》端口-》MCU-COM-波特率921600

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_COM1.jpg)

## MCU0烧录流程
### 手动烧录
#### 非空板烧录
1. 打开板子，板端Acore串口常按enter进入uboot（一定要一直按）
```c
fastboot 0
```
2. 编译好的mcu0 镜像/output_sysmcu/目录下找到相应的mcu0镜像
```c
fastboot oem interface:mtd
/* 编译出来的mcu0镜像：MCU_S100_SIP_V2.0.img */
fastboot flash MCU_a "xxx/MCU_S100_SIP_V2.0.img"
fastboot flash MCU_b "xxx/MCU_S100_SIP_V2.0.img"
```
#### 空片烧录或烧挂重新烧录
1. 通过编译RDKS100-acore获取RDKS100镜像包，结构如下所示

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/acore_product.png)

2. 烧录第一步：进入dfu模式，按照下图拨key即可(烧录完，记得拨回去！！！)

a. 左下角有两个按键，在旁边有箭头丝印，箭头的方向表示实现对应功能的拨码方向
b. 电源开关：向下拨动，给板子上电
c. 烧录开关：向上拨动，给板子烧录
d. 上述操作完成后，按图片中按键1，同时2处的灯变为红色

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/board_dfu1.png)

3. 烧录第二步：在你解压的镜像文件夹下执行下面的指令，即dfu下载进入uboot(sec版本)
```c
dfu-util.exe -d 3652:6610 -a 0 -D out/product/xmodem_tools/sec/out/s100/cmd_load_sbl
dfu-util.exe -d 3652:6610 -a 0 -D out/product/xmodem_tools/sec/out/s100/sbl.pkg
dfu-util.exe -d 3652:6610 -a 0 -D out/product/xmodem_tools/sec/out/s100/cmd_exit_sbl
dfu-util.exe -d 3652:6620 -a 0 -R -D out/product/xmodem_tools/sec/out/s100/u-boot-spl_ddr.bin
dfu-util.exe -d 3652:6620 -a 0 -R -D out/product/xmodem_tools/sec/out/s100/S100_MCU_V1.0.bin
# mcu启动可能费时比较久
dfu-util.exe -d 3652:6625 -a 0 -D out/product/xmodem_tools/sec/out/s100/hobot-s100-bl31.dtb
dfu-util.exe -d 3652:6625 -a 1 -D out/product/xmodem_tools/sec/out/s100/bl31.bin
dfu-util.exe -d 3652:6625 -a 2 -D out/product/xmodem_tools/sec/out/s100/tee-pager_v2.bin
dfu-util.exe -d 3652:6625 -a 3 -R -D out/product/xmodem_tools/sec/out/s100/u-boot.bin
```
4. 烧录第三步：整体烧录命令如下：
```c
fastboot.exe oem interface:mtd 
fastboot.exe flash hb_vspiflash out/product/img_packages/disk/miniboot_flash_nose.img

fastboot.exe oem interface:blk
fastboot.exe oem bootdevice:scsi
fastboot.exe flash 0x0 out/product/img_packages/disk/ufs_disk.simg
```
### 自动烧录
1. 能够正常进入Uboot，下载模式选择“uboot”

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_uboot.png)

2. 不能正常进入UBoot，下载模式选择“usb”

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_usb.png)

## MCU1 Undefined/Abort 异常处理原理

正常情况下系统在进入undefined/abort异常时，最终会进入死循环状态。只有重新执行上下电流程才能再次正常运行。RDK-S100由于不能对mcu1单独进行上下电，所以需要进行系统流程的修改，以实现上述的预期。
具体原理：当Undefined/Abort异常产生时，也会最终进入死循环状态。通过Acore的sysfs对mcu1进行软件下电，也即通知mcu1进入wfi模式，等下次再次start时，mcu1将重新软件启动，从而实现预期。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_exception.png)

以Undefined异常为例子，当Undefined异常产生时，uart串口输出log “EL1_Undefined_Handler”，并进入最终进入S100_Exception_Handler处理函数，并根据exception_on变量进入死循环状态。当Acore通过remoteproc框架stop mcu1后，核间中断修改exception_on变量，进而关闭tick周期性中断，并进入WFI模式（STANDBY模式）：
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
## MCU1 main函数简介
main函数是进入系统后的关键代码，下述代码也是mcu1正常启动的关键，请勿随意删除相关代码，删除可能会导致启动异常。
```c
int main(void)
{
    Ipc_MainPowerUp = TRUE;   /* IPC 上电标志，mcu1默认上电，因为在mcu0已上电 */
    PpsIcu_Irq_Init();        /* PPS相关中断配置为边沿触发函数 */
    Uart_Init();              /* UART串口初始化，debug用 */
    Log_Init();               /* log串口初始化，初始化后可在Acore获取相应mcu log信息 */
    #ifdef SHELL_ENABLE
    Shell_Init();             /* shell命令初始化，通过SHELL_ENABLE宏进行开关 */
    #endif
    Version_into_AonSram();   /* 获取mcu版本信息，初始化后可在Acore获取相应mcu version信息 */
    LogSync("MCU FreeRtos Lite Init Success!\r\n");
    FreeRtos_Irq_Init();      /* FreeRtos 中断初始化 */
    FreeRtos_Task_Init();     /* FreeRtos 任务初始化以及调度启动 */
    for(;;){};
}
```
