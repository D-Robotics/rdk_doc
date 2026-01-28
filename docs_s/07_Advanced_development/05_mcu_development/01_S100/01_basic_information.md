---
sidebar_position: 1
---

# MCU快速入门指南

## 范围

本章节概述了 RDK-S100 MCU 系统，旨在帮助读者快速了解并掌握相关内容，以便开展 MCU1 的开发工作。因为MCU0负责启动Acore、MCU1以及电源管理等功能，这部分不建议客户自行修改，默认不释放源码，提供地瓜验证过的bin文件。章节中仅对可能与 MCU1 发生冲突的部分进行简要说明，旨在帮助用户在开发过程中规避 MCU0 与 MCU1 之间的资源竞争问题。

## 基础信息

1. MCU编译工具链为GCC工具链，版本为gcc-arm-none-eabi-10.3-2021.10
2. MCU核为ARM R52+，可以用ARM R52 technical reference manual文档作为参考：[官网链接](https://developer.arm.com/documentation/100026/latest)
3. MCU运行的操作系统均为FreeRTOS，版本为FreeRTOS Kernel V10.0.1
4. MCU主要分为两部分：MCU0和MCU1。MCU0主要负责启动Acore、MCU1以及电源管理等功能，目前不开源；MCU1主要负责跑业务等功能，开源，客户可根据自己需求进行修改

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

pip install "scons>=4.0.0"
pip install ecdsa
pip install tqdm
```

## 编译MCU系统

1. 编译会使用python3，RDK S100开发使用的python3的版本为3.8.10；
2. MCU1的镜像分为debug和release两个版本。debug版本的镜像会有调试信息，而release版本不含调试信息。

:::info 工具链下载说明 

首次编译会从 arm 官网下载工具链后解压缩（10min左右），网速不好可能会导致工具链下载不成功或下载不完整的问题，建议通过以下方式下载编译编译工具链：
1. 点击[工具链下载链接](../../../01_Quick_start/download.md#工具下载)，下载编译工具链。
2. 将已有工具链移至 /Build/ToolChain/Gcc/ 内，移动工具链命令如下：

    `mv 工具链存储路径/工具链文件名 新代码/Build/ToolChain/Gcc/`

3. 编译时检测到有工具链，不会再从官网下载。

:::

```shell
# 编译 MCU1 Debug 版本
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py s100_sip_B debug

# 编译MCU1 Debug版本
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py s100_sip_B release
```

## 编译成功标志

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/build_success.png)

### 编译输出目录

```c
output/
├── debug                               # 该文件夹下包含debug版本的编译生成文件
|    ├── objs                           # 编译生成的i/s/o文件
|    └── S100_MCU_SIP_V2.0              # 编译生成的bin/map/elf等文件
|         ├── custom_compiler_flags.py
|         ├── S100_MCU_DEBUG.elf        # MCU1启动文件
|         ├── S100_MCU_DEBUG.map
|         ├── S100_MCU_SIP_V2.0.bin
├── objs                                # 编译生成的i/s/o文件，根据编译的版本变化
├── release                             # 该文件夹下包含release版本的编译生成文件
|    ├── objs                           # 编译生成的i/s/o文件
|    └── S100_MCU_SIP_V2.0              # 编译生成的bin/map/elf等文件
```

## MCU1启动/关闭流程
MCU1的启动/关闭是由Acore经过remoteproc框架传递信息给MCU0进而实现启动/关闭MCU1。
### MCU1启动原理与步骤{#start_mcu1}
#### MCU1启动原理

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_start.png)

#### MCU1启动步骤
下述启动流程以debug版本为例，release版本与其类似，只是少一些log打印。

1. 经过上述编译流程，编译debug版本会在S100_MCU_SIP_V2.0文件夹下产生S100_MCU_DEBUG.elf文件（release版本类似），该文件为MCU1的firmware文件，因此需要将该文件推送到板端的/lib/firmware目录。举例子如下：

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
stop MCU1之后，如果需要再次启动MCU1，必须等待系统进入wfi模式之后，才能再次start MCU1，见下图所示。原因解释：避免系统还没有进入wfi模式时，start MCU1会重新加载firmware至 mcu sram位置导致之前位置代码被覆盖，导致系统运行跑飞挂死

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu1_enter_wfi.png)
:::

## MCU0/MCU1模块划分
MCU整个系统含有ICU、RTC、IPC、port、CAN等模块，但是为了用户开发的方便，对于功能进行了划分，划分细节如下图所示。

|模块|模块位置|
|----|---------------|
|ppslcu|MCU0|
|port|MCU0|
|uart|MCU0/MCU1|
|log|MCU0/MCU1|
|shell_init|MCU0/MCU1|
|mDma|MCU0/MCU1|
|I2c|MCU0: i2c6, i2c7/MCU1: i2c8, i2c9|
|tca9539|MCU0|
|ICU|MCU0|
|GPT|MCU0|
|pmic|MCU0|
|fls_init|MCU0|
|otafiash|MCU0|
|ipc|MCU0: instance8/MCU1: instance0(其他instance未划分, 均可使用)|
|crypto|MCU0|
|pvt|MCU0|
|canGW|MCU1|
|Rtc|MCU0|
|RTC_pps|MCU0|
|Eth_Init|MCU1|
|Scmi|MCU0|

## MCU在sysfs上debug功能介绍

MCU目前在sysfs上支持查看系统状态alive，系统存活时间taskcounter，mcu版本mcu_version，sbl版本sbl_version等功能。
1. 系统状态alive：表示MCU0/MCU1所处状态，分别为alive和dead两种。mcu alive状态每1s更新一次，所以获取状态会有1s延迟；
2. 系统存活时间taskcounter：表示mcu启动后持续的时间，单位：秒；
3. mcu版本mcu_version：可以查看mcu版本信息，包括debug版本还是release版本，以及编译的时间；
4. sbl版本sbl_version：可以查看sbl版本信息以及编译的时间，但是只有在remoteproc_mcu0下可以查看;
5. mcu串口log: 可以查看MCU串口log信息，分别remoteproc_mcu0对应MCU0，remoteproc_mcu1对应MCU1。
6. mcu cpuloads: 可以获取到MCU0/MCU1各任务的任务状态，优先级，剩余栈，运行次数（FreeRtos tickcount）和使用率等信息，帮助用户去debug。cpuloads数据获取需要1s的延迟，因为会涉及到大量数据拷贝至sysfs文件系统下的输出buffer。cpuloads的获取需要在MCU0/MCU1**已上电**的情况下才能进行获取。

:::info 图片中的信息可能因版本更新而有所不同，文中示例仅供参考
:::
1. 系统状态alive，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/alive_state.png)

2. 系统存活时间taskcounter，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/taskcounter_state.png)

3. mcu版本mcu_version，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_version.png)

4. sbl版本sbl_version，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/sbl_version.png)

5. mcu串口log获取，图示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/log.png)

6. mcu cpuloads获取，图示:

![](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/cpuload.jpg)

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
2. 编译好的MCU0 镜像/output_sysmcu/目录下找到相应的MCU0镜像（MCU0代码仅在商业版中提供）
```c
fastboot oem interface:mtd
/* 编译出来的MCU0镜像：MCU_S100_SIP_V2.0.img */
fastboot flash MCU_a "xxx/MCU_S100_SIP_V2.0.img"
fastboot flash MCU_b "xxx/MCU_S100_SIP_V2.0.img"
```

#### 空板烧录
**空板烧录请参考以下工具烧录**

### 工具烧录
1. 能够正常进入Uboot时，按如下配置：
   1. “下载模式”选择“uboot”；
   2. “储存介质”选择“emmc”；
   3. “类型”选择“secure”；
   4. “选择镜像”位置请选择带有`img_packages`和`xmodem_tools`的文件夹；
   5. “acore串口”根据实际情况选择；
   6. “波特率”选择“921600”；
   7. 单击“其他配置”的右方的小箭头，点击“分区选择”，然后只勾选“miniboot_flash”；

  ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_uboot.png)

2. 不能正常进入UBoot，下载模式选择“usb”，不需要选择串口及波特率，其他配置与能够正常进入Uboot时保持一致：

  ![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/mcu_usb.png)

## MCU1 Undefined/Abort 异常处理原理

正常情况下系统在进入undefined/abort异常时，最终会进入死循环状态。只有重新执行上下电流程才能再次正常运行。RDK-S100由于不能对MCU1单独进行上下电，所以需要进行系统流程的修改，以实现上述的预期。
具体原理：当Undefined/Abort异常产生时，也会最终进入死循环状态。通过Acore的sysfs对MCU1进行软件下电，也即通知MCU1进入wfi模式，等下次再次start时，MCU1将重新软件启动，从而实现预期。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/basic_information/MCU_exception.png)

以Undefined异常为例子，当Undefined异常产生时，uart串口输出log “EL1_Undefined_Handler”，并进入最终进入S100_Exception_Handler处理函数，并根据exception_on变量进入死循环状态。当Acore通过remoteproc框架stop MCU1后，核间中断修改exception_on变量，进而关闭tick周期性中断，并进入WFI模式（STANDBY模式）：
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
main函数是进入系统后的关键代码，下述代码也是MCU1正常启动的关键，请勿随意删除相关代码，删除可能会导致启动异常。
```c
int main(void)
{
    Ipc_MainPowerUp = TRUE;   /* IPC 上电标志，MCU1默认上电，因为在MCU0已上电 */
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
## MCU Log简介
MCU提供了基础的日志（Log）输出功能，主要用于调试与运行状态记录。当前版本的Log模块支持通过格式化字符串的方式输出信息，便于开发者在调试过程中快速定位问题和查看变量状态。

目前，MCU Log支持的格式化输出类型包括：
- %s —— 字符串
- %d —— 十进制有符号整数
- %u —— 十进制无符号整数
- %x —— 十六进制小写格式
- %X —— 十六进制大写格式
- %c —— 单个字符

除以上类型外的其他格式化输出暂不支持，后续版本将逐步扩展更多的数据类型与格式支持，以满足更丰富的调试需求。
