---
sidebar_position: 07
---

# 7.7 VDSP开发指南

## 基础调试指南

### CPU侧开发

#### 镜像加载卸载

S100系统在启动时默认不启动VDSP FW（Firmware），需要用户通过命令的形式手动加载和卸载FW，命令如下所示：

``` shell
echo -n <firmware路径> > /sys/module/firmware_class/parameters/path

#S100 VDSP0
# 设置VDSP0 FW名称：
echo <firmware名称> > /sys/class/remoteproc/remoteproc_vdsp0/firmware
#VDSP0的FW加载：
echo start > /sys/class/remoteproc/remoteproc_vdsp0/state
#VDSP0的FW卸载：
echo stop > /sys/class/remoteproc/remoteproc_vdsp0/state
```

用户可通过以下命令修改FW路径（**必须是绝对路径**）：

``` shell
echo -n <firmware路径> > /sys/module/firmware_class/parameters/path
```

用户可根据自己命名的FW名称在加载之前进行调整：

``` shell
#S100 VDSP0
echo <firmware名称> > /sys/class/remoteproc/remoteproc_vdsp0/firmware
```

用户需要修改原始镜像，配置init.rc，kernel启动后由init进程自动加载VDSP镜像。

``` shell
#首先将编译出的FW镜像（如vdsp0）拷贝到/userdata 下
echo -n <firmware路径> > /sys/module/firmware_class/parameters/path
#S100 VDSP0
echo <firmware名称> > /sys/class/remoteproc/remoteproc_vdsp0/firmware
echo start > /sys/class/remoteproc/remoteproc_vdsp0/state
```

**FW版本查看**

``` shell
#S100 VDSP0
cat /sys/class/remoteproc/remoteproc_vdsp0/version # for vdsp0
```

**VDSP运行状态查看**

``` shell
#running表示已加载，offline表示未加载
#S100 VDSP0
cat /sys/class/remoteproc/remoteproc_vdsp0/state # for vdsp0
```

**心跳监控**

默认处于关闭状态，可通过下列命令打开或关闭；心跳监控、发送周期为100ms，监控到连续7次心跳被丢失就会上报诊断并reset VDSP。

``` shell
# 打开心跳监控
echo Y > /sys/module/hobot_remoteproc/parameters/heartbeat_enable
# 关闭心跳监控
echo N > /sys/module/hobot_remoteproc/parameters/heartbeat_enable
```

#### 通过函数接口形式操作vdsp

通过libvdsp.so动态链接库加载DSP程序，实现加载、启动、停止、复位、获取DSP状态等功能，API介绍可参考
[VDSP启停控制接口](#vdsp_boot_api)。

#### 消息连接和发送

目前用户只可使用系统预设的服务名称，可使用的服务名如下表所示：

| VDSP   | 服务名称                                                      | 作用                                   | 是否必须启动          | VDSP侧是否默认启动 |
| ------ | --------------------------------------------------------- | ------------------------------------ | --------------- | ------------ |
| DSP0/1 | dcore0_device_op/dcore1_device_op                     | 系统软件内部对DSP的调试控制，系统软件已经使用，用户不可再次注册和使用 | 是               | 是            |
| DSP0/1 | dcore0_acore_heart/dcore1_acore_heart                 | 心跳机制使用，目前未使用，用户可用作其他用途               | 否               | 否            |
| DSP0/1 | dcore0_rpmsg_bpu/dcore1_rpmsg_bpu                     | BPU相关的控制，目前未使用，用户可用作其他用途             | 否               | 否            |
| DSP0/1 | dcore0_rpmsg_op/dcore1_rpmsg_op                       | 工具链算子相关的控制，目前未使用，用户可用作其他用途           | 否               | 否            |

| VDSP | 服务名称                         | 作用                                   | 是否必须启动          | VDSP侧是否默认启动 |
| ---- | ---------------------------- | ------------------------------------ | --------------- | ------------ |
| DSP0 | dcore0_device_op           | 系统软件内部对DSP的调试控制，系统软件已经使用，用户不可再次注册和使用 | 是               | 是            |
| DSP0 | dcore0_acore_heart         | 心跳机制使用，目前未使用，用户可用作其他用途               | 否               | 否            |
| DSP0 | dcore0_rpmsg_bpu           | BPU相关的控制，目前未使用，用户可用作其他用途             | 否               | 否            |
| DSP0 | dcore0_rpmsg_op            | 工具链算子相关的控制，目前未使用，用户可用作其他用途           | 否               | 否            |

用户可使用的API可参考下表：

| 接口                            | 功能   | 头文件          | 相关库         |
| ----------------------------- | ---- | ------------ | ----------- |
| hb_rpmsg_connect_server    | 连接服务 | rpmsg_lib.h | librpmsg.so |
| hb_rpmsg_disconnect_server | 断开服务 | rpmsg_lib.h | librpmsg.so |
| hb_rpmsg_send               | 发送消息 | rpmsg_lib.h | librpmsg.so |
| hb_rpmsg_recv               | 接收消息 | rpmsg_lib.h | librpmsg.so |

其中，同一个服务通道，不支持多进程、多线程并发接收或发送。

#### heap相关开发

VDSP提供了动态分配释放heap接口，支持配置自定义内存对齐大小，以及支持查看当前heap状态，当VDSP端需要动态分配heap时，可以通过以下接口实现。

| **接口**                      | **功能**             | **相关头文件**            |
| --------------------------- | ------------------ | -------------------- |
| hb_mem_heap_initialize   | 初始化mem alloctor接口  | hb_mem_allocator.h |
| hb_mem_heap_deinitialize | 解初始化mem alloctor接口 | hb_mem_allocator.h |
| hb_mem_heap_alloc        | 分配heap空间           | hb_mem_allocator.h |
| hb_mem_heap_free         | 释放已分配的heap空间       | hb_mem_allocator.h |
| hb_mem_heap_get_status  | 获取当前heap状态         | hb_mem_allocator.h |

### VDSP侧开发

代码获取步骤如下：

``` shell
（1）首先获取到发布包，解压后确认有vdsp源码。如果无，联系地瓜相关人员进行获取。
（2）在vdsp路径下可以获取到vdsp源码。
```

#### Linux环境搭建

:::tip
获取build包建立编译环境。其中build包的获取请联系地瓜相关人员进行获取。

本文档仅提供Linux环境搭建及编译的说明介绍。对于文档中提到的xplorer中获取调试文档请联系地瓜相关人员进行获取。
:::

Linux环境下安装build的命令：

``` shell
tar -zxvf Vision_Q8_linux.tgz \
   && mv RI-2023.11-linux/Vision_Q8/ /opt/xtensa/XtDevTools/install/builds/RI-2023.11-linux/ \
   && rm -rf RI-2023.11-linux

/opt/xtensa/XtDevTools/install/builds/RI-2023.11-linux/Vision_Q8/install --xtensa-tools \
   /opt/xtensa/XtDevTools/install/tools/RI-2023.11-linux/XtensaTools/
```

#### 编译

获取代码后编译步骤如下：

``` shell
（1）cd vdsp_fw
（2）bash make.sh
```

静态库生成在library目录

``` shell
library/libvdsp0.a
```

二进制镜像生成在samples目录

``` shell
samples/{subdir}/vdsp0
```

### 调试指南

#### 日志查看

VDSP FW的日志会通过串口输出。

需要注意的是VDSP FW与其他模块共用一个串口，如BL31、optee，若VDSP FW输出日志太多，可能会阻塞这些模块的日志输出，引起watchdog。
另外VDSP FW日志和kernel日志都输出到串口中，存在相互干扰的问题，用户可通过降低kernel日志等级来防止日志干扰：

``` shell
echo 0 > /proc/sys/kernel/printk
```

当没有串口可用的情况下，用户可通过ssh登录板子，后台默认会启动hrut_remoteproc_log服务：

``` shell
#S100 VDSP0 默认开机执行的启动命令，日志保存路径：/log/dsp0/message
hrut_remoteproc_log -b /sys/class/remoteproc/remoteproc_vdsp0/log -f /log/dsp0/message -r 2048 -n 200
```

同样的，VDSP FW的日志会写入Share memory中，由CPU侧log服务进程存入文件系统中。因此用户可通过以下路径下的文件查看日志，但是需要注意的是这里的日志并不是实时的。

``` shell
#S100 VDSP0的日志路径：
/log/dsp0/message
/log/dsp0/archive/
#message是临时文件，存满之后会写入到archive/目录下，当该目录下的文件达到一定数量后，会删除时间较早产生的文件
```

#### 日志打印接口

使用printf接口日志会通过串口输出。

推荐使用DSP_ERR、DSP_WARN、DSP_INFO、DSP_DBG接口，该接口除了将日志通过串口输出，还会将日志写入Share memory中，通过CPU侧的log服务将日志信息存入文件系统中。

DSP_*接口使用注意事项：

1. 头文件``hb_vdsp_log.h``
2. 接口使用示例。比如进入异常分支需要打印日志时，使用DSP_ERR接口，``DSP_ERR("Input parameter invalid.\n");``

#### 线程状态查看

通过以下命令可在串口中查看VDSP侧的线程状态。需要注意的是以下数据的统计和输出可能会影响VDSP的性能。

使用方法：首先需要代码中使能``#define THREAD_STACK_CHECK (1)``，其次需要在新启动的线程前使能栈跟踪，如下所示：

``` shell
(void)hb_enable_stack_track(dev_thread_stack, sizeof(dev_thread_stack)/sizeof(dev_thread_stack[0]));
#S100 VDSP0：
echo on > /sys/devices/virtual/misc/vdsp0/vdsp_ctrl/dspthread
echo off > /sys/devices/virtual/misc/vdsp0/vdsp_ctrl/dspthread
```

#### coredump查看

和coredump相关的系统软件初始化主要有两部分：注册异常和使能看门狗。

``` shell
hb_wdt_on();
hb_enable_coredump();
```

目前xos能够处理的异常类型如下：

``` shell
/*  EXCCAUSE register values:  */
/*  General Exception causes (Bits 3:0 of EXCCAUSE register)  */

/* No exception */
#define EXCCAUSE_NONE                   UINT32_C(0)
/* Instruction usage */
#define EXCCAUSE_INSTRUCTION            UINT32_C(1)
/* Addressing usage */
#define EXCCAUSE_ADDRESS                UINT32_C(2)
/* External causes */
#define EXCCAUSE_EXTERNAL               UINT32_C(3)
/* Debug exception */
#define EXCCAUSE_DEBUG                  UINT32_C(4)
/* Syscall exception */
#define EXCCAUSE_SYSCALL                UINT32_C(5)
/* Hardware failure */
#define EXCCAUSE_HARDWARE               UINT32_C(6)
/* Memory management */
#define EXCCAUSE_MEMORY                 UINT32_C(7)
/* Coprocessor */
#define EXCCAUSE_CP_DISABLED            UINT32_C(8)
/*  Reserved 9-15  */
```

不应在 VQ8 上为异常原因 4（调试异常）/5（SYSCALL 异常）/8（协处理器异常）注册异常处理程序，为系统预留使用。
需要注意的是：9-15 为预留类型，也需要略过不注册。

离线调试方法如下：
VDSP发生coredump时，Acore会把VDSP所有可能使用的memory空间(iram/dram0/dram1/reserved
ddr)全部写入指定的文件系统中，路径如下：

``` shell
#vdsp0
/log/coredump/
```

新建restore.script.sh脚本，4个memory
dump文件的路径根据实际项目的存放路径设置，把获得到的CPU寄存器复制到该脚本对应处，如下所示：

``` shell
python import thread_aware_rtos
python thread_aware_rtos.k.rtos_support.dump_analysis_mode = True
b main
run

restore vdsp0_ddr_2024-05-06-02-50-03.hex binary 0xf0000000
restore vdsp0_iram_2024-05-06-02-50-03.hex binary 0x08080000
restore vdsp0_dram0_2024-05-06-02-50-03.hex binary 0x08000000
restore vdsp0_dram1_2024-05-06-02-50-03.hex binary 0x08040000

set $ar0 = 0xf00502a8
set $ar1 = 0xf3fdded0
set $ar2 = 0xf3fddd00
set $ar3 = 0x34
set $ar4 = 0x1b
set $ar5 = 0x5d
set $ar6 = 0x53
set $ar7 = 0x58
set $ar8 = 0xf0050192
set $ar9 = 0xf3fddeb0
set $ar10 = 0x4f
set $ar11 = 0xf3fddd00
set $ar12 = 0x0
set $ar13 = 0xf3fddee0
set $ar14 = 0xf3fddce0
set $ar15 = 0xf3fddd1b
set $ar16 = 0xf0065978
set $ar17 = 0xf3fddb60
set $ar18 = 0x4f
set $ar19 = 0xf002bea4
set $ar20 = 0x0
set $ar21 = 0xffffffb1
set $ar22 = 0x4f
set $ar23 = 0xffffffff
set $ar24 = 0x808100f
set $ar25 = 0xf3fddf00
set $ar26 = 0xf3fddf10
set $ar27 = 0x53113
set $ar28 = 0xf0050280
set $ar29 = 0xffffffff
set $ar30 = 0x0
set $ar31 = 0xf3fddf70
set $ps = 0x68
set $wb = 0x40000311
set $pc = 0xf0050057
python thread_aware_rtos.k.rtos_support.XOS_initialized = True
```

打开xt-gdb命令行(xplorer或者命令行模式均可)，按照顺序执行如下操作：

``` shell
xt-gdb vdsp0 (可执行文件的目标文件)
(xt-gdb) >> source restore.script.sh
(xt-gdb) >> run
ctrl+c //取消运行
(xt-gdb) >> stepi
(xt-gdb) >> info threads
(xt-gdb) >> bt
```

backtrace调试信息显示如下：

``` shell
(xt-gdb) bt
#0  _RMCDump () at /vdsp/bsp_project/coredump/RegDump.S:90
#1  0xf0050192 in Exc_Dump_Regs () at bsp_project/coredump/Exc_Dump_Regs.c:110
#2  0xf00502a8 in dafault_exchandler (frame=0xf3fddf10) at bsp_project/coredump/coredump.c:125
#3  0x0808100f in _GeneralException (cause=..., exccause=...) at ./xos_vectors_xea3_v2.S
#4  0xf00504b8 in hb_platform_init () at bsp_project/driver/devcontrol/devcontrol.c:152
#5  0xf0030306 in main (argc=1, argv=0xf0073704) at main.c:68
```

#### Stack usage查看

Stack usage的说明建议阅读Xtensa® XOS Reference Manual Reference
Manual：\<VDSP安装路径\>/xtensa/XtDevTools/downloads/\<version\>/docs/xos_rm.pdf。

#### MPU配置

目前部署的MPU主要两个作用，一是用来限制VDSP访问地址的范围，访问超过MPU允许的范围会报coredump错误，第二个作用是可以配置地址段的属性，详细介绍请参考Xtensa®
System Software Reference
Manual：\<VDSP安装路径\>/xtensa/XtDevTools/downloads/\<version\>/docs/xos_rm.pdf。

VDSP地址映射以及MPU保护部分如下图所示，访问MPU保护地址范围外会报coredump错误。

![image_2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_2.png)

报错log如下图所示，错误地址为0x0，表示访问了不允许访问的地址：

![image_3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_3.png)

目前对于VDSP地址的属性配置，主要包括三个部分：

（1）对于ION空间的属性配置： XTHAL_MEM_WRITEBACK

（2）对于两个共享内存区域的属性配置：XTHAL_MEM_NON_CACHEABLE

（3）对于除（1）（2）外其他段的属性配置：XTHAL_MEM_WRITEBACK

#### Cadence文档路径位置

安装Xplorer后，可通过如下位置\<VDSP安装路径\>/xtensa/XtDevTools/downloads/RI-2023.11/docs查看已下载的文档路径。

### FAQ

#### VDSP侧耗时统计方式有哪些？

用户可使用gettimeofday()直接获取时间，也可通过XT_RSR_CCOUNT()获取count个数来换算时间，前者需要包含\#include
\<sys/time.h\>头文件。建议使用后者来统计时间，更精确，且不建议在耗时要求较高的场景下打印日志。

此外用户还可参考Xtensa® Software Development Toolkit User's
Guide：\<VDSP安装路径\>/xtensa/XtDevTools/downloads/\<version\>/docs/sw_dev_toolkit_ug.pdf。

#### LSP如何修改？

（1）拷贝一份模板lsp

（2）编辑memmap.xmm

（3）重新生成memmap，执行命令：xt-genldscripts -b custom_lsp/q8-min-rt/

:::tip
调试使用xos工具时，提示找不到工具链，先确认xos build环境是否建立。
如果建立，配置环境变量，比如配置临时环境变量``export PATH=$PATH:[*]/XtensaTools/bin``
:::

#### CPU侧start加载和stop卸载FW时提示不成功？

可能是stop时服务根本没启动，或者start时服务已启动。

#### VDSP侧如何增加用户线程？

示例代码如下：

``` C
ret = xos_thread_create(&dev_thread_tcb, 0, dev_thread_func, 0, "dev_control", dev_thread_stack, STACK_SIZE_1, TRACE_THREAD_PRIO, 0, 0);
```

其中dev_thread_func表示创建的线程函数，用来实现用户想要在线程函数中处理的功能。dev_thread_stack表示指向线程栈的首地址(由用户分配)。STACK_SIZE_1表示栈的大小。TRACE_THREAD_PRIO表示线程优先级，取值范围在0~15，数值越小优先级越高。

#### VDSP侧如何获取设备id？

可以使用xthal_get_prid()。

#### VDSP侧idma使用哪个库？

![image_5](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_5.png)

请使用libidma-os或者libidma-debug-os，因为我们使用xos。

#### int64_t/uint64_t/float类型的变量打印出错？

如果int64_t/uint64_t/float定义的变量在打印时输出异常的值，而在使用时（如进行大小比较等操作）是正常的。

这是因为xtensa/xtutil.h头文件会将printf、vsnprintf等函数替换为xt_printf，xt_vsnprintf，用户需要将xtutil.h头文件注释掉，使用原生的printf、vsnprintf接口。

:::danger
标准C库的printf等函数，无法在中断handler中使用，否则会卡死。
:::

#### VDSP在运行过程中出现卡住问题

需要排查变量指针的地址是否是对齐：

（1）（int64_t *）类型的变量指针的地址需要八字节对齐

（2）（int32_t *）类型的变量指针的地址需要四字节对齐

（3）（int16_t *）类型的变量指针的地址需要二字节对齐

确认在中断handler是否使用了标准C库的printf等函数，目前中断handler不支持使用。

#### 使用sim软仿时出现seg段溢出问题

需要更改sim的xmm文件，路径：xtensa/xtensa/XtDevTools/install/builds/RI-2023.11-win32/Vision_Q8/xtensa-elf/lib/sim/memmap.xmm，
将相应的溢出段的值改大，在xplorer下打开cmd：

![image_6](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_6.png)

进到xtensa-elf/lib路径下：

![image_7](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_7.png)

执行xt-genldscripts -b sim，如下提示表明成功：

![image_8](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_8.png)

再次编译vdsp工程，进行sim软仿。

#### Idma搬运过程中出现异常停止问题

出现此情况可以查看idma init函数中，是否有设置运行时间，置0可以关闭时间限制：

![image_9](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_9.png)

#### 在windows环境编译出的镜像提示dcore0_rpmsg_op server not start

需要手动在Build Properties中添加CONFIG_TEST_CASE：

![image_10](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/image_10.png)

#### 在总线程个数大于32的运行环境下，线程状态查看功能不生效

这是由于线程dump功能实现时，默认定义存放线程信息的数组大小为32； 可以通过加大数组大小来支持大于32个线程状态查看。

如下所示将支持最多64线程状态查看功能：

``` C
static int32_t cycle_check(void * arg, int32_t unused)
{
    const int32_t countMax = 64;
```

#### VDSP退出流程相关的适配说明

系统中依赖dev_control线程正常运行，用户通过enable宏CONFIG_PLATFORM_INIT_BUILTIN_THREADS
来控制由hb_platform_init函数启动，如果用户没有enable就需要创建相应的线程，同时用户线程需遵循下述的流程：

（1）在用户任务循环线程中添加退出判断的逻辑：调用函数hb_is_thread_stop，返回1表示需要立即退出线程

#### VDSP log使用场景限制

（1）当前中断handler不支持直接或间接使用标准C库的printf，如果使用会出现VDSP挂死

（2）在hb_platform_init之前不支持直接或间接使用标准C库的printf，如果使用可能引发低概率boot失败的问题

#### VDSP编译注意事项

（1）需要定义平台宏，不同平台对应的宏包括：CONFIG_ARCH_HOBOT_SOC_SIGIE、CONFIG_ARCH_HOBOT_SOC_SIGIP、CONFIG_ARCH_HOBOT_SOC_SIGIB

（2）默认支持不同的VDSP
CORE可加载同一个FW，同时就不需要定义CONFIG_VDSP宏（CONFIG_VDSP0/CONFIG_VDSP1不再被使用）

#### 安全下电和休眠流程

（1）在安全下电和休眠流程的处理流程中，驱动会检查VDSP
Firmware状态并确保已经进入停止状态后才继续相应的动作；同时建议在APP中实现VDSP退出流程

## VDSP sample

### 功能概述

本章节介绍S100系列SOC芯片平台的VDSP用例，该用例实现了VDSP的启停、核间消息的收发和VDSP图像处理。

### 软件架构说明

需要同时实现ARM侧和VDSP两侧的开发，当前基于RPMSG IPC通信机制实现client/server业务交互逻辑。

![vdsp1](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/vdsp1.png)

#### ARM侧开发流程

ARM侧用户主要是加载VDSP Firmware，并连接VDSP侧的服务，作为client端向VDSP侧发送rpmsg计算请求。

![vdsp2](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/vdsp2.png)

#### VDSP侧开发流程

VDSP侧主要是初始化其运行环境、启动相关的服务（VDSP侧作为server端可启动多个服务，支持一对一的模式），
并能通过rpmsg机制接收和回复client端的信息，此外还可启动其他线程进行业务开发。

![vdsp3](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/vdsp3.png)

#### 本用例流程说明：

ARM侧和VDSP的交互流程如下所示：

![vdsp4](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/07_vdsp_development/vdsp4.png)

ARM侧：

1)  启动VDSP
2)  连接rpmsg server(dcore0_rpmsg_op)
3)  向VDSP发送rpmsg
4)  接收VDSP发送的rpmsg
5)  断开rpmsg server
6)  关闭VDSP

VDSP侧：

1)  初始化环境
2)  启动rpmsg server(dcore0_rpmsg_op)
3)  接收ARM发送的rpmsg
4)  根据接收到的rpmsg数据进行解析，并执行相应的图像处理函数
5)  向ARM回复rpmsg，其中包含执行的结果

#### 代码位置与目录结构

代码路径

1. 代码位置：
``` shell
  ARM侧：{sdk_dir}/source/hobot-sp-samples/debian/app/vdsp_demo/
  DSP侧：{sdk_dir}/vdsp_fw/samples/libxi-sample/
```

2. 目录结构：
``` shell
#源码和二进制文件目录结构，不包括编译框架文件（Makefile、Kconfig等）
  ARM:
     └── src
         └── vdsp_sample.c

  DSP:
     └── main.c
```

### 编译

#### 编译说明

编译命令

``` bash
# ARM侧sample编译可以直接在板端完成。sample在板端的路径如下所示：
/app/vdsp_demo/vdsp_sample
# 编译命令
cd /app/vdsp_demo/vdsp_sample && Makefile

# VDSP编译命令
cd vdsp_fw && ./make.sh

# VDSP侧输出文件:
{sdk_dir}/vdsp_fw/samples/libxi-sample/vdsp0-{build_type}
```

### 运行

### 支持平台

S100

#### 硬件环境搭建

NA

#### 运行参数说明

下面列出vdsp sample支持的输入参数，可以通过 `--help` 获取到所有参数的说明。

| 参数名         | 用法                                                                 | 默认值 |
|----------------|----------------------------------------------------------------------|--------|
| `dsp_id`       | `dsp_id=<0>` 指定 VDSP 0                                            | 0      |
| `vdsp_pathname`| `vdsp_pathname=*`，指定 VDSP firmware 路径                          | `/app/vdsp_demo/vdsp_sample/res/q8sample` |
| `sample-type`  | `sample-type=<0,1>`，指定 sample 类型：0 表示基础 sample，1 表示完整链路 sample | 1      |
| `help`         | 打印帮助信息                                                         | —      |

### 运行结果说明

``` bash
root@ubuntu:/app/vdsp_demo/vdsp_sample# ./vdsp_sample
vdsp_sample_cxt_s:
        vdsp_id:0
        vdsp_pathname:/app/vdsp_demo/vdsp_sample/res/q8sample
vdsp_call_params_s:
        cmd:xi-sample-flip
        type:1
        buf_width:128
        buf_height:128
        vdsp_buf0:0xfffc0000
        vdsp_buf1:0xfffd0000
recv_buf: 0
```

运行结束后会得到上述log输出，recv_buf返回0表示执行正常。

## VDSP API介绍

### 核间通信RPMSG接口

#### 核间通信RPMSG 头文件和链接库

  - VDSP侧

    头文件：hb_rpmsg_interface.h

    链接库：无

  - Acore侧

    头文件：hb_rpmsg_interface.h

    链接库：librpmsg.so

#### 核间通信RPMSG API 返回值 {#rpmsg_api_return_value}

VDSP侧

``` c
#define RPMSG_ERR_INVALID_ARG               (-1)
#define RPMSG_ERR_PATH_NOT_LINK             (-2)
#define RPMSG_ERR_SERVER_NOT_CONNECT        (-3)
#define RPMSG_ERR_OUT_OF_RES                (-4)
#define RPMSG_ERR_SEND_BUF_OVERSIZE         (-5)
#define RPMSG_ERR_NO_MEM                    (-6)
#define RPMSG_ERR_TIMEOUT                   (-7)
#define RPMSG_ERR_RECV_BUF_OVERFLOW         (-8)
#define RPMSG_ERR_INVALID_SERVER            (-9)
#define RPMSG_ERR_CRC_CHECK                 (-10)
```

Acore侧

``` c
#define RPMSG_ERR_INVALID_ARG           (-1)
#define RPMSG_ERR_INVALID_SERVER        (-2)
#define RPMSG_ERR_OUT_OF_RES            (-3)
#define RPMSG_ERR_KER_USR_TRANS         (-4)
#define RPMSG_ERR_SEND_BUF_OVERSIZE     (-5)
#define RPMSG_ERR_NO_MEM                (-6)
#define RPMSG_ERR_TIMEOUT               (-7)
#define RPMSG_ERR_SIGNAL_STOP           (-8)
#define RPMSG_ERR_RECV_BUF_OVERFLOW     (-9)
#define RPMSG_ERR_NOT_START_SERVER      (-10)
#define RPMSG_ERR_CRC_CHECK             (-11)
#define RPMSG_ERR_DRV_VERSION           (-12)
#define RPMSG_ERR_UNKNOWN_ERR           (-13)
```

#### 核间通信RPMSG(VDSP侧) API

##### hb_rpmsg_start_server

【函数声明】

``int32_t hb_rpmsg_start_server(const char* server_name, uint32_t flags, rl_ept_rx_cb_t rx_cb, void* rx_cb_data, uint32_t timeout, rpmsg_handle** handle);``

【参数描述】

  - \[IN\] server_name：服务名称
  - \[IN\] flags：通信特性
  - \[IN\] rx_cb：接收报文的callback函数
  - \[IN\] rx_cb_data：回调参数
  - \[IN\] timeout：接收超时时间（单位：ms）
  - \[OUT\] handle：代表rpmsg通信句柄

用户可用的服务名称：

VDSP0

  - dcore0_acore_heart.
  - dcore0_rpmsg_bpu.
  - dcore0_rpmsg_op.

VDSP1

  - dcore1_acore_heart.
  - dcore1_rpmsg_bpu.
  - dcore1_rpmsg_op.

:::warning
S100上没有VDSP1，使用时需要注意。
:::

【说明】

Flags参数的使用（使用位操作符）:

  - RPMSG_F_BLOCK 阻塞式传输
  - RPMSG_F_NONBLOCK 非阻塞式传输
  - RPMSG_F_CRC_CHECK 支持CRC check

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

开启rpmsg通信服务

【示例代码】

``` c
#include <hb_rpmsg_interface.h>
#include <hb_vdsp_log.h>
#include <platform.h>

#ifdef CNFIG_VDSP0
static char test_server_name[] = "dcore0_rpmsg_op";
#elif CNFIG_VDSP1
static char test_server_name[] = "dcore1_rpmsg_op";
#endif
#define CORE_COM_TX_RX_PAYLOAD_SIZE     (240)
#define MAX_PAYLAD (CORE_COM_TX_RX_PAYLOAD_SIZE)
static char buffer_rev[MAX_PAYLAD] = {0};
static char temp_hope[MAX_PAYLAD] ="I am test string,hope you can see me";

int32_t mycallback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    printf("[vdsp0],mycallback test! payload is 0x%x ,payload_len is %d, \
        src is %d\n",payload,payload_len,src);
    memcpy((void*)buffer_rev, payload, payload_len);
}

int32_t rpmsg_test()
{
    int32_t ret;
    rpmsg_handle *handle = NULL;

    //recv, block
    ret = hb_rpmsg_start_server(test_server_name, RPMSG_F_BLOCK |
    RPMSG_F_QUEUE_RECV, mycallback, NULL,0, &handle);
    if(ret != 0) {
        DSP_ERR("hb_rpmsg_start_server fail!,ret = %d\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    DSP_INF("%s server start,(block/recv) way!!\n", test_server_name);
    ret = hb_rpmsg_recv(handle, buffer_rev, MAX_PAYLAD);
    if (ret < 0) {
        DSP_ERR("hb_rpmsg_recv(block/recv) failed, ret = %d\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    DSP_INF("hb_rpmsg_recv(block/recv), buffer_rev : %s\n", buffer_rev);
    ret = hb_rpmsg_send(handle, buffer_rev, MAX_PAYLAD);
    if (ret < 0) {
        DSP_ERR("rpmsg_send error [[[%d]]]\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    ret = hb_rpmsg_stop_server(handle);
    if (ret < 0) {
        DSP_ERR("server:%s stop error [[[%d]]]\n", test_server_name, ret);
        return -1;
    }
    DSP_INF("%s server stop(block/recv)!!\n", test_server_name);
    return 0;
}
```

##### hb_rpmsg_stop_server

【函数声明】

``int32_t hb_rpmsg_stop_server(rpmsg_handle* handle);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

停止rpmsg通信服务

【示例代码】

参考 [hb_rpmsg_start_server](#hb_rpmsg_start_server)

##### hb_rpmsg_send

【函数声明】

``int32_t hb_rpmsg_send(const rpmsg_handle* handle, char *buf, uint32_t len);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[IN\] buf：要发送的数据地址
  - \[IN\] len：要发送的数据长度，可选值为: 1 \~ 240

【返回值】

  - 成功：实际发送的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

发送通信服务帧数据

【示例代码】

参考 [hb_rpmsg_start_server](#hb_rpmsg_start_server)

##### hb_rpmsg_recv

【函数声明】

``int32_t hb_rpmsg_recv(rpmsg_handle* handle, char* buf, uint32_t len);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[OUT\] buf：接收数据的地址
  - \[IN\] len：接收数据的长度，可选值为: 1 \~ 240

【返回值】

  - 成功：实际接收的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)
【功能描述】

接收通信服务帧数据

【示例代码】

参考 [hb_rpmsg_start_server](#hb_rpmsg_start_server)

##### hb_rpmsg_send_timeout

【函数声明】

``int32_t hb_rpmsg_send_timeout(const rpmsg_handle* handle, char* buf, uint32_t len, uint32_t timeout);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[IN\] buf：要发送的数据地址
  - \[IN\] len：要发送的数据长度，可选值为: 1 \~ 240
  - \[IN\] timeout：发送阻塞时长

【返回值】

  - 成功：实际发送的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

发送通信服务帧数据，带有timeout参数

【示例代码】

``` c
#include <hb_rpmsg_interface.h>
#include <hb_vdsp_log.h>
#include <platform.h>

#ifdef CNFIG_VDSP0
static char test_server_name[] = "dcore0_rpmsg_op";
#elif CNFIG_VDSP1
static char test_server_name[] = "dcore1_rpmsg_op";
#endif
#define CORE_COM_TX_RX_PAYLOAD_SIZE     (240)
#define MAX_PAYLAD (CORE_COM_TX_RX_PAYLOAD_SIZE)
static char buffer_rev[MAX_PAYLAD] = {0};
static char temp_hope[MAX_PAYLAD] ="I am test string,hope you can see me";

int32_t mycallback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    printf("[vdsp0],mycallback test! payload is 0x%x ,payload_len is %d, \
        src is %d\n",payload,payload_len,src);
    memcpy((void*)buffer_rev, payload, payload_len);
}

int32_t rpmsg_test()
{
    int32_t ret;
    rpmsg_handle *handle = NULL;

    //recv, block
    ret = hb_rpmsg_start_server(test_server_name, RPMSG_F_BLOCK |
    RPMSG_F_QUEUE_RECV, mycallback, NULL,0, &handle);
    if(ret != 0) {
        DSP_ERR("hb_rpmsg_start_server fail!,ret = %d\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    DSP_INF("%s server start,(block/recv) way!!\n", test_server_name);
    ret = hb_rpmsg_recv_timeout(handle, buffer_rev, MAX_PAYLAD, 1000u);
    if (ret < 0) {
        DSP_ERR("hb_rpmsg_recv(block/recv) failed, ret = %d\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    DSP_INF("hb_rpmsg_recv(block/recv), buffer_rev : %s\n", buffer_rev);
    ret = hb_rpmsg_send_timeout(handle, buffer_rev, MAX_PAYLAD, 1000u);
    if (ret < 0) {
        DSP_ERR("rpmsg_send error [[[%d]]]\n", ret);
        hb_rpmsg_stop_server(handle);
        return -1;
    }
    ret = hb_rpmsg_stop_server(handle);
    if (ret < 0) {
        DSP_ERR("server:%s stop error [[[%d]]]\n", test_server_name, ret);
        return -1;
    }
    DSP_INF("%s server stop(block/recv)!!\n", test_server_name);
    return 0;
}
```

##### hb_rpmsg_recv_timeout

【函数声明】

int32_t hb_rpmsg_recv_timeout(rpmsg_handle* handle, char* buf,
uint32_t len, uint32_t timeout);

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[OUT\] buf：接收数据的地址
  - \[IN\] len：接收数据的长度，可选值为: 1 \~ 240
  - \[IN\] timeout：接收阻塞时间

【返回值】

  - 成功：实际接收的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

接收通信服务帧数据，带有timeout参数

【示例代码】

参考 [hb_rpmsg_send_timeout](#hb_rpmsg_send_timeout)

#### 核间通信RPMSG(Acore侧) API

##### hb_rpmsg_connect_server

【函数声明】

``int32_t hb_rpmsg_connect_server(char *server_name, int flags, int timeout, rpmsg_handle **handle);``

【参数描述】

  - \[IN\] server_name：服务名称
  - \[IN\] flags：通信特性
  - \[IN\] timeout：阻塞模式下的超时时间（单位：ms）
  - \[OUT\] handle：返回rpmsg通信句柄

【说明】

Flags参数的使用（使用位操作符）:

  - RPMSG_F_BLOCK 阻塞式传输
  - RPMSG_F_NONBLOCK 非阻塞式传输
  - RPMSG_F_CRC_CHECK 支持CRC check

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

连接通讯服务

【示例代码】

``` c
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <rpmsg_lib.h>
void usage(char *s)
{
    printf("usage: %s server_name timeout(ms)\n", s);
}
int main(int argc, char *argv[])
{
    int ret = 0;
    struct rpmsg_handle *handle = NULL;
    char buffer[240] = {0};
    int timeout = 0;
    int cnt_max_connect = 10;
    uint32_t major, minor, patch;
    char *p;

    if (argc != 3) {
        usage(argv[0]);
        return -1;
    }
    timeout = atoi(argv[2]);
    if (timeout < 0) {
        printf("invalid timeout: %d\n", timeout);
        usage(argv[0]);
        return -1;
    }
    ret = hb_rpmsg_get_version(&major, &minor, &patch);
    if (ret < 0) {
        printf("rpmsg get version failed\n");
        return ret;
    } else {
        printf("rpmsg verion: %u.%u.%u\n", major, minor, patch);
    }
    reconnect:
    ret = hb_rpmsg_connect_server(argv[1], RPMSG_F_BLOCK, timeout,
    &handle);
    if (ret < 0) {
        --cnt_max_connect;
        if ((ret == RPMSG_ERR_NOT_START_SERVER) && (cnt_max_connect)) {
            printf("rpmsg_connect_server error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            usleep(100 * 1000);
            goto reconnect;
        } else {
            printf("rpmsg_connect_server error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            return -1;
        }
    }
    while (1) {
        ret = hb_rpmsg_send(handle, buffer, 240);
        if (ret < 0) {
            printf("rpmsg_send error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            hb_rpmsg_disconnect_server(handle);
            return -1;
        } else
            printf("recv log: %s\n", buffer);
            ret = hb_rpmsg_recv(handle, buffer, 240);
            if (ret < 0) {
                printf("rpmsg_recv error[%d]: %s\n", ret,
                    hb_rpmsg_error_message(ret));
                hb_rpmsg_disconnect_server(handle);
                return -1;
            }
        }
    }
    return 0;
}
```

##### hb_rpmsg_disconnect_server

【函数声明】

``int32_t hb_rpmsg_disconnect_server(rpmsg_handle* handle);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

断开通信服务

【示例代码】

参考 [hb_rpmsg_connect_server](#hb_rpmsg_connect_server)

##### hb_rpmsg_send

【函数声明】

``int32_t hb_rpmsg_send(const rpmsg_handle* handle, char *buf, int32_t len);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[IN\] buf：要发送的数据地址
  - \[IN\] len：要发送的数据长度，可选值为: 1 \~ 240

【返回值】

  - 成功：实际发送的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

发送特定通信服务帧数据

【示例代码】

参考 [hb_rpmsg_connect_server](#hb_rpmsg_connect_server)

##### hb_rpmsg_recv

【函数声明】

``int32_t hb_rpmsg_recv(rpmsg_handle* handle, char* buf, int32_t  len);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[OUT\] buf：接收数据的地址
  - \[IN\] len：接收数据的长度，可选值为: 1 \~ 240

【返回值】

  - 成功：实际接收的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

接收通信服务帧数据

##### hb_rpmsg_send_timeout

【函数声明】

``int32_t hb_rpmsg_send_timeout(const rpmsg_handle* handle, char *buf, int32_t len, int32_t timeout);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[IN\] buf：要发送的数据地址
  - \[IN\] len：要发送的数据长度，可选值为: 1 \~ 240
  - \[IN\] timeout：发送阻塞时长

【返回值】

  - 成功：实际发送的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

发送通信服务帧数据，带有timeout参数

【示例代码】

``` c
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <rpmsg_lib.h>
void usage(char *s)
{
    printf("usage: %s server_name timeout(ms)\n", s);
}
int main(int argc, char *argv[])
{
    int ret = 0;
    struct rpmsg_handle *handle = NULL;
    char buffer[240] = {0};
    int timeout = 0;
    int cnt_max_connect = 10;
    if (argc != 3) {
        usage(argv[0]);
        return -1;
    }
    timeout = atoi(argv[2]);
    if (timeout < 0) {
        printf("invalid timeout: %d\n", timeout);
        usage(argv[0]);
        return -1;
    }
    reconnect:
    ret = hb_rpmsg_connect_server(argv[1], RPMSG_F_BLOCK, timeout,
    &handle);
    if (ret < 0) {
        --cnt_max_connect;
        if ((ret == RPMSG_ERR_NOT_START_SERVER) && (cnt_max_connect)) {
            printf("rpmsg_connect_server error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            usleep(100 * 1000);
            goto reconnect;
        } else {
            printf("rpmsg_connect_server error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            return -1;
        }
    }
    while (1) {
        ret = hb_rpmsg_send_timeout(handle, buffer, 240, 1000);
        if (ret < 0) {
            printf("rpmsg_send error[%d]: %s\n", ret,
                hb_rpmsg_error_message(ret));
            hb_rpmsg_disconnect_server(handle);
            return -1;
        } else
            printf("recv log: %s\n", buffer);
            ret = hb_rpmsg_recv_timeout(handle, buffer, 240, 1000);
            if (ret < 0) {
                printf("rpmsg_recv error[%d]: %s\n", ret,
                    hb_rpmsg_error_message(ret));
                hb_rpmsg_disconnect_server(handle);
                return -1;
            }
        }
    }
    return 0;
}
```

##### hb_rpmsg_recv_timeout

【函数声明】

``int32_t hb_rpmsg_recv_timeout(rpmsg_handle* handle, char* buf, int32_t len, int32_t timeout);``

【参数描述】

  - \[IN\] handle：代表rpmsg通信句柄
  - \[OUT\] buf：接收数据的地址
  - \[IN\] len：接收数据的长度，可选值为: 1 \~ 240
  - \[IN\] timeout：接收阻塞时间

【返回值】

  - 成功：实际接收的字节数
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

接收通信服务帧数据，带有timeout参数

【示例代码】

参考 [hb_rpmsg_send_timeout](#hb_rpmsg_send_timeout_acore)

##### hb_rpmsg_get_version

【函数声明】

``int32_t hb_rpmsg_get_version(uint32_t* major, uint32_t* minor, uint32_t* patch);``

【参数描述】

  - \[OUT\] major：major版本号
  - \[OUT\] minor：minor版本号
  - \[OUT\] patch：patch

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [核间通信RPMSG API返回值](#rpmsg_api_return_value)

【功能描述】

获取rpmsg动态库版本号

【示例代码】

参考 [hb_rpmsg_connect_server](#hb_rpmsg_connect_server)

##### hb_rpmsg_error_message

【函数声明】

``const char* hb_rpmsg_error_message(int32_t error_code);``

【参数描述】

  - \[IN\] error_code：错误码

【返回值】

  - 成功：错误描述字符串
  - 失败：NULL

【功能描述】

将错误码转为错误描述字符串

【示例代码】

参考 [hb_rpmsg_connect_server](#hb_rpmsg_connect_server)

### 核间通信IPCFHAL接口

#### 核间通信IPCFHAL头文件和链接库

  - VDSP侧

    头文件：hb_ipcfhal_interface.h ipcf_hal_errno.h

    链接库：无

  - Acore侧

    头文件：hb_ipcfhal_interface.h ipcf_hal_errno.h

    链接库：libhbipcfhal.so

#### 核间通信IPCFHAL API返回值 {#vdsp_ipcfhal_api_return}

``` c
#define IPCF_HAL_E_OK           0/**< General OK*/
#define IPCF_HAL_E_NOK          1/**< General Not OK*/
#define IPCF_HAL_E_CONFIG_FAIL      2/**< Config fail*/
#define IPCF_HAL_E_WRONG_CONFIGURATION  3/**< Wrong configuration*/
#define IPCF_HAL_E_NULL_POINTER     4/**< A null pointer was passed as an argument*/
#define IPCF_HAL_E_PARAM_INVALID    5/**< A parameter was invalid*/
#define IPCF_HAL_E_LENGTH_TOO_SMALL 6/**< Length too small*/
#define IPCF_HAL_E_INIT_FAILED      7/**< Initialization failed*/
#define IPCF_HAL_E_UNINIT       8/**< Called before initialization*/
#define IPCF_HAL_E_BUFFER_OVERFLOW  9/**< Source address or destination address Buffer overflow*/
#define IPCF_HAL_E_ALLOC_FAIL       10/**< Source alloc fail*/
#define IPCF_HAL_E_TIMEOUT      11/**< Expired the time out*/
#define IPCF_HAL_E_REINIT       12/**< Re initilize*/
#define IPCF_HAL_E_BUSY         13/**< Busy*/
#define IPCF_HAL_E_CHANNEL_INVALID  14/**< Channel is invalid*/
```

API返回值为上述宏定义取负值。

#### 核间通信IPCFHAL(VDSP侧) API

##### hb_ipcfhal_init

【函数声明】

``int32_t hb_ipcfhal_init(ipcfhal_chan_t *channel);``

【参数描述】

  - \[IN\] channel：ipcfhal channel信息

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL初始化

【IPCFHAL初始化 示例代码】

:::info
完整代码编译和运行参考``/app/vdsp_demo/vdsp_ipcfhal_sample/READTME.md``
:::

``` c
#define CFG_FILE "config.json"
#define DATA_LEN 1024

static int32_t ipcfhal_thread_func(void *arg, int32_t unused) {
	int32_t ret;
	char *ch_name = "cpu-vdsp-ins0ch0";
	ipcfhal_chan_t channel;
	uint8_t rx_data[DATA_LEN] = {0};
	uint8_t tx_data[DATA_LEN] = {0};
	int32_t timeout = -1;
	uint32_t major, minor, patch;

	ret = vdsp_server_ready(BOOT_COMPLETE_EVENT_MASK_BIT1 | BOOT_COMPLETE_EVENT_MASK_BIT2);

	ret = hb_ipcfhal_get_version(&major, &minor, &patch);
	if (ret < 0) {
		return ret;
	} else {
		printf("ipcfhal verion: %u.%u.%u\n", major, minor, patch);
	}

	ret = hb_ipcfhal_getchan_byjson(ch_name, &channel, CFG_FILE);
	if (ret < 0)
		return ret;
	ret = hb_ipcfhal_init(&channel);
	if (ret < 0)
		return ret;
	ret = hb_ipcfhal_config(&channel);
	if (ret < 0)
		return ret;

	while (hb_is_thread_stop() != 1) {
		memset(tx_data, 0x55, DATA_LEN);
		ret = hb_ipcfhal_send(tx_data, DATA_LEN, &channel);
		if (ret < 0)
			continue;

		memset((void *)rx_data, 0, DATA_LEN);
		ret = hb_ipcfhal_recv(rx_data, DATA_LEN, timeout, &channel);
		if (ret < 0)
			continue;
		for (int i = 0; i < DATA_LEN; i++) {
			if (rx_data[i] != 0x55) {
				DSP_ERR("recv rx_data[%d] err. 0x%x\n", i, rx_data[i]);
				break;
			}
		}
		xos_thread_sleep_msec(2);
	}

	return ret;
}
```

##### hb_ipcfhal_getchan_byjson

【函数声明】

``int32_t hb_ipcfhal_getchan_byjson(const char *name, ipcfhal_chan_t *channel, const char *json_file);``

【参数描述】

  - \[IN\] name：ipcfhal channel 名字
  - \[IN\] json_file：ipcfhal json配置文件
  - \[OUT\] channel：ipcfhal channel信息

【返回值】

  - 成功: 通道id
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL从json配置文件获取channel

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_config

【函数声明】

int32_t hb_ipcfhal_config(ipcfhal_chan_t *channel);

【参数描述】

  - \[IN\] channel：ipcfhal channel信息

【返回值】

  - 成功: \>=0
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL 配置channel

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_send

【函数声明】

``int32_t hb_ipcfhal_send(const uint8_t *data, int16_t length, ipcfhal_chan_t *channel);``

【参数描述】

  - \[IN\] data：发送的数据buffer
  - \[IN\] length：发送的buffer长度
  - \[IN\] channel：ipcfhal channel信息

【返回值】

  - 成功: 实际发送的字节数
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL发送消息

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_recv

【函数声明】

``int32_t hb_ipcfhal_recv(uint8_t *data, int16_t length, int32_t  timeout, ipcfhal_chan_t *channel);``

【参数描述】

  - \[IN\] length：buffer最大长度
  - \[IN\] timeout：0 非阻塞, \>0 阻塞超时ms, -1 阻塞.
  - \[IN\] channel：ipcfhal channel信息
  - \[OUT\] data：接收的数据buffer

【返回值】

  - 成功: 实际接收的字节数
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL接收消息

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_deinit

【函数声明】

``int32_t hb_ipcfhal_deinit(ipcfhal_chan_t *channel);``

【参数描述】

  - \[IN\] channel：ipcfhal channel信息

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL反初始化

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_trans_err

【函数声明】

``int32_t hb_ipcfhal_trans_err(int32_t err_code, char **err_str);``

【参数描述】

  - \[IN\] err_code：返回值错误编码
  - \[OUT\] err_str：转换错误字符串

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

转换IPCFHAL错误码为错误字符串

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_get_version

【函数声明】

``int32_t hb_ipcfhal_get_version(uint32_t *major, uint32_t *minor, uint32_t *patch);``

【参数描述】

  - \[OUT\] major：libhbipcfhal major 版本号
  - \[OUT\] minor：libhbipcfhal minor 版本号
  - \[OUT\] patch：libhbipcfhal patch 版本号

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

获取IPCFHAL库的版本号

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init)

##### hb_ipcfhal_register_callback

【函数声明】

``int32_t hb_ipcfhal_register_callback(uint8_t *user_data, user_cb_t user_cb, ipcfhal_chan_t *channel);``

【参数描述】

  - \[IN\] user_data：用户数据
  - \[IN\] user_cb：用户回调函数
  - \[IN\] channel：ipcfhal channel信息

【返回值】

  - 成功: =0
  - 失败: \!0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL注册回调函数，VDSP侧接收方式配置为回调方式；参数为NULL时，注销回调函数，VDSP侧接收方式配置为recv方式；同一实例多通道使用callback时会阻塞执行，不同实例通道间callback会并发执行。

【IPCFHAL注册回调函数 示例代码】

``` c
static void test_ipcfhal_callback_func(uint8_t *userdata, int32_t instance, int32_t chan_id,
            uint8_t *buf, uint64_t size)
{
    printf("userdata[0x%llx] ins[%d] ch[%d] buf[0x%llx] size[%llu]\n",
        (uint64_t)userdata, instance, chan_id, (uint64_t)buf, size);
    *userdata = 1u;
}

static int32_t hb_libipchal_register_callback_func(void)
{
    int32_t ret = 0;
    uint8_t user_flag = 0;
    uint32_t cnt = 100u;
    uint8_t rx_data[1024] = {0};
    ipcfhal_chan_t channel;

    printf("hb_libipchal_register_callback_func start\n");
    (void)hb_ipcfhal_getchan_byjson("cpu-vdsp-ch0", &channel, "ipcf_config.json");
    (void)hb_ipcfhal_init(&channel);
    (void)hb_ipcfhal_config(&channel);
    (void)hb_ipcfhal_register_callback(&user_flag, test_ipcfhal_callback_func, &channel);

    while ((user_flag == 0) && (cnt--) ) {
        xos_thread_sleep_msec(100);
    }

    (void)hb_ipcfhal_register_callback(NULL, NULL, &channel);
    (void)hb_ipcfhal_deinit(&channel);
    printf("hb_libipchal_register_callback_func end\n");

    return 0;
}
```

#### 核间通信IPCFHAL(Acore侧) API

##### hb_ipcfhal_init {#hb_ipcfhal_init_arm}

【函数声明】

``int32_t hb_ipcfhal_init(ipcfhal_chan_t *channel);``

【参数描述】

-   \[IN\] channel：ipcfhal通道

【返回值】

-   成功: =0
-   失败: !0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL初始化

【IPCFHAL初始化 示例代码】

:::info
完整代码编译和运行参考``/app/vdsp_demo/vdsp_ipcfhal_sample/READTME.md``
:::

``` c
{
  "log_level": 0,
  "config_num": 2,
  "config_num_max":256,
  "config_0": {
    "name": "cpu2vdsp_ins22ch0",
    "instance": 22,
    "channel": 0,
    "pkg_size_max": 4096,
    "fifo_size": 64000,
    "fifo_type": 0,
    "ipcf_dev_path":"/dev/ipcdrv",
    "ipcf_dev_name":"ipcdrv"
  },
  "config_1": {
    "name": "cpu2vdsp_ins22ch1",
    "instance": 22,
    "channel": 1,
    "pkg_size_max": 4096,
    "fifo_size": 64000,
    "fifo_type": 0,
    "ipcf_dev_path":"/dev/ipcdrv",
    "ipcf_dev_name":"ipcdrv"
  }
}

/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2022-2024, D-Robotics Co., Ltd.
 *                   All rights reserved.
 *************************************************************************/

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <logging.h>
#include <hb_vdsp_mgr.h>
#include "hb_ipcf_hal.h"
#include "ipcf_hal_errno.h"
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define SMP_1KB_LEN		(1024)/**< sample send and recv data length*/
#define SMP_TIMEOUT_VAL		(1000)/**< sample recv timeout value*/
#define SMP_USLEEP_1MS		(1000u)/**< sample sleep 1ms*/
#define SMP_USLEEP_10MS		(10000u)/**< sample sleep 10ms*/
#define SMP_SLEEP_1S		(1u)/**< sample sleep 1s*/
#define SMP_RUN_TIME		(10l)/**< sample run time, unit: s*/
#define SMP_100CNT		(100u)/**< sample 100 counts*/
#define SMP_CHAN_NUM		(1)/**< sample channel number*/
#define SMP_THREAD_NUM		(SMP_CHAN_NUM * 2)/**< sample thread number*/
#define SMP_ROLL_POS		(4)/**< sample rolling count position*/
#define SMP_CFG_FILE	"/app/vdsp_demo/vdsp_ipcfhal_sample/ipcfhal_config.json"

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
void *send_pthread(void *argv);
void *recv_pthread(void *argv);

/******************************************************************************/
/*---------------------------- Local Function -------------------------------*/
/******************************************************************************/
/*thread arg*/
struct th_arg_t {
  ipcfhal_chan_t ch;/**< ipcfhal channel*/
  bool Is_Enable;/**< thread status*/
  uint32_t data_len;/**< send data length*/
  uint32_t sleep_time;/**< send period*/
  bool result;/**< thread running result*/
};

volatile sig_atomic_t g_running = true;

void signal_handler(int sig) {
  if (sig == SIGINT)
    g_running = false;
}

static void *tx_pthread(void *argv)
{
  int32_t ret = 0;
  struct th_arg_t *pth_arg = (struct th_arg_t *)argv;
  uint8_t tx_data[SMP_1KB_LEN] = {0u};

  if (pth_arg == NULL) {
    return NULL;
  }

  while (pth_arg->Is_Enable) {
    memset(tx_data, 0x55, SMP_1KB_LEN);
    ret = hb_ipcfhal_send(tx_data, pth_arg->data_len, &pth_arg->ch);
    if (ret != (int32_t)pth_arg->data_len) {/*return data_len*/
      pr_err("Ins[%u]Ch[%u] send failed\n",
        pth_arg->ch.instance, pth_arg->ch.id);
      pth_arg->result = false;
    }
    usleep(pth_arg->sleep_time);
  }

  pr_info("[%s End]Ins[%u]Ch[%u]\n",
    __func__, pth_arg->ch.instance, pth_arg->ch.id);

  return NULL;
}

static void *rx_pthread(void *argv)
{
  int32_t ret = 0;
  struct th_arg_t *pth_arg = (struct th_arg_t *)argv;
  uint8_t data[SMP_1KB_LEN] = {0u};
  int32_t timeout = SMP_TIMEOUT_VAL;

  if (pth_arg == NULL) {
    return NULL;
  }

  while (pth_arg->Is_Enable) {
    memset(data, 0u, pth_arg->data_len);
    ret = hb_ipcfhal_recv(data, SMP_1KB_LEN, timeout, &pth_arg->ch);
    if (ret != -IPCF_HAL_E_TIMEOUT && ret < 0) {
      pr_err("Ins[%u]Ch[%u] recv failed: %d\n",
        pth_arg->ch.instance, pth_arg->ch.id, ret);
      pth_arg->Is_Enable = false;
      pth_arg->result = false;
      break;
    }

    if (ret >= 0) {
      for (int i = 0; i < SMP_1KB_LEN; i++) {
        if (data[i] != 0x55) {
          pr_err("recv data[%d] err. 0x%x\n", i, data[i]);
          break;
        }
      }
    } else if (ret == -IPCF_HAL_E_TIMEOUT) {
      pth_arg->result = false;
    }

    usleep(pth_arg->sleep_time);
  }
  pr_info("[%s End]Ins[%u]Ch[%u]\n",
    __func__, pth_arg->ch.instance, pth_arg->ch.id);

  return NULL;
}

/******************************************************************************/
/*---------------------------- Global Function -------------------------------*/
/******************************************************************************/
int main(int argc, char *argv[])
{
  const char *json_path = SMP_CFG_FILE;
  struct timespec test_start_time, test_cur_time;
  int32_t ret = 0;
  bool result = true;
  pthread_t thread_tid[SMP_THREAD_NUM];
  struct th_arg_t thread_arg[SMP_CHAN_NUM];
  const char *chan_name[SMP_CHAN_NUM] = {
    "cpu2vdsp_ins22ch0"
  };
  int32_t vdsp_id = 0;
  int32_t timeout = 1000;
  const char *vdsp_pathname = "/app/vdsp_demo/vdsp_ipcfhal_sample/res/q8sample";

  signal(SIGINT, signal_handler);

  ret = hb_vdsp_init(vdsp_id);
  if (ret) {
    pr_err("vdsp init failed, ret %d\n", ret);
    return ret;
  }

  ret = hb_vdsp_start(vdsp_id, timeout, vdsp_pathname);
  if (ret) {
    pr_err("vdsp start failed, ret %d\n", ret);
    return ret;
  }

  for (int32_t i = 0; i < SMP_CHAN_NUM; i++) {
    /*step1: get channel info by json file*/
    ret = hb_ipcfhal_getchan_byjson(chan_name[i], &thread_arg[i].ch, json_path);
    if (ret < 0) {/* return channel id*/
      pr_err("%s not found, ret %d\n", chan_name[i], ret);
      hb_vdsp_stop(vdsp_id);
      hb_vdsp_deinit(vdsp_id);
      return ret;
    }

    /*step2: init channel*/
    ret = hb_ipcfhal_init(&thread_arg[i].ch);
    if (ret < 0) {
      pr_err("%s init failed, ret %d\n", thread_arg[i].ch.name, ret);
      for (int32_t j = 0; j < i; j++) {
        hb_ipcfhal_deinit(&thread_arg[j].ch);
      }
      hb_vdsp_stop(vdsp_id);
                        hb_vdsp_deinit(vdsp_id);
      return ret;
    }

    /*step3: config channel*/
    ret = hb_ipcfhal_config(&thread_arg[i].ch);
    if (ret < 0) {
      pr_err("%s config failed, ret %d\n", thread_arg[i].ch.name, ret);
      for (int32_t j = 0; j <= i; j++) {
        hb_ipcfhal_deinit(&thread_arg[j].ch);
      }
      hb_vdsp_stop(vdsp_id);
                        hb_vdsp_deinit(vdsp_id);
      return ret;
    }

    /*step4: set thread arg*/
    thread_arg[i].sleep_time = SMP_USLEEP_10MS;
    thread_arg[i].data_len = SMP_1KB_LEN;
    thread_arg[i].Is_Enable = true;
    thread_arg[i].result = true;
    (void)pthread_create(&thread_tid[i], NULL, rx_pthread, &thread_arg[i]);
    (void)pthread_create(&thread_tid[i+1], NULL, tx_pthread, &thread_arg[i]);
  }

  /*step5: run send and recv thread*/
  clock_gettime(CLOCK_MONOTONIC, &test_start_time);
  clock_gettime(CLOCK_MONOTONIC, &test_cur_time);
  while (g_running) {
    for (int32_t i = 0; i < SMP_CHAN_NUM; i++) {
      if (thread_arg[i].Is_Enable == false)
        break;
    }
    if (test_cur_time.tv_sec - test_start_time.tv_sec > SMP_RUN_TIME)
      break;

    sleep(SMP_SLEEP_1S);
    clock_gettime(CLOCK_MONOTONIC, &test_cur_time);
  }

  /*step6: deinit channel*/
  for (int32_t i = 0; i < SMP_CHAN_NUM; i++) {
    void *ret;
    thread_arg[i].Is_Enable = false;//stop thread
    usleep(SMP_USLEEP_10MS);
    pthread_join(thread_tid[i], &ret);
    pthread_join(thread_tid[i+1], &ret);
    (void)hb_ipcfhal_deinit(&thread_arg[i].ch);

    result = (result && thread_arg[i].result);
  }

  if (result == false) {
    pr_err("[ipc-sample] ipc sample running failed\n");
  } else {
    pr_info("[ipc-sample] ipc sample running success\n");
  }

  ret = hb_vdsp_stop(vdsp_id);
  if (ret) {
    pr_err("vdsp stop failed, ret %d\n", ret);
    return ret;
  }

  ret = hb_vdsp_deinit(vdsp_id);
  if (ret) {
    pr_err("vdsp deinit failed, ret %d\n", ret);
    return ret;
  }
  return ret;
}
```

##### hb_ipcfhal_getchan_byjson

【函数声明】

``int32_t hb_ipcfhal_getchan_byjson(const char *name, ipcfhal_chan_t *channel, const char *json_file);``

【参数描述】

-   \[IN\] name：ipcfhal channel 名字
-   \[IN\] json_file：ipcfhal json配置文件
-   \[OUT\] channel：ipcfhal channel信息

【返回值】

-   成功: \>=0
-   失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL从json配置文件获取channel，channel初始化后，不支持用户修改结构体成员值。

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_config

【函数声明】

``int32_t hb_ipcfhal_config(ipcfhal_chan_t *channel);``

【参数描述】

-   \[IN\] channel：ipcfhal channel信息

【返回值】

-   成功: \>=0
-   失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL 配置channel

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_send

【函数声明】

``int32_t hb_ipcfhal_send(const uint8_t *data, uint32_t length, ipcfhal_chan_t *channel);``

【参数描述】

-   \[IN\] data：发送的数据buffer
-   \[IN\] length：发送的buffer长度
-   \[IN\] channel：channel信息

【返回值】

-   成功: 实际发送的字节数
-   失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL发送消息

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_recv

【函数声明】

``int32_t hb_ipcfhal_recv(uint8_t *data, uint32_t length, int32_t timeout, ipcfhal_chan_t *channel);``

【参数描述】

-   \[IN\] length：buffer最大长度
-   \[IN\] timeout：0 非阻塞, \>0 阻塞超时ms, -1 阻塞
-   \[IN\] channel：channel信息
-   \[OUT\] data：接收的数据buffer

【返回值】

-   成功: 实际接收的字节数
-   失败: \<0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL接收消息

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_deinit

【函数声明】

``int32_t hb_ipcfhal_deinit(ipcfhal_chan_t *channel);``

【参数描述】

-   \[IN\] channel：channel信息

【返回值】

-   成功: =0
-   失败: !0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

IPCFHAL反初始化

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_trans_err

【函数声明】

``int32_t hb_ipcfhal_trans_err(int32_t err_code, char **err_str);``

【参数描述】

-   \[IN\] err_code：返回值错误编码
-   \[OUT\] err_str：转换错误字符串

【返回值】

-   成功: =0
-   失败: !0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

转换IPCFHAL错误码为错误字符串

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

##### hb_ipcfhal_get_version

【函数声明】

``int32_t hb_ipcfhal_get_version(uint32_t *major, uint32_t *minor, uint32_t *patch);``

【参数描述】

-   \[OUT\] major：libhbipcfhal major 版本号
-   \[OUT\] minor：libhbipcfhal minor 版本号
-   \[OUT\] patch：libhbipcfhal patch 版本号

【返回值】

-   成功: =0
-   失败: !0，参考 [核间通信IPCFHAL API返回值](#vdsp_ipcfhal_api_return)

【功能描述】

获取IPCFHAL库的版本号

【示例代码】

参考 [hb_ipcfhal_init](#hb_ipcfhal_init_arm)

### HEAP分配接口

#### HEAP分配 头文件和链接库

  - VDSP侧

    头文件：hb_mem_allocator.h

    链接库：无

#### HEAP分配 API 返回值 {#vdsp_heap_api_return}

``` c
#define HB_MEM_OK                                    0
#define HB_MEM_ERR_INVALID_PARAMS                    (-16777215)
#define HB_MEM_ERR_REPEAT_INIT                       (-16777214)
#define HB_MEM_ERR_HEAP_BUSY                         (-16777213)
#define HB_MEM_ERR_INSUFFICIENT_MEM                  (-16777212)
```

#### HEAP分配 API

##### hb_mem_heap_initialize

【函数声明】

``int32_t hb_mem_heap_initialize(hb_mem_heap_t heap_id, void *start_vaddr, size_t heap_size, uint32_t align);``

【参数描述】

  - \[IN\] heap_id：有效的heap序号，可选值: 0, 1
  - \[IN\] start_vaddr：heap起始地址
  - \[IN\] heap_size：heap大小
  - \[IN\] align：heap空间对齐的大小

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [HEAP分配 API 返回值](#vdsp_heap_api_return)

【功能描述】

初始化heap分配接口

【示例代码】

``` c
int32_t ret;
void *start_vaddr[2] = {(void *)heapDRAM0, heapDRAM1};
size_t heap_size[2] = {sizeof(heapDRAM0), sizeof(heapDRAM1)};
ret = hb_mem_heap_initialize(HB_MEMRY_HEAP_DRAM_0, start_vaddr[0],
    heap_size[0], 0x10000);
if (ret < 0) {
    printf("hb_mem_heap_initialize failed ret = %d\n", ret);
    return ret;
}
ret = hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
if (ret < 0) {
    printf("hb_mem_heap_deinitialize failed ret = %d\n", ret);
    return ret;
}
return 0;
```

##### hb_mem_heap_deinitialize

【函数声明】

``int32_t hb_mem_heap_deinitialize(hb_mem_heap_t heap_id);``

【参数描述】

  - \[IN\] heap_id：有效的heap序号，可选值: 0, 1

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [HEAP分配 API 返回值](#vdsp_heap_api_return)

【功能描述】

解初始化heap分配接口

【示例代码】

``` c
int32_t ret;
void *start_vaddr[2] = {(void *)heapDRAM0, heapDRAM1};
size_t heap_size[2] = {sizeof(heapDRAM0), sizeof(heapDRAM1)};
ret = hb_mem_heap_initialize(HB_MEMRY_HEAP_DRAM_0, start_vaddr[0],
    heap_size[0], 0x10000);
if (ret < 0) {
    printf("hb_mem_heap_initialize failed ret = %d\n", ret);
    return ret;
}
ret = hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
if (ret < 0) {
    printf("hb_mem_heap_deinitialize failed ret = %d\n", ret);
    return ret;
}
return 0;
```

##### hb_mem_heap_alloc

【函数声明】

``int32_t hb_mem_heap_alloc(hb_mem_heap_t heap_id, size_t req_size, uint32_t align, void ** vaddr);``

【参数描述】

  - \[IN\] heap_id：有效的heap序号，可选值: 0, 1
  - \[IN\] reg_size：分配的字节大小
  - \[IN\] align：heap空间对齐的大小
  - \[OUT\] vaddr：分配buffer的起始地址

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [HEAP分配 API 返回值](#vdsp_heap_api_return)

【功能描述】

heap分配接口

【示例代码】

``` c
int32_t ret;
void *start_vaddr[2] = {(void *)heapDRAM0, heapDRAM1};
size_t heap_size[2] = {sizeof(heapDRAM0), sizeof(heapDRAM1)};
uint32_t malloc_alignment = 64;
void *vaddr = NULL;
ret = hb_mem_heap_initialize(HB_MEMRY_HEAP_DRAM_0, start_vaddr[0],
heap_size[0], 0x10000);
if (ret < 0) {
    printf("hb_mem_heap_initialize failed ret = %d\n", ret);
    return ret;
}
ret = hb_mem_heap_alloc(HB_MEMRY_HEAP_DRAM_0, heap_size[0],
    malloc_alignment, &vaddr);
if (ret < 0) {
    printf("hb_mem_heap_alloc failed ret = %d\n", ret);
    hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
    return ret;
}
ret = hb_mem_heap_free(HB_MEMRY_HEAP_DRAM_0, vaddr);
if (ret < 0) {
    printf("hb_mem_heap_free failed ret = %d\n", ret);
    hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
    return ret;
}
ret = hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
return 0;
```

##### hb_mem_heap_free

【函数声明】

``int32_t hb_mem_heap_free(hb_mem_heap_t heap_id, void *vaddr);``

【参数描述】

  - \[IN\] heap_id：有效的heap序号，可选值: 0, 1
  - \[IN\] vaddr：已分配buffer的起始地址

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [HEAP分配 API 返回值](#vdsp_heap_api_return)

【功能描述】

释放heap已分配的空间

【示例代码】

``` c
int32_t ret;
void *start_vaddr[2] = {(void *)heapDRAM0, heapDRAM1};
size_t heap_size[2] = {sizeof(heapDRAM0), sizeof(heapDRAM1)};
uint32_t malloc_alignment = 64;
void *vaddr = NULL;
ret = hb_mem_heap_initialize(HB_MEMRY_HEAP_DRAM_0, start_vaddr[0],
heap_size[0], 0x10000);
if (ret < 0) {
    printf("hb_mem_heap_initialize failed ret = %d\n", ret);
    return ret;
}
ret = hb_mem_heap_alloc(HB_MEMRY_HEAP_DRAM_0, heap_size[0],
    malloc_alignment, &vaddr);
if (ret < 0) {
    printf("hb_mem_heap_alloc failed ret = %d\n", ret);
    hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
    return ret;
}
ret = hb_mem_heap_free(HB_MEMRY_HEAP_DRAM_0, vaddr);
if (ret < 0) {
    printf("hb_mem_heap_free failed ret = %d\n", ret);
    hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
    return ret;
}
ret = hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
if (ret < 0) {
    printf("hb_mem_heap_deinitialize failed ret = %d\n", ret);
    return ret;
}
return 0;
```

##### hb_mem_heap_get_status

【函数声明】

``int32_t hb_mem_heap_get_status(hb_mem_heap_t heap_id, hb_mem_heap_status_t *heap_status);``

【参数描述】

  - \[IN\] heap_id：有效的heap序号，可选值: 0, 1
  - \[OUT\] heap_status：heap状态信息

【返回值】

  - 成功: =0
  - 失败: \<0，参考 [HEAP分配 API 返回值](#vdsp_heap_api_return)

【功能描述】

获取heap分配状态

【示例代码】

``` c
int32_t ret;
void *start_vaddr[2] = {(void *)heapDRAM0, heapDRAM1};
size_t heap_size[2] = {sizeof(heapDRAM0), sizeof(heapDRAM1)};
hb_mem_heap_status_t heap_status = {0, };
ret = hb_mem_heap_initialize(HB_MEMRY_HEAP_DRAM_0, start_vaddr[0],
    heap_size[0], 0x10000);
if (ret < 0) {
    printf("hb_mem_heap_initialize failed ret = %d\n", ret);
    return ret;
}
ret = hb_mem_heap_get_status(HB_MEMRY_HEAP_DRAM_0, &heap_status);
if (ret < 0) {
    printf("hb_mem_heap_get_status failed ret = %d\n", ret);
    hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
    return ret;
}
ret = hb_mem_heap_deinitialize(HB_MEMRY_HEAP_DRAM_0);
if (ret < 0) {
    printf("hb_mem_heap_deinitialize failed ret = %d\n", ret);
    return ret;
}
return 0;
```

### VDSP启停控制接口 {#vdsp_boot_api}

#### VDSP启停控制 头文件和链接库

  - Acore侧

    头文件：hb_vdsp_mgr.h

    链接库：libvdsp.so

#### VDSP启停控制 API返回值 {#vdsp_boot_api_return}

``` c
#define HB_VDSP_OK                      (0)
#define HB_VDSP_OPEN_DEV_ERR                    (-1)
#define HB_VDSP_START_IOCTL_ERR                 (-2)
#define HB_VDSP_STOP_IOCTL_ERR                  (-4)
#define HB_VDSP_GET_STATUS_IOCTL_ERR                (-5)
#define HB_VDSP_RESET_IOCTL_ERR                 (-6)
#define HB_VDSP_PARAM_INVALID                   (-7)
#define HB_VDSP_SET_PATH_ERR                    (-8)
#define HB_VDSP_SET_NAME_ERR                    (-9)
#define HB_VDSP_CLOSE_DEV_ERR                   (-10)
#define HB_VDSP_ERR_IOC_GET_VERSION_INFO            (-11)
#define HB_VDSP_ERR_IOC_MEM_ALLOC               (-12)
#define HB_VDSP_ERR_IOC_MEM_FREE                (-13)
#define HB_VDSP_ERR_IOC_SMMU_MAP                (-14)
#define HB_VDSP_ERR_IOC_SMMU_UNMAP              (-15)
#define HB_VDSP_ERR_INIT                    (-16)
#define HB_VDSP_ERR_DEINIT                  (-17)
#define HB_VDSP_ERR_NOT_INITED                  (-18)
#define HB_VDSP_ERR_ALLOC_COM_BUF               (-19)
#define HB_VDSP_ERR_FREE_MEMBUF                 (-20)
#define HB_VDSP_ERR_GET_COM_BUF_WITH_VADDR          (-21)
#define HB_VDSP_ERR_SMMU_MAP                    (-22)
#define HB_VDSP_ERR_SMMU_UNMAP                  (-23)
#define HB_VDSP_ERR_CHECK_VERSION               (-24)
#define HB_VDSP_ERR_INSUFFICIENT_MEM                (-25)
#define HB_VDSP_ERR_MISMATCH_INTERFACE              (-26)
#define HB_VDSP_ERR_RBTREE_CREATE_NODE              (-27)
#define HB_VDSP_ERR_RBTREE_INSERT_NODE              (-28)
#define HB_VDSP_ERR_RBTREE_SEARCH_NODE              (-29)
```

#### VDSP启停控制 API

##### hb_vdsp_get_version

【函数声明】

``int32_t hb_vdsp_get_version(uint32_t *major, uint32_t *minor, uint32_t *patch);``

【参数描述】

  - \[OUT\] major：libvdsp major 版本号
  - \[OUT\] minor：libvdsp minor 版本号
  - \[OUT\] patch：libvdsp patch 版本号

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

获取VDSP启停控制库的版本号

【获取VDSP库版本号 示例代码】

``` c
#include <hb_vdsp_mgr.h>

int32_t boot_lib_version_test(int32_t dsp_id)
{
    int32_t ret = 0;
    uint32_t major, minor, patch;

    ret = hb_vdsp_get_version(&major, &minor, &patch);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_get_version ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    printf("%s Get version %d.%d.%d.\n", TAG, major, minor, patch);

    return ret;
}
```

##### hb_vdsp_init

【函数声明】

``int32_t hb_vdsp_init(int32_t dsp_id);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

初始化VDSP启停控制库

【初始化VDSP库 示例代码】

``` c
#include <hb_vdsp_mgr.h>

int32_t boot_lib_init_test(int32_t dsp_id)
{
    int32_t ret = 0;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    return ret;
}
```

##### hb_vdsp_deinit

【函数声明】

``int32_t hb_vdsp_deinit(int32_t dsp_id);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

释放VDSP启停控制库

【示例代码】

参考 [hb_vdsp_init](#hb_vdsp_init)

##### hb_vdsp_start

【函数声明】

``int32_t hb_vdsp_start(int32_t dsp_id, int32_t timeout, const char *pathname);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] timeout：0为异步; -1为同步; 大于0为同步timeout等待时间,ms单位
  - \[IN\] pathname：vdsp镜像完整路径，包含镜像名字

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

启动 VDSP

【同步启动VDSP示例代码】

``` c
#include <hb_vdsp_mgr.h>

#define VDSP_BOOT_MODE_ASYNC        (0)
#define VDSP_BOOT_MODE_SYNC         (-1)

int32_t boot_lib_sync_test(int32_t dsp_id, const char* pathname)
{
    int32_t ret = 0;
    int32_t status = 0;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_start(dsp_id, VDSP_BOOT_MODE_SYNC, pathname);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_start ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_get_status(dsp_id, &status);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_get_status ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_stop(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_stop ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    printf("%s dsp%d boot sync pass !\n", __func__, dsp_id);
    return ret;
}

boot_lib_sync_test(0, "/app/vdsp_demo/vdsp_sample/res/q8sample");
boot_lib_sync_test(1, "/app/vdsp_demo/vdsp_sample/res/q8sample");
```

【异步启动VDSP示例代码】

``` c
#include <hb_vdsp_mgr.h>
#include <poll.h>

#define VDSP_BOOT_MODE_ASYNC        (0)
#define VDSP_BOOT_MODE_SYNC         (-1)

int32_t boot_lib_async_test(int32_t dsp_id, const char* pathname)
{
    int32_t ret = 0;
    struct pollfd pollfds;
    int32_t fd;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_start(dsp_id, VDSP_BOOT_MODE_ASYNC, pathname);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_start ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_get_fd(dsp_id, &fd);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_get_fd ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    pollfds.fd = fd;
    pollfds.events = POLLIN | POLLRDNORM;

    printf("%s dsp%d poll entry pollfds.events=0x%x fd-%d.\n", __func__, dsp_id, pollfds.events, fd);

    ret = poll(&pollfds, 1, 600);
    printf("%s dsp%d poll return pollfds.revents=0x%x ret-%d.\n", __func__, dsp_id, pollfds.revents, ret);
    if (ret <= 0) {
        printf("%s dev poll err: %d, %s\n", __func__, errno, strerror(errno));
        hb_vdsp_close_fd(fd);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_close_fd(fd);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_close_fd ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_stop(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_stop ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    printf("%s dsp%d boot async pass !\n", __func__, dsp_id);
    return ret;
}

boot_lib_async_test(0, "/app/vdsp_demo/vdsp_sample/res/q8sample");
boot_lib_async_test(1, "/app/vdsp_demo/vdsp_sample/res/q8sample");
```

##### hb_vdsp_stop

【函数声明】

``int32_t hb_vdsp_stop(int32_t dsp_id);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

停止 VDSP

【示例代码】

参考 [hb_vdsp_start](#hb_vdsp_start_sync)

##### hb_vdsp_get_status

【函数声明】

``int32_t hb_vdsp_get_status(int32_t dsp_id, int32_t *status);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[OUT\] status：VDSP 运行状态

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

获取 VDSP 运行状态

::: tip
该接口不支持多进程、多线程并发使用。
:::

【示例代码】

参考 [hb_vdsp_start](#hb_vdsp_start_sync)

##### hb_vdsp_reset

【函数声明】

``int32_t hb_vdsp_reset(int32_t dsp_id);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

复位 VDSP

【示例代码】

``` c
#include <hb_vdsp_mgr.h>

#define VDSP_BOOT_MODE_ASYNC        (0)
#define VDSP_BOOT_MODE_SYNC         (-1)

int32_t boot_lib_reset_test(int32_t dsp_id, const char* pathname)
{
    int32_t ret = 0;
    int32_t status = RPROC_OFFLINE;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_start(dsp_id, VDSP_BOOT_MODE_SYNC, pathname);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_start ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_get_status(dsp_id, &status);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_get_status ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_reset(dsp_id);
    if (ret != HB_VDSP_OK) {
        ALOGE("%s dsp%d hb_vdsp_reset ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_stop(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_stop ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    printf("%s dsp%d reset pass !\n", __func__, dsp_id);
    return ret;
}

boot_lib_reset_test(0, "/app/vdsp_demo/vdsp_sample/res/q8sample");
boot_lib_reset_test(1, "/app/vdsp_demo/vdsp_sample/res/q8sample");
```

##### hb_vdsp_set_path

【函数声明】

``int32_t hb_vdsp_set_path(int32_t dsp_id, const char* path);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] path：vdsp镜像完整路径，不包含镜像名字

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

设置 VDSP 镜像路径

【示例代码】

``` c
#include <hb_vdsp_mgr.h>

#define VDSP_BOOT_MODE_ASYNC        (0)
#define VDSP_BOOT_MODE_SYNC         (-1)

int32_t boot_lib_set_pathname_test(int32_t dsp_id, const char* path, const char* name)
{
    int32_t ret = 0;
    int32_t status = 0;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_set_path(dsp_id, path);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_set_path ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_set_name(dsp_id, name);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_set_name ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_start(dsp_id, VDSP_BOOT_MODE_SYNC, NULL);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_start ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_get_status(dsp_id, &status);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_get_status ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_stop(dsp_id);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_stop(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_stop ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    printf("%s dsp%d set path & name pass !\n", __func__, dsp_id);
    return ret;
}

boot_lib_set_pathname_test(0, "/app/testcase/S05_VDSP/testsuite", "q8sample");
boot_lib_set_pathname_test(1, "/app/testcase/S05_VDSP/testsuite", "q8sample");
```

##### hb_vdsp_set_name

【函数声明】

``int32_t hb_vdsp_set_name(int32_t dsp_id, const char* name);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] name：vdsp镜像名字

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

设置 VDSP 镜像名字

【示例代码】

参考 [hb_vdsp_set_path](#hb_vdsp_setpathname)

##### hb_vdsp_get_fd

【函数声明】

``int32_t hb_vdsp_get_fd(int32_t dsp_id, int32_t *retfd);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[OUT\] retfd：vdsp设备fd句柄

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

打开并返回 VDSP 设备fd句柄

【示例代码】

参考 [hb_vdsp_start](#hb_vdsp_start_async)

##### hb_vdsp_close_fd

【函数声明】

``int32_t hb_vdsp_close_fd(int32_t fd);``

【参数描述】

  - \[IN\] fd：vdsp设备fd句柄

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

关闭 VDSP 设备

【示例代码】

参考 [hb_vdsp_start](#hb_vdsp_start_async)

##### hb_vdsp_mem_alloc

【函数声明】

``int32_t hb_vdsp_mem_alloc(int32_t dsp_id, uint64_t size, int64_t flags, uint64_t *va, uint64_t *iova);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] size：申请的内存大小
  - \[IN\] flags：使用libhbmem申请内存的flags
  - \[OUT\] va：使用libhbmem申请内存的虚拟地址
  - \[OUT\] iova：映射成的vdsp设备地址

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

通过libvdsp申请内存并映射成vdsp可访问的设备地址

【申请内存并映射 示例代码】

``` c
#include <hb_vdsp_mgr.h>

int32_t boot_lib_mem_alloc_test(int32_t dsp_id)
{
    int32_t ret = 0;
    uint64_t usize = 64 * 1024;
    int64_t hbmem_flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN;
    uint64_t p_va=0;
    uint64_t p_iova=0;

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_vdsp_mem_alloc(dsp_id, usize, hbmem_flags, &p_va, &p_iova);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_mem_alloc ret-%d fail !\n", __func__, dsp_id, ret);
    hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_mem_free(dsp_id, p_va);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_mem_free ret-%d fail !\n", __func__, dsp_id, ret);
    hb_vdsp_deinit(dsp_id);
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    return ret;
}
```

##### hb_vdsp_mem_free

【函数声明】

``int32_t hb_vdsp_mem_free(int32_t dsp_id, uint64_t va);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] va：使用libhbmem申请内存的虚拟地址

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

通过libvdsp解映射vdsp设备地址并释放内存

【示例代码】

参考 [hb_vdsp_mem_alloc](#hb_vdsp_mem_alloc)

##### hb_vdsp_mmu_map

【函数声明】

``int32_t hb_vdsp_mmu_map(int32_t dsp_id, uint64_t va, uint64_t size, uint64_t *iova);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] va：使用libhbmem申请内存的虚拟地址
  - \[IN\] size：待映射的内存大小
  - \[OUT\] iova：映射成的vdsp设备地址

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

通过libvdsp映射虚拟地址为vdsp可访问的设备地址；map支持对同一个地址进行多次映射，map、unmap需要成对使用，输入参数va、size指定的区间需要确保不超过buffer分配的实际大小

【映射内存为设备地址 示例代码】

``` c
#include <hb_vdsp_mgr.h>

int32_t boot_lib_mem_map_test(int32_t dsp_id)
{
    int32_t ret = 0;
    int64_t flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN;
    hb_mem_common_buf_t com_buf = {0, };
    hb_mem_common_buf_t info = {0, };
    uint64_t usize = 64 * 1024;
    uint64_t p_iova=0;

    ret = hb_mem_module_open();
    if (ret != 0) {
        printf("%s dsp%d hb_mem_module_open ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    ret = hb_mem_alloc_com_buf(usize, flags, &com_buf);
    if (ret != 0) {
        printf("%s dsp%d hb_mem_alloc_com_buf ret-%d fail !\n", __func__, dsp_id, ret);
        hb_mem_module_close();
        return ret;
    }
    if (com_buf.fd < 0) {
        printf("%s dsp%d com_buf.fd %d fail !\n", __func__, dsp_id, com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_mem_get_com_buf(com_buf.fd, &info);
    if (ret != 0) {
        printf("%s dsp%d hb_mem_get_com_buf ret-%d fail !\n", __func__, dsp_id, ret);
        hb_mem_free_buf(com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_vdsp_init(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_init ret-%d fail !\n", __func__, dsp_id, ret);
        hb_mem_free_buf(com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_vdsp_mmu_map(dsp_id, (uint64_t)com_buf.virt_addr, usize, &p_iova);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_mmu_map ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        hb_mem_free_buf(com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_vdsp_mmu_unmap(g_i_vdsp_id, (uint64_t)com_buf.virt_addr);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_mmu_unmap ret-%d fail !\n", __func__, dsp_id, ret);
        hb_vdsp_deinit(dsp_id);
        hb_mem_free_buf(com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_vdsp_deinit(dsp_id);
    if (ret != HB_VDSP_OK) {
        printf("%s dsp%d hb_vdsp_deinit ret-%d fail !\n", __func__, dsp_id, ret);
        hb_mem_free_buf(com_buf.fd);
        hb_mem_module_close();
        return ret;
    }

    ret = hb_mem_free_buf(com_buf.fd);
    if (ret != 0) {
        printf("%s dsp%d hb_mem_free_buf ret-%d fail !\n", __func__, dsp_id, ret);
    hb_mem_module_close();
        return ret;
    }

    ret = hb_mem_module_close();
    if (ret != 0) {
        printf("%s dsp%d hb_mem_module_close ret-%d fail !\n", __func__, dsp_id, ret);
        return ret;
    }

    return ret;
}
```

##### hb_vdsp_mmu_unmap

【函数声明】

``int32_t hb_vdsp_mmu_unmap(int32_t dsp_id, uint64_t va);``

【参数描述】

  - \[IN\] dsp_id：dsp编号，取值0、1, 对应dsp0、dsp1
  - \[IN\] va：使用libhbmem申请内存的虚拟地址

【返回值】

  - 成功：0
  - 失败：异常为负值错误码，参考 [VDSP启停控制 API 返回值](#vdsp_boot_api_return)

【功能描述】

通过libvdsp解映射vdsp设备地址

【示例代码】

参考 [hb_vdsp_mmu_map](#hb_vdsp_mmu_map)
