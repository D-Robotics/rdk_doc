---
sidebar_position: 2
---

# MCU系统说明
## 编译系统基本说明
MCU的编译系统基于Scons3.0.0创建（[Scons 3.0.0用户手册官网](https://scons.org/doc/3.0.0/HTML/scons-user.html)）。

## MCU1编译系统
MCU1编译系统位于mcu/Build/FreeRtos_mcu1，具体目录结构，如下图所示：
```c
FreeRtos_mcu1
├── build_freertos.py                   # 编译的入口脚本
├── SConstruct_Lite_FRtos_S100_sip_B    # Scons参与编译文件夹以及输出目录
├── settings_freertos.py                # Scons编译命令参数相关的文件
└── Linker                              # 编译link脚本所在目录
     └── gcc
          └── S100.ld
```

## 编译流程介绍
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/MCU_build_system/build_freertos.png)

## 编译中的重点文件关系介绍
build_freertos.py是编译的整体入口，但是实际调度到scons时，能够对scons编译环境/流程产生影响的方式有以下几个：
1. SConstruct文件：SConstruct文件是scons编译的定义文件，它和每个模块内的Sconscript组成了Cmake里Cmakefile；Make系统里makefile的作用；
2. settings_freertos.py：该文件生效的入口实际上是SConstruct里面的“Variables”类的初始化，核心在于引入一系列静态定义的编译环境变量；环境变量的变量名就是settings_freertos.py里面的变量名，变量值就是settings_freertos.py里面的变量名对应的变量值；“Variables”类实例化后的示例会被Environment类使用，用于scons的编译
3. gcc_arm.py：实际定义编译命令的定义文件，真正生效的入口是settings_freertos.py里面定义的“COMPILER_TOOL”字段，COMPILER_TOOL字段进一步会被Sconscruct文件的Variables添加并最后被env获取到其中的“CC”等配置

## MCU1镜像layout
|区域名称|起始地址|占用大小|作用|
|----|---------------|---------------|---------|
| FLASH_STARTUP| 0x0CAB0000|1K |启动代码位置 |
| FLASH| 0x0CAB0400|2731K |代码、数据、栈等使用的区域 |
| FREERTOS_HEAP| 0x0CD5B000|512K |堆空间 |
| LOG_SHARE_Reserved| 0x0CDDB000|8K |MCU1 log存放的空间，log会循环覆盖 |
| SCMI_IPC_Reserved| 0x0CDDD000|12K |SCMI IPC通信需要的空间，用于buffer及关键数据 |

在上述SRAM的排布中， 强烈不建议客户修改SCMI_IPC_Reserved及其之后的区域。 这些部分大部分都是其他域一定会使用的关键位置，修改可能导致异常。如果要修改，请先咨询地瓜相关支持人员。

下面是地瓜版本中的链接文件，解释了链接脚本中提供的一些变量作用：
```c
MEMORY
{
    FLASH_STARTUP(rx)       : org = 0x0CAB0000, len = 1K
    FLASH(rw)               : org = 0x0CAB0400, len = 2731K
    FREERTOS_HEAP(rw)       : org = 0x0CD5B000, len = 512K
    LOG_SHARE_Reserved(rw)  : org = 0x0CDDB000, len = 8K
    SCMI_IPC_Reserved(rw)   : org = 0x0CDDD000, len = 12K
}

/* Define output sections */
SECTIONS {
     .EL2_core_exceptions_table :
     {
          . = ALIGN(32);
          _start = .;
          *(.EL2_core_exceptions_table)
          . = ALIGN(32);
     } > FLASH_STARTUP

     .EL2_Reset_Handler :
     {
          . = ALIGN(32);
          *(.EL2_Reset_Handler)
          . = ALIGN(32);
     } > FLASH_STARTUP

     .EL1_core_exceptions_table :
     {
          . = ALIGN(32);
          *(.EL1_core_exceptions_table)
          . = ALIGN(32);
     } > FLASH_STARTUP

     .text :
     {
          . = ALIGN(4);
          *(.text)                 /* .text sections (code) */
          . = ALIGN(4);
     } > FLASH

     .shell :
     {
          _shell_command_start = .;
          KEEP (*(shellCommand))
          _shell_command_end = .;
     } > FLASH

     .mcal_text :
     {
          *(.mcal_text)
     } > FLASH

     .mcal_const_cfg :
     {
          *(.mcal_const_cfg)
     } > FLASH

     .mcal_const :
     {
          *(.mcal_const)
     } > FLASH

     .common_text :
     {
          *(.common_text)
          PROVIDE(__TEXT_END = .);
     } > FLASH
     /******************text end******************/

     .const :
     {
          . = ALIGN(32);
          *(.const)
          *(.rodata)
     } > FLASH


     .heap :
     {
          . = ALIGN(64);
          __HEAP_START = .;
          __end__ = .;
          __heap_start__ = .;
          PROVIDE(end = .);
          PROVIDE(_end = .);
          PROVIDE(__end = .);
          __HeapBase = .;
          . += HEAP_SIZE;
          __HeapLimit = .;
          __heap_limit = .;
          __heap_end__ = .;
     } > FLASH

     .u_boot_list :
     {
          . = ALIGN(4);
          *(SORT(.u_boot_list*))
          . = ALIGN(4);
     } > FLASH

     .global_data :
     {
          . = ALIGN(64);
          __DATA_RAM = .;
          __data_start__ = .;      /* Create a global symbol at data start. */
          *(.data)                 /* .data sections */
          . = ALIGN(64);
          __data_end__ = .;        /* Define a global symbol at data end. */
          PROVIDE(__DATA_END = .);
          PROVIDE(__DATA_ROM = .);
     } > FLASH

     .stack (NOLOAD) :
     {
          . = ALIGN(64);
          __STACK_START = .;
          __StackLimit = .;
          __stack_start__ = .;
          . += STACK_SIZE;
          __stack_end__ = .;
          __StackTop = .;
     } > FLASH

     .stack_exc (NOLOAD) :
     {
          . = ALIGN(64);
          __StackLimit_exc = .;
          __stack_start_exc__ = .;
          . += STACK_SIZE_EXC;
          __stack_end_exc__ = .;
          __StackTop_exc = .;
          __STACK_END = .;
     } > FLASH

     .init_table :
     {
          . = ALIGN(64);
          __COPY_TABLE = .;
          KEEP(*(.init_table))
     } > FLASH

     .zero_table :
     {
          . = ALIGN(64);
          __ZERO_TABLE = .;
          KEEP(*(.zero_table))
     } > FLASH

     .interrupts :
     {
          __VECTOR_TABLE = .;
          __interrupts_start__ = .;
          . = ALIGN(4);
          KEEP(*(.isr_vector))     /* Startup code */
          __interrupts_end__ = .;
          . = ALIGN(4);
     } > FLASH

     __VECTOR_RAM = __VECTOR_TABLE;
     __RAM_VECTOR_TABLE_SIZE = 0x0;
     __VECTOR_TABLE_COPY_END = __VECTOR_TABLE + __RAM_VECTOR_TABLE_SIZE;

     .interrupt_drv_shared_memory :
     {
          *(.interrupt_drv_shared_memory)
     } > FLASH

     .handlers :
     {
          . = ALIGN(32);
          *(.handlers)
     } > FLASH

     .mcal_data :
     {
          *(.mcal_data)
     } > FLASH

     .mcal_shared_data :
     {
          *(.mcal_shared_data)
     } > FLASH

     .bss (NOLOAD) :
     {
          . = ALIGN(64);
          __BSS_START = .;
          __bss_start__ = .;
          *(.bss)
     } > FLASH

     .mcal_bss (NOLOAD) :
     {
          . = ALIGN(64);
          *(.mcal_bss)
     } > FLASH

     .mcal_shared_bss (NOLOAD) :
     {
          . = ALIGN(64);
          *(.mcal_shared_bss)
          __DATA_RAM_END = .;
          __m_ram_init_end = .;
          __bss_end__ = .;
          __BSS_END = .;
     } > FLASH

     .ipc_mdma :
     {
          *(.ipc_mdma)
     } > FLASH

     .ucheap_section (NOLOAD) :
     {
          . = ALIGN(64);
          KEEP(*(.ucheap_section))
          . = ALIGN(64);
     } > FREERTOS_HEAP

     .log (NOLOAD) :
     {
          *(.log)
     } > LOG_SHARE_Reserved

     /*-------- LABELS USED IN CODE -------------------------------*/
     SRAM_START_ADDR         = ORIGIN(FLASH_STARTUP);
     MCU_LOG_START_ADDR      = ORIGIN(LOG_SHARE_Reserved);
     __SCMI_IPC_START_ADDR   = ORIGIN(SCMI_IPC_Reserved);
     NON_SECURE_START_ADDR   = ORIGIN(LOG_SHARE_Reserved);

     PROVIDE(SRAM_SIZE = 0x34FFFF);
}
```

## startup.s启动代码简介
1. 启动的第一条指令就是进入EL2_core_exceptions_table向量表
```c
    .text
    .align 4
    .section ".EL2_core_exceptions_table", "ax"
    .globl  EL2_core_exceptions_table
    .type   EL2_core_exceptions_table, %function
EL2_core_exceptions_table:
    b   EL2_Reset_Handler             /* Reset Handler */
    b   EL2_Undefined_Handler         /* Undefined Handler */
    b   EL2_HVC_Handler               /* SVCall Handler */
    b   EL2_Prefetch_Handler          /* Prefetch Handler */
    b   EL2_Abort_Handler             /* Abort Handler */
    b   EL2_Trap_Handler              /* Reserved */
    b   EL2_IRQ_Handler               /* IRQ Handler */
    b   EL2_FIQ_Handler               /* FIQ Handler */
```
2. 从而进入EL2_Reset_Handler函数，正式开始启动；
```c
EL2_Reset_Handler:
    mov r0, #0
    mov r1, r0
    mov r2, r0
    mov r3, r0
    mov r4, r0
    mov r5, r0
    mov r6, r0
    mov r7, r0
    mov r8, r0
    mov r9, r0
    mov r10, r0
    mov r11, r0
    mov r12, r0
    ldr r0, =0x23000003
    MCR p15, 0, r0, c15, c0, 0

    b MPU_Init
```
3. 在做其他操作前，通过MPU配置后续可能用到的各个地址空间，在 MPU_Init 这个标号；MPU region1是MCU sram区域， 用户可以按照自己的需求，切分sram，并配置各个切分区域的属性 ，比如sharebility/non-cache等等；启动代码对这一部分的配置仅供参考，对于MCU sram区域的划分，参考上一节MCU1镜像layout，MCU0相关区域的划分因MCU0的缘故不做展示；
```c
MPU_Init:
          //.....其他省略

          /*-----region 1 MCU sram示例配置，根据自己需求修改-------*/
          /*---------------region 1 mcu sram---------------*/
          /* normal memory attribute */
          ldr r0, =1                /* Region 1 */
          mcr p15, 4, r0, c6, c2, 1 /* Write HPRSELR */
          mcr p15, 0, r0, c6, c2, 1 /* Write PRSELR */

          ldr r0, =0x0C800000       /* Start address */
          orr r0, r0, #0x2          /* SH=0, AP=1, XN=0*/
          mcr p15, 4, r0, c6, c3, 0 /* Write HPRBAR */
          mcr p15, 0, r0, c6, c3, 0 /* Write PRBAR */

          ldr r0, =0x0CDFFFFF      /* End address */
          and r0, r0, #0xFFFFFFC0
          orr r0, r0, #0x3          /* AttrIndex=1, non-cacheable, enable region */
          mcr p15, 4, r0, c6, c3, 1 /* Write HPRLAR */
          mcr p15, 0, r0, c6, c3, 1 /* Write PRLAR */

          /*---------------region 5 internal gic & peripheral---------------*/
          /* device memory attribute            */
          ldr r0, =5                /* Region 5 */
          mcr p15, 4, r0, c6, c2, 1 /* Write HPRSELR */
          mcr p15, 0, r0, c6, c2, 1 /* Write PRSELR */

          ldr r0, =0x22000000       /* Start address */
          orr r0, r0, #0x13         /* SH=2, AP=1, XN=1*/
          mcr p15, 4, r0, c6, c3, 0 /* Write HPRBAR */
          mcr p15, 0, r0, c6, c3, 0 /* Write PRBAR */

          ldr r0, =0x223FFFFF       /* End address */
          sub r0, r0, #1
          and r0, r0, #0xFFFFFFC0
          orr r0, r0, #0x7          /* AttrIndex=3, device memory, enable region */
          mcr p15, 4, r0, c6, c3, 1 /* Write HPRLAR */
          mcr p15, 0, r0, c6, c3, 1 /* Write PRLAR */

          //.....后续省略
```
4. 地瓜版本中重要的MPU region说明如下：

:::caution
ARM R52的background region和RDK-S100芯片上实际实现的memory map是有差异的。

比如0x2200_0000在ARM的background region中默认是属于normal memory空间，但在RDK-S100芯片上这段地址空间对应是GIC这种device类的寄存器空间。

所以，像这种芯片实际与background region有差异的区域，在进行访问之前，一定要通过MPU将memory类型和RDK-S100芯片实际实现保持一致，否则会导致访问异常。

这些区域都是MCU正常运行可能需要访问的空间，缺少这些空间的配置，可能会导致运行异常。客户在自己的版本中 请保持和地瓜给的代码一样的地址空间属性配置。SRAM区域，客户则可以根据自己项目的需求做不同的切分。
:::

| MPU region| 起始地址| 结束地址|memory类型|说明|
|--------|----------------------------------------|-----------------------------|----------------|---------|
| 0| 0x0800_0000|0x0AFF_FFFF|normal memory|vdsp tcm及MCU TCM所在空间，SPL运行会使用vdsp tcm|
| 7| 0x2200_0000|0x223F_FFFF|device memory|MCU GIC寄存器所在地址空间，影响对GIC寄存器的访问|
| 8| 0x2300_0000|0x25FF_FFFF|device memory|MCU peri寄存器所在空间，影响对MCU上peri寄存器的访问|
| 9| 0x2600_0000|0x7FFF_FFFF|device memory|CPUSYS各种寄存器所在空间，影响MCU对Acore侧的寄存器访问|
| 10| 0x8000_0000|0xFFFF_FFFF|normal memory|DDR所在地址空间，影响MCU对DDR访问|
| 11| 0x1800_0000|0x1FFF_FFFF|device memory|XSPI所在地址空间，影响MCU对flash的使用|

5. 启动代码接下来做了enable_prefetch/enable_peri_secure/使能VFP/配置SYSCNT寄存器等操作，建议客户保留这些代码；
6. 紧接着让当前core从hyper模式跳转到el1；
```c
     /* Init ELR_hyp with stack_initialization address - init the return address when jumping from EL2 into EL1 */
     ldr r0, =EL1_Reset_Handler
     msr ELR_hyp, r0

     //省略部分代码

     /* Exception return - will jump to address pointed by ELR_hyp (main) */
     eret /* When executed in Hyp mode, ERET loads the PC from ELR_hyp and loads the CPSR from SPSR_hyp */
```
7. 接下来是做栈的初始化，每个core都有自己的一段栈区域。启动代码只初始化了R52+上abort/undefined/system mode的栈寄存器 ，并没有初始化配置irq/fiq mode等等栈寄存器。客户需要根据自己OS的情况，确定R52+各个模式的栈寄存器是否需要初始化；
```c
     /* Setup the stack for supervisor mode (entered from reset) */
     mrs         r0, cpsr
     and         r0, r0, #~0x00FF
     orr         r0, r0, #0x0033
     msr         cpsr_c, r0
     sub         r3, r3, r1
     mov         SP, r3         /* top of stack to SP_svc */

     ldr         r3, =__StackTop_exc
     ldr         r2, =__StackLimit_exc
     sub         r2, r3, r2     /* r2 : size in bytes */
     mov         r4, #4
     udiv        r1, r2, r4     /* r1 : size divided by 4 */
     and         r1, r1, #~0x0f /* r1 size alligned to 16 bytes */

     /* Go to FIQ mode and set stack (below the previous one) */
     mrs         r0, cpsr
     and         r0, r0, #~0x003F
     orr         r0, r0, #0x0031
     msr         cpsr_c, r0
     sub         r3, r3, r1
     mov         SP, r3

     //.....后续省略
```
8. 跳转到main
```c
    /* Enable IRQ and FIQ interrupts for the system/user mode */
    cpsie   i               /* Unmask interrupts (IRQ)*/
    cpsie   f               /* Unmask fast interrupts (FIQ)*/

    /* Go to supervisor mode */
    /* mrs         r0, cpsr */
    /* and         r0, r0, #~0x00FF */
    /* orr         r0, r0, #0x0033 */
    /* msr         cpsr_c, r0 */

    /* Jump to the main() method */
    bl main

	/* Should never get here */
	b .
    .end
```
