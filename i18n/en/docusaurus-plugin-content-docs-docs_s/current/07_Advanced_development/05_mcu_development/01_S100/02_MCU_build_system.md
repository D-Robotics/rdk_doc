---
sidebar_position: 2
---

# MCU System Description
## Basic Description of the Build System
The MCU build system is based on SCons 3.0.0 ([SCons 3.0.0 User Guide Official Website](https://scons.org/doc/3.0.0/HTML/scons-user.html)).

## MCU1 Build System
The MCU1 build system is located in `mcu/Build/FreeRtos_mcu1`. The specific directory structure is shown below:
```c
FreeRtos_mcu1
├── build_freertos.py                   # Entry script for building
├── SConstruct_Lite_FRtos_S100_sip_B    # SCons file specifying directories to compile and output directory
├── settings_freertos.py                # File containing SCons build command parameters
└── Linker                              # Directory containing linker scripts
     └── gcc
          └── S100.ld
```

## Build Process Overview
![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/MCU_build_system/build_freertos.png)

## Key File Relationships in the Build Process
`build_freertos.py` serves as the overall entry point for the build. However, when actually invoking SCons, the following methods influence the SCons build environment and process:
1. **SConstruct file**: The SConstruct file defines the SCons build configuration. Together with the SConscript files inside each module, it fulfills a role similar to CMakeLists.txt in CMake or Makefile in traditional Make systems.
2. **settings_freertos.py**: This file takes effect through the initialization of the `Variables` class inside the SConstruct file. Its core purpose is to introduce a series of statically defined build environment variables. The variable names correspond directly to those defined in `settings_freertos.py`, and their values are the corresponding values from the same file. The instantiated `Variables` object is then used by the `Environment` class to configure the SCons build.
3. **gcc_arm.py**: This file actually defines the compilation commands. Its effective entry point is the `COMPILER_TOOL` field defined in `settings_freertos.py`. This `COMPILER_TOOL` field is subsequently added by the SConstruct file's `Variables` instance and finally accessed by the SCons environment (`env`) to retrieve configurations such as `CC`.

## MCU1 Image Layout
| Region Name           | Start Address | Size     | Purpose |
|-----------------------|---------------|----------|---------|
| FLASH_STARTUP         | 0x0CAB0000    | 1K       | Startup code location |
| FLASH                 | 0x0CAB0400    | 2731K    | Region used for code, data, stack, etc. |
| FREERTOS_HEAP         | 0x0CD5B000    | 512K     | Heap space |
| LOG_SHARE_Reserved    | 0x0CDDB000    | 8K       | Space reserved for MCU1 logs (logs are overwritten cyclically) |
| SCMI_IPC_Reserved     | 0x0CDDD000    | 12K      | Space required for SCMI IPC communication (used for buffers and critical data) |

In the above SRAM layout, customers are **strongly discouraged** from modifying the `SCMI_IPC_Reserved` region and any regions that follow. These areas are critical for other domains and modifications may lead to system malfunctions. If modification is absolutely necessary, please consult Diguasupport personnel first.

Below is the linker script from the Digua version, which explains the purpose of several variables provided in the linker script:
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

## Introduction to startup.s boot code
1. The first instruction executed at boot enters the EL2_core_exceptions_table vector table:
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
2. Execution then proceeds to the EL2_Reset_Handler function, formally starting the boot process:
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
3. Before performing other operations, the code configures memory regions that may be used later via the MPU at the MPU_Init label. MPU region 1 is configured as the MCU SRAM region. Users can partition the SRAM according to their requirements and configure attributes for each partition, such as shareability or non-cacheability. The configuration provided in the boot code serves only as a reference. For SRAM partitioning on the MCU, refer to the MCU1 image layout described in the previous section; partitioning details for MCU0 are omitted due to MCU0-specific considerations:
```c
MPU_Init:
          //.....other parts omitted

          /*-----Example configuration for region 1 (MCU SRAM); modify according to your requirements-------*/
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

          //.....remaining parts omitted
```
4. Important MPU regions in the Digua version are described below:

:::caution
The background region defined by ARM R52 differs from the actual memory map implemented on the RDK-S100 chip.

For example, address 0x2200_0000 is classified as normal memory by default in ARM's background region, but on the RDK-S100 chip, this address range corresponds to device-type registers such as the GIC.

Therefore, for any memory regions where the chip's actual implementation differs from the ARM background region, you must configure the MPU to ensure the memory type matches the RDK-S100 chip's actual implementation before accessing them; otherwise, access exceptions will occur.

These regions are essential for normal MCU operation. Missing their configuration may lead to runtime exceptions. Customers must maintain the same memory attribute configuration for these address spaces as provided in Digua's reference code. For the SRAM region, customers may partition it differently according to their project requirements.
:::

| MPU region | Start Address | End Address | Memory Type | Description |
|------------|---------------|-------------|-------------|-------------|
| 0 | 0x0800_0000 | 0x0AFF_FFFF | normal memory | Contains VDSP TCM and MCU TCM; used by SPL |
| 7 | 0x2200_0000 | 0x223F_FFFF | device memory | MCU GIC register space; affects GIC register access |
| 8 | 0x2300_0000 | 0x25FF_FFFF | device memory | MCU peripheral register space; affects access to MCU peripherals |
| 9 | 0x2600_0000 | 0x7FFF_FFFF | device memory | CPUSYS register space; affects MCU access to A-core side registers |
| 10 | 0x8000_0000 | 0xFFFF_FFFF | normal memory | DDR address space; affects MCU access to DDR |
| 11 | 0x1800_0000 | 0x1FFF_FFFF | device memory | XSPI address space; affects MCU access to flash |

5. The boot code subsequently enables prefetch, configures peripherals as secure, enables VFP, configures SYSCNT registers, etc. Customers are advised to retain this code.
6. The current core then transitions from Hyp mode to EL1:
```c
     /* Init ELR_hyp with stack_initialization address - init the return address when jumping from EL2 into EL1 */
     ldr r0, =EL1_Reset_Handler
     msr ELR_hyp, r0

     //部分代码 omitted

     /* Exception return - will jump to address pointed by ELR_hyp (main) */
     eret /* When executed in Hyp mode, ERET loads the PC from ELR_hyp and loads the CPSR from SPSR_hyp */
```
7. Stack initialization follows, with each core having its own dedicated stack region. The boot code initializes stack pointers only for abort, undefined, and system modes on the R52+; it does not initialize stack pointers for IRQ/FIQ modes, etc. Customers should determine, based on their OS requirements, whether stack pointers for all R52+ modes need initialization:
```c
     /* Setup the stack for supervisor mode (entered from reset) */
     mrs         r0, cpsr
     and         r0, r0, #~0x00FF
     orr         r0, r0, #0x0033
     msr         cpsr_c, r0
     sub         r3, r3, r1
     mov         SP, r3         /* top of stack to SP_svc */
```ldr         r3, =__StackTop_exc
     ldr         r2, =__StackLimit_exc
     sub         r2, r3, r2     /* r2 : size in bytes */
     mov         r4, #4
     udiv        r1, r2, r4     /* r1 : size divided by 4 */
     and         r1, r1, #~0x0f /* r1 size aligned to 16 bytes */

     /* Go to FIQ mode and set stack (below the previous one) */
     mrs         r0, cpsr
     and         r0, r0, #~0x003F
     orr         r0, r0, #0x0031
     msr         cpsr_c, r0
     sub         r3, r3, r1
     mov         SP, r3

     //.....Subsequent code omitted
```
8. Jump to main
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