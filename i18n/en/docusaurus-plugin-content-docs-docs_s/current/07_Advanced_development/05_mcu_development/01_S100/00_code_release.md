---
sidebar_position: 0
---

# MCU Code Package Structure Introduction

:::info
MCU0 firmware compilation / McalCdd / Service / Platform, and other code are proprietary to the Enterprise Edition. If needed, please contact [D-Robotics](mailto:developer@d-robotics.cc) for support.
:::

:::tip
The Commercial Edition offers more comprehensive feature support, deeper hardware capability access, and exclusive customization content. To ensure compliance and secure delivery, we will grant access to the Commercial Edition through the following process:

**Commercial Edition Access Procedure**:
1. **Complete a questionnaire**: Submit basic information about your organization and intended use case.
2. **Sign a Non-Disclosure Agreement (NDA)**: We will contact you based on your submission, and both parties will sign an NDA upon mutual confirmation.
3. **Content release**: After the agreement is signed, we will provide access to the Commercial Edition materials through a private channel.

If you wish to obtain the Commercial Edition, please fill out the questionnaire below. We will contact you within 3–5 business days:

Questionnaire link: https://horizonrobotics.feishu.cn/share/base/form/shrcnpBby71Y8LlixYF2N3ENbre
:::

## MCU Community Edition

```c
MCU
├── Build                # Build system, including compilation/linker scripts
├── Config               # McalCdd module configurations for various boards
├── Include              # Header files primarily for driver and Service folders
├── Library              # Static library files primarily for drivers and Service
├── log                  # Compilation logs
├── OpenSource           # FreeRTOS open-source code repository
├── output               # Directory containing compiled/linked output files
├── samples              # Usage examples, including drivers for CAN, IPC, Ethernet, etc.
└── Target               # Core system code, e.g., startup-related, task definitions, interrupt handling, etc.
```


## MCU Enterprise Edition
```c
MCU
├── Build                # Build system, including compilation/linker scripts
|   ├── FreeRtos         # Used to compile MCU0 firmware
|   ├── FreeRtos_mcu1    # Used to compile MCU1 firmware
|   ├── ToolChain        # GCC toolchain
|   └── Tools            # General-purpose tools used during compilation
├── Common               # Common files and definitions required by all MCAL modules
├── Config               # McalCdd module configurations for various boards
├── log                  # Compilation logs
├── McalCdd              # Driver code for various modules
├── OpenSource           # FreeRTOS open-source code repository
├── output               # Directory containing compiled/linked output files
├── Platform             # Platform-related configurations, such as basic data definitions and Memmap configurations for each module; this part can be replaced by the customer
|   ├── Compiler         # Platform and compiler-related definitions
|   ├── Memmap           # Memmap configuration for modules
|   └── Schm             # Exclusive area definitions possibly used in module drivers; customers may need to select and populate these
├── samples              # Usage examples, including drivers for CAN, IPC, Ethernet, etc.
├── Service              # Proprietary middleware services developed by Digua, such as power management, OTA management, Log/Shell, etc.
└── Target               # Core system code, e.g., startup-related, task definitions, interrupt handling, etc.
```