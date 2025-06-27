---
sidebar_position: 0
---

# MCU代码包结构介绍

:::info
MCU0固件编译/McalCdd/Service/Platform等代码为企业版专有，如有需要，请联系[D-Robotics](mailto:developer@d-robotics.cc)获取支持。
:::

## MCU社区版

```c
MCU
├── Build                # Build系统，包含编译/链接脚本
├── Config               # 针对各种不同board的McalCdd模块配置
├── Include              # 主要为驱动和Service文件夹内的头文件
├── Library              # 主要为驱动和Service静态库文件
├── log                  # 编译log
├── OpenSource           # FreeRtos开源代码仓库
├── output               # 编译/链接生成文件的所在目录
├── samples              # 包含使用样例，包括Can，IPC，Eth等驱动
└── Target               # 系统基础代码，比如启动相关，任务定义相关，中断相关等
```


## MCU企业版
```c
MCU
├── Build                # Build系统，包含编译/链接脚本
|   ├── FreeRtos         # 用于编译MCU0的固件
|   ├── FreeRtos_mcu1    # 用于编译MCU1的固件
|   ├── ToolChain        # gcc工具链
|   └── Tools            # 编译过程中使用的通用工具
├── Common               # 包含所有MCAL模块所需的通用文件和定义
├── Config               # 针对各种不同board的McalCdd模块配置
├── log                  # 编译log
├── McalCdd              # 各种模块驱动代码
├── OpenSource           # FreeRtos开源代码仓库
├── output               # 编译/链接生成文件的所在目录
├── Platform             # 平台配置相关，比如基础数据定义，各个模块的Memmap配置，此部分可以由客户自己替换
|   ├── Compiler         # 平台配置和编译器相关的定义
|   ├── Memmap           # 模块的memmap配置
|   └── Schm             # 模块驱动中可能涉及到exclusive区域定义，可能需要客户选择填充
├── samples              # 包含使用样例，包括Can，IPC，Eth等驱动
├── Service              # 包含地瓜自研的中间服务代码，比如电源管理，OTA管理，Log/Shell等
└── Target               # 系统基础代码，比如启动相关，任务定义相关，中断相关等
```
