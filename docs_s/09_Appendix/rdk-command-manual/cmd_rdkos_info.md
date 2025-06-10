---
sidebar_position: 1
---

# rdkos_info

**rdkos_info命令** 用于一次性收集`RDK`系统的软件、硬件版本，驱动加载清单，RDK软件包安装清单和最新的系统日志，方便用户快速获取当前系统的状态信息。

## 语法说明

```
sudo rdkos_info [options]
```

## 选项说明

选项都是可选的，非必须。如果不带任何选项参数运行，`rdkos_info`会默认安装简洁模式输出信息。

- `-b`：基础输出模式，不会收集系统日志。
- `-s`：简洁输出模式（默认），输出30行最新的系统日志。
- `-d`：详细输出模式，输出300行最新的系统日志。
- `-v`：显示版本信息。
- `-h`：显示帮助信息。

## 常用命令

默认用法

```
sudo rdkos_info
```

部分输出如下：

```
sunrise@ubuntu:/root$ sudo rdkos_info
================ RDK System Information Collection ================

[Hardware Model]:
        D-Robotics RDK S100 V0P5 (Board Id = 0x6A84)

[CPU And BPU Status]:
        =====================1=====================
        temperature-->
                pvt_cmn_pvtc1_t1 : 38.932 (C)
                pvt_cmn_pvtc1_t2 : 40.587 (C)
                pvt_mcu_pvtc1_t1 : 38.013 (C)
                pvt_mcu_pvtc1_t2 : 38.381 (C)
                pvt_bpu_pvtc1_t1 : 39.300 (C)
        voltage-->
                FAKE     : 100.0 (mV)
                VDDQ_DDR2 : 504.0 (mV)
                VDD_MCU  : 749.0 (mV)
                VDDIO_PVT_MCU : 1794.0 (mV)
                VDDIO_TOP4_1V8 : 1794.0 (mV)
                VDDQ_DDR0 : 498.0 (mV)
                VDD2H_DDR0 : 1042.0 (mV)
                VAA_DDR0 : 1794.0 (mV)
                VDD_DDR0 : 743.0 (mV)
                VDDQ_DDR1 : 498.0 (mV)
                VDD2H_DDR1 : 1052.0 (mV)
                VAA_DDR1 : 1794.0 (mV)
                VDD_DDR1 : 743.0 (mV)
                VDDIO_SD_SDIO_T : 1788.0 (mV)
                VDD_CPU  : 776.0 (mV)
                VAA_DDR2 : 1780.0 (mV)
                VDD2H_DDR2 : 1042.0 (mV)
                VDD_DDR2 : 737.0 (mV)
                VDDIO_TOP2_1V8 : 1780.0 (mV)
                VDD_BPU  : 737.0 (mV)
                VDDIO_TOP0_1V8 : 1780.0 (mV)
                PLL_TOP0_VDDHV : 1768.0 (mV)
                VDDIO_SD_33_MCU : 3304.0 (mV)
                VDDIO_ADC_MCU : 1800.0 (mV)
                VDDIO_MCU_1V8 : 1800.0 (mV)
                PLL_MCU_VDDHV : 1800.0 (mV)
                PLL_MCU_VDDPOST : 751.0 (mV)
                PLL_MCU_VDDREF : 751.0 (mV)
                VDD_TOP  : 757.0 (mV)
                VDD_AON  : 751.0 (mV)
                VDDIO_SD_AON_CA : 1662.0 (mV)
                VDDIO_PVT_1V8 : 1800.0 (mV)
        cpu frequency-->
                          min   cur     max
                policy0: 1125000        1500000 1500000
                policy4: 1125000        1500000 1500000
        bpu status information---->
                        ratio
                bpu0:   0

[Total Memory]:         4.7Gi
[Used Memory]:          1.6Gi
[Free Memory]:          1.5Gi


[RDK OS Version]:
        4.0.2-Beta

[RDK Kernel Version]:
        Linux ubuntu 6.1.112-rt43-DR-4.0.2-2506091317-g08c4b8-g2e7d75 #80 SMP PREEMPT_RT Mon Jun  9 13:18:49 CST 2025 aarch64 aarch64 aarch64 GNU/Linux

[RDK Miniboot Version]:
        4.0.2-20250605231718
```
