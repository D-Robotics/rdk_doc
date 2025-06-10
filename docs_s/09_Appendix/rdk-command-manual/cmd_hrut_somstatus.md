---
sidebar_position: 1
---

# hrut_somstatus

**hrut_somstatus** 命令可以获取温度传感器温度、CPU\BPU的运行频率以及BPU负载。

## 语法说明

```
sudo hrut_somstatus
```

## 常用命令

```shell
sunrise@ubuntu:~$ sudo hrut_somstatus
=====================1=====================
temperature-->
        pvt_cmn_pvtc1_t1 : 38.565 (C)
        pvt_cmn_pvtc1_t2 : 40.219 (C)
        pvt_mcu_pvtc1_t1 : 37.829 (C)
        pvt_mcu_pvtc1_t2 : 38.013 (C)
        pvt_bpu_pvtc1_t1 : 38.932 (C)
voltage-->
        FAKE     : 100.0 (mV)
        VDDQ_DDR2 : 498.0 (mV)
        VDD_MCU  : 749.0 (mV)
        VDDIO_PVT_MCU : 1794.0 (mV)
        VDDIO_TOP4_1V8 : 1794.0 (mV)
        VDDQ_DDR0 : 504.0 (mV)
        VDD2H_DDR0 : 1042.0 (mV)
        VAA_DDR0 : 1794.0 (mV)
        VDD_DDR0 : 743.0 (mV)
        VDDQ_DDR1 : 504.0 (mV)
        VDD2H_DDR1 : 1042.0 (mV)
        VAA_DDR1 : 1794.0 (mV)
        VDD_DDR1 : 743.0 (mV)
        VDDIO_SD_SDIO_T : 1788.0 (mV)
        VDD_CPU  : 776.0 (mV)
        VAA_DDR2 : 1780.0 (mV)
        VDD2H_DDR2 : 1030.0 (mV)
        VDD_DDR2 : 737.0 (mV)
        VDDIO_TOP2_1V8 : 1780.0 (mV)
        VDD_BPU  : 737.0 (mV)
        VDDIO_TOP0_1V8 : 1780.0 (mV)
        PLL_TOP0_VDDHV : 1768.0 (mV)
        VDDIO_SD_33_MCU : 3304.0 (mV)
        VDDIO_ADC_MCU : 1800.0 (mV)
        VDDIO_MCU_1V8 : 1800.0 (mV)
        PLL_MCU_VDDHV : 1800.0 (mV)
        PLL_MCU_VDDPOST : 746.0 (mV)
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
```

**temperature（温度）**：

- **pvt_cmn_pvtc1_t1/2**：表示 CPU 温度，单位为摄氏度（C）。
- **pvt_mcu_pvtc1_t1/2**：表示 MCU 温度，单位为摄氏度（C）。
- **pvt_bpu_pvtc1_t1**：  表示 BPU 温度，单位为摄氏度（C）。

**cpu frequency（CPU 频率）**：

- `min`：CPU 可运行的最低频率。
- `cur`：CPU 的当前运行频率。
- `max`：CPU 可运行的最大频率。
- 这些信息显示了每个 CPU 簇的频率范围，包括最小、当前和最大频率。

**bpu status information（BPU 状态信息）**：

- `ratio`：BPU运行时的负载率。
- 这些信息显示了 BPU 负载。
