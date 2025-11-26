---
sidebar_position: 1
---

# hrut_somstatus

The **hrut_somstatus** command retrieves temperature sensor readings, CPU/BPU operating frequencies, and BPU load.

## Syntax

```
sudo hrut_somstatus
```

## Common Usage

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

**temperature**:

- **pvt_cmn_pvtc1_t1/2**: CPU temperature, in degrees Celsius (°C).
- **pvt_mcu_pvtc1_t1/2**: MCU temperature, in degrees Celsius (°C).
- **pvt_bpu_pvtc1_t1**: BPU temperature, in degrees Celsius (°C).

**cpu frequency**:

- `min`: Minimum operating frequency of the CPU.
- `cur`: Current operating frequency of the CPU.
- `max`: Maximum operating frequency of the CPU.
- This information shows the frequency range (minimum, current, and maximum) for each CPU cluster.

**bpu status information**:

- `ratio`: BPU load ratio during operation.
- This information indicates the BPU load.