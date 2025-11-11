---
sidebar_position: 3
---

# MCU1 Development Guide

## MCU Interrupt Numbers and Corresponding Modules
| Module | Interrupt Number | Name |
|--------|----------------------------------------|---------------------------------------------|
| SGI| 0-15| |
| PPI| 16-31| |
| MCU_STCU | 32| Bist_Stcu0Isr|
| MEDIA_TOP_STCU|33|Bist_Stcu1Isr|
| VIDEO_STCU|34|Bist_Stcu2Isr|
|VDSP_STCU|35|Bist_Stcu3Isr|
|HSIS_STCU|36|Bist_Stcu4Isr|
|GPU_STCU|37|Bist_Stcu5Isr|
|DDR2_STCU|38|Bist_Stcu6Isr|
|DDR1_STCU|39|Bist_Stcu7Isr|
|DDR0_STCU|40|Bist_Stcu8Isr|
|CPU_MP4_STCU|41|Bist_Stcu9Isr|
|CPU_MP2_STCU|42|Bist_Stcu10Isr|
|CAM_STCU|43|Bist_Stcu11Isr|
|BPU0_STCU|44|Bist_Stcu12Isr|
|UART0|45|Uart0_Isr|
|UART1|46|Uart1_Isr|
|UART2|47|Uart2_Isr|
|ADC0|48|Adc_Ch0WdIsr|
||49|Adc_Ch1WdIsr|
||50|Adc_Ch2WdIsr|
||51|Adc_Ch3WdIsr|
||52|Adc_Ch4WdIsr|
||53|Adc_Ch5WdIsr|
||54|Adc_Ch6WdIsr|
||55|Adc_Ch7WdIsr|
||56|Adc_Ch8WdIsr|
||57|Adc_Ch9WdIsr|
||58|Adc_Ch10WdIsr|
||59|Adc_Ch11WdIsr|
||60|Adc_Ch12WdIsr|
||61|Adc_Ch13WdIsr|
||62|Adc_InjIsr|
||63|Adc_NorIsr|
|I2C0|64|I2c0_Isr|
|I2C1|65|I2c1_Isr|
|I2C2|66|I2c2_Isr|
|I2C3|67|I2c3_Isr|
|GPIO0|68|Gpio_Icu0ExtIsr|
|GPIO1|69|Gpio_Icu1ExtIsr|
|GPIO2|70|Gpio_Icu2ExtIsr|
|WWDT0|71|Wdg_Ins0RstIsr|
||72|Wdg_Ins0IntIsr|
|WWDT1|73|Wdg_Ins1RstIsr|
||74|Wdg_Ins1IntIsr|
|WWDT2|75|Wdg_Ins2RstIsr|
||76|Wdg_Ins2IntIsr|
|OTF_CRC0|77|Otf_Isr|
|CRC0|78|Crc_Isr|
|GPT0|79|Gpt_Ins0Ch0Isr|
||80|Gpt_Ins0Ch1Isr|
||81|Gpt_Ins0Ch2Isr|
||82|Gpt_Ins0Ch3Isr|
|GPT1|83|Gpt_Ins1Ch0Isr|
||84|Gpt_Ins1Ch1Isr|
||85|Gpt_Ins1Ch2Isr|
||86|Gpt_Ins1Ch3Isr|
|GPT2|87|Gpt_Ins2Ch0Isr|
||88|Gpt_Ins2Ch1Isr|
||89|Gpt_Ins2Ch2Isr|
||90|Gpt_Ins2Ch3Isr|
|GPT3|91|Gpt_Ins3Ch0Isr|
||92|Gpt_Ins3Ch1Isr|
||93|Gpt_Ins3Ch2Isr|
||94|Gpt_Ins3Ch3Isr|
|GPT4|95|Gpt_Ins4Ch0Isr|
||96|Gpt_Ins4Ch1Isr|
||97|Gpt_Ins4Ch2Isr|
||98|Gpt_Ins4Ch3Isr|
|GPT5|99|Gpt_Ins5Ch0Isr|
||100|Gpt_Ins5Ch1Isr|
||101|Gpt_Ins5Ch2Isr|
||102|Gpt_Ins5Ch3Isr|
|PMU|103|Pmu_ReqDeny0Isr|
|BIFSPI|104||
|PVT|105|Pvt_McuAlarmIsr|
|L1FCHM|106|Fchm_MissionIntIsr|
||107|Fchm_NcfIntIsr|
||108|Fchm_CfIntIsr|
|CMM0|109|Cmm_Ins0Isr|
|CMM1|110|Cmm_Ins1Isr|
|PWM0|111|Pwm_Generic0Isr|
|XSPI|112|Xspi_Isr|
|CANFD0|113|Can0_TimestampIsr|
||114|Can0_WakeupIsr|
||115|Can0_ErrorIsr|
||116|Can0_DataIsr|
|CANFD1|117|Can1_TimestampIsr|
||118|Can1_WakeupIsr|
||119|Can1_ErrorIsr|
||120|Can1_DataIsr|
|CANFD2|121|Can2_TimestampIsr|
||122|Can2_WakeupIsr|
||123|Can2_ErrorIsr|
||124|Can2_DataIsr|
|CANFD3|125|Can3_TimestampIsr|
||126|Can3_WakeupIsr|
||127|Can3_ErrorIsr|
||128|Can3_DataIsr|
|CANFD4|129|Can4_TimestampIsr|
||130|Can4_WakeupIsr|
||131|Can4_ErrorIsr|
||132|Can4_DataIsr|
|CANFD5|133|Can5_TimestampIsr|
||134|Can5_WakeupIsr|
||135|Can5_ErrorIsr|
||136|Can5_DataIsr|
|CANFD6|137|Can6_TimestampIsr|
||138|Can6_WakeupIsr|
||139|Can6_ErrorIsr|
||140|Can6_DataIsr|
|CANFD7|141|Can7_TimestampIsr|
||142|Can7_WakeupIsr|
||143|Can7_ErrorIsr|
||144|Can7_DataIsr|
|CANFD8|145|Can8_TimestampIsr|
||146|Can8_WakeupIsr|
||147|Can8_ErrorIsr|
||148|Can8_DataIsr|
|CANFD9|149|Can9_TimestampIsr|
||150|Can9_WakeupIsr|
||151|Can9_ErrorIsr|
||152|Can9_DataIsr|
|mcu eth|153|Gmac_TxCh0Isr|
||154|Gmac_TxCh1Isr|
||155|Gmac_TxCh2Isr|
||156|Gmac_TxCh3Isr|
||157|Gmac_TxCh4Isr|
||158|Gmac_TxCh5Isr|
||159|Gmac_RxCh0Isr|
||160|Gmac_RxCh1Isr|
||161|Gmac_RxCh2Isr|
||162|Gmac_RxCh3Isr|
||163|Gmac_RxCh4Isr|
||164|Gmac_RxCh5Isr|
||165|Gmac_SbdIsr|
||166|Gmac_PmtIsr|
||167|Gmac_LpiIsr|
|LIN0|168|Lin0_Isr|
|LIN1|169|Lin1_Isr|
|LIN2|170|Lin2_Isr|
|SPI0|171|Spi0_Isr|
|SPI1|172|Spi1_Isr|
|SPI2|173|Spi2_Isr|
|SPI3|174|Spi3_Isr|
|SPI4|175|Spi4_Isr|
|SPI5|176|Spi5_Isr|
|IPC|177|Ipc_Ch0Isr|
||178|Ipc_Ch1Isr|
||179|Ipc_Ch2Isr|
||180|Ipc_Ch3Isr|
||181|Ipc_Ch4Isr|
||182|Ipc_Ch5Isr|
||183|Ipc_Ch6Isr|
||184|Ipc_Ch7Isr|
||185|Ipc_Ch8Isr|
||186|Ipc_Ch9Isr|
||187|Ipc_Ch10Isr|
||188|Ipc_Ch11Isr|
||189|Ipc_Ch12Isr|
||190|Ipc_Ch13Isr|
||191|Ipc_Ch14Isr|
||192|Ipc_Ch15Isr|
||193|Ipc_Ch16Isr|
||194|Ipc_Ch17Isr|
||195|Ipc_Ch18Isr|
||196|Ipc_Ch19Isr|
||197|Ipc_Ch20Isr|
||198|Ipc_Ch21Isr|
||199|Ipc_Ch22Isr|
||200|Ipc_Ch23Isr|
||201|Ipc_Ch24Isr|
||202|Ipc_Ch25Isr|
||203|Ipc_Ch26Isr|
||204|Ipc_Ch27Isr|
||205|Ipc_Ch28Isr|
||206|Ipc_Ch29Isr|
||207|Ipc_Ch30Isr|
||208|Ipc_Ch31Isr|
|MDMA0|209|Mdma0_Ch0Isr|
||210|Mdma0_Ch1Isr|
|MDMA1|211|Mdma1_Ch0Isr|
||212|Mdma1_Ch1Isr|
|PDMA0|213|PDMA0_Ch0Isr|
||214|PDMA0_Ch1Isr|
||215|PDMA0_Ch2Isr|
||216|PDMA0_Ch3Isr|
||217|PDMA0_Ch4Isr|
||218|PDMA0_Ch5Isr|
|PMC0|219|Pmc0_Isr|
|PMC1|220|Pmc1_Isr||PPS(RTC)|221|Pps_IcuRtcIsr|
|PPS(TIME_SYNC0)|222|Pps_Icu0Isr|
|PPS(TIME_SYNC1)|223|Pps_Icu1Isr|
|PPS(TIME_SYNC2)|224|Pps_Icu2Isr|
|PPS_SYNC|225|Pps0_Isr|
||226|Pps1_Isr|
||227|Pps2_Isr|
||228|Pps3_Isr|
||229|Pps4_Isr|
|HSM_IPC0|230|Ipc_HsmIpc0Ch4Isr|
||231|Ipc_HsmIpc0Ch5Isr|
||232|Ipc_HsmIpc0Ch6Isr|
||233|Ipc_HsmIpc0Ch7Isr|
|Reserved|234||
||235||
||236||
||237||
||238||
||239||
||240||
|HSM_IPC1|241|Ipc_HsmIpc1Ch4Isr|
||242|Ipc_HsmIpc1Ch5Isr|
||243|Ipc_HsmIpc1Ch6Isr|
||244|Ipc_HsmIpc1Ch7Isr|
|CPU_MP4_CMM|245|Cmm_Ins13Isr|
|CPU_MP4_CLUSTER_PMU|246|Pmu_ReqDeny1Isr|
|CPU_MP4_CDB_PMU|247|Pmu_ReqDeny2Isr|
|CPU_MP2_CMM|248|Cmm_Ins14Isr|
|CPU_MP2_CLUSTER_PMU|249|Pmu_ReqDeny3Isr|
|CPU_MP2_CDB_PMU|250|Pmu_ReqDeny4Isr|
|CPU_IPC1_CH0|251|Ipc_CpuIpc1Ch0Isr|
|CPU_IPC1_CH1|252|Ipc_CpuIpc1Ch1Isr|
|CPU_IPC1_CH2|253|Ipc_CpuIpc1Ch2Isr|
|CPU_IPC0_CH0|254|Ipc_CpuIpc0Ch0Isr|
|CPU_IPC0_CH1|255|Ipc_CpuIpc0Ch1Isr|
|CPU_IPC0_CH2|256|Ipc_CpuIpc0Ch2Isr|
|CPU_IPC0_CH3|257|Ipc_CpuIpc0Ch3Isr|
|CPU_IPC0_CH4|258|Ipc_CpuIpc0Ch4Isr|
|CPU_IPC0_CH5|259|Ipc_CpuIpc0Ch5Isr|
|CPU_IPC0_CH6|260|Ipc_CpuIpc0Ch6Isr|
|CPU_IPC0_CH7|261|Ipc_CpuIpc0Ch7Isr|
|CPU_IPC0_CH8|262|Ipc_CpuIpc0Ch8Isr|
|CPU_IPC0_CH9|263|Ipc_CpuIpc0Ch9Isr|
|CPU_IPC0_CH10|264|Ipc_CpuIpc0Ch10Isr|
|CPU_IPC0_CH11|265|Ipc_CpuIpc0Ch11Isr|
|CPU_IPC0_CH12|266|Ipc_CpuIpc0Ch12Isr|
|CPU_IPC0_CH13|267|Ipc_CpuIpc0Ch13Isr|
|CPU_IPC0_CH14|268|Ipc_CpuIpc0Ch14Isr|
|CPU_IPC0_CH15|269|Ipc_CpuIpc0Ch15Isr|
|CPU_ROUTER_SWTRIG1_0|270|Router_Swtrig1Ch0Isr|
|CPU_ROUTER_SWTRIG1_1|271|Router_Swtrig1Ch1Isr|
|CPU_ROUTER_SWTRIG1_2|272|Router_Swtrig1Ch2Isr|
|CPU_ROUTER_SWTRIG1_3|273|Router_Swtrig1Ch3Isr|
|DDR0_CMM|291|Cmm_Ins2Isr|
|DDR1_CMM|294|Cmm_Ins3Isr|
|DDR2_CMM|297|Cmm_Ins4Isr|
|PERI_I2C0|300|Peri_I2C0Isr|
|PERI_I2C1|301|Peri_I2C1Isr|
|PERI_I2C2|302|Peri_I2C2Isr|
|PERI_I2C3|303|Peri_I2C3Isr|
|PERI_I2C4|304|Peri_I2C4Isr|
|PERI_I2C5|305|Peri_I2C5Isr|
|PERI_USB|306|Peri_UsbIsr|
|PERI_CMM|307|Cmm_Ins19Isr|
|CAM_ISP0_0|308|Cam_Isp0Ch0Isr|
|CAM_ISP0_1|309|Cam_Isp0Ch1Isr|
|CAM_ISP0_2|310|Cam_Isp0Ch2Isr|
|CAM_ISP0_3|311|Cam_Isp0Ch3Isr|
|CAM_CPE0_PYM|312|Cam_Cpe0PymIsr|
|CAM_CPE0_MIPI_RX_CSI|313|Cam_Cpe0MipiRxCsiIsr|
|CAM_CPE0_CIM|314|Cam_Cpe0CimIsr|
|CAM_CPE0_PYM_PRE_UV|315|Cam_Cpe0PymPreUvIsr|
|CAM_CPE0_PYM_PRE_Y|316|Cam_Cpe0PymPreYIsr|
|CAM_CPE0_CMM|317|Cmm_Ins8Isr|
|CAM_ISP1_0|318|Cam_Isp1Ch0Isr|
|CAM_ISP1_1|319|Cam_Isp1Ch1Isr|
|CAM_ISP1_2|320|Cam_Isp1Ch2Isr|
|CAM_ISP1_3|321|Cam_Isp1Ch3Isr|
|CAM_CPE1_YNR|322|Cam_Cpe1YnrIsr|
|CAM_CPE1_PYM|323|Cam_Cpe1PymIsr|
|CAM_CPE1_MIPI_RX_CSI|324|Cam_Cpe1MipiRxCsiIsr|
|CAM_CPE1_CIM|325|Cam_Cpe1CimIsr|
|CAM_CPE1_PYM_PRE_UV|326|Cam_Cpe1PymPreUvIsr|
|CAM_CPE1_PYM_PRE_Y|327|Cam_Cpe1PymPreYIsr|
|CAM_CPE1_CMM|328|Cmm_Ins7Isr|
|CAM_STITCH|329|Cam_StichIsr|
|CAM_CPE_LITE_MIPI_RX|330|Cam_CpeLiteMipiRxCsiIsr|
|CAM_GDC0|331|Cam_Gdc0Isr|
|CAM_CPE_LITE_PYM|332|Cam_CpeLitePymIsr|
|CAM_CPE_LITE_PYM_PRE_UV|333|Cam_CpeLitePymPreUvIsr|
|CAM_CPE_LITE_PYM_PRE_Y|334|Cam_CpeLitePymPreYIsr|
|CAM_CPE_LITE_CIM|335|Cam_CpeLiteCymIsr|
|CAM_CPE_LITE_CMM|336|Cmm_Ins6Isr|
|CAM_MIPI_TX1_DSI|337|Cam_MipiTx1DsiIsr|
|CAM_MIPI_TX1_CSI2|338|Cam_MipiTx1Csi2Isr|
|CAM_MIPI_TX0_DSI|339|Cam_MipiTx0DsiIsr|
|CAM_MIPI_TX0_CSI2|340|Cam_MipiTx0Csi2Isr|
|CAM_IDU0|341|Cam_Idu0Isr|
|CAM_IDU1|342|Cam_Idu1Isr|
|CAM_IDE_CMM|343|Cmm_Ins5Isr|
|CAM_GPIO|344|Cam_GpioIsr|
|CAM_TOP_CMM|345|Cmm_Ins9Isr|
|VIDEO_VPU|346|Vid_VpuIsr|
|VIDEO_JPU|347|Vid_JpuIsr|
|VIDEO_CMM|348|Cmm_Ins10Isr|
|VIDEO_GIPO|349|Vid_GpioIsr|
|VDSP_CMM|350|Cmm_Ins11Isr|
|VDSP_Q8_EARLY_REST|351|Vdsp_EarlyRestIsr|
|VDSP_Q8_REST|352|Vdsp_ResetIsr|
|HSIS_CMM|353|Cmm_Ins12Isr|
|BPU_VM0|354|Bpu_Vm0Isr|
|BPU_VM1|355|Bpu_Vm1Isr|
|BPU_HYP|356|Bpu_HypIsr|
|BPU_PVT|357|Pvt_BpuAlarmIsr|
|BPU_CMM|358|Cmm_Ins17Isr|
|GPU_CMM|359|Cmm_Ins18Isr|
|RTC|360|Rtc_Isr|
|AON_WAKEUP_GPIO|361|Aon_WakeUpGpioIsr|
|AON_GPIO|362|Aon_GpioIsr|
|AON_PMU_REQ_MOD|363|Aon_PmuReqModIsr|
|MEDIA_BOT_CMM|364|Cmm_Ins15Isr|
|MEDIA_TOP_CMM|366|Cmm_Ins16Isr|
|CMN_CMM|368|Cmm_Ins20Isr|
|CMN_PVTC|369|Pvt_CmnAlarmIsr|
|CMN_PPU_PMU|370|Pmu_Ppu0Isr|

## MCU Interrupt Usage
Since MCU0 and MCU1 reside in the same hardware domain, both MCU0 and MCU1 receive the same interrupt when it occurs. Therefore, to ensure proper operation of the MCU system, each interrupt must be enabled exclusively on either MCU0 or MCU1. However, as MCU0 is not open-source externally, it is necessary to summarize the interrupts already used by MCU0 to prevent conflicts during customer development on MCU1.

Currently used interrupts on MCU0:
| Module | Interrupt Number | Name |
|--------|----------------------------------------|---------------------------------------------|
|GPIO0|68|Gpio_Icu0ExtIsr|
|GPIO1|69|Gpio_Icu1ExtIsr|
|GPIO2|70|Gpio_Icu2ExtIsr|
|WWDT0|71|Wdg_Ins0RstIsr|
|WWDT0|72|Wdg_Ins0IntIsr|
|WWDT1|73|Wdg_Ins1RstIsr|
|WWDT1|74|Wdg_Ins1IntIsr|
|WWDT2|75|Wdg_Ins2RstIsr|
|WWDT2|76|Wdg_Ins2IntIsr|
|GPT0|81|Gpt_Ins0Ch2Isr|
|GPT1|83|Gpt_Ins1Ch0Isr|
|GPT1|84|Gpt_Ins1Ch1Isr|
|L1FCHM|106|Fchm_MissionIntIsr|
||107|Fchm_NcfIntIsr|
||108|Fchm_CfIntIsr|
|PWM0|111|Pwm_Generic0Isr|
|MDMA1|211|Mdma1_Ch0Isr|
|PDMA0|213|PDMA0_Ch0Isr|
|PPS(RTC)|221|Pps_IcuRtcIsr|
|HSM_IPC1|241|Ipc_HsmIpc1Ch4Isr|
|HSM_IPC1|242|Ipc_HsmIpc1Ch5Isr|
|CPU_IPC1_CH0|251|Ipc_CpuIpc1Ch0Isr|
|CPU_IPC1_CH1|252|Ipc_CpuIpc1Ch1Isr|
|CPU_IPC1_CH2|253|Ipc_CpuIpc1Ch2Isr|
|CPU_IPC0_CH8|262|Ipc_CpuIpc0Ch8Isr|
|CPU_IPC0_CH9|263|Ipc_CpuIpc0Ch9Isr|
|CPU_IPC0_CH10|264|Ipc_CpuIpc0Ch10Isr|
|CPU_IPC0_CH11|265|Ipc_CpuIpc0Ch11Isr|
|CPU_IPC0_CH12|266|Ipc_CpuIpc0Ch12Isr|
|CPU_IPC0_CH13|267|Ipc_CpuIpc0Ch13Isr|
|CPU_IPC0_CH14|268|Ipc_CpuIpc0Ch14Isr|
|CPU_IPC0_CH15|269|Ipc_CpuIpc0Ch15Isr|
|CPU_ROUTER_SWTRIG1_0|270|Router_Swtrig1Ch0Isr|
|CPU_ROUTER_SWTRIG1_1|271|Router_Swtrig1Ch1Isr|
|CPU_ROUTER_SWTRIG1_2|272|Router_Swtrig1Ch2Isr|
|CPU_ROUTER_SWTRIG1_3|273|Router_Swtrig1Ch3Isr|
|RTC|360|Rtc_Isr|

## Guide to Adding Compilation Directories
### Brief Introduction to SCons
Currently, the RDK-S100 MCU only supports building for s100_sip_B, and uses SCons instead of Makefile for compilation.

SCons works similarly to Makefile: each directory contains an SConscript file (analogous to a Makefile), and a top-level SConstruct file controls the overall build process.  
For example, the MCU1 image is controlled by SConstruct_Lite_FRtos_S100_sip_B.

### Steps to Add a Compilation Directory
1. Modify the file `mcu/Build/FreeRtos_mcu1/SConstruct_Lite_FRtos_S100_sip_B` to add or remove corresponding modules.

   For instance, to add the `mcu/Service/Log` directory, simply add its path accordingly. The variable `False` indicates that source files will not be copied to the build output directory during the build process.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/scons_add_context.png)

2. Add an `SConscript` file in the newly added module directory. This `SConscript` file can be copied from any existing compiled module directory.

## Introduction to MCU FreeRTOS System
The MCU system includes several key functionalities, as illustrated below:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/freertos_system.png)

The diagram shows the relative task priorities of each function and the calling sequence within the same task. When integrating, customers must maintain the relative priorities, assigned core, and calling order within each task. Descriptions and important notes for each function are provided below:

### Power
**ScmiProcess**: Place in a high-priority task; it is recommended to run within a 2ms task. If this cannot be achieved, ensure the maximum scheduling period does not exceed 100ms. Placing it in a task with a long scheduling period will negatively impact boot time. The approximate impact can be estimated as:  
*"Number of SCMI communications during boot × task period"*.

**SysPower_State_Loop**: Place in a low-priority task.

### Boot
**AcoreBootProc**: Place in a low-priority task. This function includes necessary initializations for Acore boot-up, including initialization of the critical Housekeeping feature `Housekeeping_WriteMagicNum`. If this function is not properly initialized, Acore accesses to MCU registers will cause Acore exceptions.

**Integration Note**: This must be handled on MCU0. AcoreBoot requires flash access, so flash resource conflicts must be avoided. It should be placed in the same low-priority task as the OTA functionality described below.### OTA  
OtaFlash_MainFunction: Place in a low-priority task, as it involves OTA-related processing logic.

Integration Note: This must be handled on MCU0. The OTA feature requires flash, IPC, and crypto functionalities. To avoid conflicts caused by concurrent flash operations, it is recommended to place all flash-related operations into a single low-priority task for serialized access. For example, the previously mentioned AcoreBootProc runs in the same low-priority task.

### Sleep/Wake-up  
SysPower_McuCoreEnterLowPower: Place this function in the highest-priority task with the shortest possible cycle supported by this core.

Integration Note: This function only executes when sleep/wake-up is required; otherwise, it exits quickly without introducing additional overhead.

### System Interrupt Description  
Communication between the MCU and Acore/HSM relies on IPC. For details about interrupts used by the IPC system service, refer to the section: [IPC Introduction](../../../07_Advanced_development/05_mcu_development/01_S100/08_mcu_ipc.md).  
It is recommended to configure the priority of these interrupts higher than regular functional interrupts. These interrupts can be assigned the same priority level.

## Introduction to FreeRTOS System  
There are two mainstream startup approaches for FreeRTOS:  
1. In the main function, perform hardware initialization, RTOS system initialization, and create all tasks before finally starting the RTOS scheduler to begin multitasking.  
2. In the main function, initialize hardware and the RTOS system first, then create a single startup task and start the scheduler. Within this startup task, create all application tasks. Once all tasks are successfully created, the startup task deletes itself.  

Neither approach is inherently superior; RDK-S100 adopts the first method.

### FreeRTOS Task Creation  
Task creation is located in `/mcu/Target/Target-hobot-lite-freertos-mcu1/target/FreeRtosOsHal/Task_Hal.c`. Example:

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/task_init.png)

- `xxx_Startup` task: Handles startup initialization and executes only once.  
- `FreeRtos_OsTask_SysCore_BSW_xms` and `FreeRtos_OsTask_SysCore_ASW_xms`: Periodic tasks scheduled according to their respective xms intervals. These tasks also perform internal processing; for details, refer to the previous section "Introduction to MCU FreeRTOS System".

If customers develop their own code, they can refer to the examples above. Alternatively, they can implement their demo logic within existing tasks (see below).  
Task functions are defined in `/mcu/Target/Target-hobot-lite-freertos-mcu1/target/HorizonTask.c`.  
Taking `OsTask_SysCore_BSW_10ms` as an example, this task periodically checks for Shell transaction processing:
```c
TASK(OsTask_SysCore_BSW_10ms)
{
    #ifdef SHELL_ENABLE
        Shell_Handler();
    #endif
}
```

### FreeRTOS Interrupt Usage  
FreeRTOS interrupt handling is centralized in `/mcu/Target/Target-hobot-lite-freertos/target/FreeRtosOsHal/Isr_Hal.c`:
```c
void FreeRtos_Irq_Init(void)
{
  int interrupt_index = 0;
  for(; interrupt_index < INTERRUPT_MCU_MAX_NUM; interrupt_index++)
  {
    if((!Interrupt_McuConfigs[interrupt_index].irqNumber) && (!Interrupt_McuConfigs[interrupt_index].priority)
        && (!Interrupt_McuConfigs[interrupt_index].Handler) && (!Interrupt_McuConfigs[interrupt_index].enable_flag))
    {
      break;
    }

    if(Interrupt_McuConfigs[interrupt_index].Handler)
    {
      INT_SYS_InstallHandler(Interrupt_McuConfigs[interrupt_index].irqNumber, Interrupt_McuConfigs[interrupt_index].Handler, NULL);
    }

    if(Interrupt_McuConfigs[interrupt_index].priority)
    {
      INT_SYS_SetPriority(Interrupt_McuConfigs[interrupt_index].irqNumber, Interrupt_McuConfigs[interrupt_index].priority);
    }

    if(Interrupt_McuConfigs[interrupt_index].enable_flag)
    {
      INT_SYS_EnableIRQ(Interrupt_McuConfigs[interrupt_index].irqNumber);
    }
  }
}
```

If no interrupt handler is configured, the default interrupt handler is used (see `/mcu/Target/Target-hobot-lite-freertos-mcu1/target/SuperSoC_ISR.s`).  
For example, the RTC interrupt handler:
```c
// DefaultISR---default interrupt handler
    .align  4
    .weak   DefaultISR
    .type   DefaultISR, %function
DefaultISR:
    hlt #0
    b   .
    .pool
    .size   DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro  def_irq_handler handler_name
    .weak   \handler_name
    .set    \handler_name, DefaultISR
    .endm

// Set default RTC interrupt handler
    def_irq_handler AON_RTC_INTR
```

Note:  
When enabling interrupts on MCU1, ensure that the corresponding interrupts on MCU0 are disabled!!!

### Introduction to FreeRTOS Memory Management Scheme  
FreeRTOS memory management schemes are located in `/mcu/OpenSource/FreeRTOS/portable/MemMang/`, offering five memory management algorithms: `heap_1.c`, `heap_2.c`, `heap_3.c`, `heap_4.c`, and `heap_5.c`. The FreeRTOS memory management module optimizes memory utilization and efficiency through memory allocation and deallocation operations, while minimizing memory fragmentation.

#### heap_1.c  
`heap_1.c` is the simplest memory management scheme provided by FreeRTOS. It supports memory allocation but not deallocation. This is ideal for safety-critical embedded systems, as the inability to free memory prevents fragmentation and potential system crashes. However, its drawback is low memory utilization—once allocated, memory cannot be reclaimed even if used only once.

#### heap_2.c  
`heap_2.c` uses a best-fit algorithm. For example, if 100 bytes are requested and available memory blocks are 200, 500, and 1000 bytes, the 200-byte block is split, and the remaining portion is returned to the free list. This scheme supports deallocation and reinserts freed memory into the list sorted by size. However, it cannot coalesce adjacent free blocks, leading to fragmentation when allocation sizes vary. (In contrast, `heap_4.c` solves this by merging adjacent blocks.)

#### heap_3.c  
`heap_3.c` simply wraps the standard C library’s `malloc()` and `free()` functions, ensuring compatibility with common compilers. These wrapped functions are made thread-safe by suspending the scheduler before memory operations and resuming it afterward.

#### heap_4.c  
Like `heap_2.c`, `heap_4.c` uses a best-fit algorithm but also includes a coalescing mechanism to merge adjacent free blocks, reducing fragmentation. It is especially suitable for port layers that directly use `pvPortMalloc()` and `vPortFree()`. Unlike `heap_2.c`, its free block list is sorted by memory address (lower addresses first) to facilitate efficient merging during deallocation.

#### heap_5.c  
`heap_5.c` uses the same best-fit and coalescing algorithms as `heap_4.c` but supports memory heaps spanning multiple non-contiguous memory regions (e.g., internal RAM and external SDRAM). This scheme is more complex and slightly less real-time efficient than `heap_4.c`.

#### RDK-S100 Memory Scheme  
RDK-S100 uses `heap_4.c`, which combines best-fit allocation and coalescing. It supports allocation and deallocation of arbitrary-sized memory blocks, minimizes fragmentation, covers all real-time system scenarios, and offers good real-time performance.

## LOG Area Adjustment  
### MCU1 Area Adjustment  
Modify the relevant section in `/mcu/Build/FreeRtos_mcu1/Linker/gcc/S100.ld`. The size cannot be changed for now.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/mcu_log_address.png)

### Acore Area Adjustment  
Modify the corresponding section in `/source/hobot-drivers/kernel-dts/drobot-s100-soc.dtsi`, keeping it consistent with the MCU1 modification.

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/acore_log_address.png)

## Reserved Shared Memory Region Between MCU and Acore  
This shared memory region is allocated within the MCU0 address space. However, since both MCU0 and MCU1 belong to the same MCU SRAM domain, MCU1 can also access this address range:
```c
MCU_STATE_Reserved      : org = 0x0C800400, len = 1K
```

### Currently Occupied Regions:
```c
MCU1_VERSION:  org = 0x0C800400, len = 0x60
MCU_ALIVE:     org = 0x0C800460, len = 0x10
     ---MCU0_ALIVE: org = 0x0C800460, len = 0x04;
     ---MCU1_ALIVE: org = 0x0C800464, len = 0x04;
     ---REVERSED:   org = 0x0C800468, len = 0x08;
```

### Usage Notes  
When using shared memory for data transfer, a synchronization issue may arise: MCU updates data in SRAM, but Acore still reads stale data from its cache.

To prevent data inconsistency between Acore and MCU, prepend variables with `volatile` or use the `ioremap_np()` function. Both methods ensure direct reads from SRAM instead of cached values.