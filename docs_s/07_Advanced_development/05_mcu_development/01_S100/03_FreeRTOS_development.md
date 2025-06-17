---
sidebar_position: 3
---

# MCU1开发指南

## MCU中断号及模块对应关系
| 模块 | 中断号 | 名称 |
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
|PMC1|220|Pmc1_Isr|
|PPS(RTC)|221|Pps_IcuRtcIsr|
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

## MCU中断使用情况
由于MCU0和MCU1处于统一硬件域，所以当中断产生时，MCU0/MCU1都能接收到同一中断。因此为了保障MCU系统的正常运行，统一中断只能由MCU0或MCU1使能。但是又因为MCU0不对外开源，因此需要对MCU0使用的中断进行总结，避免MCU1客户开发过程中使用冲突。

目前MCU0已经使用的中断：
| 模块 | 中断号 | 名称 |
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

## 增加编译目录教程
### scons简述
目前RDK-S100 mcu仅仅支持s100_sip_B的编译，并且采用的是scons编译方式取代了Makefile。
Scons跟Makefile类似，每个文件夹由Sconscript编译文件（类似于Makefile），最后有个总的SConstruct文件总体去控制编译。
如mcu1的镜像就是SConstruct_Lite_FRtos_S100_sip_B控制。
### 增加编译目录流程
1. 修改mcu/Build/FreeRtos_mcu1/SConstruct_Lite_FRtos_S100_sip_B文件，增加/删除相应的模块。
   
   如增加mcu/Service/Log文件夹，只需增加相应的位置即可。变量False表示控制构建过程中不会将源文件复制到编译输出目录。

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/scons_add_context.png)

2. 在添加编译的模块下，添加SConscript文件，SConscript文件可以从任意已经编译的模块文件夹下获取

## MCU FreeRtos系统简介
MCU这边有几个系统关键功能，如下图所示：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/freertos_system.png)

上图可以看到各个功能所在任务的相对优先级及同一个任务中的调用顺序，客户集成请保持各功能的相对优先级、所在core及同一个任务中的调用顺序。各个功能的说明及注意事项如下：

### Power
ScmiProcess：放在高优先级任务中，建议放在2ms任务中。如果不能满足，最大调度周期不要超过100ms。放在调度周期长的任务中会影响启动时间，一般评估影响可以按照启动过程中的"scmi通讯次数x所在任务周期"计算。

SysPower_State_Loop：放在低优先级任务中。

### Boot
AcoreBootProc：放在低优先级任务中。这个里面会有Acore启动需要的相关初始化等。其中就有Housekeeping关键功能的初始化 Housekeeping_WriteMagicNum，如果该功能未被正常初始化，Acore对MCU的寄存器访问会导致Acore异常。

集成注意：需要放在MCU0上处理。AcoreBoot需要使用flash，需要避免flash冲突问题。和下文的OTA功能都放到同一个低优任务中处理。

### OTA
OtaFlash_MainFunction：放在低优先级任务，涉及到OTA相关处理逻辑。

集成注意：需要放在MCU0上处理。OTA功能需要使用flash、IPC以及crypto功能。需要避免flash并发操作的冲突问题，建议将所有使用到flash相关的功能放到一个低优先级的task中串行使用。比如前文提到的AcoreBootProc就是和它在同一个低优任务中。

### 休眠唤醒
SysPower_McuCoreEnterLowPower：放在本core上能支持的最短周期最高优先级任务中。

集成注意：该函数只有在需要休眠唤醒时才会真正运行，其他时候都是快速退出不会产生额外耗时。

### 系统中断说明
MCU和Acore/HSM通信依赖IPC，IPC系统服务涉及到的中断可以参考：[IPC的相关介绍](../../../07_Advanced_development/05_mcu_development/01_S100/08_mcu_ipc.md) 章节
这些中断优先级建议配置成比平常的功能类中断优先级高，这些中断本身可以配置成同样的优先级。

## FreeRtos系统简介
FreeRTOS的主流的启动方式有两种：第一种，在main函数中将硬件初始化，RTOS系统初始化，所有任务的创建这些都弄好，最后启动RTOS的调度器，开始多任务的调度；第二种，在main函数中将硬件和RTOS系统先初始化好，然后创建一个启动任务后就启动调度器，在启动任务里面创建各种应用任务，当所有任务都创建成功后，启动任务把自己删除。两种方式没有太强的优劣之分，RDK-S100选择第一种方式。
### FreeRtos系统任务创建
任务创建位于/mcu/Target/Target-hobot-lite-freertos-mcu1/target/FreeRtosOsHal/Task_Hal.c中，举例如下：

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/task_init.png)

xxx_Startup任务，为启动初始化相关的函数，只执行一次。
FreeRtos_OsTask_SysCore_BSW_xms和FreeRtos_OsTask_SysCore_ASW_xms为周期性任务，会根据xms的不同产生周期性的调度。同时周期性任务内部会有工作处理，细节见本章上一节"MCU FreeRtos系统简介"章节。

如果客户自行开发，可参考上述两种类型的例子。也可以在已经创建的任务中处理自己的demo，见下文。
任务函数位于/mcu/Target/Target-hobot-lite-freertos-mcu1/target/HorizonTask.c文件中，
以OsTask_SysCore_BSW_5ms为例，任务会周期性地检测Can事务处理：
```c
TASK(OsTask_SysCore_BSW_5ms)
{
    hb_CAN2IPC_MainFunction();
    Hb_Ipc2Can_MainFunction();
    if (Eth_Test == 1) {
        EthTest_Mainfunc();
        Eth_Receive(0, 0, &ethRxStat);
    }
}
```
### FreeRtos系统中断使用
FreeRtos的中断使用集中在/mcu/Target/Target-hobot-lite-freertos/target/FreeRtosOsHal/Isr_Hal.c文件中，
```c
void FreeRtos_Irq_Init(void)
{
  FreeRtosInstallHandler(); // 中断处理函数设置
  Isr_SetPriority(); // 中断优先级设置
  Isr_Enable(); // 中断使能
}
```

如果没有设置中断处理函数，那么中断处理函数处于默认状态，见/mcu/Target/Target-hobot-lite-freertos-mcu1/target/SuperSoC_ISR.s文件。
以RTC中断处理函数为例：
```c
// DefaultISR---默认中断处理函数
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

// 设置RTC默认中断处理函数
    def_irq_handler AON_RTC_INTR
```

注意：
在MCU1使能中断的时候一定要确保MCU0相应的中断处于关闭状态！！！

### FreeRtos内存管理方案简介
FreeRtos内存管理方案位于/mcu/OpenSource/FreeRTOS/portable/MemMang/文件夹中，共有5 种内存管理算法，分别是heap_1.c、heap_2.c、heap_3.c、heap_4.c和heap_5.c。FreeRTOS 的内存管理模块通过对内存的申请、释放操作，来管理用户和系统对内存的使用，使内存的利用率和使用效率达到最优，同时最大限度地解决系统可能产生的内存碎片问题。
#### heap_1.c
heap_1.c 管理方案是 FreeRTOS 提供所有内存管理方案中最简单的一个，它只能申请内存而不能进行内存释放，这样子对于要求安全的嵌入式设备来说是最好的，因为不允许内存释放，就不会产生内存碎片而导致系统崩溃，但是也有缺点，那就是内存利用率不高，某段内存只能用于内存申请的地方，即使该内存只使用一次，也无法让系统回收重新利用。
#### heap_2.c
heap_2.c方案与heap_1.c方案采用的内存管理算法不一样，它采用一种最佳匹配算法(best fit algorithm)，比如我们申请100字节的内存，而可申请内存中有三块对应大小200字节，500字节和1000字节大小的内存块，按照算法的最佳匹配，这时候系统会把200字节大小的内存块进行分割并返回申请内存的起始地址，剩余的内存则插回链表留待下次申请。Heap_2.c方案支持释放申请的内存，将释放的内存重新插入链表，并按照大小进行排序，但是它不能把相邻的两个小的内存块合成一个大的内存块，对于每次申请内存大小都比较固定的，这个方式是没有问题的，而对于每次申请并不是固定内存大小的则会造成内存碎片，后面要讲解的heap_4.c方案采用的内存管理算法能解决内存碎片的问题，可以把这些释放的相邻的小的内存块合并成一个大的内存块。
#### heap_3.c
heap_3.c方案只是简单的封装了标准C库中的malloc()和free()函数，并且能满足常用的编译器。重新封装后的malloc()和free()函数具有保护功能，采用的封装方式是操作内存前挂起调度器、完成后再恢复调度器。
#### heap_4.c
heap_4.c方案与heap_2.c方案一样都采用最佳匹配算法来实现动态的内存分配，但是不一样的是heap_4.c方案还包含了一种合并算法，能把相邻的空闲的内存块合并成一个更大的块，这样可以减少内存碎片。heap_4.c方案特别适用于移植层中可以直接使用pvPortMalloc()和vPortFree()函数来分配和释放内存的代码。heap_4.c内存管理方案的空闲块链表不是以内存块大小进行排序的，而是以内存块起始地址大小排序，内存地址小的在前，地址大的在后，因为heap_4.c 方案还有一个内存合并算法，在释放内存的时候，假如相邻的两个空闲内存块在地址上是连续的，那么就可以合并为一个内存块，这也是为了适应合并算法而作的改变。
#### heap_5.c
heap_5.c 方案在实现动态内存分配时与 heap4.c 方案一样，采用最佳匹配算法和合并算法，并且允许内存堆跨越多个非连续的内存区，也就是允许在不连续的内存堆中实现内存分配，比如用户在片内RAM中定义一个内存堆，还可以在外部SDRAM再定义一个或多个内存堆，这些内存都归系统管理。该方案较为复杂，实时性略逊于heap_4.c。
#### RDK-S100内存方案
RDK-S100采用的是heap_4.c方案，该方案结合最佳匹配算法和合并算法，可以分配和释放随机字节内存，在避免内存碎片的同时覆盖实时系统内存分配的全场景，并且实时性较好。

## LOG区域调整
### MCU1区域调整
修改/mcu/Build/FreeRtos_mcu1/Linker/gcc/S100.ld文件中相应位置，大小暂不支持修改

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/mcu_log_address.png)

### Acore区域调整
修改/source/hobot-drivers/kernel-dts/drobot-s100-soc.dtsi文件中相应位置，与MCU1修改保持一致

![](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/05_mcu_development/01_S100/FreeRTOS_development/acore_log_address.png)

## MCU与Acore共享内存区域预留
该共享内存区域空间开辟在MCU0所在空间，但MCU0和MCU1同属于MCU SRAM域，因此MCU1也可以使用相应地址
```c
MCU_STATE_Reserved      : org = 0x0C800400, len = 1K
```
### 目前已经被占用的区域：
```c
MCU1_VERSION:  org = 0x0C800400, len = 0x60
MCU_ALIVE:     org = 0x0C800460, len = 0x10
     ---MCU0_ALIVE：org = 0x0C800460, len = 0x04；
     ---MCU1_ALIVE：org = 0x0C800464, len = 0x04；
     ---REVERSED：  org = 0x0C800468, len = 0x08；
```
### 使用注意事项
如果使用共享内存的方式传输数据，可能会出现MCU数据更新至SRAM，但是Acore的缓存还为旧数据的问题，因此导致读取数据不同步。

为避免Acore和MCU出现数据不同步的问题，需要在变量前加"volatile"或者"ioremap_np()函数"。这两种方式都是为了避免读取缓存，而是直接读取SRAM数据。
