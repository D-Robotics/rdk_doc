---
sidebar_position: 14
---
# WiFi驱动调试指南
RDKS100的WiFi接在由PCIe拓展出来的M.2接口上。本章节会介绍部分用户层命令和内核dts配置项。

本章节后续示例以AW-XM612模组为例，用户需要根据自己使用的具体模组进行对应的修改。

## 用户层调试
### 确认PCIe ep设备
可以通过用户层命令`lspci`来确认WiFi模组是否正常被识别。
```bash
# 确认当前有哪些ep节点
root@ubuntu:~# lspci -vt
-+-[0000:01]---00.0-[02-07]----00.0-[03-07]--+-00.0-[04]----00.0  Anchor Chips Inc. Device bd31
 |                                           +-02.0-[05]----00.0  Realtek Semiconductor Co., Ltd. Device 5765
 |                                           +-06.0-[06]----00.0  ASMedia Technology Inc. Device 3042
 |                                           \-0e.0-[07]----00.0  ASMedia Technology Inc. Device 3042
 \-[0000:00]-

# 确认具体的节点：
root@ubuntu:~# lspci -v -s 04:00.0
04:00.0 Network controller: Anchor Chips Inc. Device bd31 (rev 02)
        Subsystem: Anchor Chips Inc. Device 0000
        Flags: bus master, fast devsel, latency 0, IRQ 181, IOMMU group 18
        Memory at 8000400000 (64-bit, non-prefetchable) [size=64K]
        Memory at 8000800000 (64-bit, non-prefetchable) [size=8M]
        Capabilities: [48] Power Management version 3
        Capabilities: [58] MSI: Enable+ Count=1/32 Maskable- 64bit+
        Capabilities: [68] Vendor Specific Information: Len=38 <?>
        Capabilities: [a0] MSI-X: Enable- Count=64 Masked-
        Capabilities: [ac] Express Endpoint, MSI 00
        Capabilities: [100] Advanced Error Reporting
        Capabilities: [13c] Device Serial Number 00-00-00-ff-ff-00-00-00
        Capabilities: [150] Power Budgeting <?>
        Capabilities: [160] Virtual Channel
        Capabilities: [1b0] Latency Tolerance Reporting
        Capabilities: [220] Physical Resizable BAR
        Capabilities: [240] L1 PM Substates
        Capabilities: [200] Precision Time Measurement
        Kernel driver in use: brcmfmac
        Kernel modules: brcmfmac
```
## 模组驱动代码
AW-XM612模组的驱动代码由模组厂提供，地瓜集成，集成的代码路径在：`source/kernel/drivers/net/wireless/broadcom/brcm80211/`文件夹内。

## 模组内核配置
AW-XM612模组的驱动需要使能以下配置：
```defconfig
CONFIG_CFG80211=m
CONFIG_BRCMFMAC=m
CONFIG_BRCMFMAC_PCIE=y
```

## 内核DTS配置
PCIe拓展的WiFi模组一般需要Host端对模组的reset信号/reg_on等信号进行控制，在S100上，这部分配置被定义在了`source/hobot-drivers/kernel-dts/rdk-v0p5.dtsi`内：
```dts
&hobot_pcie_rc0 {
	refclk-mode = <2>; /* 0:internal; 1:CC; 2:SRNS; 3:SRIS; */
	num-lanes = <2>;

	switch-perst-gpios = <&gpio_exp_27 14 GPIO_ACTIVE_LOW>;	/* SWITCH_PERSTB */

	ep-ponrst-gpios = <&gpio_exp_24 3 GPIO_ACTIVE_LOW>,	/* WIFI_REG_ON */
			<&gpio_exp_20 0 GPIO_ACTIVE_LOW>,	/* USBHUB1_PWRON */
			<&gpio_exp_20 7 GPIO_ACTIVE_LOW>;	/* USBHUB2_PWRON */

	ep-perst-gpios = <&gpio_exp_20 3 GPIO_ACTIVE_LOW>,	/* NVME_PERSTB */
			<&gpio_exp_24 2 GPIO_ACTIVE_LOW>,	/* WIFI_PERSTB */
			<&gpio_exp_27 15 GPIO_ACTIVE_LOW>,	/* USBHUB1_PERSTB */
			<&gpio_exp_20 5 GPIO_ACTIVE_LOW>;	/* USBHUB2_PERSTB */
};
```
S100的PCIe驱动会在初始化时，申请这些GPIO并作解复位等操作。
