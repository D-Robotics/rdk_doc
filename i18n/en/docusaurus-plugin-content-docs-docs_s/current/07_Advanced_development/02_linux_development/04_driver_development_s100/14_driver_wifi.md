---
sidebar_position: 14
---
# WiFi Driver Debugging Guide

The WiFi module on RDKS100 is connected to an M.2 interface extended via PCIe. This section introduces some user-space commands and kernel DTS configuration items.

The examples in the following sections use the AW-XM612 module as a reference. Users should modify the configurations accordingly based on their specific module.

## User-Space Debugging

### Verify PCIe Endpoint Device

You can use the user-space command `lspci` to confirm whether the WiFi module is properly recognized.

```bash
# Check current endpoint nodes
root@ubuntu:~# lspci -vt
-+-[0000:01]---00.0-[02-07]----00.0-[03-07]--+-00.0-[04]----00.0  Anchor Chips Inc. Device bd31
 |                                           +-02.0-[05]----00.0  Realtek Semiconductor Co., Ltd. Device 5765
 |                                           +-06.0-[06]----00.0  ASMedia Technology Inc. Device 3042
 |                                           \-0e.0-[07]----00.0  ASMedia Technology Inc. Device 3042
 \-[0000:00]-

# Check specific node details:
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

## Module Driver Code

The driver code for the AW-XM612 module is provided by the module vendor and integrated by Digua. The integrated code resides in the following directory:  
`source/kernel/drivers/net/wireless/broadcom/brcm80211/`.

## Kernel Configuration for the Module

The driver for the AW-XM612 module requires enabling the following configurations:

```defconfig
CONFIG_CFG80211=m
CONFIG_BRCMFMAC=m
CONFIG_BRCMFMAC_PCIE=y
```

## Kernel DTS Configuration

Generally, a PCIe-based WiFi module requires the host side to control signals such as reset and `reg_on`. On the S100, these configurations are defined in:  
`source/hobot-drivers/kernel-dts/rdk-v0p5.dtsi`:

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

During initialization, the PCIe driver on the S100 will request these GPIOs and perform operations such as deasserting reset.