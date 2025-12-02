---
sidebar_position: 3
---
# Configuration of S100X PCIe Module Functionality under the Kernel

In the kernel, PCIe configuration consists of two parts: defconfig and DTS.

## defconfig

The following basic configurations must be included in the kernel's defconfig file:

```shell
CONFIG_PCI=y
CONFIG_PCI_DOMAINS=y
CONFIG_PCI_DOMAINS_GENERIC=y
CONFIG_PCI_SYSCALL=y
CONFIG_PCIEAER=y
CONFIG_PCIE_PTM=y
CONFIG_PCI_MSI=y
CONFIG_PCI_MSI_IRQ_DOMAIN=y
CONFIG_PCI_QUIRKS=y
CONFIG_PCI_DEBUG=y
CONFIG_PCI_REALLOC_ENABLE_AUTO=y
CONFIG_PCI_IOV=y
CONFIG_PCIE_BUS_DEFAULT=y
CONFIG_PCIEPORTBUS=y
```

Configurations required to enable RC mode are as follows:

```shell
CONFIG_PCIE_HOBOT=m
CONFIG_PCIE_HOBOT_RC=m
CONFIG_PCIE_HOBOT_EP_DEV=m
CONFIG_PCIE_HOBOT_EP_DEV_MAN=m
```

Configurations required for RC mode to support NVMe are as follows:

```shell
CONFIG_BLK_DEV_NVME=m
```

Configurations required to enable EP mode are as follows:

```shell
CONFIG_PCIE_HOBOT=m
CONFIG_PCIE_HOBOT_EP=m
CONFIG_PCIE_HOBOT_EP_FUN=m
```

Among these, hybrid functionality must be enabled and is already included in the above configurations. Other features can be configured as needed.

```shell
CONFIG_PCIE_HOBOT_DEBUG=y
```

## DTS

Note that the same controller can only be configured as either RC or EP mode, not both.

### RC Mode

The configuration nodes for the controllers are: `hobot_pcie_rc0` and `hobot_pcie_rc1`.

Typically, you only need to change the `status` field from `disable` to `okay` to enable RC mode for the corresponding controller, and vice versa to disable it.

### EP Mode

The configuration nodes for the controllers are: `hobot_pcie_ep0` and `hobot_pcie_ep1`.

In addition to configuring the controller itself, its child nodes `funX` also need to be configured.  
`fun0` must be enabled.

### PCIe Link Configuration

PCIe links support three modes:
- PCIe0 x4 Lane;
- PCIe0 x2 Lane + GMAC0 + GMAC1;
- PCIe0 x1 Lane + PCIe1 x1 Lane + GMAC0 + GMAC1;

Link configuration in the DTS is as follows:
```dts
/* rdk-v0p5.dtsi */
...

    &hsis0 {
        hsi-mode = <0x4>;  /* 0x1: pcie x4, 0x4: pcie x2 + gmac0 + gmac1, 0x8: pcie0 x1 + pcie1 x1 + gmac0 + gmac1 */
    };

...
```