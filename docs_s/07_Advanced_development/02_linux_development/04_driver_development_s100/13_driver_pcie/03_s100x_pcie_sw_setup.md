---
sidebar_position: 3
---
# S100X PCIe模块功能在kernel下的配置

在Kernel中，PCIe的配置分为defconfig和DTS两部分：

## defconfig

在内核的defconfig文件中，需要包含如下基础配置：

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

使能RC模式需要的配置如下：

```shell
CONFIG_PCIE_HOBOT=m
CONFIG_PCIE_HOBOT_RC=m
CONFIG_PCIE_HOBOT_EP_DEV=m
CONFIG_PCIE_HOBOT_EP_DEV_MAN=m
```

RC模式支持NVME需要的配置如下：

```shell
CONFIG_BLK_DEV_NVME=m
```

使能EP模式需要的配置如下：

```shell
CONFIG_PCIE_HOBOT=m
CONFIG_PCIE_HOBOT_EP=m
CONFIG_PCIE_HOBOT_EP_FUN=m
```

其中hybrid为必须使能的功能，已经被上述配置包含，其他功能可以按需配置。

如果有debug的需求（具体使用参考：S100 PCIe驱动debug信息汇总）：

```shell
CONFIG_PCIE_HOBOT_DEBUG=y
```

如果需要功能安全：

```shell
CONFIG_PCIE_HOBOT_STL=y
```

## DTS
要注意同一个控制器只能配置为RC或者EP模式。

### RC模式

控制器的配置节点为： `hobot_pcie_rc0` 和 `hobot_pcie_rc1` 。

一般情况下只需要修改 `status` 字段由 `disable` 为 `okay` 即可使能对应控制器的RC模式，反之关闭。

### EP模式

控制器的配置节点为： `hobot_pcie_ep0` 和 `hobot_pcie_ep1` 。

除了控制器本身需要配置，其子节点 `funX` 也需要进行配置。
`fun0` 必须配置为使能状态。
