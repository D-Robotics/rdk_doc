---
sidebar_position: 2
---
# S100X PCIe Software Architecture and Module Partitioning

## Software Framework

The PCIe software framework is divided into two parts: RC and EP:

![S100X_PCIE_sw_arch](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/pcie/sw_arch-en.jpg)

## Driver Module Description

Driver-related source code is located under the hobot-drivers/pcie directory. Details of each module are as follows:

| side | component                          | output                    | Source file                     |
|------|------------------------------------|---------------------------|---------------------------------|
| both | S13E01C01 PCIe basic driver        | hobot-pcie-common.ko      | hobot-ep-dev/hobot-pcie-common/ |
| both | S13E01C03 PCIe controller driver   | hobot-pcie.ko             | hobot.c                         |
| RC   | S13E01C02 PCIe device manager      | hobot-pcie-dev-manager.ko | hobot-ep-dev/hobot-pcie-dev-manager |
| RC   | S13E01C04 PCIe RC controller driver | hobot-pcie-rc.ko          | hobot-rc.c                      |
| RC   | S13E01C10 PCIe hybrid device driver | hobot-pcie-ep-dev.ko      | hobot-ep-dev/hobot-pcie-ep-dev  |
| RC   | S13E01C11 PCIe device wrapper      |                           |                                 |
| EP   | S13E01C05 PCIe EP controller driver | hobot-pcie-ep.ko          | hobot-ep.c                      |
| EP   | S13E01C06 PCIe function wrapper    | hobot-pcie-ep-fun.ko      | hobot-ep-fun/                   |
| EP   | S13E01C07 PCIe hybrid function driver |                           |                                 |

## PCIe Driver Loading/Unloading

### RC-side Loading

```shell
modprobe hobot-pcie
modprobe hobot-pcie-rc
modprobe hobot-pcie-ep-dev
modprobe hobot-pcie-dev-manager
```

### EP-side Loading

```shell
modprobe hobot-pcie
modprobe hobot-pcie-ep-fun
```

### RC-side Unloading

```shell
rmmod hobot_pcie_dev_manager
rmmod hobot_pcie_ep_dev
rmmod hobot_pcie_rc
rmmod hobot_pcie_common
rmmod hobot_pcie
```

### EP-side Unloading

```shell
rmmod hobot_pcie_ep_fun
rmmod hobot_pcie_ep
rmmod hobot_pcie_common
rmmod hobot_pcie
```

Notes:

1. Before unloading drivers, ensure all PCIe applications have been terminated. Always unload RC drivers before EP drivers.
2. Before system sleep, follow this sequence:
   - Stop PCIe applications on both sides
   - Unload RC drivers
   - Unload EP drivers
3. After system wake-up, follow this sequence:
   - Load EP drivers
   - Load RC drivers
   - Start PCIe applications
4. If PCIe link instability or a peer reboot causes a PCIe link down, reboot both RC and EP to ensure PCIe functionality and system stability.
5. Do not load PCIe-related drivers when PCIe functionality is not in use.
6. Before system reboot, follow this sequence:
   - Stop PCIe applications on both sides
   - Unload RC drivers
   - Unload EP drivers
   - Reboot the RC system
   - Reboot the EP system

## User-space Module Description

User-space related source code is located under the hbre directory. Details of each module are as follows:

| side | component                     | output          | Source file  |
|------|-------------------------------|-----------------|--------------|
| both | S13E01C15 PCIe user library   | libhbpcie.so    | libhbpcie/   |
| both | S13E01C16 PCIe High Level API | libhbpciehl.so  | libhbpciehl/ |