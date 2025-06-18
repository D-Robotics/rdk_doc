---
sidebar_position: 2
---
# S100X PCIe软件架构与模块划分

## 软件框架

PCIe软件框架分为RC和EP两个部分：

![S100X_PCIE_sw_arch](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/pcie/sw_arch.png)

## 驱动模块说明

驱动相关的源码在hobot-drivers/pcie目录下，各个模块的细节信息如下：

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

## PCIe驱动加载/卸载

### RC端加载

```shell
modprobe hobot-pcie
modprobe hobot-pcie-rc
modprobe hobot-pcie-ep-dev
modprobe hobot-pcie-dev-manager
```

### EP端加载

```shell
modprobe hobot-pcie
modprobe hobot-pcie-ep-fun
```

### RC端卸载

```shell
rmmod hobot_pcie_dev_manager
rmmod hobot_pcie_ep_dev
rmmod hobot_pcie_rc
rmmod hobot_pcie_common
rmmod hobot_pcie
```

### EP端卸载

```shell
rmmod hobot_pcie_ep_fun
rmmod hobot_pcie_ep
rmmod hobot_pcie_common
rmmod hobot_pcie
```

需要注意：

1. 卸载驱动前一定要保证PCIe的应用程序都已关闭，按照先卸载RC驱动，再卸载EP驱动的顺序进行
2. 系统休眠前需要遵循以下流程：
   - 停掉两端的PCIe应用程序
   - 卸载RC驱动
   - 卸载EP驱动
3. 系统唤醒后需要遵循以下流程：
   - 加载EP驱动
   - 加载RC驱动
   - 启动PCIe应用程序
4. 链路信号不稳定或对端重启导致PCIe link down后，需要重启RC和EP以保证PCIe功能的可用性以及系统的稳定性
5. 在不使用pcie功能的时候，不要加载pcie相关驱动
6. 系统重启前需要遵循以下流程：
   - 停掉两端的PCIe应用程序
   - 卸载RC驱动
   - 卸载EP驱动
   - RC系统重启
   - EP系统重启

## 用户态模块说明

用户态相关的源码在hbre目录下，各个模块的细节信息如下：

| side | component                     | output          | Source file  |
|------|-------------------------------|-----------------|--------------|
| both | S13E01C15 PCIe user library   | libhbpcie.so    | libhbpcie/   |
| both | S13E01C16 PCIe High Level API | libhbpciehl.so  | libhbpciehl/ |
