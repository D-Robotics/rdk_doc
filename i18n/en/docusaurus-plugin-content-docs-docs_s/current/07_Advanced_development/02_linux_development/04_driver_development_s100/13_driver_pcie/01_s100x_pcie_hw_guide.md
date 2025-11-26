---
sidebar_position: 1
---
# S100X PCIe Hardware Specifications and Supported Topologies

## PCIe Overview

PCI Express (PCIe) is a multi-lane I/O interconnect that provides low pin count, high reliability, and high-speed data transfer.  
It is the third-generation I/O interconnect technology following the ISA and PCI buses, designed to serve as a universal  
serial I/O interconnect across multiple market segments, including desktops, mobile devices, servers, storage, and embedded communications.

## S100X PCIe Hardware Specifications

### S100E

- Two PCIe Gen 4.0 controllers with the following controller-to-lane configurations:
  - One controller: x2 or x4
  - Two controllers: x1
- Each controller supports configuration as either Root Complex (RC) or Endpoint (EP) mode
- SR-IOV support in EP mode: 1 Physical Function (PF) + 4 Virtual Functions (VFs)
- Supports 8 DMA channel pairs
- Supports MSI-X
- Supports SMMU
- Supports 48 Outbound regions
- Supports PTM time synchronization

## The S100X supports the following PCIe bus topologies:

![S100X_PCIE_Topology](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/07_Advanced_development/02_linux_development/driver_development_s100/pcie/topology.png)

1. **Topology 1**: Two S100X chips directly connected, with one S100X acting as RC and the other as EP

2. **Topology 2**: Three S100X chips directly connected, with one S100X acting as RC and simultaneously connecting to two S100X EPs

3. **Topology 3**: One S100X directly connected to a third-party standard PCIe EP device, such as an NVMe SSD

4. **Topology 4**: One S100X configured as a PCIe EP device connected to a third-party RC device—typically used when the S100X functions as a PCIe accelerator card

5. **Topology 5**: Multiple S100X chips and third-party standard PCIe EP devices connected via a PCIe Switch, with one S100X acting as RC and all other devices operating as EPs