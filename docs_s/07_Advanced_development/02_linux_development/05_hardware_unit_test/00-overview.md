# 概述

驱动功能单元测试是验证系统驱动程序与硬件组件稳定性、性能及功能完整性的关键阶段。本章节将详细介绍各类驱动功能单元的测试方法与测试标准，确保系统在实际应用场景中具备高度的可靠性和卓越的性能表现。

## <span id="Deploy_test_program"/>测试代码的存放路径

为了便于测试代码、脚本和工具的统一管理及使用，本章节所述的所有测试资源均已集成在 SDK 交付包的 BSP 源码中，存放路径如下：

- **SDK 交付包 BSP 路径：**
  `~/sdk/source/hobot-multimedia-samples/debian/app/multimedia_samples/chip_base_test`

开发者在使用官方发布的 SDK 交付包时，所提供的 EVB（评估板）系统镜像会默认将所有测试程序安装到开发板上的以下路径：

- **开发板路径：**
  `/app/multimedia_samples/chip_base_test`

## 测试环境准备

在进行驱动功能单元测试前，确保以下环境准备工作已完成：

1. 使用官方发布的 SDK 交付包，包含相应测试代码和工具。

2. 在开发板中确认测试工具和脚本的存放路径：

   ```bash
   ls /app/multimedia_samples/chip_base_test
   ```

3. 根据本章节描述的测试方法，逐步执行相应测试程序，并对结果进行记录与分析。

## 测试执行方式

开发者可以通过以下方式直接在开发板上运行测试程序：

1. 使用串口或 SSH 登录开发板。

2. 进入测试程序目录：

   ```bash
   cd /app/multimedia_samples/chip_base_test
   ```

3. 执行相应的测试脚本或程序，例如执行 CPU / BPU / DDR 压力测试：

   ```bash
   cd /app/chip_base_test/01_cpu_bpu_ddr/scripts
   ./stress_test.sh
   ```

## 测试目标与标准

驱动功能单元测试旨在覆盖以下几个核心目标：

- **功能验证：** 确保驱动程序正确实现设计功能。
- **稳定性测试：** 通过长时间运行及边界条件测试，验证驱动的稳定性。
- **性能测试：** 测量驱动与硬件的性能指标，确保满足系统需求。
- **兼容性测试：** 验证驱动在不同硬件配置或软件版本下的兼容性。

各类驱动的详细测试方法、步骤及判定标准将在后续章节中逐一进行阐述。
