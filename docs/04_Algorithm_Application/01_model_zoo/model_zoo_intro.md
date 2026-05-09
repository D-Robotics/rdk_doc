---
sidebar_position: 1
---

# 4.1.1 Model Zoo 概述

## 产品介绍

RDK Model Zoo 是 D-Robotics（地瓜机器人）面向 RDK 系列开发板提供的 BPU（智能计算架构 Brain Processing Unit）模型示例与工具集合，面向模型部署和智能应用开发，用于帮助开发者快速上手 BPU、跑通模型推理流程。

仓库中收录了覆盖图像分类、目标检测、实例分割、姿态估计、OCR、多模态等多个智能应用领域的 BPU 可运行模型，并提供从 **原始模型（PyTorch/ONNX）→ 定点量化转换 → 推理运行 → 结果解析 → 示例验证** 的完整参考实现，帮助用户以最小成本理解并使用 BPU 能力。

:::tip

Model Zoo GitHub 仓库地址：https://github.com/D-Robotics/rdk_model_zoo

:::

:::info

RDK Model Zoo 是一个社区开源共建项目。我们非常欢迎开发者贡献新的模型示例、优化现有代码或完善文档。如果您有任何改进建议，欢迎通过 Pull Request (PR) 的方式参与共建！

:::

## 分支与硬件平台对应关系

Model Zoo 按硬件平台提供对应分支，不同分支的目录结构、推理接口和系统要求有所不同：

| 目标硬件 | 对应分支 | Python 推理接口 | 使用入口 |
| :--- | :--- | :--- | :--- |
| RDK X5 | [`rdk_x5`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x5) | `hbm_runtime` | `samples/vision/<sample>/README.md` |
| RDK X5（历史 demo） | [`rdk_x5_legacy`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x5_legacy) | `bpu_infer_lib_x5` / `hobot_dnn.pyeasy_dnn` | 目标 demo 目录 README |
| RDK X3 | [`rdk_x3`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_x3) | `bpu_infer_lib_x3` / `hobot_dnn.pyeasy_dnn` | `demos/<task>/<demo>/README.md` |
| RDK S 系列 | [`rdk_s`](https://github.com/D-Robotics/rdk_model_zoo/tree/rdk_s) | `hbm_runtime` | `samples/<domain>/<sample>/README.md` |

## 分支说明

### rdk_x5

RDK X5 的主交付分支，需系统版本 RDK OS >= 3.5.0。

- Python sample 统一使用 `hbm_runtime` 接口，C++ sample 使用 `hb_dnn` 接口
- 目录按 `samples/vision/<model>` 规范化组织
- 包含模型下载、转换配置、Python/C++ runtime、评测工具和测试数据

### rdk_x5_legacy

原 RDK X5 历史 demo 归档分支，仅用于历史兼容和旧 demo 查询。

- 目录按 `demos/<task>/<demo>` 组织
- 不同 demo 使用不同的推理接口（`bpu_infer_lib_x5` 或板端自带的 `hobot_dnn.pyeasy_dnn`），以目标目录 README 为准

:::caution

`bpu_infer_lib_x5` 和 `hobot_dnn.pyeasy_dnn` 对 featuremap 输入模型支持不佳。如需使用 featuremap 输入模型，请使用 `rdk_x5` 分支的 `hbm_runtime`。

:::

### rdk_x3

RDK X3 设备专用分支。

- 目录按 `demos/<task>/<demo>` 组织
- 包含图像分类、目标检测、实例分割等示例
- 推理接口按目标目录 README 选择，包含 `bpu_infer_lib_x3` 与板端自带的 `hobot_dnn.pyeasy_dnn`

:::caution

`bpu_infer_lib_x3` 和 `hobot_dnn.pyeasy_dnn` 对 featuremap 输入模型支持不佳。

:::

部分 demo 支持 Jupyter Notebook 交互式体验。

### rdk_s

RDK S100 / S600 系列板卡专用分支，需系统版本 RDK OS >= 4.0.5。

- 按领域组织 sample，包含视觉和语音示例
- 推理接口为 `hbm_runtime`（与 RDK X5 同名，底层依赖不同：S 系列基于 `libhbucp`，X5 基于 `libdnn`）
- RDK S 系列的历史 demo 保留在 [RDK Model Zoo S](https://github.com/d-Robotics/rdk_model_zoo_s) 仓库

## 各平台详细使用说明

- RDK X3 使用说明请参考 [4.1.2 RDK X3 Model Zoo 使用说明](./rdk_x3_guide.md)
- RDK X5 使用说明请参考 [4.1.3 RDK X5 Model Zoo 使用说明](./rdk_x5_guide.md)
- RDK S 系列使用说明请参考 [4.1.4 RDK S Model Zoo 使用说明](./rdk_s_guide.md)
- 推理接口参考请查阅 [4.1.5 推理接口参考](./infer_api_ref.md)
