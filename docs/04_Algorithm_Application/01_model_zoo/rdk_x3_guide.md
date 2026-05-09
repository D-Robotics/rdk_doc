---
sidebar_position: 2
---

# 4.1.2 RDK X3 Model Zoo 使用说明

## 分支与系统要求

RDK X3 使用 `rdk_x3` 分支，推荐系统版本 RDK OS >= 3.0.0 。

```bash
git clone https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo
git checkout rdk_x3
```

## 仓库目录结构

`rdk_x3` 分支按 demo 目录组织，主要结构如下：

```bash
rdk_model_zoo/
|-- demos/
|   |-- classification/          # 图像分类
|   |   |-- GoogLeNet/
|   |   |-- MobileNetV1/
|   |   |-- MobileNetV2/
|   |   |-- MobileNetV4/
|   |   |-- MobileOne/
|   |   |-- RepGhost/
|   |   |-- RepVGG/
|   |   |-- RepViT/
|   |   `-- ResNet/
|   |-- detect/                  # 目标检测
|   |   |-- FCOS/
|   |   |-- PaddleOCR/
|   |   |-- YOLOv10/
|   |   |-- YOLOv5/
|   |   `-- YOLOv8/
|   `-- Instance_Segmentation/   # 实例分割
|       `-- YOLOv8-Seg/
`-- resource/                    # 文档资源
```

## Python 推理接口

RDK X3 分支的 Python 推理接口有两种：

- **`bpu_infer_lib_x3`**：需手动安装的 Python 推理库
- **`hobot_dnn.pyeasy_dnn`**：板端系统自带的 Python 推理接口

具体使用哪种接口，以目标 demo 目录 README 和源码入口为准。

:::caution

`bpu_infer_lib_x3` 和 `hobot_dnn.pyeasy_dnn` **对 featuremap 输入模型支持不佳**。如需推理 featuremap 输入模型，建议使用 C++ 接口或升级到 RDK X5 平台并使用 `hbm_runtime` 推理接口。

:::

### 安装 bpu_infer_lib_x3

RDK X3 使用 `bpu_infer_lib_x3` 的 demo，按以下命令安装：

```bash
wget -nc https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_x3/bpu_infer_lib_x3-1.0.3-py3-none-any.whl
pip install bpu_infer_lib_x3-1.0.3-py3-none-any.whl
```

使用 `hobot_dnn.pyeasy_dnn` 的 demo 直接使用板端系统自带接口，无需额外安装。

## 快速上手

### 运行 demo

```bash
cd demos/<task>/<demo>
less README.md
# 按 README 准备模型、安装依赖并运行
```

### 任务目录说明

| 任务 | 目录 | 包含模型 |
| :--- | :--- | :--- |
| 图像分类 | `demos/classification` | GoogLeNet、MobileNetV1、MobileNetV2、MobileNetV4、MobileOne、RepGhost、RepVGG、RepViT、ResNet |
| 目标检测 | `demos/detect` | FCOS、PaddleOCR、YOLOv10、YOLOv5、YOLOv8 |
| 实例分割 | `demos/Instance_Segmentation` | YOLOv8-Seg |

## 使用 Jupyter 体验

RDK X3 分支部分 demo 提供 Jupyter Notebook，支持交互式运行：

```bash
# 安装 Jupyter Lab
pip install jupyterlab

# 启动 Jupyter Lab（ip 地址为板卡实际 IP）
jupyter lab --allow-root --ip 192.168.1.10
```

启动后在浏览器中打开链接，进入对应 demo 的 notebook，点击运行按钮即可体验。

:::note

所有程序中的相对路径均以模型所在的目录开始计算。

:::
