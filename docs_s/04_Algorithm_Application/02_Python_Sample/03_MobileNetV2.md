---
sidebar_position: 3
---

# 图像分类-MobileNetV2

本示例展示如何使用基于 BPU 部署的 `MobileNetV2` 模型进行图像分类任务，使用 `hbm_runtime` 进行推理，本示例代码位于`/app/pydev_demo/01_classification_sample/02_mobilenetv2/ `目录下。

## 模型说明
- 简介：

    MobileNetV2 是一种轻量级卷积神经网络，由 Google 于 2018 年提出，设计用于在移动设备上实现高效的图像识别。其引入了 Inverted Residual 和 Linear Bottleneck 的结构，以降低计算量并提升性能。MobileNetV2 非常适合部署在边缘设备和资源受限场景中，用于图像分类、检测等任务。本示例使用的 MobileNetV2 模型为 224×224 输入、支持 NV12 格式的 BPU 量化模型。

- HBM 模型名称： mobilenetv2_224x224_nv12.hbm

- 输入格式： NV12，大小为 224x224（Y、UV 分离）

- 输出： 1000 类别的 softmax 概率分布（符合 ImageNet 1000 类标准）

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/MobileNet/mobilenetv2_224x224_nv12.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载模型文件，提取模型名称、输入输出名称及其对应 shape 信息。

- 输入预处理

    将输入图像从 BGR 格式 resize 到 224x224 后，转换为硬件要求的 NV12 格式（Y 与 UV 分离），形成字典结构输入，适配推理接口。

- 推理执行

    调用 .run() 方法执行推理，支持设置 BPU 运行核心（如 core0/core1）及推理优先级（0~255）。

- 结果后处理

    获取模型输出 tensor，解析 softmax 概率并显示 top-K（默认 top-5）预测结果，输出对应的类别名称和概率。

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构

```text
.
├── mobilenetv2.py              # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明
| 参数           | 说明                                                     | 默认值                                      |
|----------------|----------------------------------------------------------|---------------------------------------------|
| `--model-path` | 模型文件路径（.hbm 格式）                                  | `/opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm`                 |
| `--test-img`   | 测试图片路径                                              | `/app/res/assets/zebra_cls.jpg`                |
| `--label-file` | 类别标签映射文件路径                                       | `/app/res/labels/imagenet1000_clsidx_to_labels.txt` |
| `--priority`   | 模型优先级（0~255，越大优先级越高）                         | `0`                                         |
| `--bpu-cores`  | 推理使用的 BPU 核心编号列表（如 `--bpu-cores 0 1`）         | `[0]`                                       |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python mobilenetv2.py
        ```
    - 指定参数运行
        ```bash
        python mobilenetv2.py \
        --model-path /opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm \
        --test-img /app/res/assets/zebra_cls.jpg \
        --label-file /app/res/labels/imagenet1000_clsidx_to_labels.txt
        ```
- 查看结果
    ```bash
    Top-5 Predictions:
    zebra: 0.8916
    tiger, Panthera tigris: 0.0028
    hartebeest: 0.0018
    jaguar, panther, Panthera onca, Felis onca: 0.0016
    tiger cat: 0.0016
    ```

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。
