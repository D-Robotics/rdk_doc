---
sidebar_position: 6
---

# 语义分割-UNetMobileNet

本示例展示了如何基于 `hbm_runtime` 在 BPU 上运行 UNet-MobileNet 语义分割模型，支持图像预处理、推理、后处理（解析输出并叠加彩色分割掩码）等功能，本示例代码位于`/app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet/`目录下。

## 模型说明
- 简介：

    UNet 是一种经典的语义分割网络结构，采用编码器-解码器架构，在医学图像分析等领域表现出色。本示例使用 MobileNet 作为编码器主干网络，以降低模型复杂度并加快推理速度，适用于边缘设备上的实时分割任务。模型输出为每个像素的类别标签，用于实现城市街景分割等应用。

- HBM 模型名称： unet_mobilenet_1024x2048_nv12.hbm

- 输入格式： NV12，大小为 1024x2048（Y、UV 平面分离）

- 输出： 尺寸与输入一致的分割图，每个像素点对应一个 0~18 的类别编号（共 19 类）

## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载量化后的语义分割模型，提取模型名、输入输出张量名称、形状与量化信息等元数据。

- 输入预处理

    原始图像以 BGR 格式加载后被缩放至 1024×2048，转换为 NV12 格式（Y/UV 分离），并封装为推理接口要求的输入字典结构。

- 推理执行

    使用 .run() 方法执行模型前向推理，支持设定调度参数如 BPU 核心分配与优先级。输出为类别 logits 张量。

- 结果后处理

    - 对输出张量取 argmax 得到每个像素所属类别；

    - 将预测图 resize 到输入图大小；

    - 恢复为原图尺寸并映射到指定颜色调色板；

    - 使用设定的 alpha 融合系数与原图进行混合，生成分割可视化图；

    - 最终图像包含分割结果的直观覆盖图，可保存或展示。

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── unet_mobilenet.py           # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明

| 参数名               | 说明                                      | 默认值                                 |
| ----------------- | --------------------------------------- | ----------------------------------- |
| `--model-path`    | 模型文件路径（.hbm 格式）                              | `/opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm` |
| `--test-img`      | 输入测试图像路径                                      | `/app/res/assets/segmentation.png`     |
| `--img-save-path` | 推理后结果图像保存路径                                  | `result.jpg`                        |
| `--priority`      | 模型优先级（0\~255，越大优先级越高）                    | `0`                                 |
| `--bpu-cores`     | 指定运行模型的 BPU 核心编号列表（如 `--bpu-cores 0 1`） | `[0]`                               |
| `--alpha-f`       | 可视化融合系数，`0.0=仅显示掩码`，`1.0=仅原图`           | `0.75`                              |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python unet_mobilenet.py
        ```
    - 指定参数运行
        ```bash
        python unet_mobilenet.py \
        --model-path /opt/hobot/model/s100/basic/unet_mobilenet_1024x2048_nv12.hbm \
        --test-img /app/res/assets/segmentation.png \
        --img-save-path result.jpg \
        --alpha-f 0.75 \
        --priority 0 \
        --bpu-cores 0
        ```
- 查看结果

    运行成功后，会将结果绘制在原图上，并保存到 --img-save-path 指定路径
    ```bash
    [Saved] Result saved to: result.jpg
    ```
