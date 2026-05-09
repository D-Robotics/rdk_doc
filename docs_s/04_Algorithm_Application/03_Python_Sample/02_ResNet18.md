---
sidebar_position: 2
---

# 图像分类-ResNet18

本示例演示如何使用`hbm_runtime`的python接口部署`ResNet18`模型进行图像分类推理。适用于搭载BPU芯片的 RDK S100设备，本示例代码位于`/app/pydev_demo/01_classification_sample/01_resnet18/`目录下。

## 模型说明
- 简介：

  ResNet（Residual Network）是由微软研究院提出的一种深层卷积神经网络架构，其核心思想是引入“残差连接（Residual Connection）”，通过跨层的快捷连接缓解了深层网络中的梯度消失问题，从而能有效训练数十甚至上百层的深度网络。本示例采用的 ResNet18 是其中的一种较轻量级变种，具有 18 层结构，广泛应用于图像分类、特征提取等任务。
- HBM模型名称：resnet18_224x224_nv12.hbm

- 输入格式：NV12，大小为 224x224

- 输出：1000 类别的 softmax 概率分布

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ResNet/resnet18_224x224_nv12.hbm
    ```

## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载指定模型，解析输入输出名称和形状，用于后续推理。

- 输入预处理

    将 BGR 图像 resize 到 224x224, 转换为 NV12 格式（Y、UV 分离）。

- 推理执行

    通过 .run() 方法完成模型前向推理，支持可选调度参数（优先级、核心绑定）。

- 结果后处理

    读取输出 tensor，解析 top-K 分类结果（Top-5），显示类别标签和概率值。

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── resnet18.py                 # 主程序，使用 `hbm_runtime` 调用 ResNet18 进行分类
└── README.md                   # 使用说明
```

## 参数说明
| 参数           | 说明                                                     | 默认值                                      |
|----------------|----------------------------------------------------------|---------------------------------------------|
| `--model-path` | 模型文件路径（.hbm 格式）                                | `/opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm`                 |
| `--test-img`   | 测试图片路径                                            | `/app/res/assets/zebra_cls.jpg`                |
| `--label-file` | 类别标签映射文件路径                                     | `/app/res/labels/imagenet1000_clsidx_to_labels.txt` |
| `--priority`   | 模型优先级（0~255，越大优先级越高）                      | `0`                                         |
| `--bpu-cores`  | 推理使用的 BPU 核心编号列表（如 `--bpu-cores 0 1`）     | `[0]`                                       |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python resnet18.py
        ```
    - 指定参数运行
        ```bash
        python resnet18.py \
        --model-path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
        --test-img /app/res/assets/zebra_cls.jpg \
        --label-file /app/res/labels/imagenet1000_clsidx_to_labels.txt
        ```

- 查看结果
    ```bash
    Top-5 Predictions:
    zebra: 0.9979
    impala, Aepyceros melampus: 0.0005
    cheetah, chetah, Acinonyx jubatus: 0.0005
    gazelle: 0.0004
    prairie chicken, prairie grouse, prairie fowl: 0.0002
    ```

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。
