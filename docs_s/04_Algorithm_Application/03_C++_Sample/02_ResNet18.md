---
sidebar_position: 2
---

# 图像分类-ResNet18

本示例演示如何使用`C/C++`部署`ResNet18`模型进行图像分类推理，适用于搭载BPU芯片的 RDK S100 设备，本示例代码位于`/app/cdev_demo/bpu/01_classification_sample/01_resnet18/`目录下。

## 模型说明
- 简介：

  ResNet（Residual Network）是由微软研究院提出的一种深层卷积神经网络架构，其核心思想是引入“残差连接（Residual Connection）”，通过跨层的快捷连接缓解了深层网络中的梯度消失问题，从而能有效训练数十甚至上百层的深度网络。本示例采用的 ResNet18 是其中的一种较轻量级变种，具有 18 层结构，广泛应用于图像分类、特征提取等任务。
- HBM模型名称：resnet18_224x224_nv12.hbm

- 输入格式：NV12，大小为 224x224

- 输出：1000 类别的 softmax 概率分布

## 功能说明
- 模型加载

    加载指定模型，解析输入输出名称和形状，用于后续推理。

- 输入预处理

    将 BGR 图像 resize 到 224x224, 转换为 NV12 格式（Y、UV 分离）。

- 推理执行

    通过 .infer() 方法完成模型前向推理。

- 结果后处理

    读取输出 tensor，解析 top-K 分类结果（Top-5），显示类别标签和概率值。

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构

```text
.
|-- CMakeLists.txt         # CMake 构建脚本
|-- README.md              # 工程说明文档
|-- inc/                   # 头文件目录
|   `-- resnet18.hpp       # ResNet18 模型推理类定义
`-- src/                   # 源代码目录
    |-- main.cc            # 程序入口，调用 ResNet18 推理流程
    `-- resnet18.cc        # ResNet18 推理类实现
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 模型下载
若在程序运行时未找到模型，可通过下列命令下载
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ResNet/resnet18_224x224_nv12.hbm
```

## 参数说明
| 参数             | 说明                                    | 默认值                                                 |
| -------------- | ------------------------------------- | --------------------------------------------------- |
| `--model_path` | 模型文件路径（`.hbm` 格式）                     | `/opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm`         |
| `--test_img`   | 测试图片路径                                | `/app/res/assets/zebra_cls.jpg`                     |
| `--label_file` | ImageNet 类别映射（dict，每行 `index\tlabel`） | `/app/res/labels/imagenet1000_clsidx_to_labels.txt` |
| `--top_k`      | 输出 Top-K 分类结果数                        | `5`                                                 |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./resnet_18
        ```
    - 指定参数运行
        ```bash
        ./resnet_18 \
        --model_path /opt/hobot/model/s100/basic/resnet18_224x224_nv12.hbm \
        --test_img   /app/res/assets/zebra_cls.jpg \
        --label_file /app/res/labels/imagenet1000_clsidx_to_labels.txt \
        --top_k 5
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
- 输出结果显示 top-K 概率最高的类别。

- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。
