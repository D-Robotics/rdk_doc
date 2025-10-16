---
sidebar_position: 3
---

# 图像分类-MobileNetV2

本示例展示如何使用基于 BPU 部署的 `MobileNetV2` 模型进行图像分类任务，使用 `C/C++` 进行推理，本示例代码位于`/app/cdev_demo/bpu/01_classification_sample/02_mobilenetv2/`目录下。

## 模型说明
- 简介：

    MobileNetV2 是一种轻量级卷积神经网络，由 Google 于 2018 年提出，设计用于在移动设备上实现高效的图像识别。其引入了 Inverted Residual 和 Linear Bottleneck 的结构，以降低计算量并提升性能。MobileNetV2 非常适合部署在边缘设备和资源受限场景中，用于图像分类、检测等任务。本示例使用的 MobileNetV2 模型为 224×224 输入、支持 NV12 格式的 BPU 量化模型。

- HBM 模型名称： mobilenetv2_224x224_nv12.hbm

- 输入格式： NV12，大小为 224x224（Y、UV 分离）

- 输出： 1000 类别的 softmax 概率分布（符合 ImageNet 1000 类标准）

## 功能说明
- 模型加载

    加载模型文件，提取模型名称、输入输出数量等信息。

- 输入预处理

    将输入图像从 BGR 格式 resize 到 224x224 后，转换为硬件要求的 NV12 格式（Y 与 UV 分离），形成字典结构输入，适配推理接口。

- 推理执行

    调用 .infer() 方法执行推理。

- 结果后处理

    获取模型输出 tensor，解析概率并显示 top-K（默认 top-5）预测结果，输出对应的类别名称和概率。

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构

```text
.
├── CMakeLists.txt              # CMake 构建脚本
├── README.md                   # 使用说明
├── inc
│   └── mobilenetv2.hpp         # 模型推理头文件
└── src
    ├── main.cc                 # 主程序入口
    └── mobilenetv2.cc          # 模型实现
```
## 编译工程
- 构建项目
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 模型下载
若在程序运行时未找到模型，可通过下列命令下载
```bash
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/MobileNet/mobilenetv2_224x224_nv12.hbm
```

## 参数说明
| 参数             | 说明                  | 默认值                                                        |
| -------------- | ------------------- | ---------------------------------------------------------- |
| `--model_path` | 模型文件路径（.hbm 格式）     | `/opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm` |
| `--test_img`   | 测试图片路径              | `/app/res/assets/zebra_cls.jpg`                            |
| `--label_file` | 类别标签映射文件路径（dict 格式） | `/app/res/labels/imagenet1000_clsidx_to_labels.txt`        |
| `--top_k`      | 输出 top-k 分类结果       | `5`                                                        |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./mobilenetv2
        ```
    - 指定参数运行
        ```bash
        ./mobilenetv2 \
        --model_path /opt/hobot/model/s100/basic/mobilenetv2_224x224_nv12.hbm \
        --test_img /app/res/assets/zebra_cls.jpg \
        --label_file /app/res/labels/imagenet1000_clsidx_to_labels.txt \
        --top_k 5
        ```
- 查看结果
    ```bash
    TOP 0: label=zebra, prob=0.992246
    TOP 1: label=tiger, Panthera tigris, prob=0.00404656
    TOP 2: label=hartebeest, prob=0.00133707
    TOP 3: label=tiger cat, prob=0.000722661
    TOP 4: label=impala, Aepyceros melampus, prob=0.000539704
    ```

## 注意事项
- 输出结果显示 top-K 概率最高的类别。

- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。
