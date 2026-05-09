---
sidebar_position: 10
---


# 车道线检测-LaneNet

本示例基于BPU运行 LaneNet 模型，实现车道线的实例分割与二值分割，并将结果图像保存到本地，本示例代码位于`/app/cdev_demo/bpu/06_lane_detection_sample/01_lanenet/`目录下。

## 模型说明
- 简介：

    LaneNet 是一种用于实时车道线检测的语义分割模型。LaneNet 在图像预处理上采用归一化与标准化方式，适合自动驾驶与 ADAS 系统中的道路场景分析。本示例使用的是量化版本模型 lanenet256x512.hbm，支持 BPU 推理加速。

- HBM模型名称：lanenet256x512.hbm

- 输入格式：RGB，大小为 256x512，归一化到 [0,1] 后进行标准化

- 输出：

    - instance_seg_logits：用于区分不同车道线的 3 通道图

    - binary_seg_pred：二值分割结果，表示车道区域的位置

## 功能说明
- 模型加载

    加载 LaneNet 模型，自动解析模型的部分元数据。

- 输入预处理

    将输入图像转换为 RGB 格式，调整到 256x512 尺寸，并使用 ImageNet 均值与标准差进行归一化和标准化处理，最终转为 NCHW 格式并添加 batch 维度。

- 推理执行

    使用 .infer() 方法进行推理，输出包括 instance 特征图和二值掩膜图。

- 结果后处理

    对输出 tensor 进行 reshape 与归一化：

    - instance_seg_logits：输出三通道图像用于可视化每个车道实例

    - binary_seg_pred：输出单通道二值图，用于提取车道区域

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt            # CMake 构建脚本：目标/依赖/包含与链接配置
|-- README.md                 # 使用说明（当前文件）
|-- inc
|   `-- lanenet.hpp           # LaneNet 推理封装头文件：加载/预处理/推理/后处理接口
`-- src
    |-- lanenet.cc            # LaneNet 推理实现：前后处理与推理调用
    `-- main.cc               # 程序入口：解析参数→完整流程→保存 instance/binary 结果
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
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/Lanenet/lanenet256x512.hbm
```

## 参数说明
| 参数名         | 说明                       | 默认值                                              |
| -------------- |  ------------------------ | ------------------------------------------------ |
| `--model_path` | 模型文件路径（`.hbm` 格式） | `/opt/hobot/model/s100/basic/lanenet256x512.hbm` |
| `--test_img`   | 输入测试图像路径            | `/app/res/assets/input.jpg`                      |


## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./lanenet
        ```
    - 指定参数运行
        ```bash
        ./lanenet \
            --model_path /opt/hobot/model/s100/basic/lanenet256x512.hbm \
            --test_img   /app/res/assets/input.jpg
        ```
- 查看结果

    运行成功后，会将结果绘制出来，保存到 instance_pred.png 和 binary_pred.png
    ```bash
    Results saved to: instance_pred.png and binary_pred.png
    ```

## 注意事项
- 输出结果存储为instance_pred.png 和 binary_pred.png，用户可自行查看。

- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。
