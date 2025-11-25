---
sidebar_position: 7
---

# 实例分割-Ultralytics YOLO11

本示例展示了如何基在 BPU 上运行 YOLOv11 语义分割模型，支持图像预处理、推理、后处理（解析输出并叠加彩色分割掩码）等功能，本示例代码位于`/app/cdev_demo/bpu/03_instance_segmentation_sample/02_ultralytics_yolo11_seg/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLO11 是一款轻量级目标检测与实例分割模型，基于 YOLO 系列设计并融合了 anchor-free 与 anchor-based 思想结构与回归分箱（distributional regression）策略。本模型为其实例分割变体，支持同时输出边界框、类别概率和高质量的像素级掩膜，适用于实时场景中的多对象检测与分割任务。

- HBM 模型名称： yolo11n_seg_nashe_640x640_nv12.hbm

- 输入格式： NV12 格式图像（Y/UV 分离），尺寸为 640x640

- 输出：

    - 目标检测结果（bounding box + 类别 + 分数）

    - 实例分割掩膜（每个目标对应一张 mask）

## 功能说明
- 模型加载

    加载量化后的 Ultralytics YOLO11 实例分割模型，并解析运行时元数据。

- 输入预处理

    将输入 BGR 图像缩放至 640×640，并转换为 NV12 格式（Y、UV 平面分离），以适配模型输入要求。

- 推理执行

    通过 .infer() 方法触发前向推理，推理输出包含多个尺度的类别分数、边界框回归、掩膜系数以及全局掩膜原型张量。

- 结果后处理

    - 使用阈值筛选检测候选，并解码边界框与掩膜系数；

    - 合并各尺度输出，应用 NMS 筛选最终目标；

    - 利用掩膜原型与系数恢复出每个目标的掩膜；

    - 掩膜及边界框均被缩放回原始图像尺寸；

    - 支持可选形态学操作（开操作）以优化掩膜边缘；

    - 最终输出为边框、类别、分数与像素级实例掩膜。

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt                 # CMake 构建脚本
|-- README.md                      # 使用说明（当前文件）
|-- inc
|   `-- ultralytics_yolo11_seg.hpp # YOLO11_Seg 推理封装头文件（加载/预处理/推理/后处理接口）
`-- src
    |-- main.cc                    # 程序入口：解析参数、执行完整 pipeline、保存可视化结果
    `-- ultralytics_yolo11_seg.cc  # YOLO11_Seg 推理实现：解码、NMS、掩码生成与缩放等
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
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_seg_nashe_640x640_nv12.hbm
```

## 参数说明
| 参数名             | 说明                       | 默认值                                                              |
| --------------- | ------------------------ | ---------------------------------------------------------------- |
| `--model_path`  | 模型文件路径（`.hbm`）           | `/opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm` |
| `--test_img`    | 输入测试图片路径                 | `/app/res/assets/office_desk.jpg`                                |
| `--label_file`  | 类别标签文件路径                 | `/app/res/labels/coco_classes.names`                             |
| `--score_thres` | 置信度过滤阈值（低于该值的框会被丢弃）      | `0.25`                                                           |
| `--nms_thres`   | IoU 阈值（类别内 NMS 用于去除重复检测） | `0.7`                                                            |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./ultralytics_yolo11_seg
        ```
    - 指定参数运行
        ```bash
        ./ultralytics_yolo11_seg \
            --model_path /opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm \
            --test_img /app/res/assets/office_desk.jpg \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.7
        ```
- 查看结果

    运行成功后，会将结果绘制在原图上，并保存到build/result.jpg
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## 注意事项
- 输出结果存储为result.jpg，用户可自行查看。

- 如需了解更多部署方式或模型支持情况，请参考官方文档或联系平台技术支持。


## License
    ```license
    Copyright (C) 2025，XiangshunZhao D-Robotics.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    ```
