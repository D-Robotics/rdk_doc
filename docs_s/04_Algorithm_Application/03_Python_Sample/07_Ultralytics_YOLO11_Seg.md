---
sidebar_position: 7
---

# 实例分割-Ultralytics YOLO11

本示例展示了如何基于 `hbm_runtime` 在 BPU 上运行 YOLOv11 语义分割模型，支持图像预处理、推理、后处理（解析输出并叠加彩色分割掩码）等功能，本示例代码位于`/app/pydev_demo/03_instance_segmentation_sample/02_ultralytics_yolo11_seg/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLO11 是一款轻量级目标检测与实例分割模型，基于 YOLO 系列设计并融合了 anchor-free 与 anchor-based 思想结构与回归分箱（distributional regression）策略。本模型为其实例分割变体，支持同时输出边界框、类别概率和高质量的像素级掩膜，适用于实时场景中的多对象检测与分割任务。

- HBM 模型名称： yolo11n_seg_nashe_640x640_nv12.hbm

- 输入格式： NV12 格式图像（Y/UV 分离），尺寸为 640x640

- 输出：

    - 目标检测结果（bounding box + 类别 + 分数）

    - 实例分割掩膜（每个目标对应一张 mask）


- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_seg_nashe_640x640_nv12.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载量化后的 Ultralytics YOLO11 实例分割模型，并解析输入输出张量的名称、形状、量化参数等运行时元数据。

- 输入预处理

    将输入 BGR 图像缩放至 640×640，并转换为 NV12 格式（Y、UV 平面分离），以适配模型输入要求。

- 推理执行

    通过 .run() 方法触发前向推理，并支持通过接口设定调度参数（BPU 核绑定与优先级）。推理输出包含多个尺度的类别分数、边界框回归、掩膜系数以及全局掩膜原型张量。

- 结果后处理

    - 使用阈值筛选检测候选，并解码边界框与掩膜系数；

    - 合并各尺度输出，应用 NMS 筛选最终目标；

    - 利用掩膜原型与系数恢复出每个目标的掩膜；

    - 掩膜及边界框均被缩放回原始图像尺寸；

    - 支持可选形态学操作（开操作）以优化掩膜边缘；

    - 最终输出为边框、类别、分数与像素级实例掩膜。

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── ultralytics_yolo11_seg.py   # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明
| 参数                | 说明                    | 默认值 |
|--------------------|-----------------------------|--------------------------------------|
| `--model-path`     | 模型文件路径（.hbm 格式）     | `/opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm` |
| `--test-img`       | 测试图片路径                 | `/app/res/assets/office_desk.jpg`        |
| `--label-file`     | 分类标签文件                 | `/app/res/labels/coco_classes.names`     |
| `--img-save-path`  | 输出结果图片保存路径          | `result.jpg`                          |
| `--priority`       | 模型优先级 (0~255)           | `0`                                   |
| `--bpu-cores`      | BPU 核心编号                 | `[0]`                                 |
| `--nms-thres`      | NMS IoU 队值间值             | `0.7`                                 |
| `--score-thres`    | 精度阈值                     | `0.25`                                |
| `--is-open`        | 是否对分割结果进行形态形象处理 | `True`                                |
| `--is-point`       | 是否在边缘处绘制边线上的点     | `True`                                |

## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python ultralytics_yolo11_seg.py
        ```
    - 指定参数运行
        ```bash
        python ultralytics_yolo11_seg.py \
        --model-path /opt/hobot/model/s100/basic/yolo11n_seg_nashe_640x640_nv12.hbm \
        --test-img /app/res/assets/office_desk.jpg \
        --label-file /app/res/labels/coco_classes.names \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --nms-thres 0.7 \
        --score-thres 0.25 \
        --is-open True \
        --is-point True
        ```
- 查看结果

    运行成功后，会将结果绘制在原图上，并保存到 --img-save-path 指定路径
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。

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
