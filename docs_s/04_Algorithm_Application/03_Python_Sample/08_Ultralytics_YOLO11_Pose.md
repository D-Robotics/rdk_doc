---
sidebar_position: 8
---

# 姿态估计-Ultralytics YOLO11

本示例展示了如何基于 `hbm_runtime` 在 BPU 上运行 Ultralytics YOLO11 姿态估计模型，实现人体关键点检测与可视化。支持模型预处理、推理执行与后处理（含关键点解码、边界框绘制、关键点标注），本示例代码位于`/app/pydev_demo/04_pose_sample/01_ultralytics_yolo11_pose/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLO11 Pose 是一款高效的轻量级人体关键点检测模型，支持同时进行目标检测与姿态估计（多关键点预测）。它集成 Distribution Focal Loss（DFL）以增强边界框与关键点的定位精度，适用于实时应用场景中的多人体姿态识别任务。

- HBM 模型名称： yolo11n_pose_nashe_640x640_nv12.hbm

- 输入格式： NV12 格式图像（Y、UV 分离），尺寸为 640×640

- 输出：

    - 每个人的边界框坐标（xyxy）

    - 关键点位置（K×2，x/y 坐标）

    - 每个关键点的置信度得分

    - 支持 COCO 人体关键点格式（常见为 17 点）

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_pose_nashe_640x640_nv12.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载指定的 Ultralytics YOLO11 姿态估计模型，自动解析模型的输入输出张量名称、形状与量化参数。

- 输入预处理

    将输入的 BGR 图像 resize 到 640×640，并转为 NV12 格式（Y、UV 分离），用于模型推理。

- 推理执行

    调用 .run() 接口执行推理，同时支持设置运行优先级和 BPU 核心绑定，通过 set_scheduling_params() 实现。

- 结果后处理

    - 解码多尺度输出中的边界框（使用 DFL 分箱解码）；

    - 解码关键点位置与置信度（K×2 + K）；

    - 使用 NMS 去除冗余检测框；

    - 将关键点坐标和边界框映射回原始图像尺寸；

    - 提供阈值控制仅显示高置信度关键点；

    - 支持图像可视化，包括检测框与关键点绘制。


## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── ultralytics_yolo11_pose.py    # 主推理脚本
└── README.md                     # 使用说明
```

## 参数说明
| 参数名                | 说明                                              | 默认值                                |
| ------------------ | --------------------------------------------------- | ------------------------------------- |
| `--model-path`     | 模型文件路径（`.hbm` 格式）                           | `/opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm` |
| `--test-img`       | 测试图像路径                                         | `/app/res/assets/bus.jpg`                |
| `--label-file`     | 类别标签路径，每行一个类别名称                         | `/app/res/labels/coco_classes.names`     |
| `--img-save-path`  | 检测结果保存路径                                     | `result.jpg`                          |
| `--priority`       | 模型调度优先级（0\~255，数值越大优先级越高）           | `0`                                   |
| `--bpu-cores`      | 推理所使用的 BPU 核心编号列表（如：`--bpu-cores 0 1`） | `[0]`                                 |
| `--nms-thres`      | 非极大值抑制（NMS）中的 IoU 阈值                      | `0.7`                                 |
| `--score-thres`    | 目标置信度阈值（低于该值的目标将被过滤）               | `0.25`                                |
| `--kpt-conf-thres` | 关键点可视化置信度阈值（低于该值的关键点将不显示）      | `0.5`                                 |

## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python ultralytics_yolo11_pose.py
        ```
    - 指定参数运行
        ```bash
        python ultralytics_yolo11_pose.py \
        --model-path /opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm \
        --test-img /app/res/assets/bus.jpg \
        --label-file /app/res/labels/coco_classes.names \
        --img-save-path result.jpg \
        --priority 0 \
        --bpu-cores 0 \
        --score-thres 0.25 \
        --nms-thres 0.7 \
        --kpt-conf-thres 0.5
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
