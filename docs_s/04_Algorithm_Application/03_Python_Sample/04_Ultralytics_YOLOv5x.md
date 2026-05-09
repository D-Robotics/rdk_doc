---
sidebar_position: 4
---

# 目标检测-Ultralytics YOLOv5x

本示例展示如何在 BPU 上使用量化后的 Ultralytics YOLOv5x 模型执行图像目标检测。支持前处理、后处理、NMS 以及最终的目标框绘制和结果保存，本示例代码位于`/app/pydev_demo/02_detection_sample/01_ultralytics_yolov5x/`目录下。


## 模型说明
- 简介：

    Ultralytics YOLOv5x 是一类高性能的目标检测模型，其名称代表 "You Only Look Once"，可实现单次前向推理完成目标定位与分类。Ultralytics YOLOv5x 是其中最大的变体，具备更多的网络参数，检测精度高，适用于对准确率要求较高的场景。Ultralytics YOLOv5x 模型将输入图像映射为多个网格，每个网格预测多个 anchor 的类别和边界框。本模型已量化为适用于BPU芯片的 HBM 格式，输入尺寸为 672×672 的 NV12 图像。

- HBM 模型名称： yolov5x_672x672_nv12.hbm

- 输入格式： NV12，大小为 672x672（Y、UV 分离）

- 输出： N 个目标框，每个目标包含 (类别索引、概率、坐标框) 三元组


## 功能说明
- 模型加载

    通过 `hbm_runtime` 加载 Ultralytics YOLOv5x 量化模型，解析模型名称、输入输出名称、形状与量化参数等信息，为后续推理配置做好准备。

- 输入预处理

    将输入图像 resize 到 672x672 后转换为 NV12 格式（Y、UV 分离），并以嵌套字典形式组织输入，以适配推理接口。

- 推理执行

    利用 .run() 方法运行推理过程，支持设置推理优先级和 BPU 核心绑定（如 core0/core1 等）。

- 结果后处理

    - 对输出 tensor 进行 dequant 去量化；

    - 解码 YOLO 输出，得到预测框、置信度、类别索引；

    - 根据 score 阈值进行初筛；

    - 应用 NMS（非极大值抑制）去除冗余框；

    - 将预测框坐标映射回原图尺寸；

    - 叠加检测框并保存结果图像。


## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构

```text
.
├── ultralytics_yolov5x.py      # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明

| 参数           | 说明                                                     | 默认值                                      |
|----------------|----------------------------------------------------------|---------------------------------------------|
| `--model-path` | 模型文件路径（.hbm 格式）                                  | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--test-img`   | 测试图片路径                                              | `/app/res/assets/kite.jpg`                     |
| `--label-file` | 类别标签路径（每行一个类别）                                | `/app/res/labels/coco_classes.names`           |
| `--img-save-path` | 检测结果图像保存路径                                    | `result.jpg`                                |
| `--priority`  | 模型调度优先级（0~255）                                     | `0`                                         |
| `--bpu-cores` | 使用的 BPU 核心编号列表（如 `--bpu-cores 0 1`）              | `[0]`                                      |
| `--nms-thres`   | 非极大值抑制（NMS）阈值                                    | `0.45`                                    |
| `--score-thres` | 置信度阈值                                                | `0.25`                                    |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python ultralytics_yolov5x.py
        ```
    - 指定参数运行
        ```bash
        python ultralytics_yolov5x.py \
            --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --test-img /app/res/assets/kite.jpg \
            --label-file /app/res/labels/coco_classes.names \
            --img-save-path result.jpg \
            --priority 0 \
            --bpu-cores 0 \
            --nms-thres 0.45 \
            --score-thres 0.25
        ```
- 查看结果

    运行成功后，会将目标检测框绘制在原图上，并保存到 --img-save-path 指定路径
    ```bash
    [Saved] Result saved to: result.jpg
    ```

## 注意事项
- 若指定模型路径不存在，可尝试去`/opt/hobot/model/s100/basic/`查找。

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
