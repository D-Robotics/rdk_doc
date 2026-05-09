---
sidebar_position: 5
---

# 目标检测-Ultralytics YOLO11

本示例基于 Ultralytics YOLO11 模型，通过 `hbm_runtime` 接口完成图像的目标检测。支持图像预处理、推理、后处理（包含解码、置信度过滤、NMS）以及结果图像保存，本示例代码位于`/app/pydev_demo/02_detection_sample/02_ultralytics_yolo11/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLO11 是一款轻量级的 anchor-based 目标检测模型，融合了 anchor-free 与 anchor-based 思想，具备快速推理和精确定位的能力。该模型在回归阶段采用离散分桶（regression bin）方式，结合 softmax 分类和解码机制来提升定位精度。Ultralytics YOLO11 适用于实时场景下的小模型部署，如安防监控、工业检测等任务。

- HBM 模型名称： yolo11n_detect_nashe_640x640_nv12.hbm

- 输入格式： NV12 格式，大小为 640x640（Y、UV 分离）

- 输出： 多尺度特征图，每层包含类别得分张量和边界框离散回归张量，最终输出为目标框位置、类别 ID 和置信度分数

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_detect_nashe_640x640_nv12.hbm
    ```

## 功能说明
- 模型加载

    使用 `hbm_runtime` 接口加载 Ultralytics YOLO11 量化模型，提取输入输出名称、形状、量化信息等模型元数据，供后续推理流程使用。

- 输入预处理

    将原始 BGR 图像 resize 为 640×640，转换为 NV12 格式（Y、UV 分离），构造输入张量嵌套字典，适配推理接口要求。

- 推理执行

    通过 .run() 方法运行前向推理，可指定调度参数（推理优先级、BPU 核心绑定）。输出包含多个尺度分支的分类张量与回归张量。

- 结果后处理

    - 将量化输出反量化为 float32；

    - 对每个尺度分支进行分类分数筛选，保留超过设定置信度阈值的候选框；

    - 使用多分桶回归算法进行边框解码；

    - 合并所有尺度的候选框并应用 NMS（非极大值抑制）去除冗余框；

    - 将检测框从模型输入坐标系映射回原图尺寸；

    - 可选地绘制检测结果并保存图像文件。

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── ultralytics_yolo11.py       # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明
| 参数名           | 说明                                                         | 默认值                                         |
|------------------|--------------------------------------------------------------|------------------------------------------------|
| `--model-path`    | 模型文件路径（.hbm 格式）                                    | `/opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm`        |
| `--test-img`      | 输入测试图片路径                                             | `/app/res/assets/kite.jpg`                        |
| `--label-file`    | 类别标签文件路径（每行一个类别名称）                         | `/app/res/labels/coco_classes.names`              |
| `--img-save-path` | 检测结果图像保存路径                                         | `result.jpg`                                   |
| `--priority`      | 模型调度优先级（0~255，数值越大优先级越高）                  | `0`                                            |
| `--bpu-cores`     | 使用的 BPU 核心编号列表（如 `--bpu-cores 0 1`）             | `[0]`                                          |
| `--nms-thres`     | 非极大值抑制（NMS）的 IoU 阈值                                | `0.45`                                         |
| `--score-thres`   | 置信度过滤阈值（低于该值的目标将被过滤）                      | `0.25`                                         |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python ultralytics_yolo11.py
        ```
    - 指定参数运行
        ```bash
        python ultralytics_yolo11.py \
            --model-path /opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm \
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
