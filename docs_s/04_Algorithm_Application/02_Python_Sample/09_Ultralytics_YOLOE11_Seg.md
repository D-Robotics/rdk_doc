---
sidebar_position: 9
---

# 实例分割-Ultralytics YOLOE11

本示例展示了如何使用 `hbm_runtime` 在 BPU 上运行 Ultralytics YOLOE11 实例分割模型。程序实现了从输入图像的预处理、模型推理、后处理到结果可视化的完整流程，本示例代码位于`/app/pydev_demo/05_open_instance_seg_sample/01_yoloe11_seg/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLOE11 是一款高效能的端侧实例分割模型，适用于开放词汇物体检测与分割任务。该模型通过多尺度特征提取、密集分类和原型掩膜生成，有效识别图像中的物体并输出精细的实例分割结果。本示例使用的是 Ultralytics YOLOE11 的轻量级版本，输入图像为 640x640，支持 4585 类的广义物体分类与分割。

- HBM模型名称：yoloe_11s_seg_pf_nashe_640x640_nv12.hbm

- 输入格式：NV12，大小为 640x640

- 输出：

    - 检测框（xyxy 格式）

    - 类别 ID 和置信度分数

    - 实例分割掩膜（每个实例一个独立掩膜）

- 模型下载地址（程序自动下载）：

    ```bash
    https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm
    ```
## 功能说明
- 模型加载

    使用 `hbm_runtime` 加载指定量化模型，解析输入输出名称、形状、量化参数等元信息。

- 输入预处理

    将 BGR 图像 resize 到 640x640，转换为 NV12 格式（Y、UV 分离），构造成推理输入张量。

- 推理执行

    调用 .run() 接口执行前向推理，支持设置运行优先级和 BPU 核绑定等调度策略。

- 结果后处理

    对多尺度输出进行后处理，包括：

    - 分类置信度筛选（基于 score threshold）

    - DFL 边框解码

    - 掩膜原型融合与掩膜生成

    - NMS 过滤与结果融合

    - 将检测框和掩膜缩放回原图尺寸

    - 支持可选的掩膜开运算（形态学处理）与边界轮廓绘制

## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../../requirements.txt
```

## 目录结构
```text
.
├── yoloe11_seg.py              # 主推理脚本
└── README.md                   # 使用说明
```

## 参数说明
| 参数名               | 说明                                     | 默认值                                       |
| ----------------- | ------------------------------------------ | ----------------------------------------- |
| `--model-path`    | BPU 量化模型路径（\*.hbm）                  | `/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm` |
| `--test-img`      | 输入测试图像路径                            | `/app/res/assets/office_desk.jpg`            |
| `--label-file`    | 类别标签文件路径（每行一个类别）             | `/app/res/labels/coco_extended.names`        |
| `--img-save-path` | 推理结果图像保存路径                        | `result.jpg`                              |
| `--priority`      | 模型调度优先级（0\~255）                    | `0`                                       |
| `--bpu-cores`     | 使用的 BPU 核心编号（如 `--bpu-cores 0 1`） | `[0]`                                     |
| `--nms-thres`     | 非极大值抑制（NMS）的 IoU 阈值              | `0.7`                                     |
| `--score-thres`   | 目标检测置信度阈值                          | `0.25`                                    |
| `--is-open`       | 是否对掩码进行形态学操作（开操作）           | `False`                                   |
| `--is-point`      | 是否绘制掩码边缘轮廓点                      | `False`                                   |


## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        python yoloe11_seg.py
        ```
    - 指定参数运行
        ```bash
        python yoloe11_seg.py \
        --model-path /opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --test-img /app/res/assets/office_desk.jpg \
        --label-file /app/res/labels/coco_extended.names \
        --img-save-path result.jpg \
        --nms-thres 0.7 \
        --score-thres 0.25 \
        --is-open False \
        --is-point False
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
