---
sidebar_position: 4
---

# 目标检测-Ultralytics YOLOv5x

本示例展示如何在 BPU 上使用量化后的 Ultralytics YOLOv5x 模型执行图像目标检测。支持前处理、后处理、NMS 以及最终的目标框绘制和结果保存，本示例代码位于`/app/cdev_demo/bpu/02_detection_sample/01_ultralytics_yolov5x/`目录下。


## 模型说明
- 简介：

    Ultralytics YOLOv5x 是一类高性能的目标检测模型，其名称代表 "You Only Look Once"，可实现单次前向推理完成目标定位与分类。Ultralytics YOLOv5x 是其中最大的变体，具备更多的网络参数，检测精度高，适用于对准确率要求较高的场景。Ultralytics YOLOv5x 模型将输入图像映射为多个网格，每个网格预测多个 anchor 的类别和边界框。本模型已量化为适用于BPU芯片的 HBM 格式，输入尺寸为 672×672 的 NV12 图像。

- HBM 模型名称： yolov5x_672x672_nv12.hbm

- 输入格式： NV12，大小为 672x672（Y、UV 分离）

- 输出： N 个目标框，每个目标包含 (类别索引、概率、坐标框) 三元组


## 功能说明
- 模型加载

    加载 Ultralytics YOLOv5x 量化模型，解析模型相关信息，为后续推理配置做好准备。

- 输入预处理

    将输入图像 resize 到 672x672 后转换为 NV12 格式（Y、UV 分离），并以嵌套字典形式组织输入，以适配推理接口。

- 推理执行

    利用 .infer() 方法运行推理过程。

- 结果后处理

    - 对输出 tensor 进行 dequant 去量化；

    - 解码 YOLO 输出，得到预测框、置信度、类别索引；

    - 根据 score 阈值进行初筛；

    - 应用 NMS（非极大值抑制）去除冗余框；

    - 将预测框坐标映射回原图尺寸；

    - 叠加检测框并保存结果图像。


## 环境依赖
请先确保系统已安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构
```text
.
├── CMakeLists.txt                 # 构建配置文件
├── README.md                      # 使用说明
├── inc
│   └── ultralytics_yolov5x.hpp     # YOLOv5x 模型封装类定义
└── src
    ├── main.cc                     # 推理入口 (解析参数、执行)
    └── ultralytics_yolov5x.cc      # YOLOv5x 推理逻辑实现
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 参数说明
| 参数              | 说明                       | 默认值                                     |
| --------------- | --------------------------- | ------------------------------------------ |
| `--model-path`  | 模型文件路径（.hbm 格式）     | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--test-img`    | 测试图片路径                 | `/app/res/assets/kite.jpg`                 |
| `--label-file`  | 类别标签文件路径             | `/app/res/labels/coco_classes.names`       |
| `--score-thres` | 置信度阈值 (过滤低分框)      | `0.25`                                     |
| `--nms-thres`   | IoU 阈值 (NMS 非极大值抑制)  | `0.45`                                     |

## 快速运行
- 运行模型
    - 使用默认参数
        ```bash
        ./ultralytics_yolov5x
        ```
    - 指定参数运行
        ```bash
        ./ultralytics_yolov5x \
            --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --test-img /app/res/assets/kite.jpg \
            --label-file /app/res/labels/coco_classes.names \
            --score-thres 0.25 \
            --nms-thres 0.45
        ```
- 查看结果

    运行成功后，会将目标检测框绘制在原图上，并保存在 build/result.jpg
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
