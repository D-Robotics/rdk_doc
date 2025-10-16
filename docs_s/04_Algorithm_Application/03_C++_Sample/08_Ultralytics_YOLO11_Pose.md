---
sidebar_position: 8
---

# 姿态估计-Ultralytics YOLO11

本示例展示了如何在 BPU 上运行 Ultralytics YOLO11 姿态估计模型，实现人体关键点检测与可视化。支持模型预处理、推理执行与后处理（含关键点解码、边界框绘制、关键点标注），本示例代码位于`/app/cdev_demo/bpu/04_pose_sample/01_ultralytics_yolo11_pose/`目录下。

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

## 功能说明
- 模型加载

    加载指定的 Ultralytics YOLO11 姿态估计模型，自动解析模型的元数据。

- 输入预处理

    将输入的 BGR 图像 resize 到 640×640，并转为 NV12 格式（Y、UV 分离），用于模型推理。

- 推理执行

    调用 .infer() 接口执行推理。

- 结果后处理

    - 解码多尺度输出中的边界框（使用 DFL 分箱解码）；

    - 解码关键点位置与置信度（K×2 + K）；

    - 使用 NMS 去除冗余检测框；

    - 将关键点坐标和边界框映射回原始图像尺寸；

    - 提供阈值控制仅显示高置信度关键点；

    - 支持图像可视化，包括检测框与关键点绘制。


## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt                     # CMake 构建脚本：目标/依赖/包含路径配置
|-- README.md                          # 使用说明（当前文件）
|-- inc
|   `-- ultralytics_yolo11_pose.hpp    # 模型封装头文件：加载/预处理/推理/后处理接口声明
`-- src
    |-- main.cc                        # 程序入口：解析参数→完整 pipeline→保存可视化结果
    `-- ultralytics_yolo11_pose.cc     # 模型实现：解码、NMS、关键点后处理与坐标还原
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
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yolo11n_pose_nashe_640x640_nv12.hbm
```

## 参数说明
| 参数名                | 说明                     | 默认值                                                   |
| ------------------ | ---------------------- | ----------------------------------------------------- |
| `--model_path`     | 模型文件路径（`.hbm`）              | `/opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm` |
| `--test_img`       | 输入测试图片路径                    | `/app/res/assets/bus.jpg`                             |
| `--label_file`     | 类别标签文件（每行一个类别名称）       | `/app/res/labels/coco_classes.names`                  |
| `--score_thres`    | 置信度阈值（低于该值的检测将被过滤）     | `0.25`                                                |
| `--nms_thres`      | IoU 阈值（类别内 NMS 去重）                | `0.7`                                                 |
| `--kpt_conf_thres` | 关键点可视化置信度阈值（低于该值的点不显示） | `0.5`                                                 |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./ultralytics_yolo11_pose
        ```
    - 指定参数运行
        ```bash
        ./ultralytics_yolo11_pose \
        --model_path /opt/hobot/model/s100/basic/yolo11n_pose_nashe_640x640_nv12.hbm \
        --test_img   /app/res/assets/bus.jpg \
        --label_file /app/res/labels/coco_classes.names \
        --score_thres 0.25 \
        --nms_thres   0.7 \
        --kpt_conf_thres 0.5
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
