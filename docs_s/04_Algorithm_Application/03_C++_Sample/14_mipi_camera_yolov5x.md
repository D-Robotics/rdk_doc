---
sidebar_position: 14
---

# MIPI Camera YOLOv5x 推理

基于 BPU 的 Ultralytics YOLOv5x 实时推理示例，支持通过 MIPI 摄像头读取画面并进行目标检测，并以全屏方式可视化检测结果，本示例代码位于`/app/cdev_demo/bpu/10_mipi_camera_sample/`目录下。

## 功能说明

- 模型加载

    加载 `.hbm` 格式模型，初始化输入输出信息;

- 摄像头采集

    初始化 VIO 摄像头，采集 1920×1080 分辨率 NV12 图像;

- HDMI 显示

    绑定图像输出通道，实现实时显示;

- 图像预处理

    对 NV12 格式图像进行分离、缩放、转换为 BPU 所需张量格式;

- BPU 推理

    通过 .infer() 方法调用 BPU 执行推理任务;

- 结果后处理

    包括输出解码、置信度筛选、NMS 抑制、坐标缩放;

- 可视化显示

    将检测框和类别文本绘制到 overlay 层;


## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小节](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。


## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 硬件环境
- mipi camera的接口使用的自动检测模式，该sample运行时只能接入一个mipi摄像头（任意mipi接口都可以），同时接入多个会报错。
- 目前该sample仅支持MIPI sensor: IMX219, SC230AI
- mipi摄像头的安装方法可参考[相机扩展板-MIPI 相机接口](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md)部分。

## 目录结构
```text
.
|-- CMakeLists.txt                     # CMake 构建脚本：目标/依赖/包含与链接
|-- README.md                          # 使用说明（当前文件）
|-- inc
|   `-- ultralytics_yolov5x.hpp        # YOLOv5x 推理封装头：加载/预处理/推理/后处理接口
`-- src
    |-- main.cc                        # 程序入口：VIO取流→推理→在 Display 图层绘制
    `-- ultralytics_yolov5x.cc         # 推理实现：letterbox、NV12张量写入、解码、NMS、复原坐标
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 参数说明
| 参数名           | 说明                                  | 默认值                                                  |
| --------------- | ------------------------------------- | ------------------------------------------------------ |
| `--width`       | 传感器原始宽度（用于 VIO 参数/显示缩放） | `1920`                                                 |
| `--height`      | 传感器原始高度（用于 VIO 参数/显示缩放） | `1080`                                                 |
| `--model_path`  | BPU 量化模型路径（`.hbm`）             | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`  |
| `--label_file`  | 类别标签文件（逐行一个类别名）           | `/app/res/labels/coco_classes.names`                      |
| `--score_thres` | 置信度阈值                             | `0.25`                                                 |
| `--nms_thres`   | NMS 的 IoU 阈值                        | `0.45`                                                |

## 快速运行
注意：该程序需运行在桌面环境。
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./mipi_camera
        ```
    - 指定参数运行
        ```bash
        ./mipi_camera \
            --width 1920 --height 1080 \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.45
        ```
- 退出运行

    在命令行输入Ctrl C

- 查看结果

    运行成功后，屏幕会实时显示目标检测图像

## 注意事项
- 该程序需运行在桌面环境。

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
