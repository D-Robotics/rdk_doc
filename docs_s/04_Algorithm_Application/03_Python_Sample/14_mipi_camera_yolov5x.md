---
sidebar_position: 14
---

# MIPI Camera YOLOv5x 推理

基于 `hbm_runtime` 的 Ultralytics YOLOv5x 实时推理示例，支持通过 MIPI 摄像头读取画面并进行目标检测，并以全屏方式可视化检测结果，本示例代码位于`/app/pydev_demo/10_mipi_camera_sample/`目录下。

## 功能说明

- 模型加载

    通过 `hbm_runtime` 加载 `.hbm` 格式模型，初始化输入输出信息;

- 摄像头采集

    使用 `srcampy.Camera()` 初始化 VIO 摄像头，采集 1920×1080 分辨率 NV12 图像;

- HDMI 显示

    使用 `srcampy.Display()` 绑定图像输出通道，实现实时显示;

- 图像预处理

    对 NV12 格式图像进行分离、缩放、转换为 BPU 所需张量格式;

- BPU 推理

    通过 `run()` 方法调用 BPU 执行推理任务;

- 结果后处理

    包括输出解码、置信度筛选、NMS 抑制、坐标缩放;

- 可视化显示

    将检测框和类别文本绘制到 overlay 层;


## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小结](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。


## 环境依赖
- 确保安装了pydev中的环境依赖
    ```bash
    pip install -r ../requirements.txt
    ```

## 硬件环境
- mipi camera的接口使用的自动检测模式，该sample运行时只能接入一个mipi摄像头（任意mipi接口都可以），同时接入多个会报错。
- 目前该sample仅支持MIPI sensor: IMX219, SC230AI
- mipi摄像头的安装方法可参考[相机扩展板-MIPI 相机接口](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md)部分。

## 目录结构

```text
.
├── 01_mipi_camera_yolov5x.py       # 使用 YOLOv5X 模型进行摄像头实时目标检测与显示
├── 02_mipi_camera_dump.py          # 将摄像头捕获的图像帧以 YUV 格式保存为文件(与模型推理无关)
├── 03_mipi_camera_scale.py         # 对本地 YUV 图像进行分辨率缩放处理(与模型推理无关)
├── 04_mipi_camera_crop_scale.py    # 对本地 YUV 图像裁剪并缩放处理(与模型推理无关)
├── 05_mipi_camera_streamer.py      # 将摄像头图像实时显示至 HDMI 屏幕（推流测试）(与模型推理无关)
└── README.md                       # 当前文件，包含脚本功能说明、参数介绍及使用方法
```

## 参数说明
| 参数名           | 说明                              | 默认值                                                    |
| --------------- | --------------------------------- | ------------------------------------------------------ |
| `--model-path`  | BPU 量化模型路径（`.hbm`）          | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--priority`    | 推理优先级（0\~255，255为最高）     | `0`                                                    |
| `--bpu-cores`   | BPU 核心索引列表（如 `0 1`）        | `[0]`                                                  |
| `--label-file`  | 类别标签文件路径                    | `/app/res/labels/coco_classes.names`                         |
| `--nms-thres`   | 非极大值抑制的 IoU 阈值             | `0.45`                                                 |
| `--score-thres` | 检测置信度阈值                      | `0.25`                                                 |


## 快速运行
注意：该程序需运行在桌面环境。
- 运行模型
    - 使用默认参数
        ```bash
        python 01_mipi_camera_yolov5x.py
        ```
    - 指定参数运行
        ```bash
        python 01_mipi_camera_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```
- 退出运行

    在命令行输入Ctrl C

- 查看结果

    运行成功后，屏幕会实时显示目标检测图像

## 注意事项
- 该程序需运行在桌面环境。

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
