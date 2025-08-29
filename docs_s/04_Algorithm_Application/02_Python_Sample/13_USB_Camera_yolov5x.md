---
sidebar_position: 13
---

# USB Camera YOLOv5x 推理

基于 `hbm_runtime` 的 Ultralytics YOLOv5x 实时推理示例，支持通过 USB 摄像头读取画面并进行目标检测，并以全屏方式可视化检测结果，本示例代码位于`/app/pydev_demo/09_usb_camera_sample/`目录下。

## 功能说明
- 模型加载

    通过 `hbm_runtime` 加载指定的 .hbm 模型文件，提取模型名称、输入输出形状、量化信息等。

- 摄像头采集

    自动扫描 /dev/video* 下的设备，打开第一个可用的 USB 摄像头，设置为 MJPEG 编码、1080p 分辨率、30 FPS。

- 图像预处理

    将 BGR 图像 resize 至模型输入分辨率（letterbox 模式或普通缩放），并转换为 NV12 格式。

- 推理执行

    通过 run() 方法提交输入张量，在 BPU 上完成模型前向计算。

- 后处理

    包括量化输出解码、候选框筛选（按 score 阈值过滤）、NMS 去重，以及坐标还原回原始图像大小。

- 可视化显示

    将检测框及其类别、置信度绘制在图像上，并在窗口中全屏显示，支持实时处理和退出控制。

## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小结](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。

## 环境依赖
- 确保安装了pydev中的环境依赖
    ```bash
    pip install -r ../requirements.txt
    ```

## 目录结构
```text
.
├── usb_camera_yolov5x.py       # 主程序
└── README.md                   # 使用说明
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
        python usb_camera_yolov5x.py
        ```
    - 指定参数运行
        ```bash
        python usb_camera_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```
- 退出运行

    按Q键退出

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
