---
sidebar_position: 15
---

# WebSocket YOLOv5x 推理

本示例展示了如何在含有 HBM 加速器和 VIO 摄像头模块的嵌入式平台（如 RDK S100）上，使用 Ultralytics YOLOv5x 模型进行目标检测，并通过 WebSocket 实时推送 JPEG 图像和检测框，本示例代码位于`/app/pydev_demo/11_web_display_camera_sample/`目录下。

## 功能说明

- 模型加载

    初始化 `hbm_runtime`，加载模型，获取输入/输出名称和尺寸

- 前处理 (Preprocess)

    将原始的 NV12 图像分割为 Y/UV 通道，缩放到模型需求尺寸，生成正确格式的输入 Tensor

- 模型推理 (Inference)

    调用.run() 执行 BPU 推理

- 后处理 (Postprocess)

    解码推理结果，扫除缺乏精度和应用 NMS，将结果缩放回原图尺寸

- 相机管理 (CameraManager)

    打开相机，获取原始图像或模型尺寸图像，进行 JPEG 编码

- WebSocket 服务器

    接受网页端连接，持续获取相机图像，执行检测并给网页端返回 Protocol Buffer 结果

## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小结](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。

## 环境依赖
- 确保安装了pydev中的环境依赖
    ```bash
    pip install -r ../requirements.txt
    ```
- 安装WebSocket的包
    ```bash
    pip install websockets==15.0.1 protobuf==3.20.3
    ```

## 硬件环境
- mipi camera的接口使用的自动检测模式，该sample运行时只能接入一个mipi摄像头（任意mipi接口都可以），同时接入多个会报错。
- 目前该sample仅支持MIPI sensor: IMX219, SC230AI
- mipi摄像头的安装方法可参考[相机扩展板-MIPI 相机接口](../../01_Quick_start/01_hardware_introduction/02_rdk_s100_camera_expansion_board.md)部分。

## 目录结构
```text
.
├── mipi_camera_web_yolov5x.py      # 主程序
└── README.md                       # 使用说明
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
- 启动服务
    ```bash
    # 1. 进入webservice目录
    cd webservice/

    # 2. 启动服务
    sudo ./sbin/nginx -p .
    ```
- 运行模型
    - 回到当前目录
        ```bash
        cd ..
        ```
    - 使用默认参数
        ```bash
        python mipi_camera_web_yolov5x.py
        ```
    - 指定参数运行
        ```bash
        python mipi_camera_web_yolov5x.py \
        --model-path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
        --priority 0 \
        --bpu-cores 0 \
        --label-file /app/res/labels/coco_classes.names \
        --nms-thres 0.45 \
        --score-thres 0.25
        ```

- 查看结果

    运行成功后，通过访问web展示端：http://IP

    **注意：无需填写端口**

- 退出运行

    在命令行输入Ctrl C

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
