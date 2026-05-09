---
sidebar_position: 16
---

# RTSP视频拉流及YOLOv5x 推理

本示例演示如何在 RDK S100 等平台上，结合 SP 硬件模块（解码器、VIO、显示）和 BPU，实现：
RTSP/H.264 视频流 → 硬件解码 (NV12) → YOLOv5x 推理 → 叠加检测框 → 实时显示，本示例代码位于`/app/pydev_demo/12_rtsp_yolov5x_display_sample/`目录下。

## 功能说明

- 模型加载 (Model Load)

    使用 hbm_runtime.HB_HBMRuntime(model_path) 加载 YOLOv5x 模型，读取输入输出信息，可通过 set_scheduling_params() 设置 BPU 优先级与核心绑定。

- 前处理 (Preprocess)

    从解码线程获取 NV12 帧，拆分 Y/UV，调整为模型输入尺寸，打包成 BPU 输入格式。

- 模型推理 (Inference)

    调用 self.model.run() 执行前向推理，生成检测结果。

- 后处理 (Postprocess)

    对推理结果反量化、解码、过滤、NMS，并映射到显示坐标，输出检测框与类别。

- RTSP 解码 (RTSP + HW Decoder)

    子线程用 cv2.VideoCapture 拉取 H.264 码流，经 srcampy.Decoder 硬件解码为 NV12 帧。

- 分辨率与显示 (VPS + Display)

    调用 srcampy.Display() 与 srcampy.Camera().open_vps() 建立 VPS→HDMI 显示管线。

- 绘制检测结果 (Overlay Drawing)

    使用 draw.draw_detections_on_disp() 在显示层绘制检测框与类别文字。

- 信号与退出 (Signal Handling)

    捕获 SIGINT（Ctrl+C），设置 is_stop=True，安全退出主循环与子线程，依次关闭 VPS、显示与解码。

- 多线程与帧缓存 (Threading & Queue)

    DecodeRtspStream 继承 threading.Thread，维护帧队列；主线程通过 get_frame() 获取最新帧。

- 参数解析 (Argument Parsing)

    通过 argparse 提供参数：RTSP 源、模型路径、BPU 核心、优先级、标签文件、NMS 与置信度阈值。

- HDMI 分辨率探测 (Display Resolution)

    调用 /usr/bin/get_hdmi_res 获取当前 HDMI 分辨率，若无则默认 1920×1080。

## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小节](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。


## 环境依赖
本样例无特殊环境需求，只需确保安装了pydev中的环境依赖即可。
```bash
pip install -r ../requirements.txt
```

## 目录结构

```text
.
├── README.md               # 使用说明
└── rtsp_yolov5x_display.py # 主程序
```

## 参数说明
| 参数名                  | 说 明                       | 默认值                                                 |
| ----------------------- | -------------------------- | ------------------------------------------------------ |
| `--rtsp-urls` / `-u` | RTSP 视频流地址（可用分号分隔多路流，例如：`rtsp://192.168.1.10/stream1;rtsp://192.168.1.11/stream2`）                                     | `rtsp://127.0.0.1/1080P_test.h264`                          |
| `--model-path`  | BPU 量化模型路径（`.hbm`）          | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--priority`    | 推理优先级（0\~255，255为最高）     | `0`                                                    |
| `--bpu-cores`   | BPU 核心索引列表（如 `0 1`）        | `[0]`                                                  |
| `--label-file`  | 类别标签文件路径                    | `/app/res/labels/coco_classes.names`                   |
| `--nms-thres`   | 非极大值抑制的 IoU 阈值             | `0.45`                                                 |
| `--score-thres` | 检测置信度阈值                      | `0.25`                                                 |


## 快速运行
- 准备rtsp码流

    使用系统预置的推流服务,准备rtsp码流作为输入源，该服务会把1080P_test.h264视频文件处理成 rtsp 流，url 地址为rtsp://127.0.0.1/assets/1080P_test.h264，用户可通过如下命令启动推流服务：
    ```bash
    cd /app/res
    sudo chmod +x live555MediaServer
    sudo ./live555MediaServer &
    ```
- 运行模型
    - 使用默认参数
        ```bash
        python rtsp_yolov5x_display.py
        ```
    - 指定参数运行
        ```bash
        python rtsp_yolov5x_display.py \
        --rtsp-urls rtsp://127.0.0.1/assets/1080P_test.h264 \
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
