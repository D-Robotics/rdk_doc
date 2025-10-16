---
sidebar_position: 16
---

# RTSP视频拉流及YOLOv5x 推理

本示例演示如何在 RDK S100 等平台上，结合 SP 硬件模块（解码器、VIO、显示）和 BPU，实现：
RTSP/H.264 视频流 → 硬件解码 (NV12) → YOLOv5x 推理 → 叠加检测框 → 实时显示，本示例代码位于`/app/cdev_demo/bpu/12_rtsp_yolov5x_display_sample/`目录下。

## 功能说明

- 模型加载 (Model Load)

    使用 YOLOv5x(model_path) 加载 BPU 模型，并通过 load_linewise_labels 获取类别名称列表，准备后续推理使用。

- 前处理 (Preprocess)

    从 SP 解码器获取 NV12 帧（sp_decoder_get_image），转换为 BGR（cv::cvtColor），进行缩放/letterbox 处理，并写入 YOLOv5x 输入张量（pre_process）。

- 模型推理 (Inference)

    调用 yolov5x.infer() 在 BPU 上执行前向计算，生成原始检测结果。

- 后处理 (Postprocess)

    调用 yolov5x.post_process完成置信度过滤、NMS，并将检测框坐标映射回显示分辨率。

- RTSP 拉流与解码 (SP Decoder / FFmpeg)

    使用 FFmpeg 初始化网络栈 (avformat_network_init)，打开 RTSP 流 (avformat_open_input)，并通过 SP 模块拉取 H264 视频帧（sp_start_decode、sp_decoder_get_image）。

- 分辨率适配与缩放 (VPS)

    若显示分辨率与视频流分辨率不一致，使用 SP VPS 模块进行缩放 (sp_open_vps)，并通过 sp_module_bind 将解码器、VPS、显示模块绑定成管线。

- 屏幕显示 (SP Display)

    通过 sp_start_display 初始化显示通道；使用 draw_detections_on_disp 将检测结果叠加绘制到屏幕；若分辨率一致，可直接通过 sp_display_set_image 显示 YUV 帧。

- 信号控制 (Signal Handler)

    注册 signal_handler_func 捕获 SIGINT 等信号，设置全局标志 is_stop，以便主循环安全退出。

## 模型说明
    参考 [Ultralytics YOLOv5x 目标检测示例小节](04_Ultralytics_YOLOv5x.md#目标检测-ultralytics-yolov5x)。

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构

```text
.
|-- CMakeLists.txt
|-- README.md
|-- inc
|   `-- ultralytics_yolov5x.hpp       # YOLOv5x 封装头文件
`-- src
    |-- main.cc                       # 主程序入口：RTSP解码→YOLOv5x推理→显示
    `-- ultralytics_yolov5x.cc        # YOLOv5x 实现：预处理/推理/后处理/NMS
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 参数说明
| 参数名            | 说明                             | 默认值                                                    |
| ----------------- | ------------------------------- | ------------------------------------------------------ |
| `--rtsp_url`      | RTSP 流 URL                     | `rtsp://127.0.0.1/assets/1080P_test.h264`              |
| `--transfer_type` | RTSP 传输类型（tcp/udp）         | `tcp`                                                  |
| `--model_path`    | YOLOv5x 量化 BPU 模型路径 (.hbm) | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--label_file`    | 类别名文件（每行一个类别名）       | `/app/res/labels/coco_classes.names`                   |
| `--score_thres`   | 置信度阈值（过滤低分检测框）       | `0.25`                                                 |
| `--nms_thres`     | NMS IoU 阈值                    | `0.45`                                                 |


## 快速运行
- 准备rtsp码流

    使用系统预置的推流服务,准备rtsp码流作为输入源，该服务会把1080P_test.h264视频文件处理成 rtsp 流，url 地址为rtsp://127.0.0.1/assets/1080P_test.h264，用户可通过如下命令启动推流服务：
    ```bash
    cd /app/res
    sudo chmod +x live555MediaServer
    sudo ./live555MediaServer &
    ```
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./rtsp_yolov5x_display
        ```
    - 指定参数运行
        ```bash
        ./rtsp_yolov5x_display \
            --rtsp_url rtsp://127.0.0.1/assets/1080P_test.h264 \
            --transfer_type tcp \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.3 \
            --nms_thres 0.5
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
