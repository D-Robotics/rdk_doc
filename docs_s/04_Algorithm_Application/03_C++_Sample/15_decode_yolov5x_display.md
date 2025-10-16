---
sidebar_position: 15
---

# 视频解码及YOLOv5x 推理

本示例演示如何在 RDK S100 等平台上，使用 SP 解码/显示/VIO 与 BPU 组合完成：
本地 H.264 文件 → 硬件解码 (NV12) → YOLOv5X 推理 → 叠加框到显示图层 的端到端流程，本示例代码位于`/app/cdev_demo/bpu/11_decode_yolov5x_display_sample`目录下。

## 功能说明

- 模型加载

    加载模型，获取输入/输出相关信息。

- 前处理 (Preprocess)

    将从 VIO 获取的 NV12 帧转换为 BGR（cv::cvtColor），进行 letterbox/缩放，写入 NV12 输入张量。

- 模型推理 (Inference)

    调用 infer() 在 BPU 上执行前向计算。

- 后处理 (Postprocess)

    调用 yolov5x.post_process(score_thres, nms_thres, W, H)，完成解码、置信度过滤与 NMS，并将框坐标还原到原分辨率。

- 相机管理 (VIO)

    通过 sp_open_camera_v2 打开传感器通道，sp_vio_get_yuv 拉取 NV12 帧。

- 屏幕叠加 (SP Display)

    sp_start_display 初始化显示通道；draw_detections_on_disp 将检测结果绘制到屏幕。

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
|-- CMakeLists.txt                     # CMake 构建脚本（目标/依赖/包含与链接）
|-- README.md                          # 本使用说明
|-- inc
|   `-- ultralytics_yolov5x.hpp        # YOLOv5x 封装头：加载/预处理/推理/后处理接口
`-- src
    |-- main.cc                        # 程序入口：H.264 解码→推理→显示叠加（Ctrl+C 退出）
    `-- ultralytics_yolov5x.cc         # YOLOv5x 实现：letterbox、NV12 张量写入、解码、NMS、还原坐标
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 参数说明
| 参数名           | 说明                                                 | 默认值                                                   |
| --------------- | ---------------------------------------------------- | -------------------------------------------------------- |
| `--width`       | 源码流/解码期望宽度（像素）                             | `1920`                                                  |
| `--height`      | 源码流/解码期望高度（像素）                             | `1080`                                                  |
| `--input_path`  | 输入 H.264 文件路径（示例为本地文件；也可扩展为码流管道） | `/app/res/assets/1080P_test.h264`                          |
| `--model_path`  | YOLOv5X 量化模型（`.hbm`）路径                         | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm`  |
| `--label_file`  | 类别名列表文件（逐行一个类别名）                         | `/app/res/labels/coco_classes.names`                      |
| `--score_thres` | 置信度阈值（过滤低分框）                                | `0.25`                                                  |
| `--nms_thres`   | NMS IoU 阈值                                          | `0.45`                                                  |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./decode_yolov5x_display
        ```
    - 指定参数运行
        ```bash
        ./decode_yolov5x_display \
            --width 1920 --height 1080 \
            --input_path /app/res/assets/1080P_test.h264 \
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
