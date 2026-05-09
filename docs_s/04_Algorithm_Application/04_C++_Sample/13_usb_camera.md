---
sidebar_position: 13
---

# USB Camera YOLOv5x 推理

基于BPU的 Ultralytics YOLOv5x 实时推理示例，支持通过 USB 摄像头读取画面并进行目标检测，并以全屏方式可视化检测结果，本示例代码位于`/app/cdev_demo/bpu/09_usb_camera_sample/`目录下。

## 功能说明
- 模型加载

    加载指定的 .hbm 模型文件，提取模型相关的元信息等。

- 摄像头采集

    自动扫描 /dev/video* 下的设备，打开第一个可用的 USB 摄像头，设置为 MJPEG 编码、1080p 分辨率、30 FPS。

- 图像预处理

    将 BGR 图像 resize 至模型输入分辨率（letterbox 模式或普通缩放），并转换为 NV12 格式。

- 推理执行

    通过 infer() 方法提交输入张量，在 BPU 上完成模型前向计算。

- 后处理

    包括量化输出解码、候选框筛选（按 score 阈值过滤）、NMS 去重，以及坐标还原回原始图像大小。

- 可视化显示

    将检测框及其类别、置信度绘制在图像上，并在窗口中全屏显示，支持实时处理和退出控制。

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
|-- CMakeLists.txt                 # CMake 构建脚本：目标/依赖/包含与链接配置
|-- README.md                      # 使用说明（当前文件）
|-- inc
|   `-- ultralytics_yolov5x.hpp    # YOLOv5x 推理封装头文件：加载/预处理/推理/后处理接口
`-- src
    |-- main.cc                    # 程序入口：摄像头探测→取流→推理→绘制→显示（全屏窗口）
    `-- ultralytics_yolov5x.cc     # 推理实现：letterbox、NV12张量写入、解码、NMS、框复原
```

## 编译工程
- 配置与编译
    ```bash
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    ```

## 参数说明
| 参数名            | 说明                                          | 默认值                                                |
| ---------------- | --------------------------------------------- | ----------------------------------------------------- |
| `--video_device` | 指定视频设备（如 `/dev/video0`；为空则自动探测） | `""`（空：自动在 `/dev/video*` 中探测第一个可打开的设备） |
| `--model_path`   | BPU 量化模型路径（`.hbm`）                     | `/opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm` |
| `--label_file`   | 类别标签文件（逐行一个类别名）                  | `/app/res/labels/coco_classes.names`                      |
| `--score_thres`  | 置信度阈值                                    | `0.25`                                                 |
| `--nms_thres`    | NMS 的 IoU 阈值                               | `0.45`                                                 |


## 快速运行
注意：该程序需运行在桌面环境。
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./usb_camera
        ```
    - 指定参数运行
        ```bash
        ./usb_camera \
            --video_device /dev/video0 \
            --model_path /opt/hobot/model/s100/basic/yolov5x_672x672_nv12.hbm \
            --label_file /app/res/labels/coco_classes.names \
            --score_thres 0.25 \
            --nms_thres 0.45
        ```
- 退出运行

    把鼠标放置在显示框中，按q键退出

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
