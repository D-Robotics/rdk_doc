---
sidebar_position: 9
---

# 实例分割-Ultralytics YOLOE11

本示例展示了如何在 BPU 上运行 Ultralytics YOLOE11 实例分割模型。程序实现了从输入图像的预处理、模型推理、后处理到结果可视化的完整流程，本示例代码位于`/app/cdev_demo/bpu/05_open_instance_seg_sample/01_yoloe11_seg/`目录下。

## 模型说明
- 简介：

    Ultralytics YOLOE11 是一款高效能的端侧实例分割模型，适用于开放词汇物体检测与分割任务。该模型通过多尺度特征提取、密集分类和原型掩膜生成，有效识别图像中的物体并输出精细的实例分割结果。本示例使用的是 Ultralytics YOLOE11 的轻量级版本，输入图像为 640x640，支持 4585 类的广义物体分类与分割。

- HBM模型名称：yoloe_11s_seg_pf_nashe_640x640_nv12.hbm

- 输入格式：NV12，大小为 640x640

- 输出：

    - 检测框（xyxy 格式）

    - 类别 ID 和置信度分数

    - 实例分割掩膜（每个实例一个独立掩膜）

## 功能说明
- 模型加载

    加载指定量化模型，解析模型的部分元信息。

- 输入预处理

    将 BGR 图像 resize 到 640x640，转换为 NV12 格式（Y、UV 分离），构造成推理输入张量。

- 推理执行

    调用 .infer() 接口执行前向推理。

- 结果后处理

    对多尺度输出进行后处理，包括：

    - 分类置信度筛选（基于 score threshold）

    - DFL 边框解码

    - 掩膜原型融合与掩膜生成

    - NMS 过滤与结果融合

    - 将检测框和掩膜缩放回原图尺寸

    - 支持可选的掩膜开运算（形态学处理）与边界轮廓绘制

## 环境依赖
在编译运行前，请确保安装以下依赖：
```bash
sudo apt update
sudo apt install libgflags-dev
```

## 目录结构
```text
.
|-- CMakeLists.txt                      # CMake 构建脚本：目标/依赖/包含路径/链接库配置
|-- README.md                           # 使用说明（当前文件）
|-- inc
|   `-- ultralytics_yoloe11_seg.hpp     # YOLO11E_Seg 封装头文件：加载/预处理/推理/后处理接口声明
`-- src
    |-- main.cc                         # 程序入口：解析参数→完整 pipeline→渲染并保存结果
    `-- ultralytics_yoloe11_seg.cc      # 推理实现：解码、分数过滤、按类 NMS、掩码生成与还原
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
wget https://archive.d-robotics.cc/downloads/rdk_model_zoo/rdk_s100/ultralytics_YOLO/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm
```

## 参数说明
| 参数名             | 说明                    | 默认值                                                       |
| --------------- | --------------------- | --------------------------------------------------------- |
| `--model_path`  | 模型文件路径（`.hbm`）        | `/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm` |
| `--test_img`    | 输入测试图片路径              | `/app/res/assets/office_desk.jpg`                         |
| `--label_file`  | 类别标签文件（每行一个类别名）       | `/app/res/labels/coco_extended.names`                     |
| `--score_thres` | 置信度阈值（低于该值的检测将被过滤）    | `0.25`                                                    |
| `--nms_thres`   | IoU 阈值（类别内 NMS 去重）    | `0.7`                                                     |

## 快速运行
- 运行模型
    - 确保在`build`目录中
    - 使用默认参数
        ```bash
        ./ultralytics_yoloe11_seg
        ```
    - 指定参数运行
        ```bash
        ./ultralytics_yoloe11_seg \
            --model_path /opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm \
            --test_img   /app/res/assets/office_desk.jpg \
            --label_file /app/res/labels/coco_extended.names \
            --score_thres 0.25 \
            --nms_thres   0.7
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

## 注意事项
- 若指定模型路径不存在，程序将尝试自动下载模型。
